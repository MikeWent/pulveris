#include "core.hpp"

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

#include <fcntl.h>
#include <sys/random.h>
#include <unistd.h>

#include <httplib.h>
#include <opencv2/imgcodecs.hpp>


namespace {
void write_all(int fd, const std::string &data) {
    std::size_t written = 0;
    while (written < data.size()) {
        auto count = ::write(fd, data.data() + written, data.size() - written);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) throw std::runtime_error("Write failed: " + std::string(std::strerror(errno)));
        written += static_cast<std::size_t>(count);
    }
}

std::string token_file(const std::string &path) {
    int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (fd >= 0) {
        try {
            unsigned char bytes[32];
            std::size_t used = 0;
            while (used < sizeof(bytes)) {
                auto n = ::getrandom(bytes + used, sizeof(bytes) - used, 0);
                if (n < 0 && errno == EINTR) continue;
                if (n <= 0) throw std::runtime_error("Cannot generate web password");
                used += static_cast<std::size_t>(n);
            }
            std::ostringstream text;
            for (auto byte : bytes) text << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
            write_all(fd, text.str() + "\n");
            if (::close(fd) < 0) { fd = -1; throw std::runtime_error("Cannot close web password file"); }
            fd = -1;
        } catch (...) {
            if (fd >= 0) ::close(fd);
            ::unlink(path.c_str());
            throw;
        }
    } else if (errno != EEXIST) {
        throw std::runtime_error("Cannot create web password file: " + std::string(std::strerror(errno)));
    }
    std::ifstream file(path);
    std::string token;
    if (!file || !std::getline(file, token)) throw std::runtime_error("Cannot read web password file");
    auto end = token.find_last_not_of(" \t\r\n");
    if (end == std::string::npos) throw std::runtime_error("Web password file is empty");
    token.erase(end + 1);
    auto start = token.find_first_not_of(" \t\r\n");
    return token.substr(start);
}

bool constant_equal(const std::string &a, const std::string &b) {
    std::size_t length = std::max(a.size(), b.size());
    volatile unsigned int diff = static_cast<unsigned int>(a.size() ^ b.size());
    for (std::size_t i = 0; i < length; ++i)
        diff |= static_cast<unsigned char>(i < a.size() ? a[i] : 0) ^ static_cast<unsigned char>(i < b.size() ? b[i] : 0);
    return diff == 0;
}

void respond(httplib::Response &response, const std::string &value, int status = 200) {
    response.status = status;
    response.set_content(value, "application/json");
}
}

struct WebServer::Impl {
    Impl(Shared &state, const std::string &host, int port, const std::string &token_path, const std::string &html_path)
        : shared(state), token(token_file(token_path)) {
        std::ifstream file(html_path, std::ios::binary);
        if (!file) throw std::runtime_error("Cannot read web page: " + html_path);
        page.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
        auto expected = httplib::make_basic_authentication_header("robot", token).second;
        server.set_default_headers({{"Cache-Control", "no-store"}, {"X-Content-Type-Options", "nosniff"}});
        server.set_payload_max_length(16384);
        server.set_read_timeout(3);
        server.set_write_timeout(3);
        server.set_pre_routing_handler([expected](const httplib::Request &req, httplib::Response &res) {
            auto header = req.get_header_value("Authorization");
            if (header.size() >= 6 && header.substr(0, 6) == "basic ") header.replace(0, 6, "Basic ");
            if (constant_equal(header, expected)) return httplib::Server::HandlerResponse::Unhandled;
            res.status = 401;
            res.set_header("WWW-Authenticate", "Basic realm=\"Pulveris\", charset=\"UTF-8\"");
            res.set_content("", "text/plain");
            return httplib::Server::HandlerResponse::Handled;
        });
        server.Get("/", [this](const auto &, auto &res) { res.set_content(page, "text/html; charset=utf-8"); });
        server.Get("/api/state", [this](const auto &, auto &res) {
            respond(res, api_state(shared));
        });
        server.Get("/stream", [this](const auto &, auto &res) {
            auto sequence = std::make_shared<std::uint64_t>(0);
            res.set_chunked_content_provider("multipart/x-mixed-replace; boundary=frame",
                [this, sequence](std::size_t, httplib::DataSink &sink) {
                    cv::Mat image;
                    {
                        std::unique_lock<std::mutex> lock(shared.mutex);
                        shared.changed.wait_for(lock, std::chrono::milliseconds(200), [&] {
                            return !shared.running || shared.frame_sequence != *sequence;
                        });
                        if (!shared.running || !sink.is_writable()) { sink.done(); return true; }
                        if (shared.frame_sequence == *sequence || shared.frame.empty()) return true;
                        *sequence = shared.frame_sequence;
                        image = shared.frame;
                    }
                    std::vector<unsigned char> jpeg;
                    if (!cv::imencode(".jpg", image, jpeg, {cv::IMWRITE_JPEG_QUALITY, 70})) return false;
                    std::string header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + std::to_string(jpeg.size()) + "\r\n\r\n";
                    return sink.write(header.data(), header.size()) &&
                           sink.write(reinterpret_cast<const char *>(jpeg.data()), jpeg.size()) && sink.write("\r\n", 2);
                });
        });
        server.Post(R"(/api/(settings|mode|control))", [this](const auto &req, auto &res) {
            auto content_type = req.get_header_value("Content-Type");
            auto origin = req.get_header_value("Origin");
            if (content_type.substr(0, 16) != "application/json" ||
                (!origin.empty() && origin != "http://" + req.get_header_value("Host"))) {
                respond(res, "{\"error\":\"Invalid origin or content type\"}", 403); return;
            }
            try {
                respond(res, api_post(shared, req.path, req.body));
            } catch (const std::exception &error) { respond(res, api_error(error.what()), 400); }
        });
        if (!server.bind_to_port(host, port)) throw std::runtime_error("Cannot bind web server to " + host + ":" + std::to_string(port));
        worker = std::thread([this] { server.listen_after_bind(); });
    }
    ~Impl() {
        { std::lock_guard<std::mutex> lock(shared.mutex); shared.running = false; }
        shared.changed.notify_all();
        server.stop();
        if (worker.joinable()) worker.join();
    }
    Shared &shared;
    std::string token, page;
    httplib::Server server;
    std::thread worker;
};

WebServer::WebServer(Shared &shared, const std::string &host, int port, const std::string &token_path, const std::string &html_path)
    : impl(std::make_unique<Impl>(shared, host, port, token_path, html_path)) {}
WebServer::~WebServer() = default;
