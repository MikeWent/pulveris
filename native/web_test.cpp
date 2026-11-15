#include "core.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <httplib.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

int main() {
    char directory[] = "/tmp/pulveris-web-test-XXXXXX";
    assert(mkdtemp(directory));
    const std::string path = directory;
    int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    assert(socket_fd >= 0);
    sockaddr_in address{}; address.sin_family = AF_INET; address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    assert(bind(socket_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0);
    socklen_t length = sizeof(address);
    assert(getsockname(socket_fd, reinterpret_cast<sockaddr *>(&address), &length) == 0);
    int port = ntohs(address.sin_port); close(socket_fd);
    Shared shared; shared.config_path = path + "/settings.json";
    {
        WebServer server(shared, "127.0.0.1", port, path + "/token", "web.html");
        std::ifstream f(path + "/token"); std::string token; f >> token;
        struct stat info{}; assert(stat((path + "/token").c_str(), &info) == 0 && (info.st_mode & 0777) == 0600);
        httplib::Headers auth{httplib::make_basic_authentication_header("robot", token)};
        httplib::Client client("127.0.0.1", port);
        client.set_read_timeout(3);
        auto unauth = client.Get("/"); assert(unauth && unauth->status == 401);
        auto page = client.Get("/", auth); assert(page && page->status == 200 && page->body.find("Pulveris") != std::string::npos);
        auto post = [&](const char *url, const json &body) { return client.Post(url, auth, body.dump(), "application/json"); };
        auto saved = post("/api/settings", {{"speed", .4}, {"threshold", 150}, {"camera_width", 320}, {"camera_height", 240}});
        assert(saved && saved->status == 200);
        Settings restored; load_settings(restored, shared.config_path);
        assert(restored.speed == .4 && restored.threshold == 150 && restored.camera_width == 320 && restored.camera_height == 240);
        auto invalid_size = post("/api/settings", {{"camera_width", 5000}}); assert(invalid_size && invalid_size->status == 400);
        auto invalid = post("/api/settings", {{"speed", 2}}); assert(invalid && invalid->status == 400);
        restored = Settings{}; load_settings(restored, shared.config_path); assert(restored.speed == .4);
        httplib::Headers foreign = auth; foreign.emplace("Origin", "http://elsewhere");
        auto rejected = client.Post("/api/settings", foreign, R"({"speed":0.5})", "application/json");
        assert(rejected && rejected->status == 403);
        std::string id = "native-test-client-2026", gesture = "native-test-gesture-2026";
        auto mode = post("/api/mode", {{"mode", "remote"}, {"client", id}}); assert(mode && mode->status == 200);
        auto control = post("/api/control", {{"client", id}, {"gesture", gesture}, {"sequence", 1}, {"direction", "forward"}});
        assert(control && control->status == 200 && snapshot(shared).direction == "forward");
        std::this_thread::sleep_for(std::chrono::milliseconds(550));
        assert(snapshot(shared).direction == "stop");
        auto stale = post("/api/control", {{"client", id}, {"gesture", gesture}, {"sequence", 2}, {"direction", "forward"}});
        assert(stale && stale->status == 400);
        auto fresh = post("/api/control", {{"client", id}, {"gesture", "new-native-test-gesture"}, {"sequence", 3}, {"direction", "left"}});
        assert(fresh && fresh->status == 200 && snapshot(shared).direction == "left");
        {
            std::lock_guard<std::mutex> lock(shared.mutex);
            shared.frame = cv::Mat(8, 8, CV_8UC3, cv::Scalar(0, 0, 0));
            ++shared.frame_sequence;
        }
        shared.changed.notify_all();
        std::string received;
        client.Get("/stream", auth, [&](const char *data, std::size_t size) {
            received.append(data, size);
            return received.find("\xff\xd8") == std::string::npos;
        });
        assert(received.find("Content-Type: image/jpeg") != std::string::npos && received.find("\xff\xd8") != std::string::npos);
    }
    std::filesystem::remove_all(path);
    std::cout << "Native web authentication, settings, deadman and stream tests passed\n";
}
