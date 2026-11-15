#include "core.hpp"

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
namespace fs = std::filesystem;
namespace {
json to_json(const Settings &settings) {
    json result = json::object();
    for (const auto &spec : setting_specs) result[spec.name] = settings.*(spec.field);
    result["light_line"] = settings.light_line;
    return result;
}

json limits_json() {
    json result = json::object();
    for (const auto &spec : setting_specs)
        result[spec.name] = json::array({spec.min, std::isfinite(spec.max) ? json(spec.max) : json(nullptr), spec.step});
    return result;
}

Settings merged(Settings settings, const json &values) {
    if (!values.is_object()) throw std::invalid_argument("Expected an object");
    for (auto it = values.begin(); it != values.end(); ++it) {
        if (it.key() == "light_line") {
            if (!it.value().is_boolean()) throw std::invalid_argument("Invalid light_line");
            settings.light_line = it.value().get<bool>();
            continue;
        }
        auto spec = std::find_if(setting_specs.begin(), setting_specs.end(), [&](const auto &s) { return s.name == it.key(); });
        if (spec == setting_specs.end() || !it.value().is_number())
            throw std::invalid_argument("Unknown or invalid setting: " + it.key());
        settings.*(spec->field) = it.value().get<double>();
    }
    validate(settings);
    return settings;
}

void write_all(int fd, const std::string &data) {
    std::size_t written = 0;
    while (written < data.size()) {
        auto count = ::write(fd, data.data() + written, data.size() - written);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) throw std::runtime_error("Write failed: " + std::string(std::strerror(errno)));
        written += static_cast<std::size_t>(count);
    }
}

void save_settings(const Settings &settings, const std::string &path) {
    fs::path target(path), parent = target.parent_path();
    if (parent.empty()) parent = ".";
    fs::create_directories(parent);
    std::string pattern = (parent / ".pulveris-XXXXXX").string();
    std::vector<char> name(pattern.begin(), pattern.end()); name.push_back('\0');
    int fd = ::mkstemp(name.data());
    if (fd < 0) throw std::runtime_error("Cannot create settings file: " + std::string(std::strerror(errno)));
    try {
        write_all(fd, to_json(settings).dump(2) + "\n");
        if (::close(fd) < 0) { fd = -1; throw std::runtime_error("Cannot close settings file"); }
        fd = -1;
        if (::rename(name.data(), target.c_str()) < 0) throw std::runtime_error("Cannot replace settings file: " + std::string(std::strerror(errno)));
    } catch (...) {
        if (fd >= 0) ::close(fd);
        ::unlink(name.data());
        throw;
    }
}

std::string id_field(const json &body, const char *name) {
    if (!body.contains(name) || !body[name].is_string()) throw std::invalid_argument(std::string("Invalid ") + name);
    auto value = body[name].get<std::string>();
    if (value.size() < 16 || value.size() > 128) throw std::invalid_argument(std::string("Invalid ") + name);
    return value;
}

}

void load_settings(Settings &settings, const std::string &path) {
    std::ifstream file(path);
    if (!file) {
        if (!fs::exists(path)) { validate(settings); return; }
        throw std::runtime_error("Cannot read settings: " + path);
    }
    json saved;
    file >> saved;
    settings = merged(settings, saved);
}


std::string api_state(Shared &shared) {
    std::lock_guard<std::mutex> lock(shared.mutex);
    const auto &s = shared.status;
    const auto &d = shared.detection;
    auto points = [](const std::vector<cv::Point> &source) {
        json result = json::array();
        for (const auto &p : source) result.push_back({p.x, p.y});
        return result;
    };
    json status = {{"mode", s.mode}, {"error", s.error ? json(*s.error) : json(nullptr)},
           {"turn", s.turn}, {"left", s.left}, {"right", s.right},
           {"forward_input", s.forward}, {"backward_input", s.backward},
           {"fps", s.fps ? json(*s.fps) : json(nullptr)}, {"processing_ms", s.processing_ms},
           {"frame_age_ms", s.frame_age_ms}, {"resolution", json::array({s.width, s.height})}};
    json detection = {{"region", points(d.polygon)}, {"outline", points(d.outline)},
                      {"processing_resolution", json::array({d.processing_width, d.processing_height})},
                      {"point", d.found ? json::array({d.point.x, d.point.y}) : json(nullptr)}};
    return json({{"settings", to_json(shared.settings)}, {"limits", limits_json()}, {"mode", shared.mode}, {"detection", detection},
                 {"direction", now() < shared.expires ? shared.direction : "stop"}, {"status", status}}).dump();
}

std::string api_post(Shared &shared, const std::string &path, const std::string &payload) {
    json body = json::parse(payload);
    if (!body.is_object()) throw std::invalid_argument("Expected an object");
    if (path == "/api/settings") {
        std::lock_guard<std::mutex> update_lock(shared.settings_update);
        Settings next;
        { std::lock_guard<std::mutex> lock(shared.mutex); next = merged(shared.settings, body); }
        save_settings(next, shared.config_path);
        { std::lock_guard<std::mutex> lock(shared.mutex); shared.settings = next; }
        return to_json(next).dump();
    } else if (path == "/api/mode") {
        auto client = id_field(body, "client");
        if (!body.contains("mode") || !body["mode"].is_string()) throw std::invalid_argument("Invalid mode");
        auto mode = body["mode"].get<std::string>();
        if (mode != "physical" && mode != "remote") throw std::invalid_argument("Invalid mode");
        std::lock_guard<std::mutex> lock(shared.mutex);
        shared.mode = mode; shared.owner = mode == "remote" ? client : "";
        shared.sequence = -1; shared.direction = "stop"; shared.expires = 0; shared.gesture.clear();
        return json({{"ok", true}}).dump();
    } else {
        auto client = id_field(body, "client"), gesture = id_field(body, "gesture");
        if (!body.contains("sequence") || !body["sequence"].is_number_integer() ||
            !body.contains("direction") || !body["direction"].is_string()) throw std::invalid_argument("Invalid control command");
        double number = body["sequence"].get<double>();
        if (number < 0 || number > 9007199254740991.0) throw std::invalid_argument("Invalid sequence");
        auto sequence = static_cast<std::int64_t>(number);
        auto direction = body["direction"].get<std::string>();
        if (direction != "forward" && direction != "backward" && direction != "left" && direction != "right" && direction != "stop")
            throw std::invalid_argument("Invalid direction");
        std::lock_guard<std::mutex> lock(shared.mutex);
        if (shared.mode != "remote" || shared.owner != client) throw std::invalid_argument("Remote control is not owned by this client");
        if (sequence > shared.sequence) {
            shared.sequence = sequence;
            if (direction != "stop" && gesture == shared.gesture && now() >= shared.expires) {
                shared.direction = "stop";
                throw std::invalid_argument("Control timed out; release and press again");
            }
            shared.gesture = gesture; shared.direction = direction;
            shared.expires = direction == "stop" ? 0 : now() + .5;
        }
        return json({{"ok", true}}).dump();
    }
}

std::string api_error(const std::string &message) { return json({{"error", message}}).dump(); }
