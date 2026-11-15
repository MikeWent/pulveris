#include "core.hpp"

namespace {
constexpr double inf = std::numeric_limits<double>::infinity();
}

const std::vector<SettingSpec> setting_specs = {
    {"camera_width", &Settings::camera_width, 32, 1920, 1, false, false, true},
    {"camera_height", &Settings::camera_height, 32, 1080, 1, false, false, true},
    {"threshold", &Settings::threshold, 0, 255, 1, false, false, true},
    {"contrast", &Settings::contrast, 0, 255, 1, false, false, true},
    {"max_line_width", &Settings::max_line_width, 0, 1, .01, true},
    {"smoothing", &Settings::smoothing, 0, inf, .01},
    {"roi_start", &Settings::roi_start, 0, 1, .01, false, true},
    {"roi_top_width", &Settings::roi_top_width, 0, 1, .01, true},
    {"roi_bottom_width", &Settings::roi_bottom_width, 0, 1, .01, true},
    {"min_fraction", &Settings::min_fraction, 0, 1, .001},
    {"max_fraction", &Settings::max_fraction, 0, 1, .001},
    {"speed", &Settings::speed, 0, 1, .01},
    {"kp", &Settings::kp, 0, inf, .01}, {"ki", &Settings::ki, 0, inf, .01},
    {"kd", &Settings::kd, 0, inf, .01},
    {"ramp_rate", &Settings::ramp_rate, 0, inf, .01, true},
    {"timeout", &Settings::timeout, 0, inf, .05, true},
};

void validate(const Settings &settings) {
    for (const auto &spec : setting_specs) {
        double v = settings.*(spec.field);
        if (!std::isfinite(v) || v < spec.min || v > spec.max ||
            (spec.positive && v == 0) || (spec.exclusive_max && v == spec.max) ||
            (spec.integer && std::floor(v) != v))
            throw std::invalid_argument(std::string("Invalid value for ") + spec.name);
    }
    if (settings.min_fraction >= settings.max_fraction)
        throw std::invalid_argument("Invalid line fraction limits");
}

Snapshot snapshot(const Shared &shared) {
    std::lock_guard<std::mutex> lock(shared.mutex);
    return {shared.settings, shared.mode, now() < shared.expires ? shared.direction : "stop"};
}

double PID::update(double error, double dt, const Settings &settings) {
    double derivative = previous ? (error - *previous) / dt : 0;
    double candidate = clamp(integral + error * dt, -1, 1);
    double output = settings.kp * error + settings.ki * candidate + settings.kd * derivative;
    if (std::abs(output) <= 1 || error * output < 0) integral = candidate;
    previous = error;
    return clamp(settings.kp * error + settings.ki * integral + settings.kd * derivative, -1, 1);
}

std::pair<double, double> Motion::physical(bool forward, bool backward, bool visible,
                                            std::pair<double, double> target, const Settings &settings, double dt) {
    if ((forward && backward) || (forward && !visible)) { stop(); return {left, right}; }
    int requested = static_cast<int>(forward) - static_cast<int>(backward);
    if (requested && requested != direction && (left || right)) {
        target = {0, 0};
    } else {
        if (requested) direction = requested;
        if (!forward) target = backward ? std::make_pair(-settings.speed, -settings.speed) : std::make_pair(0.0, 0.0);
    }
    double delta = settings.ramp_rate * clamp(dt, 0, .1);
    left += clamp(target.first - left, -delta, delta);
    right += clamp(target.second - right, -delta, delta);
    return {left, right};
}

std::pair<double, double> Motion::remote(const std::string &command, const Settings &settings, double dt) {
    if (command == "stop") { stop(); return {left, right}; }
    std::pair<double, double> target;
    if (command == "forward") target = {settings.speed, settings.speed};
    else if (command == "backward") target = {-settings.speed, -settings.speed};
    else if (command == "left") target = {-settings.speed, settings.speed};
    else if (command == "right") target = {settings.speed, -settings.speed};
    else { stop(); return {left, right}; }
    double delta = settings.ramp_rate * clamp(dt, 0, .1);
    left += clamp(target.first - left, -delta, delta);
    right += clamp(target.second - right, -delta, delta);
    return {left, right};
}
