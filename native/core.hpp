#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

using Clock = std::chrono::steady_clock;
inline double now() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
inline double clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }

struct Settings {
    double camera_width = 176, camera_height = 144;
    double threshold = 100, contrast = 8, max_line_width = .35, smoothing = .12;
    double roi_start = .5, roi_top_width = .6, roi_bottom_width = 1;
    double min_fraction = .01, max_fraction = .6, speed = .3;
    double kp = .5, ki = 0, kd = .05, ramp_rate = .6, timeout = .5;
    bool light_line = true;
};

struct SettingSpec {
    const char *name;
    double Settings::*field;
    double min, max, step;
    bool positive = false, exclusive_max = false, integer = false;
};
extern const std::vector<SettingSpec> setting_specs;
void validate(const Settings &settings);
void load_settings(Settings &settings, const std::string &path);

struct Detection {
    bool found = false;
    double error = 0;
    int processing_width = 0, processing_height = 0;
    cv::Point point;
    cv::Mat selected;
    std::vector<cv::Point> polygon;
    std::vector<cv::Point> outline;
};
Detection detect_line(const cv::Mat &frame, const Settings &settings, bool debug = false);
Detection detect_for_control(const cv::Mat &frame, const Settings &settings, bool debug = false);

struct PID {
    double integral = 0;
    std::optional<double> previous;
    void reset() { integral = 0; previous.reset(); }
    double update(double error, double dt, const Settings &settings);
};

struct Motion {
    struct Segment { double left, right, duration; };
    double left = 0, right = 0;
    int direction = 0;
    std::deque<Segment> path;
    void advance(double dt);
    void stop() { left = right = 0; direction = 0; }
    std::pair<double, double> physical(bool forward, bool backward, bool visible,
                                       std::pair<double, double> target, const Settings &settings, double dt);
    std::pair<double, double> remote(const std::string &direction, const Settings &settings, double dt);
};

struct Status {
    std::string mode = "starting";
    std::optional<double> error, fps;
    double turn = 0, left = 0, right = 0, processing_ms = 0, frame_age_ms = 0;
    bool forward = false, backward = false;
    int width = 0, height = 0;
};

struct Shared {
    mutable std::mutex mutex;
    std::mutex settings_update;
    std::condition_variable changed;
    Settings settings;
    std::string config_path, mode = "physical", owner, direction = "stop", gesture;
    std::int64_t sequence = -1;
    double expires = 0;
    cv::Mat frame;
    Detection detection;
    std::uint64_t frame_sequence = 0;
    Status status;
    bool running = true;
};

struct Snapshot {
    Settings settings;
    std::string mode, direction;
};
Snapshot snapshot(const Shared &shared);

struct InputValues { bool forward, backward, dark_line; };

class Inputs {
public:
    explicit Inputs(const std::array<int, 2> &pins, int tape_pin);
    ~Inputs();
    Inputs(const Inputs &) = delete;
    Inputs &operator=(const Inputs &) = delete;
    InputValues read();
private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

class StatusLeds {
public:
    explicit StatusLeds(const std::array<int, 3> &pins);
    ~StatusLeds();
    StatusLeds(const StatusLeds &) = delete;
    StatusLeds &operator=(const StatusLeds &) = delete;
    void set(bool ready, bool line, bool error);
private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

class Motors {
public:
    Motors(const std::array<int, 4> &pins, int frequency, int enable_pin);
    ~Motors();
    Motors(const Motors &) = delete;
    Motors &operator=(const Motors &) = delete;
    void drive(double left, double right);
private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

class WebServer {
public:
    WebServer(Shared &shared, const std::string &host, int port, const std::string &token_path,
              const std::string &html_path);
    ~WebServer();
    WebServer(const WebServer &) = delete;
    WebServer &operator=(const WebServer &) = delete;
private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

std::string api_state(Shared &shared);
std::string api_post(Shared &shared, const std::string &path, const std::string &payload);
std::string api_error(const std::string &message);
