#include "core.hpp"

#include <atomic>
#include <csignal>
#include <iostream>
#include <set>
#include <thread>

#include <opencv2/videoio.hpp>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void handle_signal(int) { stop_requested = 1; }

struct Options {
    Settings settings;
    std::string camera = "/dev/video0", config = "pulveris.json", host = "0.0.0.0", token = ".web-token";
    int frequency = 100, port = 8080, enable_pin = 5, tape_pin = 7, error_led_pin = 23;
    std::array<int, 4> pins{11, 13, 15, 12};
    std::array<int, 2> inputs{22, 3}, led_pins{21, 19};
    bool web = false, dry_run = false, dry_inputs = false;
};

double number(const std::string &text) {
    std::size_t used = 0;
    double value = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(value)) throw std::invalid_argument("Invalid number: " + text);
    return value;
}

int integer(const std::string &text) {
    double value = number(text);
    if (std::floor(value) != value || value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max())
        throw std::invalid_argument("Invalid integer: " + text);
    return static_cast<int>(value);
}

Options parse_options(int argc, char **argv) {
    Options result;
    auto next = [&](int &i) -> std::string {
        if (++i >= argc) throw std::invalid_argument("Missing option value");
        return argv[i];
    };
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") {
            std::cout << "Usage: pulveris [--camera DEVICE] [--width N] [--height N] [--web] [--dry-run] [options]\n"
                         "Control: --pins L_BACK L_FWD R_BACK R_FWD --input-pins FORWARD BACKWARD --enable-pin PIN --tape-pin PIN --led-pins READY LINE --error-led-pin PIN\n"
                         "Tuning: --threshold --contrast --max-line-width --smoothing --light-line --roi-start\n"
                         "        --roi-top-width --roi-bottom-width --min-fraction --max-fraction --speed\n"
                         "        --kp --ki --kd --ramp-rate --pwm-hz --timeout\n"
                         "Web: --web-host ADDRESS --web-port PORT --web-token-file PATH --config PATH\n"
                         "Use --dry-inputs with --dry-run to read GPIO without driving motors.\n";
            std::exit(0);
        }
        if (arg == "--camera") result.camera = next(i);
        else if (arg == "--config") result.config = next(i);
        else if (arg == "--web-host") result.host = next(i);
        else if (arg == "--web-token-file") result.token = next(i);
        else if (arg == "--width") result.settings.camera_width = integer(next(i));
        else if (arg == "--height") result.settings.camera_height = integer(next(i));
        else if (arg == "--pwm-hz") result.frequency = integer(next(i));
        else if (arg == "--web-port") result.port = integer(next(i));
        else if (arg == "--enable-pin") result.enable_pin = integer(next(i));
        else if (arg == "--tape-pin") result.tape_pin = integer(next(i));
        else if (arg == "--error-led-pin") result.error_led_pin = integer(next(i));
        else if (arg == "--pins") for (auto &pin : result.pins) pin = integer(next(i));
        else if (arg == "--input-pins") for (auto &pin : result.inputs) pin = integer(next(i));
        else if (arg == "--led-pins") for (auto &pin : result.led_pins) pin = integer(next(i));
        else if (arg == "--web") result.web = true;
        else if (arg == "--dry-run") result.dry_run = true;
        else if (arg == "--dry-inputs") result.dry_inputs = true;
        else if (arg == "--light-line") result.settings.light_line = true;
        else {
            if (arg.rfind("--", 0) != 0) throw std::invalid_argument("Unknown option: " + arg);
            std::string name = arg.substr(2);
            std::replace(name.begin(), name.end(), '-', '_');
            auto spec = std::find_if(setting_specs.begin(), setting_specs.end(), [&](const auto &s) { return name == s.name; });
            if (spec == setting_specs.end()) throw std::invalid_argument("Unknown option: " + arg);
            result.settings.*(spec->field) = number(next(i));
        }
    }
    validate(result.settings);
    const std::set<int> allowed{3, 5, 7, 11, 12, 13, 15, 19, 21, 22, 23};
    std::set<int> used(result.pins.begin(), result.pins.end());
    used.insert(result.inputs.begin(), result.inputs.end()); used.insert(result.led_pins.begin(), result.led_pins.end());
    used.insert(result.enable_pin); used.insert(result.tape_pin); used.insert(result.error_led_pin);
    if (result.frequency < 20 || result.frequency > 1000 ||
        result.port < 1 || result.port > 65535 || used != allowed || (result.dry_inputs && !result.dry_run))
        throw std::invalid_argument("Invalid settings or pins; choose eleven distinct pins from 3, 5, 7, 11, 12, 13, 15, 19, 21, 22, 23");
    return result;
}

struct Frame {
    cv::Mat image;
    double timestamp = 0;
    std::uint64_t sequence = 0;
    std::uint64_t generation = 0;
    std::string error;
};

class Camera {
    struct Data {
        std::mutex mutex;
        std::condition_variable finished;
        std::atomic<bool> stop{false};
        bool done = false;
        int width, height;
        std::uint64_t generation = 1;
        Frame latest;
        Data(int w, int h) : width(w), height(h) {}
    };
public:
    Camera(const std::string &device, int width, int height) : data(std::make_shared<Data>(width, height)) {
        worker = std::thread([state = data, device, width, height] {
            try {
                bool physical_camera = device.rfind("/dev/", 0) == 0;
                cv::VideoCapture camera(device, physical_camera ? cv::CAP_V4L2 : cv::CAP_ANY);
                if (!camera.isOpened()) throw std::runtime_error("Cannot open camera " + device);
                camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
                camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
                camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
                camera.set(cv::CAP_PROP_FPS, 30);
                camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
                double file_fps = camera.get(cv::CAP_PROP_FPS);
                if (!std::isfinite(file_fps) || file_fps < 1 || file_fps > 120) file_fps = 30;
                auto next_frame = Clock::now();
                std::uint64_t generation = 1;
                while (!state->stop) {
                    int requested_width, requested_height;
                    std::uint64_t requested_generation;
                    {
                        std::lock_guard<std::mutex> lock(state->mutex);
                        requested_width = state->width; requested_height = state->height;
                        requested_generation = state->generation;
                    }
                    if (requested_generation != generation) {
                        camera.set(cv::CAP_PROP_FRAME_WIDTH, requested_width);
                        camera.set(cv::CAP_PROP_FRAME_HEIGHT, requested_height);
                        camera.set(cv::CAP_PROP_FPS, 30);
                        generation = requested_generation;
                    }
                    if (!physical_camera) {
                        std::this_thread::sleep_until(next_frame);
                        next_frame += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1 / file_fps));
                    }
                    cv::Mat frame;
                    if (!camera.read(frame) || frame.empty()) throw std::runtime_error("Camera read failed");
                    double timestamp = now();
                    std::lock_guard<std::mutex> lock(state->mutex);
                    state->latest.image = frame; state->latest.timestamp = timestamp;
                    state->latest.generation = generation; ++state->latest.sequence;
                }
            } catch (const std::exception &error) {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->latest.error = error.what();
            }
            { std::lock_guard<std::mutex> lock(state->mutex); state->done = true; }
            state->finished.notify_all();
        });
    }
    ~Camera() {
        data->stop = true;
        std::unique_lock<std::mutex> lock(data->mutex);
        bool finished = data->finished.wait_for(lock, std::chrono::milliseconds(250), [&] { return data->done; });
        lock.unlock();
        if (worker.joinable()) {
            if (finished) worker.join();
            else worker.detach(); // A stalled camera must not delay motor shutdown.
        }
    }
    Frame latest() const { std::lock_guard<std::mutex> lock(data->mutex); return data->latest; }
    std::uint64_t request_size(int width, int height) {
        std::lock_guard<std::mutex> lock(data->mutex);
        if (data->width != width || data->height != height) {
            data->width = width; data->height = height; ++data->generation;
        }
        return data->generation;
    }
private:
    std::shared_ptr<Data> data;
    std::thread worker;
};

int run(const Options &options) {
    Shared shared;
    shared.settings = options.settings;
    shared.config_path = options.config;
    std::unique_ptr<WebServer> server;
    std::unique_ptr<Inputs> inputs;
    std::unique_ptr<StatusLeds> leds;
    std::unique_ptr<Motors> motors;
    std::unique_ptr<Camera> camera;
    int exit_code = 0;
    try {
        if (!options.dry_run || options.dry_inputs) {
            leds = std::make_unique<StatusLeds>(std::array<int, 3>{options.led_pins[0], options.led_pins[1], options.error_led_pin});
        }
        load_settings(shared.settings, options.config);
        if (options.web) {
            server = std::make_unique<WebServer>(shared, options.host, options.port, options.token, "web.html");
            std::cout << "Web interface listening on " << options.host << ':' << options.port << "; token file: " << options.token << std::endl;
        }
        if (!options.dry_run || options.dry_inputs) inputs = std::make_unique<Inputs>(options.inputs, options.tape_pin);
        if (!options.dry_run) motors = std::make_unique<Motors>(options.pins, options.frequency, options.enable_pin);
        camera = std::make_unique<Camera>(options.camera, static_cast<int>(shared.settings.camera_width),
                                          static_cast<int>(shared.settings.camera_height));
        PID pid;
        Motion motion;
        std::string previous_mode = snapshot(shared).mode;
        std::optional<double> previous_time, filtered, error;
        std::optional<bool> previous_polarity;
        std::pair<double, double> target{0, 0};
        double turn = 0, started = now(), last_update = started;
        std::uint64_t last_sequence = 0;
        std::uint64_t generation = 1;
        double size_started = started;
        std::deque<double> frame_times;
        auto stop_for_camera = [&](const char *mode, bool camera_error = false) {
            if (motors) motors->drive(0, 0);
            if (leds) leds->set(false, false, camera_error);
            motion.stop(); pid.reset(); previous_time.reset(); filtered.reset(); error.reset();
            target = {0, 0}; turn = 0; frame_times.clear();
            if (server) {
                std::lock_guard<std::mutex> lock(shared.mutex);
                shared.status.mode = mode; shared.status.left = shared.status.right = 0;
                shared.status.error.reset(); shared.status.turn = 0; shared.status.fps.reset(); shared.detection = {};
            }
        };
        while (!stop_requested) {
            auto control = snapshot(shared);
            auto settings = control.settings;
            auto physical = inputs ? inputs->read() : InputValues{true, false, !settings.light_line};
            if (inputs) settings.light_line = !physical.dark_line;
            if (previous_polarity && *previous_polarity != settings.light_line) {
                if (motors) motors->drive(0, 0);
                if (leds) leds->set(false, false, false);
                motion.stop(); pid.reset(); previous_time.reset(); filtered.reset(); error.reset();
                target = {0, 0}; turn = 0;
            }
            previous_polarity = settings.light_line;
            if (control.mode != previous_mode) {
                if (motors) motors->drive(0, 0);
                motion.stop(); pid.reset(); previous_time.reset(); filtered.reset(); previous_mode = control.mode;
            }
            auto requested_generation = camera->request_size(static_cast<int>(settings.camera_width), static_cast<int>(settings.camera_height));
            if (requested_generation != generation) { generation = requested_generation; size_started = now(); }
            Frame latest = camera->latest();
            if (!latest.error.empty()) throw std::runtime_error(latest.error);
            double current = now();
            if (latest.generation != generation && latest.sequence) {
                stop_for_camera(current - size_started > 5 ? "camera timeout" : "camera reconfiguring", current - size_started > 5);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if (!latest.sequence || current - latest.timestamp > settings.timeout) {
                bool starting = !latest.sequence && current - started < 5;
                stop_for_camera(starting ? "starting" : "camera timeout", !starting);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            bool fresh = latest.sequence != last_sequence;
            double processing_started = 0;
            if (fresh) {
                processing_started = now();
                last_sequence = latest.sequence;
                if (server) {
                    { std::lock_guard<std::mutex> lock(shared.mutex); shared.frame = latest.image; ++shared.frame_sequence; }
                    shared.changed.notify_all();
                }
                auto detection = detect_for_control(latest.image, settings, server != nullptr);
                if (server) { std::lock_guard<std::mutex> lock(shared.mutex); shared.detection = detection; }
                if (now() - latest.timestamp > settings.timeout) { stop_for_camera("camera timeout", true); continue; }
                if (leds) leds->set(true, detection.found, false);
                if (!detection.found) {
                    error.reset(); turn = 0; target = {0, 0}; pid.reset(); previous_time.reset(); filtered.reset();
                } else {
                    double dt = previous_time ? std::max(latest.timestamp - *previous_time, .001) : .001;
                    double value = detection.error;
                    if (filtered && settings.smoothing != 0) value = *filtered + (1 - std::exp(-dt / settings.smoothing)) * (value - *filtered);
                    filtered = error = value;
                    turn = pid.update(value, dt, settings); previous_time = latest.timestamp;
                    target = {clamp(settings.speed + turn, -1, 1), clamp(settings.speed - turn, -1, 1)};
                }
            }
            bool forward = physical.forward, backward = physical.backward;
            if (!forward || backward) { pid.reset(); previous_time.reset(); filtered.reset(); }
            double update_time = now();
            if (update_time - latest.timestamp > settings.timeout) { stop_for_camera("camera timeout", true); continue; }
            auto updated = snapshot(shared);
            if (updated.mode != control.mode) {
                if (motors) motors->drive(0, 0);
                motion.stop();
                continue;
            }
            if (!fresh && update_time - last_update < .01) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            std::string mode = control.mode == "remote" ? "remote " + updated.direction :
                forward && backward ? "conflict" : forward ? "forward" : backward ? "backward" : "stopped";
            auto result = control.mode == "remote" ? motion.remote(updated.direction, settings, update_time - last_update) :
                motion.physical(forward, backward, error.has_value(), target, settings, update_time - last_update);
            last_update = update_time;
            if (motors) motors->drive(result.first, result.second);
            else if (fresh)
                std::cout << "mode=" << mode << " error=" << (error ? std::to_string(*error) : "lost")
                          << " turn=" << turn << " left=" << result.first << " right=" << result.second << std::endl;
            if (server) {
                double finished = now();
                if (fresh) {
                    frame_times.push_back(finished);
                    while (frame_times.size() > 1 && finished - frame_times.front() > 1) frame_times.pop_front();
                }
                std::lock_guard<std::mutex> lock(shared.mutex);
                auto &status = shared.status;
                status.mode = mode; status.error = error; status.turn = turn; status.left = result.first; status.right = result.second;
                status.forward = forward; status.backward = backward;
                if (fresh) {
                    status.fps = frame_times.size() > 1 && frame_times.back() > frame_times.front() ?
                        std::optional<double>((frame_times.size() - 1) / (frame_times.back() - frame_times.front())) : std::nullopt;
                    status.processing_ms = (finished - processing_started) * 1000;
                    status.frame_age_ms = (finished - latest.timestamp) * 1000;
                    status.width = latest.image.cols; status.height = latest.image.rows;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    } catch (const std::exception &error) {
        std::cerr << "Error: " << error.what() << std::endl;
        exit_code = 1;
        motors.reset(); inputs.reset(); camera.reset();
        if (server) {
            std::lock_guard<std::mutex> lock(shared.mutex);
            shared.status.mode = "error"; shared.status.left = shared.status.right = 0;
            shared.status.error.reset(); shared.status.turn = 0; shared.status.fps.reset(); shared.detection = {};
        }
        if (leds) {
            try {
                while (!stop_requested) {
                    leds->set(false, false, true);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            } catch (const std::exception &led_error) {
                std::cerr << "Error setting fault LED: " << led_error.what() << std::endl;
            }
        }
    }
    motors.reset(); leds.reset(); inputs.reset(); camera.reset(); server.reset();
    return exit_code;
}
}

int main(int argc, char **argv) {
    std::signal(SIGTERM, handle_signal);
    std::signal(SIGINT, handle_signal);
    std::signal(SIGPIPE, SIG_IGN);
    try { return run(parse_options(argc, argv)); }
    catch (const std::exception &error) { std::cerr << "Error: " << error.what() << std::endl; return 1; }
}
