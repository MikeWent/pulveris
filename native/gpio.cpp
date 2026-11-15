#include "core.hpp"
#include "debounce.hpp"
#include "blink.hpp"

#include <cerrno>
#include <cstring>
#include <gpiod.h>
#include <iostream>
#include <thread>

namespace {
unsigned int offset(int pin) {
    switch (pin) {
        case 3: return 12; case 5: return 11; case 7: return 203; case 11: return 0; case 12: return 6;
        case 13: return 2; case 15: return 3; case 19: return 64; case 21: return 65; case 22: return 1; case 23: return 66;
        default: throw std::invalid_argument("Unsupported GPIO pin");
    }
}

std::runtime_error gpio_error(const std::string &operation) {
    return std::runtime_error(operation + ": " + std::strerror(errno));
}

class Lines {
public:
    Lines(const std::vector<unsigned int> &pins, bool output, const char *consumer) {
        auto chip = std::unique_ptr<gpiod_chip, decltype(&gpiod_chip_close)>(gpiod_chip_open("/dev/gpiochip0"), gpiod_chip_close);
        if (!chip) throw gpio_error("Open /dev/gpiochip0");
        auto settings = std::unique_ptr<gpiod_line_settings, decltype(&gpiod_line_settings_free)>(gpiod_line_settings_new(), gpiod_line_settings_free);
        auto config = std::unique_ptr<gpiod_line_config, decltype(&gpiod_line_config_free)>(gpiod_line_config_new(), gpiod_line_config_free);
        auto request_config = std::unique_ptr<gpiod_request_config, decltype(&gpiod_request_config_free)>(gpiod_request_config_new(), gpiod_request_config_free);
        if (!settings || !config || !request_config) throw std::bad_alloc();
        if (!output) gpiod_line_settings_set_active_low(settings.get(), true);
        if (gpiod_line_settings_set_direction(settings.get(), output ? GPIOD_LINE_DIRECTION_OUTPUT : GPIOD_LINE_DIRECTION_INPUT) < 0 ||
            (output ? gpiod_line_settings_set_output_value(settings.get(), GPIOD_LINE_VALUE_INACTIVE)
                    : gpiod_line_settings_set_bias(settings.get(), GPIOD_LINE_BIAS_PULL_UP)) < 0 ||
            gpiod_line_config_add_line_settings(config.get(), pins.data(), pins.size(), settings.get()) < 0)
            throw gpio_error("Configure GPIO lines");
        gpiod_request_config_set_consumer(request_config.get(), consumer);
        request = gpiod_chip_request_lines(chip.get(), request_config.get(), config.get());
        if (!request) throw gpio_error("Request GPIO lines");
    }
    ~Lines() { if (request) gpiod_line_request_release(request); }
    Lines(const Lines &) = delete;
    Lines &operator=(const Lines &) = delete;
    gpiod_line_request *request = nullptr;
};
}

struct Inputs::Impl {
    explicit Impl(const std::array<int, 2> &pins, int tape_pin)
        : lines({offset(pins[0]), offset(pins[1]), offset(tape_pin)}, false, "pulveris-inputs") {}
    Lines lines;
    std::array<DebouncedInput, 2> debounce;
};

Inputs::Inputs(const std::array<int, 2> &pins, int tape_pin) : impl(std::make_unique<Impl>(pins, tape_pin)) {}
Inputs::~Inputs() = default;
InputValues Inputs::read() {
    std::array<gpiod_line_value, 3> values{};
    if (gpiod_line_request_get_values(impl->lines.request, values.data()) < 0) throw gpio_error("Read GPIO inputs");
    double timestamp = now();
    return {impl->debounce[0].update(values[0] == GPIOD_LINE_VALUE_ACTIVE, timestamp),
            impl->debounce[1].update(values[1] == GPIOD_LINE_VALUE_ACTIVE, timestamp),
            values[2] == GPIOD_LINE_VALUE_ACTIVE};
}

struct StatusLeds::Impl {
    explicit Impl(const std::array<int, 3> &pins)
        : lines({offset(pins[0]), offset(pins[1]), offset(pins[2])}, true, "pulveris-status") {}
    ~Impl() {
        try { set(false, false, false); }
        catch (const std::exception &e) { std::cerr << "Status LED shutdown: " << e.what() << '\n'; }
    }
    void set(bool ready, bool line, bool error) {
        bool error_on = blink.update(error, now());
        std::array<gpiod_line_value, 3> values{
            ready ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE,
            line ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE,
            error_on ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE};
        if (values == previous) return;
        if (gpiod_line_request_set_values(lines.request, values.data()) < 0) throw gpio_error("Write status LEDs");
        previous = values;
    }
    Lines lines;
    ErrorBlink blink;
    std::array<gpiod_line_value, 3> previous{GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_INACTIVE};
};

StatusLeds::StatusLeds(const std::array<int, 3> &pins) : impl(std::make_unique<Impl>(pins)) {}
StatusLeds::~StatusLeds() = default;
void StatusLeds::set(bool ready, bool line, bool error) { impl->set(ready, line, error); }

struct Motors::Impl {
    Impl(const std::array<int, 4> &pins, int frequency, int enable)
        : lines({offset(pins[0]), offset(pins[1]), offset(pins[2]), offset(pins[3]), offset(enable)}, true, "pulveris-motors"),
          period(1.0 / frequency), worker(&Impl::pwm, this) {}
    ~Impl() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            running = false; enabled = false; duty.fill(0);
            try { write({false, false, false, false}); }
            catch (const std::exception &e) { std::cerr << "GPIO shutdown: " << e.what() << '\n'; }
            ++version;
        }
        changed.notify_one();
        if (worker.joinable()) worker.join();
    }
    void write(const std::array<bool, 4> &active) {
        std::array<gpiod_line_value, 5> values{};
        for (std::size_t i = 0; i < active.size(); ++i)
            values[i] = active[i] ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
        values[4] = enabled ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
        if (gpiod_line_request_set_values(lines.request, values.data()) < 0) throw gpio_error("Write GPIO outputs");
    }
    void pwm() {
        try {
            std::unique_lock<std::mutex> lock(mutex);
            while (running) {
                double phase = std::fmod(now(), period) / period;
                std::array<bool, 4> active{};
                bool timed = false;
                double delay = 1 - phase;
                for (std::size_t i = 0; i < duty.size(); ++i) {
                    active[i] = duty[i] > phase;
                    if (duty[i] > 0 && duty[i] < 1) timed = true;
                    if (phase < duty[i] && duty[i] < 1) delay = std::min(delay, duty[i] - phase);
                }
                write(active);
                auto current = version;
                if (timed) changed.wait_for(lock, std::chrono::duration<double>(std::max(delay * period, .0001)), [&] { return !running || version != current; });
                else changed.wait(lock, [&] { return !running || version != current; });
            }
        } catch (const std::exception &e) {
            std::lock_guard<std::mutex> lock(mutex);
            error = e.what(); enabled = false; duty.fill(0);
            try { write({false, false, false, false}); } catch (...) {}
        }
    }
    void drive(double left, double right) {
        const std::array<double, 4> wanted = {std::max(-left, 0.0), std::max(left, 0.0), std::max(-right, 0.0), std::max(right, 0.0)};
        const std::array<int, 2> next = {(left > 0) - (left < 0), (right > 0) - (right < 0)};
        bool reversing = false;
        for (int i = 0; i < 2; ++i) reversing |= directions[i] && next[i] && directions[i] != next[i];
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (!error.empty()) throw std::runtime_error("GPIO PWM failed: " + error);
            bool desired_enable = next[0] || next[1];
            if (!reversing && wanted == duty && desired_enable == enabled) return;
            bool was_enabled = enabled;
            enabled = desired_enable;
            duty = reversing ? std::array<double, 4>{0, 0, 0, 0} : wanted;
            if (reversing || !enabled || !was_enabled) write({false, false, false, false});
            ++version;
        }
        changed.notify_one();
        if (reversing) {
            std::this_thread::sleep_for(std::chrono::duration<double>(period + .01));
            std::lock_guard<std::mutex> lock(mutex);
            if (!error.empty()) throw std::runtime_error("GPIO PWM failed: " + error);
            duty = wanted; ++version; changed.notify_one();
        }
        for (int i = 0; i < 2; ++i) if (next[i]) directions[i] = next[i];
    }
    Lines lines;
    double period;
    std::array<int, 2> directions{0, 0};
    std::array<double, 4> duty{0, 0, 0, 0};
    bool enabled = false, running = true;
    std::string error;
    std::mutex mutex;
    std::condition_variable changed;
    std::uint64_t version = 0;
    std::thread worker;
};

Motors::Motors(const std::array<int, 4> &pins, int frequency, int enable_pin)
    : impl(std::make_unique<Impl>(pins, frequency, enable_pin)) {}
Motors::~Motors() = default;
void Motors::drive(double left, double right) { impl->drive(left, right); }
