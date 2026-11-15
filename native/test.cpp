#include "core.hpp"
#include "debounce.hpp"

#include <cassert>
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char **argv) {
    Settings settings;
    if (argc > 1) {
        settings.light_line = false;
        cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);
        if (image.empty()) { std::cerr << "Cannot open image\n"; return 1; }
        settings.threshold = 160; settings.roi_top_width = 1;
        const double start = now();
        Detection result;
        for (int i = 0; i < 100; ++i) result = detect_line(image, settings, true);
        std::cout << "found=" << result.found << " error=" << result.error
                  << " point=" << result.point.x << "," << result.point.y
                  << " pixels=" << (result.found ? cv::countNonZero(result.selected) : 0)
                  << " mean_ms=" << (now() - start) * 10 << '\n';
        return 0;
    }
    assert(settings.light_line);
    cv::Mat image(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
    settings.roi_start = .6; settings.roi_top_width = 1;
    cv::rectangle(image, {70, 60}, {79, 99}, cv::Scalar(255, 255, 255), cv::FILLED);
    auto white = detect_line(image, settings);
    assert(white.found && white.error > .4);
    settings.light_line = false;
    image.setTo(cv::Scalar(255, 255, 255));
    cv::rectangle(image, {70, 60}, {79, 99}, cv::Scalar(0, 0, 0), cv::FILLED);
    auto right = detect_line(image, settings, true);
    assert(right.found && right.error > .4 && !right.selected.empty() && !right.outline.empty());
    cv::flip(image, image, 1);
    auto left = detect_line(image, settings);
    assert(left.found && left.error < -.4);
    image.setTo(cv::Scalar(255, 255, 255));
    assert(!detect_line(image, settings).found);

    cv::Mat edge(240, 320, CV_8UC3, cv::Scalar(195, 195, 195));
    std::vector<cv::Point> stripe{{225, 120}, {258, 120}, {319, 180}, {319, 239}, {275, 239}, {273, 185}};
    cv::fillPoly(edge, std::vector<std::vector<cv::Point>>{stripe}, cv::Scalar(120, 120, 120));
    cv::rectangle(edge, {270, 190}, {319, 195}, cv::Scalar(195, 195, 195), cv::FILLED);
    cv::rectangle(edge, {155, 218}, {185, 239}, cv::Scalar(110, 110, 110), cv::FILLED);
    settings.threshold = 160; settings.roi_start = .5; settings.roi_top_width = .6;
    auto track = detect_line(edge, settings);
    assert(track.found && track.error > .5);
    cv::flip(edge, edge, 1);
    track = detect_line(edge, settings);
    assert(track.found && track.error < -.5);

    cv::Mat large(720, 1280, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::rectangle(large, {900, 144}, {1030, 719}, cv::Scalar(0, 0, 0), cv::FILLED);
    settings.roi_start = .2; settings.roi_top_width = 1;
    auto scaled = detect_for_control(large, settings, true);
    assert(scaled.found && scaled.processing_width == 320 && scaled.processing_height == 180);
    assert(scaled.point.x > 850 && scaled.point.x < 1100 && scaled.polygon.front().y >= 140 && scaled.polygon.front().y <= 150);
    assert(!scaled.outline.empty());

    DebouncedInput forward, backward;
    assert(!forward.update(true, 1.0));
    assert(!forward.update(true, 1.02));
    assert(!forward.update(false, 1.021)); // Bounce resets the timer.
    assert(!forward.update(true, 1.022));
    assert(!backward.update(true, 1.03));
    assert(forward.update(true, 1.053));
    assert(!backward.update(true, 1.053)); // Inputs debounce independently.
    assert(!forward.update(false, 1.054)); // Release is immediate.
    assert(backward.update(true, 1.061));
    assert(!backward.update(false, 1.062));

    PID pid;
    settings.kp = 2; settings.ki = 1; settings.kd = .5;
    assert(pid.update(1, .1, settings) == 1 && pid.integral == 0);
    Motion motion;
    settings.speed = .5; settings.ramp_rate = 1;
    assert(motion.physical(true, false, true, {.5, .5}, settings, .1).first == .1);
    assert(motion.physical(true, true, true, {.5, .5}, settings, .1).first == 0);
    assert(motion.remote("forward", settings, .1).first == .1);
    assert(motion.remote("stop", settings, .1).first == 0);
    std::cout << "Native vision, PID, motion and stop tests passed\n";
}
