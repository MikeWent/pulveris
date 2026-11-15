#include "core.hpp"

#include <opencv2/imgproc.hpp>

Detection detect_line(const cv::Mat &frame, const Settings &settings, bool debug) {
    Detection result;
    const int height = frame.rows, width = frame.cols;
    if (width < 2 || height < 2) return result;
    const int top = std::min(static_cast<int>(height * settings.roi_start), height - 1);
    const double center = (width - 1) / 2.0;
    const auto rounded = [](double x) { return static_cast<int>(std::nearbyint(x)); };
    result.polygon = {
        {rounded(center * (1 - settings.roi_top_width)), top},
        {rounded(center * (1 + settings.roi_top_width)), top},
        {rounded(center * (1 + settings.roi_bottom_width)), height - 1},
        {rounded(center * (1 - settings.roi_bottom_width)), height - 1},
    };
    cv::Mat region = cv::Mat::zeros(height, width, CV_8UC1);
    cv::fillPoly(region, std::vector<std::vector<cv::Point>>{result.polygon}, cv::Scalar(255));
    const int window = std::max(3, rounded(width * .25) | 1);
    const int gray_top = std::max(0, top - window / 2 - 1);
    cv::Mat gray, filtered, absolute, local;
    cv::Mat view = frame.rowRange(gray_top, height);
    if (frame.channels() == 3) cv::cvtColor(view, gray, cv::COLOR_BGR2GRAY);
    else if (frame.channels() == 1) gray = view;
    else throw std::runtime_error("Unsupported camera frame format");
    cv::medianBlur(gray, filtered, 3);
    const int mode = settings.light_line ? cv::THRESH_BINARY : cv::THRESH_BINARY_INV;
    cv::threshold(filtered, absolute, settings.threshold, 255, mode);
    cv::adaptiveThreshold(filtered, local, 255, cv::ADAPTIVE_THRESH_MEAN_C, mode, window,
                          settings.light_line ? -settings.contrast : settings.contrast);
    cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Mat part = mask.rowRange(top, height);
    cv::bitwise_and(absolute.rowRange(top - gray_top, absolute.rows),
                    local.rowRange(top - gray_top, local.rows), part);
    cv::bitwise_and(part, region.rowRange(top, height), part);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat::ones(3, 3, CV_8UC1));

    const int kw = std::max(3, rounded(width * .028) | 1);
    const int kh = std::max(3, rounded(height * .028) | 1);
    const int work_top = std::max(0, top - kh / 2);
    cv::Mat narrow = cv::Mat::zeros(height - work_top, width, CV_8UC1);
    const int max_width = static_cast<int>(width * settings.max_line_width);
    for (int y = work_top; y < height; ++y) {
        const auto *source = mask.ptr<unsigned char>(y);
        auto *dest = narrow.ptr<unsigned char>(y - work_top);
        for (int x = 0; x < width;) {
            if (!source[x]) { ++x; continue; }
            const int start = x;
            while (x < width && source[x]) ++x;
            if (x - start >= 2 && x - start <= max_width)
                std::fill(dest + start, dest + x, 255);
        }
    }
    cv::Mat connected, labels, stats, centroids;
    cv::morphologyEx(narrow, connected, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kw, kh)));
    cv::bitwise_and(connected, region.rowRange(work_top, height), connected);
    int count = cv::connectedComponentsWithStats(connected, labels, stats, centroids, 8, CV_32S);
    std::vector<int> pixels(count, 0);
    for (int y = 0; y < narrow.rows; ++y) {
        const auto *bits = narrow.ptr<unsigned char>(y);
        const auto *ids = labels.ptr<int>(y);
        for (int x = 0; x < width; ++x) if (bits[x]) ++pixels[ids[x]];
    }
    const int region_area = cv::countNonZero(region), road_height = height - top;
    double best_score = -1;
    int best_label = -1;
    for (int i = 1; i < count; ++i) {
        int y = stats.at<int>(i, cv::CC_STAT_TOP) + work_top;
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        double fraction = static_cast<double>(pixels[i]) / region_area;
        if (h >= std::max(3.0, road_height * .2) && fraction >= settings.min_fraction && fraction <= settings.max_fraction) {
            double proximity = 1 + static_cast<double>(y + h - 1 - top) / road_height;
            double score = static_cast<double>(pixels[i]) * h * proximity;
            if (score >= best_score) { best_score = score; best_label = i; }
        }
    }
    if (best_label < 0) return result;
    if (debug) result.selected = cv::Mat::zeros(height, width, CV_8UC1);
    double sum = 0, sum_x = 0, sum_y = 0;
    std::vector<cv::Point> right_edge;
    const double scale = std::max(height - 1 - top, 1);
    for (int y = 0; y < narrow.rows; ++y) {
        const auto *bits = narrow.ptr<unsigned char>(y);
        const auto *ids = labels.ptr<int>(y);
        auto *dest = debug ? result.selected.ptr<unsigned char>(y + work_top) : nullptr;
        const double weight = 1 + (y + work_top - top) / scale;
        int first = -1, last = -1;
        for (int x = 0; x < width; ++x) if (bits[x] && ids[x] == best_label) {
            if (dest) dest[x] = 255;
            if (debug) { if (first < 0) first = x; last = x; }
            sum += weight; sum_x += x * weight; sum_y += (y + work_top) * weight;
        }
        if (debug && first >= 0 && y % std::max(1, height / 40) == 0) {
            result.outline.emplace_back(first, y + work_top);
            right_edge.emplace_back(last, y + work_top);
        }
    }
    result.outline.insert(result.outline.end(), right_edge.rbegin(), right_edge.rend());
    if (sum == 0) return result;
    double x = sum_x / sum, y = sum_y / sum;
    result.found = true;
    result.error = 2 * x / (width - 1) - 1;
    result.point = {rounded(x), rounded(y)};
    return result;
}

Detection detect_for_control(const cv::Mat &frame, const Settings &settings, bool debug) {
    if (frame.empty()) return {};
    const double scale = std::min({1.0, 320.0 / frame.cols, 240.0 / frame.rows});
    cv::Mat working = frame;
    if (scale < 1) cv::resize(frame, working, cv::Size(), scale, scale, cv::INTER_AREA);
    auto result = detect_line(working, settings, debug);
    result.processing_width = working.cols; result.processing_height = working.rows;
    if (scale < 1) {
        const double x = static_cast<double>(frame.cols - 1) / std::max(working.cols - 1, 1);
        const double y = static_cast<double>(frame.rows - 1) / std::max(working.rows - 1, 1);
        auto restore = [x, y](cv::Point &p) { p.x = cvRound(p.x * x); p.y = cvRound(p.y * y); };
        for (auto &p : result.polygon) restore(p);
        for (auto &p : result.outline) restore(p);
        if (result.found) restore(result.point);
    }
    return result;
}
