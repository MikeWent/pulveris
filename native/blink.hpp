#pragma once

#include <cmath>
#include <optional>

class ErrorBlink {
public:
    bool update(bool error, double timestamp) {
        if (!error) { since.reset(); return false; }
        if (!since) since = timestamp;
        return std::fmod(timestamp - *since, 1.0) < .5;
    }
private:
    std::optional<double> since;
};
