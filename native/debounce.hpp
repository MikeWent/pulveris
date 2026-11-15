#pragma once

#include <optional>

// Debounce relay closure; release is passed through immediately.
class DebouncedInput {
public:
    bool update(bool active, double timestamp) {
        if (!active) { since.reset(); return false; }
        if (!since) since = timestamp;
        return timestamp - *since >= .030;
    }
private:
    std::optional<double> since;
};
