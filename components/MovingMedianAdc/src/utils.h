#ifndef UTILS_H_
#define UTILS_H_

#include <vector>

namespace utils {

// returns 0 if the vector is empty
template <typename T>
T median(const std::vector<T> &a) {
    if (a.empty()) {
        return 0;
    }

    std::vector<T> b = a;
    int index = b.size() / 2;
    std::nth_element(b.begin(), b.begin() + index, b.end());
    return b[index];
}

template <typename T>
class MovingMedian {
   public:
    MovingMedian(int windowSize, T initialValue) : windowSize(windowSize) {
        for (int i = 0; i < windowSize; i++) {
            values.push_back(initialValue);
        }
    }

    void reset(T initialValue) {
        for (int i = 0; i < windowSize; i++) {
            values[i] = initialValue;
        }
    }

    void add(T value) {
        // `values` is always kept at `windowSize` elements
        values.erase(values.begin());
        values.push_back(value);
    }

    T getMedian() { return median(values); }

    T getLatest() { return values.back(); }

   private:
    const int windowSize;
    std::vector<T> values;
};

}  // namespace utils

#endif  // UTILS_H_