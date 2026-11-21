#pragma once

#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <ostream>

namespace rla::core {

constexpr int64_t BILLION = 1'000'000'000LL;

class Duration {
   private:
    int32_t sec_{0};      //!< Part of the time relative to the seconds.
    int32_t nanosec_{0};  //!< Part of the time relative to the nanoseconds.

    void normalize();

   public:
    Duration() = default;
    Duration(const int32_t s, const int32_t ns) : sec_(s), nanosec_(ns) {
        normalize();
    }
    Duration(const int64_t ns) {
        sec_ = static_cast<int32_t>(ns / BILLION);
        nanosec_ = static_cast<int32_t>(ns % BILLION);
        normalize();
    }

    [[nodiscard]] double getTimeInSeconds() const {
        return sec_ + nanosec_ * 1e-9;
    }

    /**
     * @brief get the time in nanoseconds.
     */
    [[nodiscard]] int64_t getTimeInNanoseconds() const {
        return static_cast<int64_t>(sec_ * BILLION) +
               static_cast<int64_t>(nanosec_);
    }

    int32_t getSeconds() const { return sec_; }
    int32_t getNanoseconds() const { return nanosec_; }

    // Duration + Duration = Duration
    [[nodiscard]] Duration operator+(const Duration& other) const;
    // Duration - Duration = Duration
    [[nodiscard]] Duration operator-(const Duration& other) const;
    // Duration * double = Duration
    [[nodiscard]] Duration operator*(double s) const;

    // Self comparisons
    bool operator==(const Duration& other) const;
    bool operator!=(const Duration& other) const;
    bool operator<(const Duration& other) const;
    bool operator>(const Duration& other) const;
    bool operator<=(const Duration& other) const;
    bool operator>=(const Duration& other) const;

    std::ostream& operator<<(std::ostream& os) const;

    friend std::ostream& operator<<(std::ostream& os, const Duration& ts) {
        os << ts.sec_ << "." << std::setw(9) << std::setfill('0')
           << ts.nanosec_;
        return os;
    }

    void print() const;
};

}  // namespace rla::core