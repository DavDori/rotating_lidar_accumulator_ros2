#pragma once

#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <ostream>

#include "rotating_lidar_accumulator/core/duration.hpp"

namespace core {

class Timestamp {
   private:
    int32_t sec_{0};      //!< Part of the time relative to the seconds.
    int32_t nanosec_{0};  //!< Part of the time relative to the nanoseconds.

    /**
     * @brief Keep nanosec < 1e9 and >= 0.
     */
    void normalize();

   public:
    Timestamp() = default;
    Timestamp(const int32_t s, const int32_t ns) : sec_(s), nanosec_(ns) {
        normalize();
    }
    Timestamp(const int64_t ns) {
        sec_ = static_cast<int32_t>(ns / BILLION);
        nanosec_ = static_cast<int32_t>(ns % BILLION);
        normalize();
    }

    /**
     * @brief get the time in seconds.
     */
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

    /**
     * @brief get the time part containing the seconds.
     */
    [[nodiscard]] int32_t getSeconds() const { return sec_; }

    /**
     * @brief get the time part containing the nanoseconds.
     */
    [[nodiscard]] int32_t getNanoseconds() const { return nanosec_; }

    [[nodiscard]] Timestamp operator+(const Duration& d) const;
    [[nodiscard]] Timestamp operator-(const Duration& d) const;
    [[nodiscard]] Duration operator-(const Timestamp& other) const;

    bool operator==(const Timestamp& other) const;
    bool operator!=(const Timestamp& other) const;
    bool operator<(const Timestamp& other) const;
    bool operator>(const Timestamp& other) const;
    bool operator<=(const Timestamp& other) const;
    bool operator>=(const Timestamp& other) const;
    // Forbidden operations with Duration
    bool operator==(const Duration&) const = delete;
    bool operator!=(const Duration&) const = delete;
    bool operator<(const Duration&) const = delete;
    bool operator>(const Duration&) const = delete;
    bool operator<=(const Duration&) const = delete;
    bool operator>=(const Duration&) const = delete;
    // Delete Time + Time (not allowed)
    Timestamp operator+(const Timestamp&) const = delete;

    std::ostream& operator<<(std::ostream& os) const;
    friend std::ostream& operator<<(std::ostream& os, const Timestamp& ts) {
        os << ts.sec_ << "." << std::setw(9) << std::setfill('0')
           << ts.nanosec_;
        return os;
    }

    void print() const;
};

}  // namespace core