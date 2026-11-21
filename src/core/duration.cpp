#include "rotating_lidar_accumulator/core/duration.hpp"

namespace rla::core {
void Duration::normalize() {
    int64_t total_ns = static_cast<int64_t>(sec_) * BILLION + nanosec_;

    sec_ = static_cast<int32_t>(total_ns / BILLION);
    nanosec_ = static_cast<int32_t>(total_ns % BILLION);

    if (nanosec_ < 0) {
        nanosec_ += BILLION;
        sec_ -= 1;
    }
}

Duration Duration::operator+(const Duration& other) const {
    Duration out(sec_ + other.sec_, nanosec_ + other.nanosec_);
    out.normalize();
    return out;
}

Duration Duration::operator-(const Duration& other) const {
    Duration out(sec_ - other.sec_, nanosec_ - other.nanosec_);
    out.normalize();
    return out;
}

Duration Duration::operator*(double s) const {
    // Convert to total nanoseconds
    int64_t total_ns = static_cast<int64_t>(sec_) * BILLION + nanosec_;

    // Scale
    int64_t scaled_ns = static_cast<int64_t>(total_ns * s);

    // Convert back
    Duration out;
    out.sec_ = static_cast<int32_t>(scaled_ns / BILLION);
    out.nanosec_ = static_cast<int32_t>(scaled_ns % BILLION);

    out.normalize();
    return out;
}

bool Duration::operator==(const Duration& other) const {
    return sec_ == other.sec_ && nanosec_ == other.nanosec_;
}

bool Duration::operator!=(const Duration& other) const {
    return !(*this == other);
}

bool Duration::operator<(const Duration& other) const {
    return (sec_ < other.sec_) ||
           (sec_ == other.sec_ && nanosec_ < other.nanosec_);
}

bool Duration::operator>(const Duration& other) const { return other < *this; }

bool Duration::operator<=(const Duration& other) const {
    return !(*this > other);
}

bool Duration::operator>=(const Duration& other) const {
    return !(*this < other);
}

std::ostream& Duration::operator<<(std::ostream& os) const {
    os << sec_ << "." << std::setw(9) << std::setfill('0') << nanosec_;
    return os;
}

inline void Duration::print() const { printf("%d.%09d\n", sec_, nanosec_); }

}  // namespace rla::core
