#include "rotating_lidar_accumulator/core/timestamp.hpp"

namespace rla::core {

void Timestamp::normalize() {
    int64_t total_ns = static_cast<int64_t>(sec_) * BILLION + nanosec_;

    sec_ = static_cast<int32_t>(total_ns / BILLION);
    nanosec_ = static_cast<int32_t>(total_ns % BILLION);

    if (nanosec_ < 0) {
        nanosec_ += BILLION;
        sec_ -= 1;
    }
}

Timestamp Timestamp::operator+(const Duration& d) const {
    Timestamp out(sec_ + d.getSeconds(), nanosec_ + d.getNanoseconds());
    out.normalize();
    return out;
}

Timestamp Timestamp::operator-(const Duration& d) const {
    int64_t s = sec_ - d.getSeconds();
    int64_t ns = static_cast<int64_t>(nanosec_) - d.getNanoseconds();

    if (ns < 0) {
        ns += 1'000'000'000;
        s -= 1;
    }

    return Timestamp(static_cast<int32_t>(s), static_cast<int32_t>(ns));
}

Duration Timestamp::operator-(const Timestamp& other) const {
    int64_t s = sec_ - other.sec_;
    int64_t ns = static_cast<int64_t>(nanosec_) - other.nanosec_;

    if (ns < 0) {
        ns += 1'000'000'000;
        s -= 1;
    }

    return Duration(static_cast<int32_t>(s), static_cast<int32_t>(ns));
}

bool Timestamp::operator==(const Timestamp& other) const {
    return sec_ == other.sec_ && nanosec_ == other.nanosec_;
}

bool Timestamp::operator!=(const Timestamp& other) const {
    return !(*this == other);
}

bool Timestamp::operator<(const Timestamp& other) const {
    return (sec_ < other.sec_) ||
           (sec_ == other.sec_ && nanosec_ < other.nanosec_);
}

bool Timestamp::operator>(const Timestamp& other) const {
    return other < *this;
}

bool Timestamp::operator<=(const Timestamp& other) const {
    return !(other < *this);
}

bool Timestamp::operator>=(const Timestamp& other) const {
    return !(*this < other);
}

std::ostream& Timestamp::operator<<(std::ostream& os) const {
    os << sec_ << "." << std::setw(9) << std::setfill('0') << nanosec_;
    return os;
}

void Timestamp::print() const { printf("%d.%09d\n", sec_, nanosec_); }

}  // namespace rla::core