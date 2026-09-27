#ifndef DESTRUCTION_TRANSFORM_HPP
#define DESTRUCTION_TRANSFORM_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/quat.hpp"
#include "destruction/math/math_utils.hpp"

namespace destruction::math {

struct Transform {
    Vec3 position{Vec3::zero()};
    Quat orientation{Quat::identity()};

    constexpr Transform() = default;
    constexpr Transform(const Vec3& pos, const Quat& rot)
        : position(pos), orientation(rot) {}

    static constexpr Transform identity() {
        return Transform(Vec3::zero(), Quat::identity());
    }

    Vec3 transform_point(const Vec3& local_point) const {
        return position + orientation.rotate(local_point);
    }

    Vec3 inverse_transform_point(const Vec3& world_point) const {
        return orientation.conjugate().rotate(world_point - position);
    }

    Vec3 transform_direction(const Vec3& local_direction) const {
        return orientation.rotate(local_direction);
    }

    Vec3 inverse_transform_direction(const Vec3& world_direction) const {
        return orientation.conjugate().rotate(world_direction);
    }

    Transform combine(const Transform& child) const {
        return Transform(
            position + orientation.rotate(child.position),
            (orientation * child.orientation).normalize()
        );
    }

    Transform inverse() const {
        Quat inv_rot = orientation.conjugate();
        Vec3 inv_pos = inv_rot.rotate(-position);
        return Transform(inv_pos, inv_rot);
    }

    bool operator==(const Transform& rhs) const {
        return (position == rhs.position) && (orientation == rhs.orientation);
    }

    bool operator!=(const Transform& rhs) const {
        return !(*this == rhs);
    }

    bool is_valid() const {
        return position.is_valid() && orientation.is_valid();
    }
};

} // namespace destruction::math

#endif // DESTRUCTION_TRANSFORM_HPP
