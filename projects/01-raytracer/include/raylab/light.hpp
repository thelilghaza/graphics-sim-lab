#pragma once

#include "raylab/color.hpp"
#include "raylab/vec3.hpp"

namespace raylab {

class PointLight {
public:
    PointLight(const Point3& pos, const Color& col = Color(1.0, 1.0, 1.0), double intens = 1.0)
        : light_position(pos), light_color(col), light_intensity(intens) {}

    const Point3& position() const { return light_position; }
    const Color& color() const { return light_color; }
    double intensity() const { return light_intensity; }

private:
    Point3 light_position;
    Color light_color;
    double light_intensity;
};

} // namespace raylab
