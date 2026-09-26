#pragma once

#include "raylab/vec3.hpp"
#include <string>
#include <vector>

namespace raylab {

class PPMWriter {
public:
    static bool write_ppm(const std::string& filename,
                          int width,
                          int height,
                          const std::vector<Color>& buffer);
};

} // namespace raylab
