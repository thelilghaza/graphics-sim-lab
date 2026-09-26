#include "raylab/ppm_writer.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

namespace raylab {

bool PPMWriter::write_ppm(const std::string& filename,
                          int width,
                          int height,
                          const std::vector<Color>& buffer) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file for writing PPM output: " << filename << "\n";
        return false;
    }

    file << "P3\n" << width << " " << height << "\n255\n";

    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            const auto& pixel_color = buffer[static_cast<size_t>(j * width + i)];

            // Exact pipeline required:
            // linear color -> clamp to [0, 1] -> gamma 2.0 via sqrt() -> scale to [0, 255] -> int
            double r_clamp = std::clamp(pixel_color.x(), 0.0, 1.0);
            double g_clamp = std::clamp(pixel_color.y(), 0.0, 1.0);
            double b_clamp = std::clamp(pixel_color.z(), 0.0, 1.0);

            double r_gamma = std::sqrt(r_clamp);
            double g_gamma = std::sqrt(g_clamp);
            double b_gamma = std::sqrt(b_clamp);

            int r = static_cast<int>(255.999 * r_gamma);
            int g = static_cast<int>(255.999 * g_gamma);
            int b = static_cast<int>(255.999 * b_gamma);

            file << r << " " << g << " " << b << "\n";
        }
    }

    return true;
}

} // namespace raylab
