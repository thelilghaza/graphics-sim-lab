#ifndef DESTRUCTION_OBJ_EXPORTER_HPP
#define DESTRUCTION_OBJ_EXPORTER_HPP

#include "destruction/fracture/shard.hpp"
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>

namespace destruction::fracture {

class ObjExporter {
public:
    static bool export_shards(const std::string& filepath, const std::vector<Shard>& shards) {
        std::ofstream out(filepath);
        if (!out.is_open()) {
            return false;
        }

        out << "# Project 04 — Procedural Destruction Sandbox\n";
        out << "# Voronoi Fracture Shard Geometry Export\n";
        out << "# Total Shards: " << shards.size() << "\n\n";

        size_t vertex_offset = 1;

        for (size_t s = 0; s < shards.size(); ++s) {
            const auto& shard = shards[s];
            if (!shard.is_valid || shard.mesh.vertices.empty()) continue;

            out << "g shard_" << std::setfill('0') << std::setw(2) << s << "\n";

            for (const auto& v : shard.mesh.vertices) {
                out << "v " << v.x << " " << v.y << " " << v.z << "\n";
            }

            for (const auto& face : shard.mesh.faces) {
                size_t n = face.vertex_indices.size();
                if (n < 3) continue;

                // Fan triangulation for OBJ face output
                for (size_t k = 1; k + 1 < n; ++k) {
                    uint32_t i0 = static_cast<uint32_t>(vertex_offset + face.vertex_indices[0]);
                    uint32_t ik = static_cast<uint32_t>(vertex_offset + face.vertex_indices[k]);
                    uint32_t ik1 = static_cast<uint32_t>(vertex_offset + face.vertex_indices[k + 1]);

                    out << "f " << i0 << " " << ik << " " << ik1 << "\n";
                }
            }

            vertex_offset += shard.mesh.vertices.size();
            out << "\n";
        }

        out.close();
        return true;
    }
};

} // namespace destruction::fracture

#endif // DESTRUCTION_OBJ_EXPORTER_HPP
