#ifndef DEMO_FRACTURE_CPP
#define DEMO_FRACTURE_CPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/math_utils.hpp"
#include "destruction/fracture/fracture_volume.hpp"
#include "destruction/fracture/site_generator.hpp"
#include "destruction/fracture/voronoi3d.hpp"
#include "destruction/fracture/obj_exporter.hpp"
#include "destruction/fracture/mesh_validator.hpp"

#include <iostream>
#include <iomanip>
#include <string>

using namespace destruction::math;
using namespace destruction::fracture;

int main() {
    std::cout << "Project 04 — Milestone 2 Voronoi Fracture Geometry Demo\n";
    std::cout << "--------------------------------------------------------\n";

    // Define source box: [-1, 1] x [-1, 1] x [-1, 1] with density 2.5 kg/m^3
    FractureVolume source_box(Vec3(-1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f), 2.5f);

    size_t requested_sites = 12;
    float min_site_dist = 0.45f;
    uint32_t random_seed = 42;

    std::cout << "Source Volume Bounds  : [" << source_box.min_pt.x << ", " << source_box.max_pt.x << "] x ["
              << source_box.min_pt.y << ", " << source_box.max_pt.y << "] x ["
              << source_box.min_pt.z << ", " << source_box.max_pt.z << "]\n";
    std::cout << "Source Volume         : " << std::fixed << std::setprecision(4) << source_box.volume() << " m^3\n";
    std::cout << "Source Mass (density) : " << source_box.mass() << " kg (" << source_box.density << " kg/m^3)\n";
    std::cout << "Target Site Count     : " << requested_sites << " (min dist = " << min_site_dist << " m, seed = " << random_seed << ")\n\n";

    std::vector<Vec3> sites = SiteGenerator::generate_3d(source_box, requested_sites, min_site_dist, random_seed);
    std::cout << "Generated Sites       : " << sites.size() << "\n";

    std::cout << "Computing 3D Voronoi Half-Space Partitioning...\n";
    auto shards = Voronoi3D::compute_partition(source_box, sites);
    auto summary = Voronoi3D::evaluate_summary(source_box, shards);

    std::cout << "\nFracture Summary Statistics:\n";
    std::cout << "--------------------------------------------------------\n";
    std::cout << "Valid Shard Count     : " << summary.valid_shard_count << " / " << summary.requested_site_count << "\n";
    std::cout << "Sum Shard Volume      : " << summary.sum_shard_volume << " m^3\n";
    std::cout << "Relative Volume Error : " << (summary.relative_volume_error * 100.0f) << " %\n";
    std::cout << "Sum Shard Mass        : " << summary.sum_shard_mass << " kg\n";
    std::cout << "Relative Mass Error   : " << (summary.relative_mass_error * 100.0f) << " %\n\n";

    std::cout << "Per-Shard Geometry Summary:\n";
    std::cout << "--------------------------------------------------------\n";
    for (const auto& shard : shards) {
        auto val = MeshValidator::validate(shard.mesh);
        std::cout << "Shard #" << std::setw(2) << shard.id << " | Site: ("
                  << std::setw(6) << shard.site.x << ", "
                  << std::setw(6) << shard.site.y << ", "
                  << std::setw(6) << shard.site.z << ") | Vol: "
                  << std::setw(6) << shard.volume << " m^3 | Verts: "
                  << std::setw(2) << shard.mesh.vertices.size() << " | Faces: "
                  << std::setw(2) << shard.mesh.faces.size() << " | Manifold: "
                  << (val.is_closed_manifold ? "YES" : "NO") << "\n";
    }

    std::string export_path = "fracture_shards.obj";
    std::cout << "\nExporting Wavefront OBJ geometry to '" << export_path << "'...\n";
    bool export_ok = ObjExporter::export_shards(export_path, shards);
    if (export_ok) {
        std::cout << "OBJ Export Status      : SUCCESS\n";
    } else {
        std::cout << "OBJ Export Status      : FAILED\n";
    }

    std::cout << "--------------------------------------------------------\n";
    std::cout << "Milestone 2 Geometry Validation Complete!\n";

    return 0;
}

#endif // DEMO_FRACTURE_CPP
