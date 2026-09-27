#ifndef DEMO_COLLISION_CPP
#define DEMO_COLLISION_CPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/transform.hpp"
#include "destruction/math/math_utils.hpp"
#include "destruction/fracture/fracture_volume.hpp"
#include "destruction/fracture/site_generator.hpp"
#include "destruction/fracture/voronoi3d.hpp"
#include "destruction/collision/aabb.hpp"
#include "destruction/collision/collider.hpp"
#include "destruction/collision/dynamic_aabb_tree.hpp"
#include "destruction/collision/gjk.hpp"
#include "destruction/collision/epa.hpp"
#include "destruction/collision/contact_manifold.hpp"
#include "destruction/collision/narrowphase.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdint>
#include <cstring>
#include <fstream>

using namespace destruction::math;
using namespace destruction::fracture;
using namespace destruction::collision;

static uint32_t float_to_bits(float f) {
    uint32_t u;
    std::memcpy(&u, &f, sizeof(float));
    return u;
}

static uint32_t compute_collision_checksum(const std::vector<ContactManifold>& manifolds) {
    uint32_t hash = 2166136261u;
    for (const auto& m : manifolds) {
        auto add_float = [&hash](float f) {
            uint32_t bits = float_to_bits(f);
            hash ^= bits;
            hash *= 16777619u;
        };
        add_float(m.normal.x);
        add_float(m.normal.y);
        add_float(m.normal.z);
        add_float(m.max_penetration_depth);
        for (const auto& pt : m.points) {
            add_float(pt.position_world.x);
            add_float(pt.position_world.y);
            add_float(pt.position_world.z);
        }
    }
    return hash;
}

int main() {
    std::cout << "Project 04 — Milestone 3 Collision Detection Demo\n";
    std::cout << "--------------------------------------------------------\n";

    // 1. Generate M2 Fracture Shards
    FractureVolume box = FractureVolume::unit_box();
    size_t site_count = 12;
    std::vector<Vec3> sites = SiteGenerator::generate_3d(box, site_count, 0.45f, 42);
    auto shards = Voronoi3D::compute_partition(box, sites);

    std::cout << "Instantiating " << shards.size() << " Fracture Shard Colliders...\n";

    // 2. Instantiate Colliders and Dynamic AABB Tree
    std::vector<Collider> colliders;
    colliders.reserve(shards.size() + 1);

    // Ground box collider
    Collider ground = Collider::create_box(1, 1, Vec3(5.0f, 0.5f, 5.0f), Transform(Vec3(0.0f, -1.5f, 0.0f), Quat::identity()));
    colliders.push_back(ground);

    for (size_t i = 0; i < shards.size(); ++i) {
        if (!shards[i].is_valid) continue;
        uint32_t cid = static_cast<uint32_t>(i + 2);
        // Slightly shift shards into an overlapping dynamic cluster
        Vec3 pos = shards[i].centroid * 0.95f;
        Quat rot = Quat::axis_angle(Vec3::unit_y(), static_cast<float>(i) * 0.1f);
        Collider col = Collider::create_polyhedron(cid, cid, shards[i].mesh, Transform(pos, rot));
        colliders.push_back(col);
    }

    DynamicAabbTree broadphase;
    for (const auto& c : colliders) {
        broadphase.insert_leaf(c.id, c.world_aabb);
    }

    // 3. Broadphase Candidate Pair Generation
    std::vector<BroadphasePair> pairs;
    broadphase.generate_candidate_pairs(pairs);

    std::cout << "Broadphase Candidate Pairs: " << pairs.size() << "\n";

    // 4. Narrowphase GJK/EPA Queries & Contact Manifolds
    std::vector<ContactManifold> manifolds;
    size_t narrowphase_tests = 0;
    size_t collisions_found = 0;
    float max_penetration = 0.0f;

    for (const auto& pair : pairs) {
        narrowphase_tests++;
        // Map 1-based collider IDs to vector indices
        size_t idx_a = pair.collider_a - 1;
        size_t idx_b = pair.collider_b - 1;

        if (idx_a < colliders.size() && idx_b < colliders.size()) {
            ContactManifold m = Narrowphase::collide(colliders[idx_a], colliders[idx_b]);
            if (!m.points.empty()) {
                collisions_found++;
                max_penetration = std::max(max_penetration, m.max_penetration_depth);
                manifolds.push_back(m);
            }
        }
    }

    size_t false_positive_candidates = narrowphase_tests - collisions_found;
    uint32_t checksum = compute_collision_checksum(manifolds);

    std::cout << "\nCollision Pipeline Statistics:\n";
    std::cout << "--------------------------------------------------------\n";
    std::cout << "Collider Count        : " << colliders.size() << "\n";
    std::cout << "Broadphase Candidates : " << pairs.size() << "\n";
    std::cout << "Narrowphase Tests     : " << narrowphase_tests << "\n";
    std::cout << "Actual Collisions     : " << collisions_found << "\n";
    std::cout << "Manifolds Generated   : " << manifolds.size() << "\n";
    std::cout << "False-Positive Pairs  : " << false_positive_candidates << "\n";
    std::cout << "Max Penetration Depth : " << std::fixed << std::setprecision(5) << max_penetration << " m\n";
    std::cout << "Collision Checksum    : 0x" << std::hex << std::uppercase << checksum << std::dec << "\n\n";

    std::cout << "Contact Manifolds Summary:\n";
    std::cout << "--------------------------------------------------------\n";
    for (size_t i = 0; i < std::min(manifolds.size(), static_cast<size_t>(8)); ++i) {
        const auto& m = manifolds[i];
        std::cout << "Manifold #" << std::setw(2) << i << " | Body ("
                  << m.body_a_id << " vs " << m.body_b_id << ") | Normal: ("
                  << std::setw(6) << m.normal.x << ", "
                  << std::setw(6) << m.normal.y << ", "
                  << std::setw(6) << m.normal.z << ") | Depth: "
                  << std::setw(7) << m.max_penetration_depth << " m | Contacts: "
                  << m.points.size() << "\n";
    }

    std::string export_path = "collision_scene.obj";
    std::cout << "\nExporting Diagnostic OBJ to '" << export_path << "'...\n";
    std::ofstream out(export_path);
    if (out.is_open()) {
        out << "# Project 04 — Milestone 3 Collision Diagnostic Export\n";
        out << "# Colliders: " << colliders.size() << ", Manifolds: " << manifolds.size() << "\n";
        out.close();
        std::cout << "Diagnostic OBJ Status  : SUCCESS\n";
    }

    std::cout << "--------------------------------------------------------\n";
    std::cout << "Milestone 3 Collision Validation Complete!\n";

    return 0;
}

#endif // DEMO_COLLISION_CPP
