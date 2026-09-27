#include "destruction/math/math_utils.hpp"
#include "destruction/math/vec3.hpp"
#include "destruction/math/mat3.hpp"
#include "destruction/math/quat.hpp"
#include "destruction/math/transform.hpp"
#include "destruction/collision/aabb.hpp"
#include "destruction/collision/collider.hpp"
#include "destruction/collision/dynamic_aabb_tree.hpp"
#include "destruction/collision/support.hpp"
#include "destruction/collision/gjk.hpp"
#include "destruction/collision/epa.hpp"
#include "destruction/collision/sat.hpp"
#include "destruction/collision/contact_manifold.hpp"
#include "destruction/collision/narrowphase.hpp"
#include "destruction/fracture/fracture_volume.hpp"
#include "destruction/fracture/site_generator.hpp"
#include "destruction/fracture/voronoi3d.hpp"

#include <iostream>
#include <cmath>
#include <cassert>

using namespace destruction::math;
using namespace destruction::fracture;
using namespace destruction::collision;

void test_aabb() {
    Aabb b(Vec3(-1, -1, -1), Vec3(1, 1, 1));
    assert(b.center() == Vec3(0, 0, 0));
    assert(b.extents() == Vec3(1, 1, 1));
    assert(is_nearly_equal(b.surface_area(), 24.0f));
    assert(b.contains(Vec3(0, 0, 0)));
    assert(!b.contains(Vec3(2, 0, 0)));

    Aabb b2(Vec3(0.5f, 0.5f, 0.5f), Vec3(2.0f, 2.0f, 2.0f));
    assert(b.overlaps(b2));

    Transform t(Vec3(10, 0, 0), Quat::axis_angle(Vec3::unit_y(), HALF_PI));
    Aabb tb = b.transform(t);
    assert(is_nearly_equal(tb.center().x, 10.0f));
    assert(is_nearly_equal(tb.center().y, 0.0f));
    assert(is_nearly_equal(tb.center().z, 0.0f));
}

void test_dynamic_tree() {
    DynamicAabbTree tree;
    Aabb box1(Vec3(-1, -1, -1), Vec3(1, 1, 1));
    Aabb box2(Vec3(0.5f, 0, 0), Vec3(2, 2, 2));
    Aabb box3(Vec3(10, 10, 10), Vec3(12, 12, 12));

    int leaf1 = tree.insert_leaf(1, box1);
    int leaf2 = tree.insert_leaf(2, box2);
    int leaf3 = tree.insert_leaf(3, box3);
    (void)leaf1;
    (void)leaf2;

    std::vector<BroadphasePair> pairs;
    tree.generate_candidate_pairs(pairs);

    assert(pairs.size() == 1);
    assert(pairs[0].collider_a == 1 && pairs[0].collider_b == 2);

    tree.remove_leaf(leaf3);
    tree.generate_candidate_pairs(pairs);
    assert(pairs.size() == 1);
}

void test_support_mapping() {
    Collider box = Collider::create_box(1, 1, Vec3(1, 2, 3));
    Vec3 sup_x = box.get_support_world(Vec3::unit_x());
    assert(is_nearly_equal(sup_x.x, 1.0f));

    Transform t(Vec3(0, 0, 0), Quat::axis_angle(Vec3::unit_z(), HALF_PI));
    box.update_world_transform(t);
    Vec3 sup_rot_x = box.get_support_world(Vec3::unit_x());
    assert(is_nearly_equal(sup_rot_x.x, 2.0f, 1e-3f));
}

void test_gjk_epa_collision() {
    Collider box_a = Collider::create_box(1, 1, Vec3(1, 1, 1), Transform(Vec3(0, 0, 0), Quat::identity()));
    Collider box_b = Collider::create_box(2, 2, Vec3(1, 1, 1), Transform(Vec3(1.5f, 0, 0), Quat::identity()));

    GjkResult gjk_res = Gjk::intersect(box_a, box_b);
    assert(gjk_res.intersecting);

    EpaResult epa_res = Epa::expand(box_a, box_b, gjk_res.simplex);
    assert(epa_res.success);
    assert(is_nearly_equal(epa_res.penetration_depth, 0.5f, 1e-3f));
    assert(is_nearly_equal(epa_res.normal.x, 1.0f, 1e-3f));

    // Separated boxes
    Collider box_c = Collider::create_box(3, 3, Vec3(1, 1, 1), Transform(Vec3(5, 0, 0), Quat::identity()));
    GjkResult gjk_sep = Gjk::intersect(box_a, box_c);
    assert(!gjk_sep.intersecting);
}

void test_sat_and_gjk_cross_validation() {
    Collider box_a = Collider::create_box(1, 1, Vec3(1, 1, 1), Transform(Vec3(0, 0, 0), Quat::identity()));
    Collider box_b = Collider::create_box(2, 2, Vec3(1, 1, 1), Transform(Vec3(1.2f, 0.3f, 0), Quat::identity()));

    GjkResult gjk_res = Gjk::intersect(box_a, box_b);
    SatResult sat_res = Sat::test_collision(box_a, box_b);

    assert(gjk_res.intersecting == sat_res.colliding);
    assert(sat_res.colliding);

    EpaResult epa_res = Epa::expand(box_a, box_b, gjk_res.simplex);
    assert(epa_res.success);
    assert(is_nearly_equal(epa_res.penetration_depth, sat_res.penetration_depth, 1e-2f));
}

void test_contact_manifold_and_reduction() {
    ContactManifold manifold;
    manifold.collider_a_id = 1;
    manifold.collider_b_id = 2;
    manifold.normal = Vec3(1, 0, 0);

    for (int i = 0; i < 6; ++i) {
        Vec3 pos(static_cast<float>(i), static_cast<float>(i * 0.5f), 0.0f);
        manifold.add_point(ContactPoint(pos, pos, pos, 0.1f * (i + 1), static_cast<uint32_t>(i)));
    }

    assert(manifold.points.size() == 6);
    manifold.reduce_to_max_4();
    assert(manifold.points.size() == 4);
}

void test_voronoi_shard_collision() {
    FractureVolume box = FractureVolume::unit_box();
    std::vector<Vec3> sites = SiteGenerator::generate_3d(box, 4, 0.5f, 42);
    auto shards = Voronoi3D::compute_partition(box, sites);

    assert(shards.size() >= 2);

    Collider col1 = Collider::create_polyhedron(1, 1, shards[0].mesh, Transform(Vec3(0, 0, 0), Quat::identity()));
    Collider col2 = Collider::create_polyhedron(2, 2, shards[1].mesh, Transform(Vec3(0.05f, 0, 0), Quat::identity()));

    ContactManifold manifold = Narrowphase::collide(col1, col2);
    (void)manifold;
}

void test_broadphase_narrowphase_pipeline() {
    DynamicAabbTree tree;
    std::vector<Collider> colliders;

    Collider c1 = Collider::create_box(1, 1, Vec3(1, 1, 1), Transform(Vec3(0, 0, 0), Quat::identity()));
    Collider c2 = Collider::create_box(2, 2, Vec3(1, 1, 1), Transform(Vec3(1.5f, 0, 0), Quat::identity()));
    Collider c3 = Collider::create_box(3, 3, Vec3(1, 1, 1), Transform(Vec3(10, 10, 10), Quat::identity()));

    colliders.push_back(c1);
    colliders.push_back(c2);
    colliders.push_back(c3);

    for (const auto& c : colliders) {
        tree.insert_leaf(c.id, c.world_aabb);
    }

    std::vector<BroadphasePair> pairs;
    tree.generate_candidate_pairs(pairs);
    assert(pairs.size() == 1);

    size_t collisions_found = 0;
    for (const auto& pair : pairs) {
        ContactManifold manifold = Narrowphase::collide(colliders[pair.collider_a - 1], colliders[pair.collider_b - 1]);
        if (!manifold.points.empty()) {
            collisions_found++;
        }
    }

    assert(collisions_found == 1);
}

void test_collision_stress() {
    DynamicAabbTree tree;
    std::vector<Collider> colliders;
    size_t count = 60;

    for (size_t i = 0; i < count; ++i) {
        float x = static_cast<float>(i % 10) * 1.5f;
        float y = static_cast<float>(i / 10) * 1.5f;
        Collider c = Collider::create_box(static_cast<uint32_t>(i + 1), static_cast<uint32_t>(i + 1), Vec3(0.8f, 0.8f, 0.8f), Transform(Vec3(x, y, 0), Quat::identity()));
        colliders.push_back(c);
        tree.insert_leaf(c.id, c.world_aabb);
    }

    std::vector<BroadphasePair> pairs;
    tree.generate_candidate_pairs(pairs);
    assert(!pairs.empty());

    size_t collisions_found = 0;
    for (const auto& pair : pairs) {
        ContactManifold manifold = Narrowphase::collide(colliders[pair.collider_a - 1], colliders[pair.collider_b - 1]);
        if (!manifold.points.empty()) {
            collisions_found++;
        }
    }
    assert(collisions_found > 0);
}

int main() {
    std::cout << "Running Project 04 Milestone 3 Collision Detection Unit Tests...\n";

    test_aabb();
    test_dynamic_tree();
    test_support_mapping();
    test_gjk_epa_collision();
    test_sat_and_gjk_cross_validation();
    test_contact_manifold_and_reduction();
    test_voronoi_shard_collision();
    test_broadphase_narrowphase_pipeline();
    test_collision_stress();

    std::cout << "All Project 04 Milestone 3 Unit Tests Passed Successfully!\n";
    return 0;
}
