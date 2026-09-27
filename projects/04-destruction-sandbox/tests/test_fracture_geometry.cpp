#include "destruction/math/math_utils.hpp"
#include "destruction/math/vec2.hpp"
#include "destruction/math/vec3.hpp"
#include "destruction/fracture/plane.hpp"
#include "destruction/fracture/polygon2d.hpp"
#include "destruction/fracture/voronoi2d.hpp"
#include "destruction/fracture/polyhedron.hpp"
#include "destruction/fracture/clipper3d.hpp"
#include "destruction/fracture/fracture_volume.hpp"
#include "destruction/fracture/site_generator.hpp"
#include "destruction/fracture/shard.hpp"
#include "destruction/fracture/mesh_validator.hpp"
#include "destruction/fracture/voronoi3d.hpp"

#include <iostream>
#include <cmath>
#include <cassert>

using namespace destruction::math;
using namespace destruction::fracture;

void test_2d_polygon_and_clipping() {
    Polygon2D rect = Polygon2D::create_rectangle(-1.0f, -1.0f, 1.0f, 1.0f);
    assert(is_nearly_equal(rect.area(), 4.0f));
    assert(rect.centroid() == Vec2(0.0f, 0.0f));

    // Clip rect at x = 0 (n = (1, 0), d = 0 -> x <= 0)
    Polygon2D left = rect.clip_by_line(Vec2(1.0f, 0.0f), 0.0f);
    assert(is_nearly_equal(left.area(), 2.0f));
    assert(is_nearly_equal(left.centroid().x, -0.5f));
}

void test_2d_voronoi_conservation() {
    std::vector<Vec2> sites = SiteGenerator::generate_2d(-2.0f, -2.0f, 2.0f, 2.0f, 8, 0.5f, 123);
    assert(sites.size() == 8);

    auto cells = Voronoi2D::compute_partition(-2.0f, -2.0f, 2.0f, 2.0f, sites);
    float sum_area = 0.0f;
    for (const auto& cell : cells) {
        assert(cell.is_valid);
        assert(cell.area > 0.0f);
        sum_area += cell.area;
    }

    float source_area = 16.0f; // [-2, 2] x [-2, 2]
    assert(is_nearly_equal(sum_area, source_area, 1e-3f));
    (void)source_area;
}

void test_3d_box_polyhedron() {
    FractureVolume unit_box = FractureVolume::unit_box();
    ConvexPolyhedron poly = unit_box.to_polyhedron();

    assert(is_nearly_equal(poly.volume(), 8.0f));
    assert(poly.centroid() == Vec3(0.0f, 0.0f, 0.0f));

    auto val = MeshValidator::validate(poly);
    assert(val.is_valid);
    assert(val.is_closed_manifold);
}

void test_3d_plane_clipping() {
    FractureVolume unit_box = FractureVolume::unit_box();
    ConvexPolyhedron poly = unit_box.to_polyhedron();

    // Plane x = 0 (n = (1, 0, 0), d = 0) -> x <= 0
    Plane plane_x(Vec3(1.0f, 0.0f, 0.0f), 0.0f);
    ConvexPolyhedron left = Clipper3D::clip_halfspace(poly, plane_x);
    ConvexPolyhedron right = Clipper3D::clip_halfspace(poly, plane_x.flipped());

    assert(is_nearly_equal(left.volume(), 4.0f, 1e-4f));
    assert(is_nearly_equal(right.volume(), 4.0f, 1e-4f));

    auto val_left = MeshValidator::validate(left);
    auto val_right = MeshValidator::validate(right);

    assert(val_left.is_valid && val_left.is_closed_manifold);
    assert(val_right.is_valid && val_right.is_closed_manifold);
}

void test_3d_single_site_voronoi() {
    FractureVolume box(Vec3(-1, -1, -1), Vec3(1, 1, 1), 2.5f);
    std::vector<Vec3> sites = { Vec3(0.1f, -0.2f, 0.3f) };

    auto shards = Voronoi3D::compute_partition(box, sites);
    assert(shards.size() == 1);
    assert(shards[0].is_valid);
    assert(is_nearly_equal(shards[0].volume, 8.0f, 1e-4f));
    assert(is_nearly_equal(shards[0].mass, 20.0f, 1e-4f)); // 8.0 * 2.5
}

void test_3d_symmetric_two_site() {
    FractureVolume box = FractureVolume::unit_box();
    std::vector<Vec3> sites = { Vec3(-0.5f, 0.0f, 0.0f), Vec3(0.5f, 0.0f, 0.0f) };

    auto shards = Voronoi3D::compute_partition(box, sites);
    assert(shards.size() == 2);
    assert(is_nearly_equal(shards[0].volume, 4.0f, 1e-3f));
    assert(is_nearly_equal(shards[1].volume, 4.0f, 1e-3f));
    assert(is_nearly_equal(shards[0].volume + shards[1].volume, 8.0f, 1e-3f));
}

void test_3d_eight_symmetric_corners() {
    FractureVolume box = FractureVolume::unit_box();
    std::vector<Vec3> sites = {
        Vec3(-0.5f, -0.5f, -0.5f), Vec3(0.5f, -0.5f, -0.5f),
        Vec3(-0.5f,  0.5f, -0.5f), Vec3(0.5f,  0.5f, -0.5f),
        Vec3(-0.5f, -0.5f,  0.5f), Vec3(0.5f, -0.5f,  0.5f),
        Vec3(-0.5f,  0.5f,  0.5f), Vec3(0.5f,  0.5f,  0.5f)
    };

    auto shards = Voronoi3D::compute_partition(box, sites);
    assert(shards.size() == 8);
    float sum_vol = 0.0f;
    for (const auto& shard : shards) {
        assert(shard.is_valid);
        assert(is_nearly_equal(shard.volume, 1.0f, 1e-2f));
        sum_vol += shard.volume;
    }
    assert(is_nearly_equal(sum_vol, 8.0f, 1e-3f));
}

void test_3d_translated_box() {
    FractureVolume box(Vec3(10.0f, 20.0f, 30.0f), Vec3(12.0f, 22.0f, 32.0f), 1.0f);
    assert(is_nearly_equal(box.volume(), 8.0f));
    assert(box.centroid() == Vec3(11.0f, 21.0f, 31.0f));

    std::vector<Vec3> sites = SiteGenerator::generate_3d(box, 6, 0.4f, 999);
    auto shards = Voronoi3D::compute_partition(box, sites);

    auto summary = Voronoi3D::evaluate_summary(box, shards);
    assert(summary.valid_shard_count == 6);
    assert(summary.relative_volume_error < 0.01f);
}

void test_mass_and_volume_conservation() {
    FractureVolume box(Vec3(-2.0f, -2.0f, -2.0f), Vec3(2.0f, 2.0f, 2.0f), 3.0f); // Volume = 64 m^3, Mass = 192 kg
    std::vector<Vec3> sites = SiteGenerator::generate_3d(box, 15, 0.6f, 42);
    assert(sites.size() == 15);

    auto shards = Voronoi3D::compute_partition(box, sites);
    auto summary = Voronoi3D::evaluate_summary(box, shards);

    assert(summary.valid_shard_count == 15);
    assert(summary.relative_volume_error < 0.01f); // < 1% error
    assert(summary.relative_mass_error < 0.01f);
}

void test_mesh_manifold_validation() {
    FractureVolume box = FractureVolume::unit_box();
    std::vector<Vec3> sites = SiteGenerator::generate_3d(box, 8, 0.5f, 555);
    auto shards = Voronoi3D::compute_partition(box, sites);

    for (const auto& shard : shards) {
        assert(shard.is_valid);
        auto val = MeshValidator::validate(shard.mesh);
        assert(val.is_valid);
        assert(val.is_closed_manifold);
        assert(val.boundary_edge_count == 0);
        assert(!val.has_duplicate_vertices);
    }
}

void test_deterministic_reproducibility() {
    FractureVolume box = FractureVolume::unit_box();
    std::vector<Vec3> sites1 = SiteGenerator::generate_3d(box, 10, 0.4f, 777);
    std::vector<Vec3> sites2 = SiteGenerator::generate_3d(box, 10, 0.4f, 777);

    assert(sites1.size() == sites2.size());
    for (size_t i = 0; i < sites1.size(); ++i) {
        assert(sites1[i] == sites2[i]);
    }

    auto shards1 = Voronoi3D::compute_partition(box, sites1);
    auto shards2 = Voronoi3D::compute_partition(box, sites2);

    assert(shards1.size() == shards2.size());
    for (size_t i = 0; i < shards1.size(); ++i) {
        assert(shards1[i].volume == shards2[i].volume);
        assert(shards1[i].centroid == shards2[i].centroid);
        assert(shards1[i].mesh.vertices.size() == shards2[i].mesh.vertices.size());
    }
}

int main() {
    std::cout << "Running Project 04 Milestone 2 Fracture Geometry Unit Tests...\n";

    test_2d_polygon_and_clipping();
    test_2d_voronoi_conservation();
    test_3d_box_polyhedron();
    test_3d_plane_clipping();
    test_3d_single_site_voronoi();
    test_3d_symmetric_two_site();
    test_3d_eight_symmetric_corners();
    test_3d_translated_box();
    test_mass_and_volume_conservation();
    test_mesh_manifold_validation();
    test_deterministic_reproducibility();

    std::cout << "All Project 04 Milestone 2 Unit Tests Passed Successfully!\n";
    return 0;
}
