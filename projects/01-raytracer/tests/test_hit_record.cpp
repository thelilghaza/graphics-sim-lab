#include "raylab/hit_record.hpp"
#include "test_harness.hpp"

int main() {
    using namespace raylab;

    constexpr double eps = 1e-7;

    HitRecord rec;
    Ray r_incoming(Point3(0, 0, 0), Vec3(0, 0, -1));
    Vec3 outward_normal(0, 0, 1);

    // Incoming ray opposes outward normal => front face hit
    rec.set_face_normal(r_incoming, outward_normal);
    TEST_ASSERT(rec.front_face);
    TEST_ASSERT_NEAR(rec.normal.z(), 1.0, eps);

    // Ray pointing in same direction as outward normal => back face hit
    Ray r_outgoing(Point3(0, 0, 0), Vec3(0, 0, 1));
    rec.set_face_normal(r_outgoing, outward_normal);
    TEST_ASSERT(!rec.front_face);
    TEST_ASSERT_NEAR(rec.normal.z(), -1.0, eps); // Inverted normal

    return raylab::test::g_failures == 0 ? 0 : 1;
}
