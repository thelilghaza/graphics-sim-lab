#include "raylab/scene_builder.hpp"
#include "test_harness.hpp"

int main() {
    using namespace raylab;

    // 1. SceneBuilder fluent interface check
    SceneBuilder builder;
    builder.add_diffuse(Point3(0, 0, -1), 0.5, Color(0.8, 0.2, 0.2))
           .add_metal(Point3(1, 0, -1), 0.5, Color(0.9, 0.9, 0.9), 0.0)
           .add_dielectric(Point3(-1, 0, -1), 0.5, 1.5)
           .set_light(Point3(2, 4, 1), Color(1, 1, 1), 1.0);

    SceneData scene = builder.build();
    TEST_ASSERT(scene.world.objects.size() == 3);
    TEST_ASSERT_NEAR(scene.light.position().x(), 2.0, 1e-6);

    // 2. Procedural random scene determinism check
    SceneData proc1 = create_procedural_random_scene(42);
    SceneData proc2 = create_procedural_random_scene(42);
    SceneData proc3 = create_procedural_random_scene(999);

    TEST_ASSERT(proc1.world.objects.size() == proc2.world.objects.size());
    TEST_ASSERT(proc1.world.objects.size() > 10); // Generates 50+ spheres

    // Compare first procedural sphere positions
    Sphere* s1 = dynamic_cast<Sphere*>(proc1.world.objects[4].get());
    Sphere* s2 = dynamic_cast<Sphere*>(proc2.world.objects[4].get());
    Sphere* s3 = dynamic_cast<Sphere*>(proc3.world.objects[4].get());

    TEST_ASSERT(s1 != nullptr && s2 != nullptr && s3 != nullptr);
    TEST_ASSERT_NEAR(s1->center().x(), s2->center().x(), 1e-7);
    TEST_ASSERT(s1->center().x() != s3->center().x()); // Different seed produces different positions

    return raylab::test::g_failures == 0 ? 0 : 1;
}
