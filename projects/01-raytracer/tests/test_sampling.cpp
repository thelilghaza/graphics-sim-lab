#include "raylab/rng.hpp"
#include "test_harness.hpp"

int main() {
    using namespace raylab;

    constexpr double eps = 1e-9;

    // 1. SplitMix64 seed mixer determinism test
    uint64_t s1 = make_sample_seed(42, 100, 200, 3);
    uint64_t s2 = make_sample_seed(42, 100, 200, 3);
    TEST_ASSERT(s1 == s2);

    // 2. Order independence: seed generated for (x, y, sample) is independent of traversal order
    uint64_t s_other_pixel = make_sample_seed(42, 101, 200, 3);
    TEST_ASSERT(s1 != s_other_pixel);

    uint64_t s_other_sample = make_sample_seed(42, 100, 200, 4);
    TEST_ASSERT(s1 != s_other_sample);

    uint64_t s_other_global = make_sample_seed(999, 100, 200, 3);
    TEST_ASSERT(s1 != s_other_global);

    // 3. RNG sequence matching for identical sample seeds
    RNG rng1(s1);
    RNG rng2(s2);
    for (int i = 0; i < 10; ++i) {
        TEST_ASSERT_NEAR(rng1.next_double(), rng2.next_double(), eps);
    }

    return raylab::test::g_failures == 0 ? 0 : 1;
}
