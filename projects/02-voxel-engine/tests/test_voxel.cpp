#include "voxel_lab/voxel.hpp"
#include <cassert>
#include <iostream>

void test_voxel_size_and_equality() {
    static_assert(sizeof(voxel_lab::Voxel) == 2, "Voxel must be 2 bytes");
    assert(sizeof(voxel_lab::Voxel) == 2);

    voxel_lab::Voxel v1(1, 4);
    voxel_lab::Voxel v2(1, 4);
    voxel_lab::Voxel v3(2, 4);

    assert(v1 == v2);
    assert(v1 != v3);

    std::cout << "[PASS] test_voxel_size_and_equality\n";
}

int main() {
    test_voxel_size_and_equality();
    std::cout << "All Voxel payload tests passed successfully!\n";
    return 0;
}
