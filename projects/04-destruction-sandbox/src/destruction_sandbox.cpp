#include "destruction/render/sandbox_app.hpp"
#include <iostream>
#include <string>

using namespace destruction::render;

int main(int argc, char** argv) {
    bool headless = false;
    int max_frames = -1;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--headless") {
            headless = true;
        } else if (arg == "--timeout" && i + 1 < argc) {
            max_frames = std::stoi(argv[++i]);
        }
    }

    std::cout << "========================================================\n";
    std::cout << "Project 04 — Procedural Destruction Sandbox\n";
    std::cout << "========================================================\n";
    std::cout << "Controls:\n";
    std::cout << "  W / A / S / D     : Move camera forward / left / back / right\n";
    std::cout << "  Left Shift        : Move faster\n";
    std::cout << "  Q / E (or C/Space): Move camera down / up\n";
    std::cout << "  Mouse Look        : Orient camera pitch and yaw\n";
    std::cout << "  Left Click        : Launch high-velocity projectile\n";
    std::cout << "  F                 : Trigger procedural Voronoi fracture on tower\n";
    std::cout << "  P                 : Pause / Unpause physical simulation\n";
    std::cout << "  O                 : Single-step advance physical simulation\n";
    std::cout << "  R                 : Reset scene to deterministic initial state\n";
    std::cout << "  1                 : Toggle Wireframe overlay\n";
    std::cout << "  2                 : Toggle AABBs overlay\n";
    std::cout << "  3                 : Toggle Contact points and normals\n";
    std::cout << "  4                 : Toggle Structural support graph edges\n";
    std::cout << "  5                 : Toggle Shard centers of mass\n";
    std::cout << "  Escape            : Exit application\n";
    std::cout << "========================================================\n";

    SandboxApp app;
    app.set_headless(headless, max_frames);

    if (!app.init()) {
        std::cerr << "[Error] Failed to initialize Destruction Sandbox Application.\n";
        return 1;
    }

    app.run();

    std::cout << "[Shutdown] Destruction Sandbox exited cleanly.\n";
    return 0;
}
