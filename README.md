# Graphics & Simulation Lab

Welcome to the **Graphics & Simulation Lab**, a personal technical laboratory and public portfolio dedicated to low-level computer graphics, rendering systems, voxel representations, dynamic simulation, GPU programming, and developer tooling.

---

## 🎯 Vision & Core Philosophy

The primary objective of this repository is to build deep, first-principles engineering projects with rigorous code quality, measurements, reproducibility, and architectural clarity.

### Core Architectural Principle
> **DO NOT build a giant shared engine/framework upfront.**

Each project in this repository is designed to be **independently understandable and buildable**. Shared libraries (`libs/`) are extracted only after there is proven reuse across multiple working projects.

---

## 🗺️ Project Roadmap

| # | Project | Planned Scope & Focus | Status |
|---|---|---|---|
| **01** | [CPU Ray Tracer](projects/01-raytracer/) | First-principles ray tracing: spheres, camera, materials, reflections, refractions, BVH acceleration, multithreading. | 🚧 Phase 0 Complete |
| **02** | [Voxel Engine](projects/02-voxel-engine/) | Chunked voxel storage, greedy meshing, procedural terrain generation, fast ray casting. | 📋 Planned |
| **03** | [Performance Lab](projects/03-performance-lab/) | CPU/GPU profiling, cache locality, SIMD vectorization, memory access pattern benchmarks. | 📋 Planned |
| **04** | [Procedural Destruction Sandbox](projects/04-destruction-sandbox/) | Voronoi fracturing, rigid body dynamic simulation, impulse solvers, structural connectivity. | 📋 Planned |
| **05** | [GPU Crater Simulator](projects/05-crater-simulator/) | Compute shaders, heightmap deformation, impact energy distribution, particle ejecta. | 📋 Planned |
| **06** | [Tiny Game Engine](projects/06-tiny-engine/) | Minimalist 3D render pipeline, scene graph, entity component system, input handling. | 📋 Planned |
| **07** | [WebGPU 3D Engine](projects/07-webgpu-engine/) | Modern web-native graphics pipeline, WGSL shaders, PBR rendering, glTF loading. | 📋 Planned |
| **08** | [Godot Project Analyzer](projects/08-godot-analyzer/) | Static analysis, asset dependency graphs, performance diagnostics, developer tooling. | 📋 Planned |

---

## 🛠️ Build & Quick Start

### Prerequisites
- **C++ Compiler**: Modern C++20 compliant compiler (MSVC 2022+, GCC 11+, or Clang 13+)
- **Build System**: [CMake](https://cmake.org/) (v3.20+) and [Ninja](https://ninja-build.org/)

### Building via CMake Presets

```bash
# Configure the default debug preset
cmake --preset default

# Build all available targets
cmake --build --preset default

# Run test suite via CTest
ctest --preset default
```

### Manual Configuration

```bash
# Configure out-of-tree build
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug

# Compile
cmake --build build

# Test
ctest --test-dir build --output-on-failure
```

---

## 📑 Repository Structure

```text
graphics-sim-lab/
├── CMakeLists.txt              # Root build configuration
├── CMakePresets.json           # Standardized build presets
├── README.md                   # Repository overview (this file)
├── LICENSE                     # MIT License
├── docs/                       # Technical documentation & principles
│   ├── roadmap.md              # Detailed project roadmap
│   ├── architecture.md         # Repository architectural rules
│   ├── engineering-principles.md # Core engineering principles
│   ├── benchmarking.md         # Benchmarking protocols
│   └── lab-notes/              # Ongoing experimental notes
├── projects/                   # Independent engineering projects
│   ├── 01-raytracer/           # CPU Ray Tracer
│   └── ...                     # Projects 02 - 08
├── libs/                       # Shared components (extracted only on proven reuse)
└── tools/                      # Benchmark & build scripts
```

---

## 📜 License

This project is licensed under the [MIT License](LICENSE).
