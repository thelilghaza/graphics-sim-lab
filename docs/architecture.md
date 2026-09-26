# Architectural Vision & Repository Design

This document details the architectural boundaries and guidelines governing the **Graphics & Simulation Lab**.

---

## 1. Project Isolation Principle

```text
graphics-sim-lab/
├── projects/
│   ├── 01-raytracer/        <-- Fully self-contained
│   ├── 02-voxel-engine/     <-- Fully self-contained
│   └── 03-performance-lab/  <-- Fully self-contained
└── libs/                    <-- Extracted code ONLY after proven reuse
```

* **No Monolithic Engines**: Each project inside `projects/` MUST be independently buildable and understandable without depending on other projects.
* **No Speculative Frameworks**: Do not construct abstractions or "engine foundations" for future projects before their concrete need arises.

---

## 2. Delayed Shared Library Extraction Rule

Shared code in `libs/` is strictly prohibited from being created upfront.

Code may only be moved into `libs/` when:
1. It has been implemented, tested, and used in **at least two independent projects**.
2. The implementation has stabilized and demonstrated clear design reuse.
3. Decoupling the component into `libs/` simplifies both consumer projects without adding hidden dependencies.

---

## 3. Technology & Language Standards

* **Native Language Baseline**: Modern **C++20** standard for native projects.
* **Build System**: **CMake** (v3.20+) with `CMakePresets.json`.
* **Testing**: Standard **CTest** integration.
* **Dependencies**: Minimize or eliminate third-party libraries. Prefer standard library solutions unless performance or platform constraints dictate otherwise.
