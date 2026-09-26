# Benchmarking Guidelines & Protocol

To uphold the principle **"Measure before optimizing"**, all benchmark records within this laboratory must follow a standardized format.

---

## Benchmark Reporting Protocol

When submitting or documenting benchmark results in any project, the following hardware and environment context MUST be recorded:

1. **System Metadata**:
   - **CPU**: Model name, architecture, core count, thread count.
   - **GPU**: (if applicable) GPU model, driver version.
   - **Operating System**: OS distribution and version.
   - **Compiler & Build Type**: Compiler version (e.g., MSVC 19.51, GCC 12.2, Clang 15), build configuration (`Release` with optimizations enabled).

2. **Benchmark Execution Context**:
   - **Workload Specifications**: Resolution, iteration count, sample depth, scene complexity.
   - **Threading Configuration**: Thread count, execution policies.
   - **Timing Metrics**: Min, max, mean execution times, variance/standard deviation where available.

---

## Rules of Benchmarking

- **No Fabricated Data**: Only record real measurements produced by execution of benchmark binaries.
- **Release Mode Mandatory**: Benchmarks must always be executed in optimized `Release` builds.
- **Cold vs Warm Runs**: Discard initial warm-up runs when measuring steady-state performance.
- **Reproducibility**: Provide exact CLI invocation parameters so benchmarks can be executed independently.
