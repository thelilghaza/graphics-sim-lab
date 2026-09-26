# Engineering Principles

All development in the Graphics & Simulation Lab adheres to these baseline principles:

1. **Prefer understanding over abstraction.**
   Build from first principles to understand how things work under the hood before wrapping them in high-level interfaces.

2. **Measure before optimizing.**
   Never optimize based on intuition alone. Every performance claim must be verified with reproducible micro-benchmarks or profiler output.

3. **Keep projects independently buildable.**
   Every project must compile and run as a standalone unit without implicit dependencies on sibling projects.

4. **Avoid premature shared libraries.**
   Only extract code into `libs/` after it has proven utility across multiple working projects.

5. **Prefer simple dependencies.**
   Rely primarily on the C++ Standard Library. Avoid bringing in complex dependency trees unless there is a strong, concrete justification.

6. **Document important engineering decisions.**
   Record trade-offs, architecture choices, lessons learned, and failure modes in project documentation and lab notes.

7. **Every substantial project must have tests.**
   Maintain automated behavior-driven tests via CTest for core logic, geometry algorithms, and math components.

8. **Every performance claim must have a reproducible benchmark.**
   Document CPU, compiler settings, dataset parameters, sample counts, and timing data. Do not fabricate benchmark numbers.

9. **Generated build artifacts must never be committed.**
   Keep binaries, object files, generated build directories, and intermediate renders strictly out of Git.

10. **Client/proprietary material must never be placed into this public repository.**
    All code, assets, and documentation must be original, open, or appropriately licensed for public distribution.

11. **Maintain professional plain-text documentation style.**
    All repository documentation, commit messages, code comments, benchmark reports, and generated technical text must use clean, professional plain text without emojis.
