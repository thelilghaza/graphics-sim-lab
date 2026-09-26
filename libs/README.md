# Shared Libraries (`libs/`)

## Policy on Shared Code

To maintain strict project independence and prevent premature monolithic framework design:

1. **NO upfront shared code**: This directory is intentionally empty at repository foundation.
2. **Proven Reuse Rule**: Code is extracted into `libs/` **only after** it has been built, tested, and actively reused across at least two separate projects in `projects/`.
3. Each extracted library must have its own clean interface, tests, and documentation.
