# Repository Guidelines

## Project Structure & Module Organization
- `code.cpp` and `code.h`: application entry point.
- `common/include` and `common/src`: shared utilities (camera, grids, config).
- `fluid2d/Lagrangian`, `fluid2d/Eulerian`, `fluid3d/Lagrangian`, `fluid3d/Eulerian`: simulation modules, each with `include/` and `src/`.
- `ui/include` and `ui/src`: ImGui-based UI and renderer; links simulation libraries.
- `resources/shaders` and `resources/pictures`: runtime assets.
- `third_party/`: vendored dependencies (glad, glm, imgui, glfw, boost); avoid local edits unless updating vendors.
- `out/`: CMake build and install outputs.

## Build, Test, and Development Commands
- Configure (matches `CMakeSettings.json`):
  `cmake -S . -B out/build/x64-Release -G "Ninja"`
- Build:
  `cmake --build out/build/x64-Release --config RelWithDebInfo`
- Run:
  `out/build/x64-Release/FluidSimulationSystem.exe`
- Tests (only if added):
  `ctest --test-dir out/build/x64-Release`

## Coding Style & Naming Conventions
- Language is C++ with CMake; headers are `.h` or `.hpp`, sources are `.cpp`.
- Indentation: follow the existing file style (tabs and Allman braces are common).
- Naming: types and files use PascalCase (e.g., `Camera.h`), namespaces and modules use lowerCamel or lowercase (e.g., `Eulerian2dPara`, `lagrangian2d`), and functions/variables use lowerCamel (e.g., `imageWidth`, `run`).
- No repo-wide formatter or linter is configured; keep style consistent with nearby code.

## Testing Guidelines
- No automated tests are configured in CMake today. If you add tests, register them with CTest and place sources under a `tests/` folder or a module-local `tests/` directory.
- Use descriptive test names that include the scheme and dimension, for example `Fluid2dEulerian_Advection`.

## Commit & Pull Request Guidelines
- Recent commits are short, lowercase, and imperative (for example `fix encode error`, `update doc/intro.md`). Keep messages concise and specific.
- PRs should describe the simulation module touched, link related issues, and include screenshots or GIFs when UI or rendering changes.
