---
name: gz-build
description: Configure and build gz-sim from source using either CMake/Ninja directly or a colcon workspace. Trigger when the user asks to build, recompile, configure, or set up the gz-sim binary tree.
---

# Building gz-sim

`gz-sim` follows the Gazebo project's standard build process. Two equally
valid front-ends exist; pick based on what the user already has set up.

## 1. Direct CMake (single repo)

Assumes every Gazebo dependency (`gz-cmake`, `gz-common`, `gz-physics`, …)
is already installed system-wide.

```sh
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_TESTING=ON
cmake --build build -j"$(nproc)"
```

Useful flags:

| Flag | Effect |
|------|--------|
| `-DCMAKE_BUILD_TYPE=Debug` | Asserts on, no optimization, full symbols. |
| `-DENABLE_PROFILER=ON` | Compiles in `GZ_PROFILER` hooks. |
| `-DBUILD_TESTING=OFF` | Skip test targets (faster, smaller). |
| `-DCMAKE_INSTALL_PREFIX=/opt/gz` | Override install location. |
| `-DBUILD_DOCS=ON` | Build Doxygen + tutorials (needs `doxygen`). |

Install (optional, into prefix):

```sh
cmake --install build
```

## 2. colcon workspace (Gazebo source build)

When the user is building the full Gazebo stack:

```sh
# from <workspace>/src/ that contains gz-sim and friends
colcon build --merge-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

Limit to gz-sim and its deps:

```sh
colcon build --merge-install \
  --packages-up-to gz-sim \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## Bazel (alternative)

```sh
bazel build //... --config=linux
bazel test  //... --config=linux
```

`MODULE.bazel` pins the dependencies; the matrix is exercised by
`.github/workflows/bazel.yml`.

## After building

* Run the smoke test: `gz sim -v 4 shapes.sdf` (needs `source install/setup.bash`
  when built via colcon).
* For headless CI, prefix with `xvfb-run -s '-screen 0 1280x800x24'`.
* For sanitizers, configure with `-DCMAKE_CXX_FLAGS="-fsanitize=address,undefined"`
  and `-DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"`.

## Common failures

* `find_package(gz-cmake REQUIRED)` failing → user hasn't installed gz-cmake.
* Linker errors about Qt6 → ensure `qt6-base-dev` + `qt6-declarative-dev`
  + `qt6-5compat-dev` are present.
* Missing `sdformat` → install the matching `libsdformat<NN>-dev` for the
  current Gazebo distribution (`Rotary` → check `package.xml`).
* Out-of-date `gz-msgs` headers → rebuild `gz-msgs` before `gz-sim`.

Always cd into the repo root (`~/gz-sim`) before running these.
