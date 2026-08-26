# 1 - Getting Started

This tutorial installs VantagePath, runs its tests, and adds it to a project.
You need Git, CMake 3.16 or newer, and a C++17 compiler.

## Test VantagePath on your computer

Open a terminal and run each command:

```bash
git clone --branch v0.1.1 --depth 1 \
  https://github.com/iSaaacH/VantagePath.git
cd VantagePath
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The last command should finish with `100% tests passed`.

## Install in a CMake project

From your project root:

```bash
git submodule add https://github.com/iSaaacH/VantagePath.git vendor/VantagePath
git -C vendor/VantagePath checkout v0.1.1
```

Add these lines to your `CMakeLists.txt`:

```cmake
add_subdirectory(vendor/VantagePath)
target_link_libraries(your_robot_target PRIVATE VantagePath::VantagePath)
```

Replace `your_robot_target` with the target passed to your `add_executable` or
`add_library` command. Then configure and build your project:

```bash
cmake -S . -B build
cmake --build build --parallel
```

## Install in a PROS project

Open the PROS integrated terminal at the root of an existing project.

```bash
git submodule add \
  https://github.com/iSaaacH/VantagePath.git src/vendor/VantagePath
git -C src/vendor/VantagePath checkout v0.1.1
```

Add the following to the user-configurable section of `Makefile`, before
`include ./common.mk`:

```make
EXTRA_INCDIR+=src/vendor/VantagePath/include
EXCLUDE_SRCDIRS+=./src/vendor/VantagePath/tests
```

PROS recursively compiles the `.cpp` files under `src/`. The exclusion prevents
the native test program— which has its own `main()`—from entering robot
firmware.

Now build:

```bash
pros make
```

If your PROS installation exposes `make` directly, this is equivalent:

```bash
make
```

## Include the API

Add this to code that uses the library:

```cpp
#include <vantage/vantage.hpp>
```

!!! important

    VantagePath does not construct motors, start an RTOS task, or choose units.
    Your robot project owns those details and calls the library at a fixed
    period. The next tutorial builds that configuration.

You are now ready for [2 - Configuration](2-configuration.md).
