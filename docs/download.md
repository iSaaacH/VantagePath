# Download

The latest release is available from
[GitHub Releases](https://github.com/iSaaacH/VantagePath/releases/latest).

## Clone the source

```bash
git clone --branch v0.2.0 --depth 1 \
  https://github.com/iSaaacH/VantagePath.git
cd VantagePath
```

## Add it as a submodule

Run this from your robot project root:

```bash
git submodule add https://github.com/iSaaacH/VantagePath.git vendor/VantagePath
git -C vendor/VantagePath checkout v0.2.0
git add .gitmodules vendor/VantagePath
git commit -m "Add VantagePath v0.2.0"
```

When somebody clones the parent project, they should use:

```bash
git clone --recurse-submodules https://github.com/YOUR-TEAM/YOUR-ROBOT.git
```

For an existing clone whose submodule directory is empty:

```bash
git submodule update --init --recursive
```

Continue to [1 - Getting Started](tutorials/1-getting-started.md) for complete
CMake and PROS setup instructions.
