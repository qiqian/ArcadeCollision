# ArcCollision native library

`arccollision.h` is the only public C header. The implementation is C++17, but
every exported symbol uses a stable C ABI and never exposes STL types or native
allocator ownership.

Build desktop targets with CMake presets, for example:

```sh
cmake --preset windows-x64
cmake --build --preset windows-x64
```

Available presets are `windows-x64`, `windows-x64-static`, `linux-x64`,
`macos-universal`, `android-arm64`, and `ios-arm64`. The two Windows presets
also have matching test presets. Android requires `ANDROID_NDK_HOME`. iOS is
built as a static library and linked into the application; exported entry
points retain default visibility for the main-program export table
(`__Internal` semantics). Other platforms load `arccollision.dll`,
`libarccollision.so`, or `libarccollision.dylib`.

Opaque `arc_polygon` and `arc_world` objects are reference-owned through the
functions in the header. World pair/query/cast-all APIs return borrowed views of
world-owned result buffers so each operation crosses the C ABI only once. The
view is invalidated by the next call on that world and must not be modified or
freed. Standalone broadphase and polygon batch APIs retain caller-provided
buffers and report the required count with `ARC_STATUS_BUFFER_TOO_SMALL`.

The implementation is split by reference module (`fixed`, `shapes`, `distance`,
`collide`, `sweep`, dynamic/static broadphase, spatial hash, and world). All
authoritative geometry uses integer fixed point; floating-point values only
cross the C ABI boundary.
