# ArcCollision.Wrapper

`ArcCollision.Wrapper` mirrors the public API of `ArcCollision.Ref` in the
`ArcCollision.Wrapper` namespace and forwards collision work to the native C++
library through its C ABI.

High-frequency blittable scratch storage (World batch-shape conversion and the
standalone broadphase build/query buffers) is held in reusable unmanaged buffers.
After capacity warmup these paths do not create temporary managed arrays; public
`List<T>` results and Polygon vertex storage remain managed by design.

Switch implementations by changing the namespace import:

```csharp
// using ArcCollision;
using ArcCollision.Wrapper;
```

Place the platform library in the application output or package it under the
standard RID native-asset path:

- `runtimes/win-x64/native/arccollision.dll`
- `runtimes/linux-x64/native/libarccollision.so`
- `runtimes/osx-arm64/native/libarccollision.dylib`
- Android: `lib/arm64-v8a/libarccollision.so`
- iOS: link `libarccollision.a` into the application; Wrapper resolves symbols
  from the main program export table.

For local development, `ARCCOLLISION_NATIVE_PATH` may point directly to the
native library.
