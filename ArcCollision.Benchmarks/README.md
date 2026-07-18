# ArcCollision static + dynamic benchmark

This console benchmark compares `ArcCollision.Ref` and `ArcCollision.Wrapper`
on the same mixed static/dynamic world. It runs entirely on the calling thread:
there is no `Task`, `Parallel`, worker pool, or backend concurrency.

The scene generator uses a repository-owned XorShift64* implementation and
creates every coordinate directly on the 24.8 fixed grid. The seed, static
shapes, dynamic initial shapes, and every per-frame dynamic update are therefore
reproducible. Shape generation and conversion are completed before timing.

Each measured frame performs:

1. Update every dynamic collider.
2. Compute dynamic-dynamic and dynamic-static candidate pairs.
3. Run narrowphase contact generation for every candidate.
4. Hash pair entity ids and complete manifold values.

The benchmark refuses to report timings if candidate counts, collision counts,
or hashes differ between backends.

Measured trials use multiple warmups and report medians plus relative
interquartile ranges. Query timings automatically repeat the same deterministic
operation inside each sample until the configured minimum sample duration is
reached, avoiding timer and scheduler noise for small query batches. Use
`--sample-ms` to change that duration; `--quick` intentionally uses a shorter
duration for smoke testing. The main trial summary reports managed bytes allocated
per trial, and every query row reports managed bytes allocated per query for the
Ref-loop, Native-loop, and Native-batch paths.

Native-batch selects among a small scalar loop, input-order SIMD packets for
sparse work, and Morton-sorted SIMD packets for dense work. The choice is based
on a fixed deterministic candidate-density probe and never changes result order.

Build the native Release library first, then run:

```sh
dotnet run --project ArcCollision.Benchmarks -c Release
```

Use `--help` for all parameters. A fast smoke run is available with `--quick`.
The default seed is printed along with a complete reproduction command.
