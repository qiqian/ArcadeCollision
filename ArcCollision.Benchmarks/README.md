# ArcCollision static + dynamic benchmark

This console benchmark compares `ArcCollision.Ref` and `ArcCollision.Wrapper`
on the same mixed static/dynamic world. It runs entirely on the calling thread:
there is no `Task`, `Parallel`, worker pool, or backend concurrency.

To keep results comparable across runs, only the calling benchmark thread is
pinned to one logical CPU (default: logical CPU 2, avoiding interrupt-heavy core
0), so samples do not drift between cores with different cache and boost states.
The process itself is not pinned: GC, finalizer, and tiered-JIT workers remain
free to run on other CPUs and cannot be forced to contend with the measured
thread. Only the benchmark thread is raised to the highest thread priority; the
process and its helper threads retain their default priority. Use `--cpu <index>`
to choose the core and `--cpu -1` to disable pinning.

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

Release benchmarks use the normal .NET tiered JIT/PGO configuration. The full
benchmark's warmup is intended for performance comparisons; `--quick` is only a
functional smoke run and its managed timings may still include lower JIT tiers.
Debug timings are not comparable and the executable prints a warning in Debug.

Native-batch selects among a small scalar loop, input-order SIMD packets for
light batches whose groups of four are spatially bundled, and Morton-sorted
SIMD packets for dense work. Scattered light batches stay scalar: unrelated
packet lanes share no subtree, so the 4-wide union traversal costs more than
four scalar descents. The choice is based on a fixed deterministic
candidate-density and group-coherence probe and never changes result order.
The `Nat/Ref` and `Batch/Ref` columns are speedups calculated as Ref-loop time
divided by Native-loop or Native-batch time respectively; values above `1x`
mean that native path is faster. Interactive console output shows those faster
ratios in green and ratios at or below `1x` in red.

Build the native Release library first, then run:

```sh
dotnet run --project ArcCollision.Benchmarks -c Release
```

Use `--help` for all parameters. A fast smoke run is available with `--quick`.
The default seed is printed along with a complete reproduction command.
