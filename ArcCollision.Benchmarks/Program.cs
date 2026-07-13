using System.Runtime.InteropServices;

namespace ArcCollision.Benchmarks;

internal static class Program
{
    public static int Main(string[] args)
    {
        try
        {
            BenchmarkOptions options = BenchmarkOptions.Parse(args);
            Run(options);
            return 0;
        }
        catch (HelpRequestedException)
        {
            BenchmarkOptions.PrintHelp();
            return 0;
        }
        catch (Exception exception)
        {
            Console.Error.WriteLine(exception);
            return 1;
        }
    }

    private static void Run(BenchmarkOptions options)
    {
#if DEBUG
        Console.WriteLine("WARNING: Debug build; use -c Release for meaningful results.");
#endif
        int threadId = Environment.CurrentManagedThreadId;
        Console.WriteLine("ArcCollision static + dynamic collision benchmark");
        Console.WriteLine($"Runtime: {RuntimeInformation.FrameworkDescription}");
        Console.WriteLine($"OS/architecture: {RuntimeInformation.OSDescription} / {RuntimeInformation.ProcessArchitecture}");
        Console.WriteLine($"Execution: single calling thread (managed id {threadId}), no Task/Parallel usage");
        Console.WriteLine($"Seed: {options.Seed} (0x{options.Seed:X})");
        Console.WriteLine($"Scene: static={options.StaticCount}, dynamic={options.DynamicCount}, "
            + $"frames={options.Frames}, fat-margin={options.FatMargin}");
        Console.WriteLine($"Trials: warmup={options.WarmupIterations}, measured={options.Iterations}");
        Console.WriteLine($"Reproduce: dotnet run --project ArcCollision.Benchmarks -c Release -- {options.ReproductionArguments}");
        Console.WriteLine();

        BenchmarkScenario scenario = BenchmarkScenario.Create(options);
        var refScene = new RefPreparedScene(scenario);
        var wrapperScene = new WrapperPreparedScene(scenario);

        for (int i = 0; i < options.WarmupIterations; i++)
        {
            TrialResult reference = BackendRunners.RunRef(refScene, options, threadId);
            TrialResult wrapper = BackendRunners.RunWrapper(wrapperScene, options, threadId);
            ValidateEquivalent(reference, wrapper, $"warmup {i + 1}");
        }

        var refResults = new TrialResult[options.Iterations];
        var wrapperResults = new TrialResult[options.Iterations];
        for (int i = 0; i < options.Iterations; i++)
        {
            // Alternate order to avoid consistently favoring the first backend
            // through temperature, cache state, or frequency ramp behavior.
            if ((i & 1) == 0)
            {
                CollectBeforeTrial();
                refResults[i] = BackendRunners.RunRef(refScene, options, threadId);
                CollectBeforeTrial();
                wrapperResults[i] = BackendRunners.RunWrapper(wrapperScene, options, threadId);
            }
            else
            {
                CollectBeforeTrial();
                wrapperResults[i] = BackendRunners.RunWrapper(wrapperScene, options, threadId);
                CollectBeforeTrial();
                refResults[i] = BackendRunners.RunRef(refScene, options, threadId);
            }
            ValidateEquivalent(refResults[i], wrapperResults[i], $"iteration {i + 1}");
            Console.WriteLine($"Completed iteration {i + 1}/{options.Iterations}");
        }

        PrintResults(options, refResults, wrapperResults);
        GC.KeepAlive(refScene);
        GC.KeepAlive(wrapperScene);
    }

    private static void CollectBeforeTrial()
    {
        GC.Collect();
        GC.WaitForPendingFinalizers();
        GC.Collect();
    }

    private static void ValidateEquivalent(
        in TrialResult reference, in TrialResult wrapper, string phase)
    {
        if (reference.CandidateCount != wrapper.CandidateCount
            || reference.CollisionCount != wrapper.CollisionCount
            || reference.Checksum != wrapper.Checksum)
        {
            throw new InvalidOperationException(
                $"Backend result mismatch during {phase}: "
                + $"Ref candidates/collisions/hash={reference.CandidateCount}/"
                + $"{reference.CollisionCount}/0x{reference.Checksum:X16}, "
                + $"Wrapper={wrapper.CandidateCount}/{wrapper.CollisionCount}/"
                + $"0x{wrapper.Checksum:X16}.");
        }
    }

    private static void PrintResults(
        BenchmarkOptions options,
        TrialResult[] reference,
        TrialResult[] wrapper)
    {
        Summary refSummary = Summary.From(reference);
        Summary wrapperSummary = Summary.From(wrapper);
        Console.WriteLine();
        Console.WriteLine("Median of measured trials (best shown in parentheses):");
        Console.WriteLine("Backend                 Build ms          Simulation ms       ms/frame");
        PrintSummary("ArcCollision.Ref", refSummary, options.Frames);
        PrintSummary("ArcCollision.Wrapper", wrapperSummary, options.Frames);
        Console.WriteLine();
        Console.WriteLine($"Wrapper build speedup:      {refSummary.BuildMedian / wrapperSummary.BuildMedian:0.00}x");
        Console.WriteLine($"Wrapper simulation speedup: {refSummary.SimulationMedian / wrapperSummary.SimulationMedian:0.00}x");
        Console.WriteLine($"Candidates per trial: {reference[0].CandidateCount:N0}");
        Console.WriteLine($"Collisions per trial: {reference[0].CollisionCount:N0}");
        Console.WriteLine($"Result checksum: 0x{reference[0].Checksum:X16} (identical)");
        double updates = (double)options.DynamicCount * options.Frames;
        Console.WriteLine($"Wrapper dynamic updates/s: {updates / (wrapperSummary.SimulationMedian / 1000):N0}");
        Console.WriteLine($"Ref dynamic updates/s:     {updates / (refSummary.SimulationMedian / 1000):N0}");
    }

    private static void PrintSummary(string name, Summary summary, int frames)
    {
        Console.WriteLine($"{name,-23} "
            + $"{summary.BuildMedian,8:0.00} ({summary.BuildBest,8:0.00})  "
            + $"{summary.SimulationMedian,10:0.00} ({summary.SimulationBest,8:0.00})  "
            + $"{summary.SimulationMedian / frames,8:0.000}");
    }

    private readonly record struct Summary(
        double BuildMedian,
        double BuildBest,
        double SimulationMedian,
        double SimulationBest)
    {
        public static Summary From(TrialResult[] source)
        {
            double[] builds = source.Select(result => result.BuildTime.TotalMilliseconds).ToArray();
            double[] simulations = source.Select(result => result.SimulationTime.TotalMilliseconds).ToArray();
            Array.Sort(builds);
            Array.Sort(simulations);
            return new Summary(
                Median(builds), builds[0], Median(simulations), simulations[0]);
        }

        private static double Median(double[] sorted)
        {
            int middle = sorted.Length / 2;
            return (sorted.Length & 1) != 0
                ? sorted[middle]
                : (sorted[middle - 1] + sorted[middle]) * 0.5;
        }
    }
}
