using System.Globalization;

namespace ArcCollision.Benchmarks;

internal sealed record BenchmarkOptions(
    ulong Seed,
    int StaticCount,
    int DynamicCount,
    int Frames,
    int Iterations,
    int WarmupIterations,
    int MinimumSampleMilliseconds,
    float FatMargin)
{
    public const ulong DefaultSeed = 0xA11CE5EEDUL;

    public static BenchmarkOptions Defaults => new(
        DefaultSeed,
        StaticCount: 1500,
        DynamicCount: 750,
        Frames: 120,
        Iterations: 9,
        WarmupIterations: 3,
        MinimumSampleMilliseconds: 200,
        FatMargin: 4f);

    public static BenchmarkOptions Parse(string[] args)
    {
        BenchmarkOptions options = Defaults;
        for (int i = 0; i < args.Length; i++)
        {
            string argument = args[i];
            if (argument == "--help" || argument == "-h")
                throw new HelpRequestedException();
            if (argument == "--quick")
            {
                options = options with
                {
                    StaticCount = 400,
                    DynamicCount = 200,
                    Frames = 20,
                    Iterations = 3,
                    WarmupIterations = 1,
                    MinimumSampleMilliseconds = 20,
                };
                continue;
            }

            string Value()
            {
                if (++i >= args.Length)
                    throw new ArgumentException($"Missing value after {argument}.");
                return args[i];
            }

            options = argument switch
            {
                "--seed" => options with { Seed = ParseSeed(Value()) },
                "--static" => options with { StaticCount = ParseInt(Value(), argument) },
                "--dynamic" => options with { DynamicCount = ParseInt(Value(), argument) },
                "--frames" => options with { Frames = ParseInt(Value(), argument) },
                "--iterations" => options with { Iterations = ParseInt(Value(), argument) },
                "--warmup" => options with { WarmupIterations = ParseInt(Value(), argument) },
                "--sample-ms" => options with
                {
                    MinimumSampleMilliseconds = ParseInt(Value(), argument),
                },
                "--fat-margin" => options with
                {
                    FatMargin = float.Parse(Value(), NumberStyles.Float, CultureInfo.InvariantCulture),
                },
                _ => throw new ArgumentException($"Unknown argument: {argument}"),
            };
        }

        if (options.StaticCount <= 0) throw new ArgumentOutOfRangeException("--static");
        if (options.DynamicCount <= 0) throw new ArgumentOutOfRangeException("--dynamic");
        if (options.Frames <= 0) throw new ArgumentOutOfRangeException("--frames");
        if (options.Iterations <= 0) throw new ArgumentOutOfRangeException("--iterations");
        if (options.WarmupIterations < 0) throw new ArgumentOutOfRangeException("--warmup");
        if (options.MinimumSampleMilliseconds <= 0)
            throw new ArgumentOutOfRangeException("--sample-ms");
        if (!float.IsFinite(options.FatMargin) || options.FatMargin < 0)
            throw new ArgumentOutOfRangeException("--fat-margin");
        const int maxColliderCount = 1 << 20;
        if ((long)options.StaticCount + options.DynamicCount > maxColliderCount)
            throw new ArgumentOutOfRangeException(
                $"Combined collider count exceeds {maxColliderCount}.");
        return options;
    }

    public string ReproductionArguments => string.Format(
        CultureInfo.InvariantCulture,
        "--seed 0x{0:X} --static {1} --dynamic {2} --frames {3} "
        + "--iterations {4} --warmup {5} --sample-ms {6} --fat-margin {7}",
        Seed, StaticCount, DynamicCount, Frames,
        Iterations, WarmupIterations, MinimumSampleMilliseconds, FatMargin);

    public static void PrintHelp()
    {
        Console.WriteLine("ArcCollision static+dynamic single-thread benchmark");
        Console.WriteLine();
        Console.WriteLine("Options:");
        Console.WriteLine("  --seed <n|0xHEX>       Deterministic seed");
        Console.WriteLine("  --static <count>       Static collider count");
        Console.WriteLine("  --dynamic <count>      Dynamic collider count");
        Console.WriteLine("  --frames <count>       Simulated frames per trial");
        Console.WriteLine("  --iterations <count>   Measured trials per backend");
        Console.WriteLine("  --warmup <count>       Untimed warmup trials per backend");
        Console.WriteLine("  --sample-ms <ms>       Minimum duration of each query timing sample");
        Console.WriteLine("  --fat-margin <value>   Dynamic-tree fat margin");
        Console.WriteLine("  --quick                Small smoke benchmark preset");
    }

    private static int ParseInt(string value, string argument)
    {
        if (!int.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out int result))
            throw new ArgumentException($"Invalid integer for {argument}: {value}");
        return result;
    }

    private static ulong ParseSeed(string value)
    {
        NumberStyles style = NumberStyles.Integer;
        if (value.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
        {
            value = value[2..];
            style = NumberStyles.AllowHexSpecifier;
        }
        if (!ulong.TryParse(value, style, CultureInfo.InvariantCulture, out ulong seed))
            throw new ArgumentException($"Invalid seed: {value}");
        return seed;
    }
}

internal sealed class HelpRequestedException : Exception
{
}
