/*
 * CollisionLimits.cs
 * ArcCollision.Ref - deterministic C# reference implementation.
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System;

namespace ArcCollision.Ref
{
    /// <summary>Public limits of the deterministic 24.8 fixed-point core.</summary>
    public static class CollisionLimits
    {
        public const float GridSize = 1f / 256f;
        public const float MaxCoordinate = 500_000_000f / 256f;
    }

    /// <summary>Initial capacity and broadphase settings for an <see cref="ArcWorld"/>.</summary>
    public readonly struct ArcWorldOptions
    {
        public readonly float FatMargin;
        public readonly int InitialColliderCapacity;
        public readonly int InitialPairCapacity;

        /// <summary>
        /// The documented defaults (fat margin 16, capacities 16). C# 9 -- the
        /// newest language version Unity 2022 LTS compiles -- has no
        /// parameterless struct constructor, so <c>new ArcWorldOptions()</c> and
        /// <c>default</c> both yield an all-zero value here. Use this property,
        /// or the parameterized constructor, to get the defaults.
        /// </summary>
        public static ArcWorldOptions Default => new ArcWorldOptions(16f, 16, 16);

        public ArcWorldOptions(
            float fatMargin = 16f,
            int initialColliderCapacity = 16,
            int initialPairCapacity = 16)
        {
            if (!float.IsFinite(fatMargin) || fatMargin < 0f)
                throw new ArgumentOutOfRangeException(nameof(fatMargin));
            if (initialColliderCapacity is < 0 or > ArcWorld.MaxColliderCount)
                throw new ArgumentOutOfRangeException(nameof(initialColliderCapacity));
            if (initialPairCapacity < 0)
                throw new ArgumentOutOfRangeException(nameof(initialPairCapacity));
            FatMargin = fatMargin;
            InitialColliderCapacity = initialColliderCapacity;
            InitialPairCapacity = initialPairCapacity;
        }
    }
}
