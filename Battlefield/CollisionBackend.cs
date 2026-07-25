/*
 * CollisionBackend.cs
 * Battlefield - ArcCollision beat-'em-up gameplay sample adapted from Quiver.
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

// Battlefield runs on the native ArcCollision backend via the P/Invoke wrapper.
// Its code lives in namespace ArcCollision.Battlefield and refers to collision
// types unqualified (ArcWorld, Circle, Shape, CollisionFilter, ...); this global
// using resolves them to ArcCollision.Wrapper instead of the managed reference
// implementation. To switch back to the reference backend, swap the project
// reference to ArcCollision.Ref and change this to `global using ArcCollision.Ref;`.
global using ArcCollision.Wrapper;
