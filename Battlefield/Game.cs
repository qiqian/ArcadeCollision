using System;
using System.Collections.Generic;

namespace ArcCollision.Battlefield;

internal enum Faction { Player, Enemy }
internal enum EnemyKind { None, Soldier, Archer }

/// <summary>A combatant on the battlefield. Bodies are circles on the floor plane.</summary>
internal sealed class Fighter
{
    public Faction Faction;
    public EnemyKind Kind;
    public Vec2 Pos;
    public Vec2 Vel;
    public float Radius = 18f;
    public float Speed = 150f;
    public float Health;
    public float MaxHealth;
    public float Facing = 1f;              // -1 left, +1 right
    public Vec2 KnockVel;                  // decaying knockback velocity
    public float HurtFlash;                // seconds remaining of red flash

    // attack state machine
    public float AttackTimer = -1f;        // >=0 while swinging
    public float AttackDuration = 0.34f;
    public float AttackCooldown;           // seconds until next allowed swing
    public float Reach = 46f;
    public float Damage = 18f;
    public readonly HashSet<Fighter> HitThisSwing = new();

    public float ThinkTimer;               // AI decision cadence
    public bool Alive => Health > 0f;
    public bool Attacking => AttackTimer >= 0f;

    /// <summary>Fraction (0..1) through the swing where contact actually lands.</summary>
    public bool AttackActive
    {
        get
        {
            if (AttackTimer < 0f) return false;
            float t = AttackDuration - AttackTimer; // elapsed
            return t >= 0.10f && t <= 0.22f;
        }
    }

    public Circle Body => new(Pos, Radius);

    /// <summary>The melee hitbox: a capsule reaching out in the facing direction.</summary>
    public Capsule Swing()
    {
        Vec2 dir = new(Facing, 0f);
        Vec2 start = Pos + dir * (Radius * 0.4f);
        Vec2 end = Pos + dir * (Radius + Reach);
        return new Capsule(start, end, Radius * 0.7f);
    }
}

internal sealed class Projectile
{
    public Faction Faction;
    public Vec2 Pos;
    public Vec2 Vel;
    public float Radius = 5f;
    public float Damage = 12f;
    public bool Alive = true;
    public float Life = 3f;
}

/// <summary>
/// The whole simulation. Rendering lives in <see cref="GameForm"/>; this class is
/// pure logic and is the primary consumer of the ArcCollision library:
///   * SpatialHash          broadphase for body-vs-body separation
///   * CircleVsCircle        push characters apart
///   * CircleVsCapsule       melee hit detection
///   * MovingCircleVsCircle  swept arrows (no tunnelling at high speed)
/// </summary>
internal sealed class Game
{
    public const float ArenaWidth = 1180f;
    public const float ArenaHeight = 560f;

    public readonly List<Fighter> Fighters = new();
    public readonly List<Projectile> Projectiles = new();
    public Fighter Player = null!;

    public int Score;
    public int Wave;
    public int BroadphasePairs;   // exposed for the HUD to show the library working
    public bool GameOver;

    private readonly SpatialHash _hash = new(64f);
    private readonly List<int> _queryScratch = new();
    private readonly Random _rng = new();
    private float _spawnTimer;
    private int _aliveEnemies;

    // player input for the current frame
    private Vec2 _moveInput;
    private bool _attackInput;

    public Game() => Reset();

    public void Reset()
    {
        Fighters.Clear();
        Projectiles.Clear();
        Score = 0;
        Wave = 0;
        GameOver = false;
        _spawnTimer = 0.5f;

        Player = new Fighter
        {
            Faction = Faction.Player,
            Pos = new Vec2(ArenaWidth * 0.5f, ArenaHeight * 0.5f),
            Radius = 20f,
            Speed = 210f,
            MaxHealth = 200f,
            Health = 200f,
            Reach = 58f,
            Damage = 34f,
            AttackDuration = 0.30f,
        };
        Fighters.Add(Player);
    }

    public void SetInput(Vec2 move, bool attack)
    {
        _moveInput = move;
        _attackInput = attack;
    }

    public void Update(float dt)
    {
        if (GameOver)
            return;

        UpdatePlayer(dt);
        UpdateEnemies(dt);
        UpdateAttacks(dt);
        Integrate(dt);
        ResolveBodies();
        UpdateProjectiles(dt);
        ClampToArena();
        Cleanup();
        Spawning(dt);

        if (!Player.Alive)
            GameOver = true;
    }

    // --------------------------------------------------------------- player

    private void UpdatePlayer(float dt)
    {
        Vec2 dir = _moveInput.Normalized(Vec2.Zero);
        Player.Vel = dir * Player.Speed;
        if (MathF.Abs(dir.X) > 0.01f)
            Player.Facing = dir.X < 0 ? -1f : 1f;

        Player.AttackCooldown -= dt;
        if (_attackInput && !Player.Attacking && Player.AttackCooldown <= 0f)
            StartSwing(Player);
    }

    // ---------------------------------------------------------------- enemy AI

    private void UpdateEnemies(float dt)
    {
        foreach (var f in Fighters)
        {
            if (f.Faction != Faction.Enemy || !f.Alive)
                continue;

            f.ThinkTimer -= dt;
            f.AttackCooldown -= dt;
            Vec2 toPlayer = Player.Pos - f.Pos;
            float dist = toPlayer.Length;
            Vec2 dir = toPlayer.Normalized(Vec2.UnitX);
            if (MathF.Abs(dir.X) > 0.01f)
                f.Facing = dir.X < 0 ? -1f : 1f;

            if (f.Kind == EnemyKind.Soldier)
            {
                float touch = f.Radius + Player.Radius + f.Reach * 0.5f;
                if (dist > touch)
                    f.Vel = dir * f.Speed;
                else
                {
                    f.Vel = Vec2.Zero;
                    if (!f.Attacking && f.AttackCooldown <= 0f)
                        StartSwing(f);
                }
            }
            else // Archer: kite the player and shoot
            {
                const float preferred = 280f;
                if (dist < preferred - 40f)
                    f.Vel = dir * -f.Speed;          // back away
                else if (dist > preferred + 40f)
                    f.Vel = dir * f.Speed;           // close in
                else
                    f.Vel = dir.Perp * (f.Speed * 0.4f); // strafe

                if (f.AttackCooldown <= 0f && dist < 460f)
                {
                    ShootArrow(f, dir);
                    f.AttackCooldown = 1.6f + (float)_rng.NextDouble();
                }
            }
        }
    }

    private void StartSwing(Fighter f)
    {
        f.AttackTimer = f.AttackDuration;
        f.HitThisSwing.Clear();
        f.AttackCooldown = f.Faction == Faction.Player ? 0.16f : 0.9f;
    }

    private void ShootArrow(Fighter f, Vec2 dir)
    {
        Projectiles.Add(new Projectile
        {
            Faction = Faction.Enemy,
            Pos = f.Pos + dir * (f.Radius + 6f),
            Vel = dir * 620f,     // fast: relies on swept collision to connect
            Damage = 14f,
        });
    }

    // ------------------------------------------------------------- attacks

    private void UpdateAttacks(float dt)
    {
        foreach (var f in Fighters)
        {
            if (!f.Attacking)
                continue;
            f.AttackTimer -= dt;
            if (f.AttackTimer < 0f)
                continue;
            if (!f.AttackActive)
                continue;

            Capsule swing = f.Swing();
            foreach (var target in Fighters)
            {
                if (target == f || !target.Alive || target.Faction == f.Faction)
                    continue;
                if (f.HitThisSwing.Contains(target))
                    continue;

                Manifold hit = Collide.CircleVsCapsule(target.Body, swing);
                if (hit.Colliding)
                {
                    f.HitThisSwing.Add(target);
                    ApplyDamage(target, f.Damage, new Vec2(f.Facing, 0f), 260f);
                }
            }
        }
    }

    private void ApplyDamage(Fighter target, float dmg, Vec2 dir, float knock)
    {
        target.Health -= dmg;
        target.HurtFlash = 0.15f;
        target.KnockVel += dir.Normalized(Vec2.UnitX) * knock;
        if (!target.Alive && target.Faction == Faction.Enemy)
        {
            Score += target.Kind == EnemyKind.Archer ? 150 : 100;
            _aliveEnemies--;
        }
    }

    // ------------------------------------------------------------ integrate

    private void Integrate(float dt)
    {
        foreach (var f in Fighters)
        {
            if (!f.Alive)
                continue;
            // slow while swinging so attacks commit
            float moveScale = f.Attacking ? 0.35f : 1f;
            f.Pos += (f.Vel * moveScale + f.KnockVel) * dt;
            // decay knockback
            float decay = MathF.Exp(-8f * dt);
            f.KnockVel *= decay;
            if (f.HurtFlash > 0f)
                f.HurtFlash -= dt;
        }
    }

    // ---------------------------------------------- body-vs-body separation

    private void ResolveBodies()
    {
        // Broadphase: rebuild the spatial hash from every living body.
        _hash.Clear();
        for (int i = 0; i < Fighters.Count; i++)
            if (Fighters[i].Alive)
                _hash.Insert(i, Fighters[i].Body.Bounds);

        BroadphasePairs = 0;
        // Two iterations of positional relaxation keeps stacks stable.
        for (int pass = 0; pass < 2; pass++)
        {
            foreach (var (ia, ib) in _hash.Pairs())
            {
                if (pass == 0)
                    BroadphasePairs++;
                Fighter a = Fighters[ia];
                Fighter b = Fighters[ib];
                if (!a.Alive || !b.Alive)
                    continue;

                Manifold m = Collide.CircleVsCircle(a.Body, b.Body);
                if (!m.Colliding)
                    continue;

                // Heavier player takes a smaller share of the push.
                float wa = a.Faction == Faction.Player ? 0.2f : 0.5f;
                float wb = b.Faction == Faction.Player ? 0.2f : 0.5f;
                float sum = wa + wb;
                if (sum <= 0f)
                    sum = 1f;
                Vec2 push = m.Normal * m.Depth;
                a.Pos -= push * (wa / sum);
                b.Pos += push * (wb / sum);
            }
        }
    }

    // ------------------------------------------------------- projectiles

    private void UpdateProjectiles(float dt)
    {
        foreach (var p in Projectiles)
        {
            if (!p.Alive)
                continue;
            p.Life -= dt;
            if (p.Life <= 0f)
            {
                p.Alive = false;
                continue;
            }

            Vec2 motion = p.Vel * dt;
            var mover = new Circle(p.Pos, p.Radius);

            // Swept test against every valid target; keep the earliest impact so
            // a fast arrow can never tunnel through a body in a single frame.
            SweepHit best = SweepHit.Miss;
            Fighter? victim = null;
            foreach (var f in Fighters)
            {
                if (!f.Alive || f.Faction == p.Faction)
                    continue;
                SweepHit h = Sweep.MovingCircleVsCircle(mover, motion, f.Body);
                if (h.Hit && (victim == null || h.Time < best.Time))
                {
                    best = h;
                    victim = f;
                }
            }

            if (victim != null)
            {
                ApplyDamage(victim, p.Damage, p.Vel, 120f);
                p.Alive = false;
            }
            else
            {
                p.Pos += motion;
                if (p.Pos.X < -40 || p.Pos.X > ArenaWidth + 40 ||
                    p.Pos.Y < -40 || p.Pos.Y > ArenaHeight + 40)
                    p.Alive = false;
            }
        }
    }

    private void ClampToArena()
    {
        foreach (var f in Fighters)
        {
            if (!f.Alive)
                continue;
            f.Pos = new Vec2(
                Math.Clamp(f.Pos.X, f.Radius, ArenaWidth - f.Radius),
                Math.Clamp(f.Pos.Y, f.Radius, ArenaHeight - f.Radius));
        }
    }

    private void Cleanup()
    {
        Fighters.RemoveAll(f => f.Faction == Faction.Enemy && !f.Alive);
        Projectiles.RemoveAll(p => !p.Alive);
    }

    // -------------------------------------------------------------- spawning

    private void Spawning(float dt)
    {
        _aliveEnemies = 0;
        foreach (var f in Fighters)
            if (f.Faction == Faction.Enemy && f.Alive)
                _aliveEnemies++;

        _spawnTimer -= dt;
        int cap = 4 + Wave;
        if (_aliveEnemies < cap && _spawnTimer <= 0f)
        {
            SpawnEnemy();
            _spawnTimer = MathF.Max(0.35f, 1.4f - Wave * 0.08f);
        }

        if (_aliveEnemies == 0 && _spawnTimer <= 0f)
            Wave++;
    }

    private void SpawnEnemy()
    {
        // spawn just outside a random edge, then walk in
        int edge = _rng.Next(4);
        Vec2 pos = edge switch
        {
            0 => new Vec2(_rng.Next((int)ArenaWidth), 8f),
            1 => new Vec2(_rng.Next((int)ArenaWidth), ArenaHeight - 8f),
            2 => new Vec2(8f, _rng.Next((int)ArenaHeight)),
            _ => new Vec2(ArenaWidth - 8f, _rng.Next((int)ArenaHeight)),
        };

        bool archer = _rng.NextDouble() < 0.28 && Wave >= 1;
        var f = new Fighter
        {
            Faction = Faction.Enemy,
            Kind = archer ? EnemyKind.Archer : EnemyKind.Soldier,
            Pos = pos,
            Radius = archer ? 16f : 18f,
            Speed = archer ? 130f : 105f + Wave * 4f,
            MaxHealth = archer ? 40f : 60f,
            Health = archer ? 40f : 60f,
            Reach = 30f,
            Damage = archer ? 0f : 12f,
            AttackDuration = 0.4f,
        };
        Fighters.Add(f);
    }
}
