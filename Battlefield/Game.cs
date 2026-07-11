using System;
using System.Collections.Generic;

namespace ArcCollision.Battlefield;

internal enum Faction { Player, Enemy }
internal enum EnemyKind { None, Grunt, Brute }
internal enum AnimState { Idle, Walk, Attack, Hurt, Dead }

/// <summary>A combatant. The body is a circle on the floor plane; everything the
/// player sees (limbs, weapon, poses) is derived from these fields by CharacterArt.</summary>
internal sealed class Fighter
{
    public Faction Faction;
    public EnemyKind Kind;
    public Vec2 Pos;
    public Vec2 Vel;
    public float Radius = 16f;
    public float Speed = 150f;
    public float Health;
    public float MaxHealth;
    public float Facing = 1f;            // -1 left, +1 right
    public float Scale = 1f;
    public Vec2 KnockVel;
    public float HurtFlash;
    public float Invuln;

    // animation
    public AnimState State = AnimState.Idle;
    public float StateTime;
    public float WalkPhase;
    public float AnimClock;
    public float DeathTimer;
    public float Lean;                    // signed body lean for poses

    // attack
    public float AttackTimer = -1f;       // counts down while swinging
    public float AttackDuration = 0.4f;
    public float AttackWindup = 0.14f;
    public float AttackActiveDur = 0.12f;
    public float AttackCooldown;
    public float Reach = 46f;
    public float Damage = 16f;
    public float KnockPower = 220f;
    public bool Launcher;
    public int ComboStep;
    public float ComboResetTimer;
    public readonly HashSet<Fighter> HitThisSwing = new();

    public float ThinkTimer;

    public bool Dead => State == AnimState.Dead;
    public bool Attacking => State == AnimState.Attack;
    public bool CanAct => State is AnimState.Idle or AnimState.Walk;
    public Circle Body => new(Pos, Radius);

    /// <summary>Elapsed time since the current swing started.</summary>
    public float AttackElapsed => AttackDuration - AttackTimer;

    public bool AttackActive =>
        Attacking && AttackElapsed >= AttackWindup && AttackElapsed <= AttackWindup + AttackActiveDur;

    /// <summary>Melee hitbox: a capsule reaching out in the facing direction.</summary>
    public Capsule Swing()
    {
        Vec2 dir = new(Facing, 0f);
        Vec2 start = Pos + dir * (Radius * 0.3f);
        Vec2 end = Pos + dir * (Radius + Reach);
        return new Capsule(start, end, Radius * 0.85f);
    }
}

internal struct HitSpark
{
    public Vec2 Pos;
    public float Age;
    public float Life;
    public float Angle;
    public float Size;
    public bool Heavy;
}

internal struct DamageNumber
{
    public Vec2 Pos;
    public Vec2 Vel;
    public float Age;
    public float Life;
    public int Value;
    public bool Crit;
}

internal struct Dust
{
    public Vec2 Pos;
    public Vec2 Vel;
    public float Age;
    public float Life;
    public float Size;
}

/// <summary>
/// The whole beat-'em-up simulation. It is a game first, but the moment-to-moment
/// physics runs entirely on the ArcCollision library:
///   * SpatialHash + CircleVsCircle  keep the crowd of bodies from overlapping
///   * CircleVsCapsule               resolves every sword / spear / hammer hit
/// </summary>
internal sealed class Game
{
    public const float ArenaWidth = 1180f;
    public const float ArenaHeight = 540f;
    public const float FloorTop = 90f;   // characters can't walk above this (keeps heads off the HUD)

    public readonly List<Fighter> Fighters = new();
    public readonly List<HitSpark> Sparks = new();
    public readonly List<DamageNumber> Numbers = new();
    public readonly List<Dust> Dusts = new();
    public Fighter Player = null!;

    public int Score;
    public int Wave;
    public int Lives = 3;
    public int Combo;
    public int BestCombo;
    public float ComboTimer;
    public bool GameOver;

    // juice
    public float Hitstop;
    public float ShakeMag;
    public float FlashFx;                 // full-screen white flash on big hits

    private readonly SpatialHash _hash = new(72f);
    private readonly Random _rng = new();
    private float _spawnTimer;
    private float _respawnTimer;

    private Vec2 _move;
    private bool _attackPressed;
    private bool _heavyPressed;
    private bool _dashPressed;
    private float _dashCd;

    public Game() => Reset();

    public void Reset()
    {
        Fighters.Clear();
        Sparks.Clear();
        Numbers.Clear();
        Dusts.Clear();
        Score = 0;
        Wave = 0;
        Lives = 3;
        Combo = 0;
        BestCombo = 0;
        ComboTimer = 0;
        GameOver = false;
        Hitstop = 0;
        ShakeMag = 0;
        _spawnTimer = 1.0f;
        _respawnTimer = 0;
        SpawnPlayer();
    }

    private void SpawnPlayer()
    {
        Player = new Fighter
        {
            Faction = Faction.Player,
            Pos = new Vec2(ArenaWidth * 0.5f, ArenaHeight * 0.62f),
            Radius = 19f,
            Scale = 1.08f,
            Speed = 235f,
            MaxHealth = 240f,
            Health = 240f,
            Reach = 62f,
            Invuln = 1.2f,
        };
        Fighters.Add(Player);
    }

    public void SetInput(Vec2 move, bool attack, bool heavy, bool dash)
    {
        _move = move;
        _attackPressed = attack;
        _heavyPressed = heavy;
        _dashPressed = dash;
    }

    // -------------------------------------------------------------- main loop

    public void Update(float dt)
    {
        // These tick in real time so impacts still "shake" while frozen.
        if (ShakeMag > 0f) ShakeMag = MathF.Max(0f, ShakeMag - dt * 26f);
        if (FlashFx > 0f) FlashFx = MathF.Max(0f, FlashFx - dt * 4f);
        if (_dashCd > 0f) _dashCd -= dt;

        if (Hitstop > 0f)
        {
            Hitstop -= dt;   // freeze frame for impact weight
            return;
        }

        if (GameOver)
        {
            AnimateOnly(dt);
            return;
        }

        UpdateCombo(dt);
        UpdatePlayer(dt);
        UpdateEnemies(dt);
        AdvanceStates(dt);
        ResolveAttacks();
        Integrate(dt);
        ResolveBodies();
        ClampToArena();
        UpdateEffects(dt);
        Cleanup();
        Spawning(dt);
    }

    private void AnimateOnly(float dt)
    {
        AdvanceStates(dt);
        Integrate(dt);
        UpdateEffects(dt);
    }

    private void UpdateCombo(float dt)
    {
        if (ComboTimer > 0f)
        {
            ComboTimer -= dt;
            if (ComboTimer <= 0f)
                Combo = 0;
        }
    }

    // -------------------------------------------------------------- player

    private void UpdatePlayer(float dt)
    {
        var p = Player;
        if (p.Dead)
            return;

        p.AttackCooldown -= dt;

        if (p.CanAct)
        {
            Vec2 dir = _move.Normalized(Vec2.Zero);
            p.Vel = dir * p.Speed;
            if (MathF.Abs(dir.X) > 0.01f)
                p.Facing = dir.X < 0 ? -1f : 1f;
            p.State = dir.LengthSquared > 0.01f ? AnimState.Walk : AnimState.Idle;

            if (_dashPressed && _dashCd <= 0f)
                StartDash(p, dir);
            else if (_heavyPressed && p.AttackCooldown <= 0f)
                StartSwing(p, heavy: true);
            else if (_attackPressed && p.AttackCooldown <= 0f)
                StartSwing(p, heavy: false);
        }
        else if (p.Attacking)
        {
            // small forward drift while committing to a swing
            p.Vel = new Vec2(p.Facing, 0f) * (p.Launcher ? 60f : 90f);
        }
        else
        {
            p.Vel = Vec2.Zero;
        }
    }

    private void StartDash(Fighter p, Vec2 dir)
    {
        Vec2 d = dir.LengthSquared > 0.01f ? dir : new Vec2(p.Facing, 0f);
        p.KnockVel += d * 520f;
        p.Invuln = MathF.Max(p.Invuln, 0.32f);
        _dashCd = 0.7f;
        for (int i = 0; i < 8; i++)
            SpawnDust(p.Pos, -d * 60f + RandVec(40f));
    }

    // ------------------------------------------------------------- enemy AI

    private void UpdateEnemies(float dt)
    {
        foreach (var f in Fighters)
        {
            if (f.Faction != Faction.Enemy || f.Dead)
                continue;

            f.AttackCooldown -= dt;
            Vec2 toPlayer = Player.Pos - f.Pos;
            float dist = toPlayer.Length;
            Vec2 dir = toPlayer.Normalized(Vec2.UnitX);

            if (!f.CanAct)
                continue;

            if (MathF.Abs(dir.X) > 0.05f)
                f.Facing = dir.X < 0 ? -1f : 1f;

            float strikeRange = f.Radius + Player.Radius + f.Reach * 0.55f;
            if (!Player.Dead && dist <= strikeRange && f.AttackCooldown <= 0f)
            {
                f.Vel = Vec2.Zero;
                StartSwing(f, heavy: f.Kind == EnemyKind.Brute);
            }
            else if (dist > strikeRange)
            {
                // approach with a little sideways jitter so they don't stack in a line
                f.ThinkTimer -= dt;
                if (f.ThinkTimer <= 0f)
                    f.ThinkTimer = 0.4f + (float)_rng.NextDouble() * 0.6f;
                Vec2 strafe = dir.Perp * (MathF.Sin(f.AnimClock * 2.2f) * 0.35f);
                f.Vel = (dir + strafe).Normalized(dir) * f.Speed;
                f.State = AnimState.Walk;
            }
            else
            {
                f.Vel = Vec2.Zero;
                f.State = AnimState.Idle;
            }
        }
    }

    // ------------------------------------------------------------- swings

    private void StartSwing(Fighter f, bool heavy)
    {
        f.State = AnimState.Attack;
        f.StateTime = 0;
        f.HitThisSwing.Clear();

        if (f.Faction == Faction.Player)
        {
            if (heavy)
            {
                f.ComboStep = 0;
                f.Launcher = true;
                f.AttackDuration = 0.52f;
                f.AttackWindup = 0.20f;
                f.AttackActiveDur = 0.14f;
                f.Damage = 46f;
                f.KnockPower = 620f;
                f.Reach = 78f;
                f.AttackCooldown = 0.55f;
                ShakeMag = MathF.Max(ShakeMag, 5f);
            }
            else
            {
                f.ComboStep = f.ComboResetTimer > 0f ? (f.ComboStep + 1) % 3 : 0;
                f.Launcher = f.ComboStep == 2;
                f.AttackDuration = 0.30f;
                f.AttackWindup = 0.07f;
                f.AttackActiveDur = 0.11f;
                f.Damage = f.ComboStep == 2 ? 30f : 18f;
                f.KnockPower = f.ComboStep == 2 ? 460f : 180f;
                f.Reach = f.ComboStep == 2 ? 70f : 58f;
                f.AttackCooldown = 0.16f;
                f.ComboResetTimer = f.AttackDuration + 0.45f;
            }
        }
        else if (f.Kind == EnemyKind.Brute)
        {
            f.Launcher = true;
            f.AttackDuration = 0.9f;
            f.AttackWindup = 0.5f;      // big telegraph
            f.AttackActiveDur = 0.14f;
            f.Damage = 34f;
            f.KnockPower = 520f;
            f.AttackCooldown = 1.4f;
        }
        else
        {
            f.Launcher = false;
            f.AttackDuration = 0.55f;
            f.AttackWindup = 0.30f;     // readable telegraph
            f.AttackActiveDur = 0.12f;
            f.Damage = 12f;
            f.KnockPower = 240f;
            f.AttackCooldown = 0.9f + (float)_rng.NextDouble() * 0.5f;
        }

        f.AttackTimer = f.AttackDuration;
    }

    private void ResolveAttacks()
    {
        foreach (var f in Fighters)
        {
            if (!f.AttackActive)
                continue;

            Capsule swing = f.Swing();
            foreach (var t in Fighters)
            {
                if (t == f || t.Dead || t.Faction == f.Faction || t.Invuln > 0f)
                    continue;
                if (f.HitThisSwing.Contains(t))
                    continue;
                if (!Collide.CircleVsCapsule(t.Body, swing).Colliding)
                    continue;

                f.HitThisSwing.Add(t);
                Vec2 hitPoint = t.Pos + new Vec2(-f.Facing, 0f) * t.Radius + new Vec2(0f, -t.Radius * 0.4f);
                ApplyHit(f, t, hitPoint);
            }
        }
    }

    private void ApplyHit(Fighter attacker, Fighter target, Vec2 point)
    {
        bool crit = attacker.Launcher;
        target.Health -= attacker.Damage;
        target.HurtFlash = 0.14f;

        Vec2 knockDir = new(attacker.Facing, 0f);
        target.KnockVel += knockDir * attacker.KnockPower;
        if (crit)
            target.KnockVel += new Vec2(0f, (_rng.Next(2) == 0 ? -1 : 1) * attacker.KnockPower * 0.15f);

        // juice scaled by damage
        float power = Math.Clamp(attacker.Damage / 40f, 0.25f, 1.2f);
        Hitstop = MathF.Max(Hitstop, crit ? 0.10f : 0.04f + power * 0.03f);
        ShakeMag = MathF.Max(ShakeMag, crit ? 9f : 3f + power * 3f);
        if (crit) FlashFx = MathF.Max(FlashFx, 0.6f);

        SpawnSpark(point, MathF.Atan2(knockDir.Y, knockDir.X), crit);
        SpawnNumber(point + new Vec2(0, -14), (int)attacker.Damage, crit);

        if (target.Health <= 0f)
        {
            EnterDeath(target, knockDir);
            if (target.Faction == Faction.Enemy)
            {
                Score += (target.Kind == EnemyKind.Brute ? 300 : 100) + Combo * 5;
                if (attacker.Faction == Faction.Player)
                    BumpCombo();
            }
        }
        else
        {
            EnterHurt(target);
            if (attacker.Faction == Faction.Player && target.Faction == Faction.Enemy)
                BumpCombo();
        }
    }

    private void BumpCombo()
    {
        Combo++;
        BestCombo = Math.Max(BestCombo, Combo);
        ComboTimer = 2.2f;
    }

    private void EnterHurt(Fighter f)
    {
        if (f.Dead)
            return;
        f.State = AnimState.Hurt;
        f.StateTime = 0;
        f.AttackTimer = -1f;
        f.Lean = -MathF.Sign(f.KnockVel.X == 0 ? f.Facing : f.KnockVel.X);
    }

    private void EnterDeath(Fighter f, Vec2 dir)
    {
        f.State = AnimState.Dead;
        f.StateTime = 0;
        f.DeathTimer = 1.1f;
        f.Health = 0;
        f.AttackTimer = -1f;
        for (int i = 0; i < 6; i++)
            SpawnDust(f.Pos, RandVec(90f));

        if (f.Faction == Faction.Player)
        {
            Lives--;
            _respawnTimer = 1.4f;
        }
    }

    // ------------------------------------------------------------- states

    private void AdvanceStates(float dt)
    {
        foreach (var f in Fighters)
        {
            f.AnimClock += dt;
            if (f.ComboResetTimer > 0f) f.ComboResetTimer -= dt;
            if (f.Invuln > 0f) f.Invuln -= dt;
            if (f.HurtFlash > 0f) f.HurtFlash -= dt;
            f.StateTime += dt;

            float speed = f.Vel.Length;
            if (f.State == AnimState.Walk || (f.State == AnimState.Idle && speed > 5f))
                f.WalkPhase += dt * (6f + speed * 0.03f);

            switch (f.State)
            {
                case AnimState.Attack:
                    f.AttackTimer -= dt;
                    f.Lean = MathF.Min(1f, f.Lean + dt * 6f); // lean into the strike
                    if (f.AttackTimer <= 0f)
                    {
                        f.State = AnimState.Idle;
                        f.Lean = 0;
                    }
                    break;
                case AnimState.Hurt:
                    if (f.StateTime > 0.26f)
                    {
                        f.State = AnimState.Idle;
                        f.Lean = 0;
                    }
                    break;
                case AnimState.Dead:
                    f.DeathTimer -= dt;
                    break;
            }
        }
    }

    // ------------------------------------------------------------- physics

    private void Integrate(float dt)
    {
        foreach (var f in Fighters)
        {
            float moveScale = f.State switch
            {
                AnimState.Attack => 0.25f,
                AnimState.Hurt => 0f,
                AnimState.Dead => 0f,
                _ => 1f,
            };
            f.Pos += (f.Vel * moveScale + f.KnockVel) * dt;
            f.KnockVel *= MathF.Exp(-9f * dt);
        }
    }

    private void ResolveBodies()
    {
        _hash.Clear();
        for (int i = 0; i < Fighters.Count; i++)
            if (!Fighters[i].Dead)
                _hash.Insert(i, Fighters[i].Body.Bounds);

        for (int pass = 0; pass < 2; pass++)
        {
            foreach (var (ia, ib) in _hash.Pairs())
            {
                Fighter a = Fighters[ia];
                Fighter b = Fighters[ib];
                if (a.Dead || b.Dead)
                    continue;

                Manifold m = Collide.CircleVsCircle(a.Body, b.Body);
                if (!m.Colliding)
                    continue;

                float wa = a.Faction == Faction.Player ? 0.15f : 0.5f;
                float wb = b.Faction == Faction.Player ? 0.15f : 0.5f;
                float sum = wa + wb;
                if (sum <= 0f) sum = 1f;
                Vec2 push = m.Normal * m.Depth;
                a.Pos -= push * (wa / sum);
                b.Pos += push * (wb / sum);
            }
        }
    }

    private void ClampToArena()
    {
        foreach (var f in Fighters)
        {
            f.Pos = new Vec2(
                Math.Clamp(f.Pos.X, f.Radius + 6f, ArenaWidth - f.Radius - 6f),
                Math.Clamp(f.Pos.Y, FloorTop, ArenaHeight - 12f));
        }
    }

    // ------------------------------------------------------------- effects

    private void UpdateEffects(float dt)
    {
        for (int i = Sparks.Count - 1; i >= 0; i--)
        {
            var s = Sparks[i];
            s.Age += dt;
            if (s.Age >= s.Life) { Sparks.RemoveAt(i); continue; }
            Sparks[i] = s;
        }
        for (int i = Numbers.Count - 1; i >= 0; i--)
        {
            var n = Numbers[i];
            n.Age += dt;
            n.Pos += n.Vel * dt;
            n.Vel *= MathF.Exp(-3f * dt);
            n.Vel += new Vec2(0, 26f * dt); // gentle gravity so it arcs
            if (n.Age >= n.Life) { Numbers.RemoveAt(i); continue; }
            Numbers[i] = n;
        }
        for (int i = Dusts.Count - 1; i >= 0; i--)
        {
            var d = Dusts[i];
            d.Age += dt;
            d.Pos += d.Vel * dt;
            d.Vel *= MathF.Exp(-4f * dt);
            if (d.Age >= d.Life) { Dusts.RemoveAt(i); continue; }
            Dusts[i] = d;
        }
    }

    private void SpawnSpark(Vec2 pos, float angle, bool heavy)
    {
        Sparks.Add(new HitSpark { Pos = pos, Life = heavy ? 0.32f : 0.22f, Angle = angle, Size = heavy ? 1.5f : 1f, Heavy = heavy });
    }

    private void SpawnNumber(Vec2 pos, int value, bool crit)
    {
        Numbers.Add(new DamageNumber
        {
            Pos = pos,
            Vel = new Vec2((float)(_rng.NextDouble() - 0.5) * 40f, -70f),
            Life = 0.8f,
            Value = value,
            Crit = crit,
        });
    }

    private void SpawnDust(Vec2 pos, Vec2 vel)
    {
        Dusts.Add(new Dust { Pos = pos, Vel = vel, Life = 0.4f + (float)_rng.NextDouble() * 0.2f, Size = 3f + (float)_rng.NextDouble() * 4f });
    }

    private Vec2 RandVec(float mag) =>
        new((float)(_rng.NextDouble() - 0.5) * 2f * mag, (float)(_rng.NextDouble() - 0.5) * 2f * mag);

    // ------------------------------------------------------------- lifecycle

    private void Cleanup()
    {
        for (int i = Fighters.Count - 1; i >= 0; i--)
        {
            var f = Fighters[i];
            if (f.Dead && f.DeathTimer <= 0f)
            {
                if (f.Faction == Faction.Player)
                    Fighters.RemoveAt(i);      // removed; respawn handled below
                else
                    Fighters.RemoveAt(i);
            }
        }
    }

    private void Spawning(float dt)
    {
        // player respawn / game over
        bool playerPresent = Fighters.Contains(Player);
        if (!playerPresent && !Player.Dead)
            playerPresent = false;
        if (Player.Dead && !Fighters.Contains(Player))
        {
            _respawnTimer -= dt;
            if (_respawnTimer <= 0f)
            {
                if (Lives > 0)
                    SpawnPlayer();
                else
                    GameOver = true;
            }
            return;
        }

        int alive = 0;
        foreach (var f in Fighters)
            if (f.Faction == Faction.Enemy && !f.Dead)
                alive++;

        _spawnTimer -= dt;
        int cap = 3 + Wave;
        if (alive < cap && _spawnTimer <= 0f)
        {
            SpawnEnemy();
            _spawnTimer = MathF.Max(0.4f, 1.5f - Wave * 0.09f);
        }
        if (alive == 0 && _spawnTimer <= 0f)
            Wave++;
    }

    private void SpawnEnemy()
    {
        int edge = _rng.Next(2);
        float y = FloorTop + (float)_rng.NextDouble() * (ArenaHeight - FloorTop - 20f);
        Vec2 pos = edge == 0 ? new Vec2(-20f, y) : new Vec2(ArenaWidth + 20f, y);

        bool brute = _rng.NextDouble() < 0.18 && Wave >= 1;
        var f = new Fighter
        {
            Faction = Faction.Enemy,
            Kind = brute ? EnemyKind.Brute : EnemyKind.Grunt,
            Pos = pos,
            Radius = brute ? 26f : 16f,
            Scale = brute ? 1.5f : 0.98f,
            Speed = brute ? 78f : 118f + Wave * 4f,
            MaxHealth = brute ? 160f : 46f,
            Health = brute ? 160f : 46f,
            Reach = brute ? 60f : 40f,
        };
        Fighters.Add(f);
    }
}
