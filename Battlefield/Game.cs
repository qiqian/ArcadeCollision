using System;
using System.Collections.Generic;

namespace ArcCollision.Battlefield;

/// <summary>
/// Combat-only translation of Quiver's Downtown Beatdown source. Source-space
/// values are mapped once from 1920x1080 by WorldScale; there is no vertical
/// viewport crop. Source-timed Tax Man VFX are rendered directly by GameForm
/// from attack/state time.
/// </summary>
internal sealed class Game
{
    public const float ArenaW = 1280f;
    public const float WorldScale = 2f / 3f;
    public const float WorldW = 16070f * WorldScale;
    // Walkable edges from Stage_01's CeilingLimits and BottomLimit collision
    // shapes (character radius is applied in ClampArena).
    public const float FloorY0 = 653f * WorldScale;
    public const float FloorY1 = 1107f * WorldScale;
    public const float XMin = 0f;
    public const float XMax = WorldW;

    private const float GravityUp = 3000f * WorldScale;
    private const float GravityDown = 3000f * 2.5f * WorldScale;
    private const float MaxLaunch = 2000f * WorldScale;
    private const float HitLane = 60f * WorldScale;
    private const float HitFreeze = 3f / 60f;
    private const float ArriveRange = 10f * WorldScale;

    public readonly List<Fighter> Fighters = new();
    public Fighter Player = null!;

    public bool GameOver, Win;
    public float Hitstop;

    internal sealed class FightRoom
    {
        public required float TriggerX;
        public required float Left, Top, Right, Bottom;
        public required SpawnData[][] Waves;
        public float Zoom = 1f;
        public float AfterLeft, AfterTop, AfterRight, AfterBottom, AfterZoom = 1f;
        public bool Boss, EngagesBoss;
        public bool Started, Cleared, WaveSpawned;
        public int WaveIndex;
        public float SpawnTimer;
    }

    internal readonly record struct SpawnData(Vec2 Start, Vec2 Target, bool WalkIn);

    public readonly List<FightRoom> Rooms = new();
    public FightRoom? ActiveRoom;
    public int RoomsCleared;
    public float CameraLeft, CameraRight = WorldW, CameraZoom = 1f;
    public float CameraTop = -280f * WorldScale, CameraBottom = 1200f * WorldScale;
    public float CamMinX => CameraLeft;
    public float CamMaxX => CameraRight;
    public float CamMinY => CameraTop;
    public float CamMaxY => CameraBottom;
    public Fighter TaxMan => _taxMan;

    private readonly SpatialHash _hash = new(80f);          // body capsules (separation)
    private readonly SpatialHash _hurtHash = new(160f);     // hurt boxes (hit broadphase)
    private readonly List<(int A, int B)> _bodyPairs = new();
    private readonly List<int> _hitCandidates = new();
    private int _bodyProxySlots;
    private int _hurtProxySlots;
    private readonly Random _rng = new();
    private Fighter _taxMan = null!;
    private Vec2 _move;
    // Presses are buffered for a few frames so an input during the jump impulse
    // or a hit-freeze isn't silently dropped (the buffer is consumed on use).
    private const float InputBufferTime = .15f;
    private float _attackBuffer, _jumpBuffer;

    public Game() => Reset();

    private static Vec2 P(float x, float y) => new(x * WorldScale, y * WorldScale);

    public void Reset()
    {
        Fighters.Clear();
        _hash.Clear();
        _hurtHash.Clear();
        _bodyProxySlots = _hurtProxySlots = 0;
        GameOver = Win = false;
        Hitstop = 0f;
        ActiveRoom = null;
        RoomsCleared = 0;
        CameraLeft = 0f;
        CameraRight = WorldW;
        CameraTop = -280f * WorldScale;
        CameraBottom = 1200f * WorldScale;
        CameraZoom = 1f;

        BuildStage();
        Player = new Fighter
        {
            Def = CharacterDef.Chad,
            Faction = Faction.Player,
            Pos = P(824, 800),
            Health = CharacterDef.Chad.MaxHealth,
        };
        Fighters.Add(Player);

        // stage_01.tscn contains Tax Man from scene load at (15208, 861).
        // He exists now but is not put in the active character list until the
        // guard room engages him; this preserves disabled collisions and lets
        // the existing renderer draw the seated presentation.
        _taxMan = new Fighter
        {
            Def = CharacterDef.TaxMan,
            Faction = Faction.Enemy,
            Pos = P(15208, 861),
            Health = CharacterDef.TaxMan.MaxHealth,
            Facing = -1f,
            CombatActive = false,
            TaxSeat = TaxSeatState.AwaitReveal,
            TaxHealthBaseline = CharacterDef.TaxMan.MaxHealth,
        };
    }

    private void BuildStage()
    {
        Rooms.Clear();
        static SpawnData S(float sx, float sy, float tx, float ty, bool walk = true) =>
            new(P(sx, sy), P(tx, ty), walk);

        Rooms.Add(new FightRoom
        {
            TriggerX = 1053 * WorldScale,
            Left = 267 * WorldScale,
            Top = -280 * WorldScale,
            Right = 2526 * WorldScale,
            Bottom = 1191 * WorldScale,
            Zoom = .849934f,
            AfterLeft = 267 * WorldScale,
            AfterTop = -280 * WorldScale,
            AfterRight = 5775 * WorldScale,
            AfterBottom = 1190 * WorldScale,
            Waves = new[] { new[] { S(2312, 794, 2312, 794, false) } },
        });
        Rooms.Add(new FightRoom
        {
            TriggerX = 4417 * WorldScale,
            Left = 2924 * WorldScale,
            Top = -280 * WorldScale,
            Right = 6259 * WorldScale,
            Bottom = 1195 * WorldScale,
            AfterLeft = 2924 * WorldScale,
            AfterTop = -280 * WorldScale,
            AfterRight = 8635 * WorldScale,
            AfterBottom = 1195 * WorldScale,
            Waves = new[]
            {
                new[] { S(6372, 790, 5656, 697), S(2755, 862, 3630, 1019) },
                new[]
                {
                    S(6372, 790, 5656, 697), S(6372, 790, 5726, 1011),
                    S(2755, 862, 3630, 1019), S(2755, 862, 3471, 704),
                },
            },
        });
        Rooms.Add(new FightRoom
        {
            TriggerX = 7525 * WorldScale,
            Left = 6838 * WorldScale,
            Top = -320 * WorldScale,
            Right = 9734 * WorldScale,
            Bottom = 1195 * WorldScale,
            Zoom = .9f,
            AfterLeft = 6838 * WorldScale,
            AfterTop = -320 * WorldScale,
            AfterRight = 15832 * WorldScale,
            AfterBottom = 1195 * WorldScale,
            Waves = new[]
            {
                new[]
                {
                    S(9906, 840, 9046, 696), S(9906, 840, 9182, 864),
                    S(9906, 840, 9306, 1004), S(9906, 840, 9326, 696),
                    S(9906, 840, 9462, 864), S(9906, 840, 9586, 1004),
                },
            },
        });
        Rooms.Add(new FightRoom
        {
            TriggerX = 13498 * WorldScale,
            Left = 13499 * WorldScale,
            Top = -320 * WorldScale,
            Right = 15815 * WorldScale,
            Bottom = 1195 * WorldScale,
            Zoom = .829016f,
            EngagesBoss = true,
            Waves = new[]
            {
                new[] { S(16019, 728, 14987, 620), S(16019, 728, 15247, 928) },
            },
        });
        Rooms.Add(new FightRoom
        {
            TriggerX = float.MaxValue,
            Left = 12587 * WorldScale,
            Top = -320 * WorldScale,
            Right = 15827 * WorldScale,
            Bottom = 1195 * WorldScale,
            Zoom = .712871f,
            Boss = true,
            Waves = Array.Empty<SpawnData[]>(),
        });
    }

    public void SetInput(Vec2 move, bool attack, bool jump)
    {
        _move = move;
        if (attack) _attackBuffer = InputBufferTime;
        if (jump) _jumpBuffer = InputBufferTime;
    }

    public void Update(float dt)
    {
        _attackBuffer = MathF.Max(0f, _attackBuffer - dt);
        _jumpBuffer = MathF.Max(0f, _jumpBuffer - dt);

        if (Hitstop > 0f)
        {
            Hitstop = MathF.Max(0f, Hitstop - dt);
            return;
        }

        // QuiverActionKnockoutLaunch slows the tree to 0.2 when the player dies.
        if (!Player.Alive && !Player.Dead)
            dt *= .2f;

        UpdateHiddenTax(dt);
        if (!GameOver)
        {
            UpdatePlayer();
            UpdateEnemies(dt);
        }

        AdvanceStates(dt);
        Integrate(dt);
        ResolveBodies();
        ClampArena();
        ResolveHits();
        Cleanup();
        StageUpdate(dt);
    }

    // ---------------------------------------------------------------- player

    private void UpdatePlayer()
    {
        Fighter p = Player;
        if (p.Dead || !p.CombatActive) return;

        if (p.State == FState.Jump)
        {
            if (p.JumpLanding || !p.JumpLaunched) return;
            ApplyAirControl(p);
            if (_attackBuffer > 0f && !p.AirAttackUsed && p.Def.AirAttack != null)
            {
                _attackBuffer = 0f;
                StartAirAttack(p);
            }
            return;
        }
        if (p.State == FState.AirAttack)
            return;

        if (p.State == FState.Attack && p.Scripted == ScriptedState.None)
        {
            if (_attackBuffer > 0f && p.Attack != null && p.Attack.ComboNext >= 0
                && p.AttackTime <= p.Attack.ComboWindow)
            {
                _attackBuffer = 0f;
                p.ComboQueued = true;
            }
            return;
        }

        if (!p.CanAct) return;
        Vec2 direction = _move.Normalized(Vec2.Zero);
        if (MathF.Abs(direction.X) > .01f)
            Face(p, MathF.Sign(direction.X));
        p.Vel = direction * p.Def.Speed * (p.TurnTimer > 0f ? .6f : 1f);
        SetLocomotion(p, direction.LengthSquared > .001f ? FState.Walk : FState.Idle);

        if (_jumpBuffer > 0f)
        {
            _jumpBuffer = 0f;
            StartJump(p);
        }
        else if (_attackBuffer > 0f)
        {
            _attackBuffer = 0f;
            StartAttack(p, 0, 1);
        }
    }

    private void ApplyAirControl(Fighter fighter)
    {
        float input = MathF.Sign(_move.X);
        if (input == 0f) return;
        float max = fighter.Def.Speed * fighter.Def.AirControl;
        bool opposite = MathF.Sign(fighter.Vel.X) != input;
        if (opposite || MathF.Abs(fighter.Vel.X) <= max)
            fighter.Vel = new Vec2(input * max, 0f);
        Face(fighter, input);
    }

    private void StartJump(Fighter fighter)
    {
        fighter.SetState(FState.Jump);
        fighter.Vel = new Vec2(fighter.Vel.X, 0f);
        fighter.SkinY = 0f;
        fighter.SkinVelY = 0f;
        fighter.JumpLaunched = false;
        fighter.JumpLanding = false;
        fighter.AirAttackUsed = false;
    }

    private void StartAirAttack(Fighter fighter)
    {
        fighter.SetState(FState.AirAttack);
        fighter.Attack = fighter.Def.AirAttack;
        fighter.AttackTime = 0f;
        fighter.AirAttackUsed = true;
        fighter.ResetAttackHits();
    }

    // --------------------------------------------------------------- enemy AI

    private void UpdateEnemies(float dt)
    {
        if (Player.Dead) return;
        foreach (Fighter fighter in Fighters)
        {
            if (fighter.Faction != Faction.Enemy || fighter.Dead || !fighter.CombatActive)
                continue;

            // Action states have priority over the spawn walk. Otherwise a hit
            // during walk-in is overwritten by Walk on the next frame, which
            // can strand a zero-health Sarge outside the KO/death pipeline.
            if (!fighter.CanAct) continue;

            if (fighter.SpawnWalking)
            {
                Vec2 delta = fighter.SpawnTarget - fighter.Pos;
                if (delta.LengthSquared <= ArriveRange * ArriveRange)
                {
                    fighter.Pos = fighter.SpawnTarget;
                    fighter.SpawnWalking = false;
                    EnterAiWait(fighter);
                }
                else
                {
                    Face(fighter, MathF.Sign(delta.X));
                    AiWalk(fighter, delta);
                }
                continue;
            }

            switch (fighter.Ai)
            {
                case AiState.Wait:
                    AiIdle(fighter);
                    fighter.AiTimer -= dt;
                    if (fighter.AiTimer <= 0f)
                        AiDecide(fighter);
                    break;

                case AiState.Chase:
                    UpdateChase(fighter, dt);
                    break;

                case AiState.PreAttackPause:
                    AiIdle(fighter);
                    fighter.AiTimer -= dt;
                    if (fighter.AiTimer <= 0f)
                    {
                        fighter.Ai = AiState.SingleAttack;
                        StartAttack(fighter, 0, 1);
                    }
                    break;

                case AiState.ComboPause:
                    AiIdle(fighter);
                    fighter.AiTimer -= dt;
                    if (fighter.AiTimer <= 0f)
                    {
                        fighter.Ai = AiState.Attacking;
                        StartAttack(fighter, 0, 3);
                    }
                    break;

                case AiState.MoveToDashMarker:
                {
                    fighter.AiTimer -= dt;
                    Vec2 delta = fighter.AiTarget - fighter.Pos;
                    float span = fighter.Def.BodyHalfSpine + fighter.Def.Radius;
                    (float screenLeft, float screenRight) = GetScreenXBounds();
                    bool blockedByScreenWall =
                        (delta.X < 0f && fighter.Pos.X <= screenLeft + span + ArriveRange)
                        || (delta.X > 0f && fighter.Pos.X >= screenRight - span - ArriveRange);
                    bool arrived = delta.LengthSquared <= ArriveRange * ArriveRange;
                    // The timeout mirrors the chase timeout: with body separation a
                    // blocking player could otherwise pin the boss short of the
                    // marker indefinitely.
                    if (arrived || blockedByScreenWall || fighter.AiTimer <= 0f)
                    {
                        if (arrived)
                            fighter.Pos = fighter.AiTarget;
                        fighter.Ai = AiState.Align;
                        fighter.AiTimer = 5f;
                    }
                    else
                    {
                        Face(fighter, MathF.Sign(delta.X));
                        AiWalk(fighter, delta);
                    }
                    break;
                }

                case AiState.Align:
                {
                    fighter.AiTimer -= dt;
                    float dy = Player.Pos.Y - fighter.Pos.Y;
                    if (MathF.Abs(dy) <= ArriveRange || fighter.AiTimer <= 0f)
                    {
                        Face(fighter, MathF.Sign(Player.Pos.X - fighter.Pos.X));
                        StartDash(fighter);
                    }
                    else
                        AiWalk(fighter, new Vec2(0f, dy));
                    break;
                }
            }
        }
    }

    private void UpdateChase(Fighter fighter, float dt)
    {
        fighter.AiTimer -= dt;
        float offset = (fighter.Def.SpriteSet == "taxman" ? 280f : 230f) * WorldScale;
        float side = fighter.Pos.X >= Player.Pos.X ? 1f : -1f;
        Vec2 target = Player.Pos + new Vec2(side * offset, 0f);
        Vec2 delta = target - fighter.Pos;
        bool arrived = MathF.Abs(fighter.Pos.Y - Player.Pos.Y) <= HitLane
            && delta.LengthSquared <= ArriveRange * ArriveRange;

        if (!arrived && fighter.AiTimer > 0f)
        {
            Face(fighter, MathF.Sign(Player.Pos.X - fighter.Pos.X));
            AiWalk(fighter, delta);
            return;
        }

        AiIdle(fighter);
        if (fighter.Def.SpriteSet == "sarge")
        {
            // sarge.tscn Chase sequence: Chase -> fixed 0.2 wait -> Attack1.
            fighter.Ai = AiState.PreAttackPause;
            fighter.AiTimer = .2f;
        }
        else
        {
            fighter.Ai = AiState.Attacking;
            if (fighter.AiAttackChoice == 2) StartArea(fighter);
            else StartAttack(fighter, 0, 1);
        }
    }

    private void AiDecide(Fighter fighter)
    {
        if (fighter.Def.SpriteSet != "taxman")
        {
            fighter.Ai = AiState.Chase;
            fighter.AiTimer = 5f;
            return;
        }

        double combo, dash, area;
        if (fighter.BossPhase == 0)
            (combo, dash, area) = (1.40816, .591839, 0);
        else if (fighter.BossPhase == 1)
            (combo, dash, area) = (1.19884, 1.19315, .60801);
        else
            (combo, dash, area) = (.608845, 1.49911, .892046);

        double draw = _rng.NextDouble() * (combo + dash + area);
        if (draw < combo)
        {
            fighter.AiAttackChoice = 0;
            fighter.Ai = AiState.Chase;
            fighter.AiTimer = 5f;
        }
        else if (draw < combo + dash)
        {
            fighter.AiAttackChoice = 1;
            Vec2 left = P(12866, 763);
            Vec2 right = P(15014, 772);
            fighter.AiTarget = fighter.Pos.DistanceSquared(left) <= fighter.Pos.DistanceSquared(right)
                ? left : right;
            fighter.Ai = AiState.MoveToDashMarker;
            fighter.AiTimer = 5f;
        }
        else
        {
            fighter.AiAttackChoice = 2;
            fighter.Ai = AiState.Chase;
            fighter.AiTimer = 5f;
        }
    }

    private void EnterAiWait(Fighter fighter)
    {
        fighter.Ai = AiState.Wait;
        if (fighter.Def.SpriteSet != "taxman")
            fighter.AiTimer = 1f + (float)_rng.NextDouble() * 2f;
        else if (fighter.BossPhase == 0)
            fighter.AiTimer = 1f;
        else if (fighter.BossPhase == 1)
            fighter.AiTimer = .5f + (float)_rng.NextDouble() * .5f;
        else
            fighter.AiTimer = .2f + (float)_rng.NextDouble() * .5f;
    }

    private static void AiIdle(Fighter fighter)
    {
        fighter.Vel = Vec2.Zero;
        SetLocomotion(fighter, FState.Idle);
    }

    private static void AiWalk(Fighter fighter, Vec2 delta)
    {
        Vec2 direction = delta.Normalized(Vec2.Zero);
        float modifier = fighter.TurnTimer > 0f ? .6f : 1f;
        fighter.Vel = direction * fighter.Def.Speed * modifier;
        SetLocomotion(fighter, direction.LengthSquared > .001f ? FState.Walk : FState.Idle);
    }

    private static void SetLocomotion(Fighter fighter, FState state)
    {
        if (fighter.State != state)
            fighter.SetState(state);
    }

    private static void Face(Fighter fighter, float facing)
    {
        if (facing == 0f || facing == fighter.Facing) return;
        fighter.Facing = facing;
        fighter.TurnTimer = fighter.Def.SpriteSet == "sarge" ? .25f : .125f;
    }

    // ---------------------------------------------------------------- attacks

    private void StartAttack(Fighter fighter, int index, int autoCombo)
    {
        fighter.SetState(FState.Attack);
        fighter.ComboIndex = index;
        fighter.Attack = fighter.Def.Combo[index];
        fighter.AttackTime = 0f;
        fighter.ComboQueued = false;
        fighter.AutoCombo = autoCombo;
        fighter.Scripted = ScriptedState.None;
        fighter.ResetAttackHits();
        BeginAttackFlags(fighter);
    }

    private void StartDash(Fighter fighter)
    {
        fighter.SetState(FState.Attack);
        fighter.Attack = fighter.Def.Dash;
        fighter.AttackTime = 0f;
        fighter.DashConnected = false;
        fighter.Ai = AiState.DashCharging;
        fighter.Scripted = ScriptedState.None;
        fighter.ResetAttackHits();
        BeginAttackFlags(fighter);
    }

    private void StartArea(Fighter fighter)
    {
        fighter.SetState(FState.Attack);
        fighter.Attack = fighter.Def.Area;
        fighter.AttackTime = 0f;
        fighter.Ai = AiState.Attacking;
        fighter.Scripted = ScriptedState.None;
        fighter.ResetAttackHits();
        BeginAttackFlags(fighter);
    }

    private void StartRetaliate(Fighter fighter)
    {
        fighter.SetState(FState.Attack);
        fighter.Attack = fighter.Def.Retaliate;
        fighter.AttackTime = 0f;
        fighter.Ai = AiState.Attacking;
        fighter.Scripted = ScriptedState.None;
        fighter.ResetAttackHits();
        BeginAttackFlags(fighter);
    }

    private static void BeginAttackFlags(Fighter fighter)
    {
        if (fighter.Def.SpriteSet == "taxman")
            fighter.Invuln = 0f; // Tax attack animations explicitly clear invulnerability at frame 0.
        bool armor = fighter.Attack?.HasSuperArmorAt(0f) == true;
        if (armor && !fighter.HasSuperArmor)
            fighter.KnockAmount = 0f;
        fighter.HasSuperArmor = armor;
        if (fighter.Attack?.IsInvulnerableAt(0f) == true)
            fighter.KnockAmount = 0f;
    }

    // ----------------------------------------------------------------- states

    private void AdvanceStates(float dt)
    {
        foreach (Fighter fighter in Fighters)
        {
            fighter.StateTime += dt;
            if (fighter.Invuln > 0f && fighter.Invuln < 900f)
                fighter.Invuln = MathF.Max(0f, fighter.Invuln - dt);
            if (fighter.HurtFlash > 0f)
                fighter.HurtFlash = MathF.Max(0f, fighter.HurtFlash - dt);
            if (fighter.TurnTimer > 0f)
                fighter.TurnTimer = MathF.Max(0f, fighter.TurnTimer - dt);

            switch (fighter.State)
            {
                case FState.Jump:
                    if (!fighter.JumpLaunched && !fighter.JumpLanding
                        && fighter.StateTime >= fighter.Def.JumpImpulseTime)
                    {
                        fighter.JumpLaunched = true;
                        fighter.SkinVelY = fighter.Def.JumpSpeed;
                    }
                    else if (fighter.JumpLanding)
                    {
                        fighter.LandedTimer -= dt;
                        if (fighter.LandedTimer <= 0f)
                        {
                            fighter.JumpLanding = false;
                            fighter.SetState(_move.LengthSquared > .001f ? FState.Walk : FState.Idle);
                        }
                    }
                    break;

                case FState.Attack:
                    AdvanceAttack(fighter, dt);
                    break;

                case FState.AirAttack:
                    fighter.AttackTime += dt;
                    break;

                case FState.Hurt:
                    fighter.HurtTimer -= dt;
                    if (fighter.HurtTimer <= 0f)
                    {
                        if (fighter.Def.SpriteSet == "taxman")
                            FinishTaxHurt(fighter);
                        else
                        {
                            fighter.SetState(FState.Idle);
                            if (fighter.Faction == Faction.Enemy
                                && fighter.Ai == AiState.SingleAttack)
                            {
                                fighter.Ai = AiState.ComboPause;
                                fighter.AiTimer = .5f;
                            }
                            else if (fighter.Faction == Faction.Enemy
                                && fighter.Ai == AiState.Attacking)
                                EnterAiWait(fighter);
                        }
                    }
                    break;

                case FState.Landed:
                    fighter.LandedTimer -= dt;
                    if (fighter.LandedTimer <= 0f)
                    {
                        if (fighter.Def.SpriteSet == "taxman")
                        {
                            fighter.Invuln = 0f;
                            if (fighter.Alive)
                            {
                                fighter.TaxPendingArea = false;
                                StartArea(fighter);
                            }
                            else EnterDeath(fighter);
                        }
                        else
                            fighter.SetState(fighter.Alive ? FState.GetUp : FState.Dead);
                    }
                    break;

                case FState.GetUp:
                    if (fighter.StateTime >= fighter.Def.GetUpTime)
                        fighter.SetState(FState.Idle);
                    break;

                case FState.Dead:
                    fighter.DeathTimer -= dt;
                    break;
            }
        }
    }

    private void AdvanceAttack(Fighter fighter, float dt)
    {
        AttackData? attack = fighter.Attack;
        if (attack == null) return;

        if (attack == fighter.Def.Dash && !fighter.DashConnected)
            fighter.AttackTime = MathF.Min(.833333f, fighter.AttackTime + dt);
        else
            fighter.AttackTime += dt;

        bool armor = attack.HasSuperArmorAt(fighter.AttackTime);
        if (armor && !fighter.HasSuperArmor)
            fighter.KnockAmount = 0f;
        fighter.HasSuperArmor = armor;

        if (fighter.Scripted != ScriptedState.None)
        {
            if (fighter.AttackTime < attack.Total) return;
            if (fighter.Scripted == ScriptedState.Bounce)
            {
                if (fighter.Alive)
                {
                    fighter.SetState(FState.Landed);
                    fighter.LandedTimer = fighter.Def.LandedTime;
                }
                else
                    EnterDeath(fighter);
            }
            else
                FinishTaxHurtKnockout(fighter);
            return;
        }

        if (attack.ComboNext >= 0 && fighter.AttackTime >= attack.ComboWindow)
        {
            bool chain = fighter.Faction == Faction.Player
                ? fighter.ComboQueued
                : fighter.AutoCombo > 1;
            if (chain)
            {
                int remaining = fighter.Faction == Faction.Player ? 1 : fighter.AutoCombo - 1;
                StartAttack(fighter, attack.ComboNext, remaining);
                return;
            }
        }

        if (fighter.AttackTime >= attack.Total)
            FinishAttack(fighter);
    }

    private void FinishAttack(Fighter fighter)
    {
        fighter.SetState(FState.Idle);
        if (fighter.Faction != Faction.Enemy) return;

        if (fighter.Def.SpriteSet == "sarge" && fighter.Ai == AiState.SingleAttack)
        {
            fighter.Ai = AiState.ComboPause;
            fighter.AiTimer = .5f;
        }
        else
        {
            if (fighter.Def.SpriteSet == "taxman")
                fighter.ConsecutiveHits = 0;
            EnterAiWait(fighter);
        }
    }

    private void EnterHurt(Fighter target, HitWindow hit)
    {
        target.SetState(FState.Hurt);
        target.HurtHigh = hit.Hurt == HurtType.High;
        target.HurtTimer = .5f;
        target.Vel = Vec2.Zero;
    }

    private void EnterTaxHurt(Fighter target, HitWindow hit)
    {
        target.TaxCumulatedDamage +=
            (target.TaxHealthBaseline - target.Health) / (float)target.MaxHealth;
        float previous = target.TaxHealthBaseline / (float)target.MaxHealth;
        float current = target.Health / (float)target.MaxHealth;

        bool changedPhase = false;
        if (target.Alive && target.BossPhase == 0 && previous > .5f && current <= .5f)
        {
            target.BossPhase = 1;
            changedPhase = true;
        }
        else if (target.Alive && target.BossPhase == 1 && previous > .15f && current <= .15f)
        {
            target.BossPhase = 2;
            changedPhase = true;
        }

        target.ConsecutiveHits++;
        if (target.ConsecutiveHits >= 7)
        {
            target.Invuln = 999f;
            target.KnockAmount = 0f;
        }

        if (!target.Alive || changedPhase || target.TaxCumulatedDamage > .1f)
        {
            if (changedPhase)
            {
                target.TaxPendingArea = true;
                ResetTaxDamage(target);
            }
            target.SetState(FState.Attack);
            target.Attack = CharacterDef.TaxHurtKnockout;
            target.AttackTime = 0f;
            target.Scripted = ScriptedState.TaxHurtKnockout;
            target.ResetAttackHits();
            target.KnockAmount = 0f;
            target.HasSuperArmor = true;
            return;
        }

        target.SetState(FState.Hurt);
        target.HurtHigh = target.TaxCumulatedDamage > .05f;
        target.HurtTimer = .833334f;
        target.Vel = Vec2.Zero;
    }

    private void FinishTaxHurt(Fighter target)
    {
        bool retaliate = target.TaxCumulatedDamage >= .1f;
        ResetTaxDamage(target);
        if (retaliate) StartRetaliate(target);
        else
        {
            target.SetState(FState.Idle);
            if (target.ConsecutiveHits >= 7)
            {
                // TaxManAiStateMachine.WaitForIdle clears the temporary
                // seven-hit invulnerability before resuming the chosen attack.
                target.Invuln = 0f;
                AiDecide(target);
            }
            else ResumeTaxPlan(target);
        }
    }

    // TaxManAiStateMachine resumes _state_to_resume after WaitForIdle: an
    // interrupted approach (chase, dash setup) restarts rather than re-rolling
    // the attack choice. The resumed state re-enters with a fresh timeout.
    private void ResumeTaxPlan(Fighter target)
    {
        switch (target.Ai)
        {
            case AiState.Chase:
            case AiState.MoveToDashMarker:
            case AiState.Align:
                target.AiTimer = 5f;
                break;
            default:
                EnterAiWait(target);
                break;
        }
    }

    private void FinishTaxHurtKnockout(Fighter target)
    {
        bool kneeled = !target.Alive || target.TaxPendingArea;
        if (kneeled)
        {
            target.SetState(FState.Landed);
            target.LandedTimer = 1f;
            target.Invuln = 1f;
            target.HasSuperArmor = true;
        }
        else
        {
            ResetTaxDamage(target);
            StartRetaliate(target);
        }
    }

    private static void ResetTaxDamage(Fighter target)
    {
        target.TaxCumulatedDamage = 0f;
        target.TaxHealthBaseline = target.Health;
    }

    private void EnterKo(Fighter target, Fighter attacker, HitWindow hit)
    {
        Vec2 direction = AttackData.LaunchDir(hit.LaunchAngleDeg);
        float horizontal = attacker.Pos.X <= target.Pos.X ? 1f : -1f;
        Vec2 current = new(target.Vel.X, target.SkinVelY);
        Vec2 added = new Vec2(direction.X * horizontal, direction.Y)
            * (target.KnockAmount * WorldScale);
        Vec2 velocity = current + added;
        if (velocity.Length > MaxLaunch)
            velocity = velocity.Normalized(Vec2.Zero) * MaxLaunch;

        target.SetState(FState.Ko);
        target.Vel = new Vec2(velocity.X, 0f);
        target.SkinVelY = velocity.Y;
        target.KnockAmount = 0f;
        target.Bounces = 0;
        target.JumpLanding = false;
        target.InThroneBounce = false;
        target.WallContact = 0;
        if (target.Faction == Faction.Enemy)
            EnterAiWait(target);
    }

    private void EnterBounce(Fighter fighter)
    {
        fighter.SkinY = 0f;
        fighter.SkinVelY = 0f;
        fighter.Vel = Vec2.Zero;
        fighter.Bounces++;
        fighter.SetState(FState.Attack);
        fighter.Attack = new AttackData { Clip = "ko_bounce", Total = fighter.Def.BounceTime };
        fighter.AttackTime = 0f;
        fighter.Scripted = ScriptedState.Bounce;
    }

    private static void EnterDeath(Fighter fighter)
    {
        fighter.SetState(FState.Dead);
        fighter.DeathTimer = fighter.Def.SpriteSet == "taxman" ? 4.54167f : .5f;
        fighter.CombatActive = false;
        fighter.Vel = Vec2.Zero;
        fighter.SkinY = 0f;
        fighter.SkinVelY = 0f;
    }

    // ---------------------------------------------------------------- physics

    private void Integrate(float dt)
    {
        foreach (Fighter fighter in Fighters)
        {
            switch (fighter.State)
            {
                case FState.Idle:
                case FState.Walk:
                    fighter.Pos += fighter.Vel * dt;
                    break;

                case FState.Attack when fighter.Attack != null:
                    if (fighter.Attack == fighter.Def.Dash)
                        IntegrateDash(fighter, dt);
                    else if (fighter.AttackTime >= fighter.Attack.LungeStart
                        && fighter.AttackTime < fighter.Attack.LungeEnd)
                        fighter.Pos += new Vec2(fighter.Facing * fighter.Attack.LungeSpeed, 0f) * dt;
                    break;

                case FState.Jump:
                    if (!fighter.JumpLaunched || fighter.JumpLanding) break;
                    IntegrateJump(fighter, dt);
                    break;

                case FState.AirAttack:
                    // Chad's air attack is configured with DISTANCE_FROM_GROUND(250),
                    // but the source compares a negative in-air skin y against +250 —
                    // the condition can never fire, so the attack (and its hitbox,
                    // enabled from 0.125s) effectively lasts until landing.
                    // IntegrateJump's ground contact handles the landing transition.
                    IntegrateJump(fighter, dt);
                    break;

                case FState.Ko:
                    fighter.Pos += new Vec2(fighter.Vel.X, 0f) * dt;
                    fighter.SkinY += fighter.SkinVelY * dt;
                    fighter.SkinVelY -= (fighter.SkinVelY > 0f ? GravityUp : GravityDown) * dt;
                    HandleThroneBounce(fighter);
                    if (fighter.SkinY <= 0f && fighter.SkinVelY < 0f)
                        EnterBounce(fighter);
                    break;
            }
        }
    }

    private void IntegrateJump(Fighter fighter, float dt)
    {
        fighter.Pos += new Vec2(fighter.Vel.X, 0f) * dt;
        fighter.SkinY += fighter.SkinVelY * dt;
        fighter.SkinVelY -= (fighter.SkinVelY > 0f ? GravityUp : GravityDown) * dt;
        if (fighter.SkinY <= 0f && fighter.SkinVelY < 0f)
        {
            fighter.SkinY = 0f;
            fighter.SkinVelY = 0f;
            fighter.Vel = Vec2.Zero;
            fighter.SetState(FState.Jump);
            fighter.JumpLaunched = true;
            fighter.JumpLanding = true;
            fighter.StateTime = MathF.Max(fighter.Def.JumpImpulseTime, fighter.StateTime);
            fighter.LandedTimer = .125f;
        }
    }

    private void IntegrateDash(Fighter fighter, float dt)
    {
        if (!fighter.DashConnected)
        {
            if (fighter.AttackTime >= .166667f)
                fighter.Pos += new Vec2(fighter.Facing * 2700f * WorldScale, 0f) * dt;

            // dash_attack_begin's animated obstacle detector.
            Vec2 detectorCenter = fighter.Pos
                + new Vec2(fighter.Facing * 449.5f * WorldScale, -256f * WorldScale);
            Aabb detector = new(detectorCenter, P(663f * .5f, 514.75f * .5f));
            bool contactedPlayer = fighter.AttackTime >= .208334f
                && Collide.CapsuleVsAabb(Player.Body, detector).Colliding;
            float span = fighter.Def.BodyHalfSpine + fighter.Def.Radius;
            (float left, float right) = GetScreenXBounds();
            // DashObstacleDetector collides with QuiverLevelCamera's moving
            // screen-limit bodies, not the wider fight-room limits.
            bool contactedWall = fighter.Pos.X - span <= left || fighter.Pos.X + span >= right;
            if (contactedPlayer || contactedWall)
            {
                fighter.DashConnected = true;
                fighter.AttackTime = .833333f;
            }
        }
        else if (fighter.AttackTime < 1.25f)
            fighter.Pos += new Vec2(fighter.Facing * 2700f * WorldScale, 0f) * dt;
    }

    private void HandleThroneBounce(Fighter fighter)
    {
        if (ActiveRoom?.Boss != true)
        {
            fighter.InThroneBounce = false;
            return;
        }

        // stage_01/ThroneBounce: Area (15127,537) + shape (220,-56),
        // RectangleShape2D(124,1688).
        Aabb throne = new(P(15347, 481), P(62, 844));
        bool overlapping = BoundsOf(fighter.CurrentHurtShape()).Overlaps(throne);
        if (overlapping && !fighter.InThroneBounce)
        {
            fighter.Health = Math.Max(0, fighter.Health - 5);
            fighter.Vel = new Vec2(-fighter.Vel.X, 0f);
            fighter.StateTime = 0f;
            Hitstop = HitFreeze;
        }
        fighter.InThroneBounce = overlapping;
    }

    private void ResolveBodies()
    {
        for (int i = 0; i < Fighters.Count; i++)
        {
            Fighter fighter = Fighters[i];
            // Source characters never body-collide (collision_mask covers only
            // obstacles/limits). Separation here is a port nicety for combat, so
            // walk-in spawns must be exempt: crossing entrance paths from a shared
            // spawn point would otherwise push each other away from their markers
            // forever and stall the wave.
            if (!fighter.Dead && fighter.CombatActive && !fighter.SpawnWalking)
                _hash.Update(i, fighter.Body);
            else
                _hash.Remove(i);
        }
        for (int i = Fighters.Count; i < _bodyProxySlots; i++)
            _hash.Remove(i);
        _bodyProxySlots = Fighters.Count;

        _hash.ComputePairs(_bodyPairs);
        for (int pairIndex = 0; pairIndex < _bodyPairs.Count; pairIndex++)
        {
            (int ia, int ib) = _bodyPairs[pairIndex];
            Fighter a = Fighters[ia], b = Fighters[ib];
            if (a.Dead || b.Dead || !a.CombatActive || !b.CombatActive) continue;
            Manifold manifold = Collide.CapsuleVsCapsule(a.Body, b.Body);
            if (!manifold.Colliding) continue;
            Vec2 separation = manifold.Normal * (manifold.Depth * .5f);
            a.Pos -= separation;
            b.Pos += separation;
        }
    }

    private void ClampArena()
    {
        (float screenLeft, float screenRight) = GetScreenXBounds();
        foreach (Fighter fighter in Fighters)
        {
            float span = fighter.Def.BodyHalfSpine + fighter.Def.Radius;
            // QuiverLevelCamera carries the left/right one-way collision walls.
            // Walk-in spawns stay exempt until they reach their marker.
            float left = fighter.SpawnWalking ? XMin : screenLeft;
            float right = fighter.SpawnWalking ? XMax : screenRight;
            // The second ceiling polygon opens the Tax Man district upward.
            float topEdge = fighter.Pos.X >= 10784f * WorldScale
                ? 575f * WorldScale
                : FloorY0;
            float minX = left + span;
            float maxX = right - span;
            int wall = fighter.Pos.X < minX ? -1 : fighter.Pos.X > maxX ? 1 : 0;
            if (fighter.State == FState.Ko && wall != 0 && wall != fighter.WallContact)
            {
                // QuiverLevelCamera's matching WallHitBox applies five damage
                // and relaunches by reflecting the current horizontal velocity.
                fighter.Health = Math.Max(0, fighter.Health - 5);
                fighter.Vel = new Vec2(-fighter.Vel.X, 0f);
                fighter.StateTime = 0f;
                Hitstop = HitFreeze;
            }
            fighter.WallContact = wall;
            fighter.Pos = new Vec2(
                Math.Clamp(fighter.Pos.X, minX, maxX),
                Math.Clamp(fighter.Pos.Y, topEdge + fighter.Def.Radius,
                    FloorY1 - fighter.Def.Radius));
        }
    }

    private (float Left, float Right) GetScreenXBounds()
    {
        float visibleWidth = ArenaW / MathF.Max(.1f, CameraZoom);
        float cameraMax = MathF.Max(CameraLeft, CameraRight - visibleWidth);
        float left = Math.Clamp(Player.Pos.X - visibleWidth * .5f, CameraLeft, cameraMax);
        return (left, left + visibleWidth);
    }

    // ------------------------------------------------------------------ hits

    private void ResolveHits()
    {
        // Keep dynamic proxies alive across frames; fat AABBs avoid tree
        // reinsertion while fighters move within their predicted margin.
        for (int i = 0; i < Fighters.Count; i++)
        {
            Fighter target = Fighters[i];
            if (target.Dead || !target.CombatActive || target.IsInvulnerable)
            {
                _hurtHash.Remove(i);
                continue;
            }
            BoxShape hurt = target.CurrentHurtShape();
            _hurtHash.Update(i, new Obb(hurt.Center, hurt.HalfSize, hurt.Rotation));
        }
        for (int i = Fighters.Count; i < _hurtProxySlots; i++)
            _hurtHash.Remove(i);
        _hurtProxySlots = Fighters.Count;
        if (_hurtHash.DynamicCount == 0) return;

        for (int attackerIndex = 0; attackerIndex < Fighters.Count; attackerIndex++)
        {
            Fighter attacker = Fighters[attackerIndex];
            if (!attacker.CombatActive
                || attacker.State is not (FState.Attack or FState.AirAttack)
                || attacker.Attack == null
                || !attacker.Attack.TryWindowAt(attacker.AttackTime, out HitWindow hit))
                continue;

            Vec2 hitCenter = HitCenter(attacker, hit);
            if (hit.ShapeKind == HitShapeKind.HorizontalCapsule)
            {
                float halfSpine = MathF.Max(0f, hit.CapsuleHeight * .5f - hit.CapsuleRadius);
                _hurtHash.Query(new Capsule(
                    hitCenter - new Vec2(halfSpine, 0f),
                    hitCenter + new Vec2(halfSpine, 0f),
                    hit.CapsuleRadius), _hitCandidates);
            }
            else
            {
                _hurtHash.Query(new Obb(hitCenter, hit.Box.HalfSize,
                    hit.Box.Rotation * attacker.Facing), _hitCandidates);
            }

            for (int candidateIndex = 0; candidateIndex < _hitCandidates.Count; candidateIndex++)
            {
                int index = _hitCandidates[candidateIndex];
                Fighter target = Fighters[index];
                if (target == attacker || target.Faction == attacker.Faction)
                    continue;
                // Re-check state that an earlier hit this frame may have changed
                // (the broadphase snapshot is from the start of the pass).
                if (target.Dead || !target.CombatActive || target.IsInvulnerable)
                    continue;
                if (MathF.Abs(attacker.Pos.Y - target.Pos.Y) > HitLane)
                    continue;
                BoxShape hurt = target.CurrentHurtShape();
                if (!HitOverlaps(attacker, hit, hurt))
                    continue;
                if (attacker.WasHitBy(target, hit.HitId))
                    continue;
                ApplyHit(attacker, target, hit);
            }
        }
    }

    private static Vec2 HitCenter(Fighter attacker, in HitWindow hit) =>
        attacker.Pos + new Vec2(attacker.Facing * hit.Box.Center.X, hit.Box.Center.Y - attacker.SkinY);

    private static bool HitOverlaps(Fighter attacker, HitWindow hit, BoxShape hurt)
    {
        Vec2 center = HitCenter(attacker, hit);
        if (hit.ShapeKind == HitShapeKind.HorizontalCapsule)
        {
            float halfSpine = MathF.Max(0f, hit.CapsuleHeight * .5f - hit.CapsuleRadius);
            Capsule capsule = new(center - new Vec2(halfSpine, 0f),
                center + new Vec2(halfSpine, 0f), hit.CapsuleRadius);
            if (MathF.Abs(hurt.Rotation) < .0001f)
                return Collide.CapsuleVsAabb(capsule, new Aabb(hurt.Center, hurt.HalfSize)).Colliding;
            return capsule.Bounds.Overlaps(BoundsOf(hurt));
        }

        BoxShape attack = new(center, hit.Box.HalfSize, hit.Box.Rotation * attacker.Facing);
        return OrientedBoxesOverlap(attack, hurt);
    }

    private void ApplyHit(Fighter attacker, Fighter target, HitWindow hit)
    {
        // CombatSystem order: damage (which starts HitFreeze), then knockback.
        target.Health = Math.Max(0, target.Health - (int)hit.Damage);
        target.HurtFlash = .14f;
        Hitstop = HitFreeze;

        target.KnockAmount += (int)hit.Knockback;
        bool armor = target.HasSuperArmor;

        if (target == Player && _taxMan.TaxSeat is TaxSeatState.Swirl
                or TaxSeatState.Drink or TaxSeatState.Laugh)
        {
            _taxMan.TaxSeat = TaxSeatState.Laugh;
            _taxMan.TaxSeatTimer = 4.08334f;
        }

        if (target.Def.SpriteSet == "taxman")
        {
            if (!target.Alive || !armor)
                EnterTaxHurt(target, hit);
            return;
        }

        if (target.Airborne && !armor)
            EnterKo(target, attacker, hit);
        else if (target.ShouldKnockout)
            EnterKo(target, attacker, hit);
        else if (!armor)
            EnterHurt(target, hit);
    }

    private static bool OrientedBoxesOverlap(BoxShape a, BoxShape b)
    {
        GetAxes(a.Rotation, out Vec2 ax, out Vec2 ay);
        GetAxes(b.Rotation, out Vec2 bx, out Vec2 by);
        return OverlapOn(ax, a, b) && OverlapOn(ay, a, b)
            && OverlapOn(bx, a, b) && OverlapOn(by, a, b);
    }

    private static bool OverlapOn(Vec2 axis, BoxShape a, BoxShape b)
    {
        GetAxes(a.Rotation, out Vec2 ax, out Vec2 ay);
        GetAxes(b.Rotation, out Vec2 bx, out Vec2 by);
        float ra = a.HalfSize.X * MathF.Abs(axis.Dot(ax))
            + a.HalfSize.Y * MathF.Abs(axis.Dot(ay));
        float rb = b.HalfSize.X * MathF.Abs(axis.Dot(bx))
            + b.HalfSize.Y * MathF.Abs(axis.Dot(by));
        return MathF.Abs((b.Center - a.Center).Dot(axis)) <= ra + rb;
    }

    private static void GetAxes(float rotation, out Vec2 x, out Vec2 y)
    {
        float c = MathF.Cos(rotation), s = MathF.Sin(rotation);
        x = new Vec2(c, s);
        y = new Vec2(-s, c);
    }

    private static Aabb BoundsOf(BoxShape box)
    {
        GetAxes(box.Rotation, out Vec2 x, out Vec2 y);
        Vec2 half = new(
            MathF.Abs(x.X) * box.HalfSize.X + MathF.Abs(y.X) * box.HalfSize.Y,
            MathF.Abs(x.Y) * box.HalfSize.X + MathF.Abs(y.Y) * box.HalfSize.Y);
        return new Aabb(box.Center, half);
    }

    // ------------------------------------------------------------ stage/boss

    private void UpdateHiddenTax(float dt)
    {
        if (_taxMan.TaxSeat == TaxSeatState.Active || Fighters.Contains(_taxMan)) return;
        switch (_taxMan.TaxSeat)
        {
            case TaxSeatState.Reveal:
            case TaxSeatState.Drink:
            case TaxSeatState.Laugh:
                _taxMan.TaxSeatTimer -= dt;
                if (_taxMan.TaxSeatTimer <= 0f)
                    StartTaxSwirl();
                break;
            case TaxSeatState.Swirl:
                _taxMan.TaxSeatTimer -= dt;
                if (_taxMan.TaxSeatTimer <= 0f)
                {
                    _taxMan.TaxSeat = TaxSeatState.Drink;
                    _taxMan.TaxSeatTimer = 1.625f;
                }
                break;
        }
    }

    private void StartTaxSwirl()
    {
        _taxMan.TaxSeat = TaxSeatState.Swirl;
        _taxMan.TaxNextDrinkSeconds = _rng.Next(3, 9);
        _taxMan.TaxSeatTimer = _taxMan.TaxNextDrinkSeconds;
    }

    private void BeginTaxEngage()
    {
        if (!Fighters.Contains(_taxMan))
            Fighters.Add(_taxMan);
        _taxMan.TaxSeat = TaxSeatState.Engage;
        _taxMan.TaxSeatTimer = 1.75f;
        _taxMan.CombatActive = false;
        _taxMan.SetState(FState.Idle);
    }

    private void AdvanceTaxEngage(float dt)
    {
        if (_taxMan.TaxSeat != TaxSeatState.Engage) return;
        _taxMan.TaxSeatTimer -= dt;
        if (_taxMan.TaxSeatTimer > 0f) return;
        _taxMan.Pos = P(15072, 861);
        _taxMan.TaxSeat = TaxSeatState.Active;
        _taxMan.CombatActive = true;
        _taxMan.SetState(FState.Idle);
        EnterAiWait(_taxMan);
    }

    private void StageUpdate(float dt)
    {
        AdvanceTaxEngage(dt);
        if (Player.Dead)
        {
            if (Player.DeathTimer <= 0f)
                GameOver = true;
            return;
        }

        int aliveEnemies = 0;
        foreach (Fighter fighter in Fighters)
            if (fighter.Faction == Faction.Enemy && fighter.CombatActive && !fighter.Dead)
                aliveEnemies++;

        if (ActiveRoom != null)
        {
            FightRoom room = ActiveRoom;
            if (room.Boss)
            {
                if (_taxMan.Dead && _taxMan.DeathTimer <= 0f)
                {
                    room.Cleared = true;
                    RoomsCleared++;
                    Win = GameOver = true;
                    ActiveRoom = null;
                }
                return;
            }

            if (!room.WaveSpawned)
            {
                room.SpawnTimer -= dt;
                if (room.SpawnTimer <= 0f)
                {
                    SpawnWave(room.Waves[room.WaveIndex]);
                    room.WaveSpawned = true;
                }
            }
            else if (aliveEnemies == 0)
            {
                room.WaveIndex++;
                if (room.WaveIndex < room.Waves.Length)
                {
                    room.WaveSpawned = false;
                    room.SpawnTimer = .8f;
                }
                else
                {
                    room.Cleared = true;
                    RoomsCleared++;
                    if (room.EngagesBoss)
                    {
                        StartRoom(Rooms[4]);
                        BeginTaxEngage();
                    }
                    else
                    {
                        ActiveRoom = null;
                        CameraLeft = room.AfterLeft;
                        CameraTop = room.AfterTop;
                        CameraRight = room.AfterRight;
                        CameraBottom = room.AfterBottom;
                        CameraZoom = room.AfterZoom;
                    }
                }
            }
            return;
        }

        foreach (FightRoom room in Rooms)
        {
            if (room.Started || room.Cleared) continue;
            if (Player.Pos.X >= room.TriggerX)
                StartRoom(room);
            break;
        }
    }

    private void StartRoom(FightRoom room)
    {
        room.Started = true;
        room.WaveIndex = 0;
        room.WaveSpawned = room.Boss;
        room.SpawnTimer = .5f;
        ActiveRoom = room;
        CameraLeft = room.Left;
        CameraTop = room.Top;
        CameraRight = room.Right;
        CameraBottom = room.Bottom;
        CameraZoom = room.Zoom;
        if (room.EngagesBoss && _taxMan.TaxSeat == TaxSeatState.AwaitReveal)
        {
            _taxMan.TaxSeat = TaxSeatState.Reveal;
            _taxMan.TaxSeatTimer = 2.625f;
        }
    }

    private void SpawnWave(SpawnData[] wave)
    {
        foreach (SpawnData spawn in wave)
        {
            Fighter fighter = new()
            {
                Def = CharacterDef.Sarge,
                Faction = Faction.Enemy,
                Pos = spawn.Start,
                SpawnTarget = spawn.Target,
                SpawnWalking = spawn.WalkIn,
                Health = CharacterDef.Sarge.MaxHealth,
                Facing = spawn.Target.X < spawn.Start.X ? -1f : 1f,
            };
            fighter.AiTimer = 1f + (float)_rng.NextDouble() * 2f;
            Fighters.Add(fighter);
        }
    }

    private void Cleanup()
    {
        for (int i = Fighters.Count - 1; i >= 0; i--)
        {
            Fighter fighter = Fighters[i];
            if (fighter.Faction == Faction.Enemy && fighter.Dead && fighter.DeathTimer <= 0f)
                Fighters.RemoveAt(i);
        }
    }

}
