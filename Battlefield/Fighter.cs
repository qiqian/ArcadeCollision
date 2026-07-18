using System;
using System.Collections.Generic;

namespace ArcCollision.Battlefield;

internal enum FState { Idle, Walk, Attack, Jump, AirAttack, Hurt, Ko, Landed, GetUp, Dead }

internal enum AiState
{
    Wait,
    Chase,
    PreAttackPause,
    SingleAttack,
    ComboPause,
    Attacking,
    MoveToDashMarker,
    Align,
    DashCharging,
}

internal enum ScriptedState { None, Bounce, TaxHurtKnockout }
internal enum TaxSeatState { AwaitReveal, Reveal, Swirl, Drink, Laugh, Engage, Active }

/// <summary>Mutable state for one translated QuiverCharacter.</summary>
internal sealed class Fighter
{
    public int EntityId = -1;
    public ArcHandle BodyHandle;
    public ArcHandle HurtHandle;
    public Vec2 HurtColliderHalfSize;
    public required CharacterDef Def;
    public Faction Faction;
    public Vec2 Pos;                         // character root / feet on the ground plane
    public Vec2 Vel;
    public float Facing = 1f;
    public int Health;
    public float KnockAmount;
    public float Invuln;                     // compatibility timer; source animation flags are separate
    public bool HasSuperArmor;
    public float HurtFlash;
    // Refreshed while touching poison and allowed to linger briefly after the
    // player leaves, so the tint/bubbles do not disappear on a single frame.
    public float PoisonEffectTime;
    // Per-fighter fractional damage accumulation keeps poison DPS independent
    // when several characters occupy one or more pools simultaneously.
    public float PoisonDamageAccumulator;
    public bool HurtHigh;
    public bool CombatActive = true;

    public FState State = FState.Idle;
    public float StateTime;
    public float TurnTimer;
    public ScriptedState Scripted;

    public AttackData? Attack;
    public int ComboIndex;
    public float AttackTime;
    public bool ComboQueued;
    public int AutoCombo;
    public readonly Dictionary<Fighter, HashSet<int>> HitGroups = new();

    public float SkinY;                      // height above root, up-positive
    public float SkinVelY;
    public bool JumpLaunched;
    public bool JumpLanding;
    public bool AirAttackUsed;
    public int Bounces;
    public bool DashConnected;
    public bool InThroneBounce;
    public int WallContact;

    public float HurtTimer;
    public float LandedTimer;
    public float DeathTimer;

    public AiState Ai = AiState.Wait;
    public float AiTimer;
    public int AiAttackChoice;
    public int BossPhase;
    public bool SpawnWalking;
    public Vec2 SpawnTarget;
    public Vec2 AiTarget;

    // Tax Man's custom source state.
    public TaxSeatState TaxSeat;
    public float TaxSeatTimer;
    public int TaxNextDrinkSeconds;
    public int TaxHealthBaseline;
    public float TaxCumulatedDamage;
    public int ConsecutiveHits;
    public bool TaxPendingArea;

    public int MaxHealth => Def.MaxHealth;
    public bool Alive => Health > 0;
    public bool Dead => State == FState.Dead;
    public bool Airborne => State is FState.Ko or FState.Jump or FState.AirAttack
        || (State == FState.Attack && Scripted == ScriptedState.Bounce);
    public bool CanAct => CombatActive && State is FState.Idle or FState.Walk;

    public Capsule Body
    {
        get
        {
            Vec2 center = Pos + new Vec2(Facing * Def.BodyOffset.X, Def.BodyOffset.Y);
            Vec2 along = new(Def.BodyHalfSpine, 0f);
            return new Capsule(center - along, center + along, Def.Radius);
        }
    }

    public bool IsInvulnerable
    {
        get
        {
            if (!CombatActive || Invuln > 0f || State is FState.Landed or FState.GetUp or FState.Dead)
                return true;
            return (State is FState.Attack or FState.AirAttack)
                && Attack != null
                && Attack.IsInvulnerableAt(AttackTime);
        }
    }

    public bool ShouldKnockout =>
        !Alive || (!HasSuperArmor && KnockAmount >= (int)Knockback.Medium);

    public BoxShape CurrentHurtShape()
    {
        BoxShape local;
        if (State == FState.Hurt)
            local = HurtHigh ? Def.HurtHigh : Def.HurtMid;
        else if ((State is FState.Attack or FState.AirAttack) && Attack != null)
            local = Attack.HurtShapeAt(AttackTime, Def.HurtBox);
        else
            local = Def.HurtBox;

        Vec2 center = Pos + new Vec2(Facing * local.Center.X, local.Center.Y - SkinY);
        return new BoxShape(center, local.HalfSize, local.Rotation * Facing);
    }

    public bool WasHitBy(Fighter attacker, int hitId)
    {
        if (!HitGroups.TryGetValue(attacker, out HashSet<int>? ids))
        {
            ids = new HashSet<int>();
            HitGroups.Add(attacker, ids);
        }
        return !ids.Add(hitId);
    }

    public void ResetAttackHits() => HitGroups.Clear();

    public void SetState(FState state)
    {
        State = state;
        StateTime = 0f;
        if (state is not FState.Attack and not FState.AirAttack)
        {
            Attack = null;
            AttackTime = 0f;
            Scripted = ScriptedState.None;
            HasSuperArmor = false;
        }
    }
}
