namespace ArcCollision.Battlefield;

/// <summary>Collision categories and reusable per-collider filters for combat.</summary>
internal static class BattlefieldCollisionFilters
{
    private const uint Body = 1u << 0;
    private const uint PlayerAttack = 1u << 1;
    private const uint EnemyAttack = 1u << 2;
    private const uint PlayerHurtbox = 1u << 3;
    private const uint EnemyHurtbox = 1u << 4;
    private const uint Poison = 1u << 5;
    private const uint StandingProbe = 1u << 6;

    public static readonly CollisionFilter FighterBody = new(Body, Body);

    private static readonly CollisionFilter s_playerAttack = new(
        PlayerAttack, EnemyHurtbox);

    private static readonly CollisionFilter s_enemyAttack = new(
        EnemyAttack, PlayerHurtbox);

    private static readonly CollisionFilter s_playerHurtbox = new(
        PlayerHurtbox, EnemyAttack);

    private static readonly CollisionFilter s_enemyHurtbox = new(
        EnemyHurtbox, PlayerAttack);

    // Poison is stored in the main world but deliberately rejects every
    // persistent fighter collider. Only a transient probe shaped from the
    // fighter's standing Body can see it, so poison slots never enter body
    // pairs or attack/hurtbox contacts.
    public static readonly CollisionFilter PoisonPool = new(
        Poison, StandingProbe);

    public static readonly CollisionFilter PoisonStandingProbe = new(
        StandingProbe, Poison);

    public static CollisionFilter Attack(Faction faction) =>
        faction == Faction.Player ? s_playerAttack : s_enemyAttack;

    public static CollisionFilter Hurtbox(Faction faction) =>
        faction == Faction.Player ? s_playerHurtbox : s_enemyHurtbox;
}
