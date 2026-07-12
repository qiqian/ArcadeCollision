namespace ArcCollision.Battlefield;

/// <summary>Collision categories and reusable per-collider filters for combat.</summary>
internal static class BattlefieldCollisionFilters
{
    private const uint Body = 1u << 0;
    private const uint PlayerAttack = 1u << 1;
    private const uint EnemyAttack = 1u << 2;
    private const uint PlayerHurtbox = 1u << 3;
    private const uint EnemyHurtbox = 1u << 4;

    public static readonly CollisionFilter FighterBody = new(Body, Body);

    private static readonly CollisionFilter s_playerAttack = new(
        PlayerAttack, EnemyHurtbox);

    private static readonly CollisionFilter s_enemyAttack = new(
        EnemyAttack, PlayerHurtbox);

    private static readonly CollisionFilter s_playerHurtbox = new(
        PlayerHurtbox, EnemyAttack);

    private static readonly CollisionFilter s_enemyHurtbox = new(
        EnemyHurtbox, PlayerAttack);

    public static CollisionFilter Attack(Faction faction) =>
        faction == Faction.Player ? s_playerAttack : s_enemyAttack;

    public static CollisionFilter Hurtbox(Faction faction) =>
        faction == Faction.Player ? s_playerHurtbox : s_enemyHurtbox;
}
