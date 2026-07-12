using Xunit;

namespace ArcCollision.Tests;

/// <summary>
/// Backend-neutral unit tests for the <c>CollisionFilter</c> matching semantics.
/// Uses only the public <c>CollisionFilter</c> / <c>CollisionCategories</c>
/// surface, so the shared compile runs it against BOTH the reference backend
/// (ArcCollision.Tests) and the native wrapper backend (ArcCollision.Tests.Wrapper).
/// The filter logic is pure managed bitwise math in both backends, so these run
/// without the native library being present.
///
/// World-level filtering (pairs/queries respecting filters) is covered by
/// <see cref="ArcWorldTests"/>; this file pins the struct's own contract.
/// </summary>
public class CollisionFilterTests
{
    private const uint Attack = 1u << 1;
    private const uint Hurt = 1u << 2;
    private const uint Scenery = 1u << 3;

    [Fact]
    public void Default_BelongsToDefaultCategory_AndAcceptsEverything()
    {
        Assert.Equal(CollisionCategories.Default, CollisionFilter.Default.Categories);
        Assert.Equal(CollisionCategories.All, CollisionFilter.Default.CollidesWith);
        Assert.False(CollisionFilter.Default.IsDisabled);
        Assert.True(CollisionFilter.Default.CanCollideWith(CollisionFilter.Default));
    }

    [Fact]
    public void Disabled_CollidesWithNothing_InEitherDirection()
    {
        Assert.True(CollisionFilter.Disabled.IsDisabled);
        Assert.False(CollisionFilter.Disabled.CanCollideWith(CollisionFilter.Default));
        Assert.False(CollisionFilter.Default.CanCollideWith(CollisionFilter.Disabled));
    }

    [Fact]
    public void Constructor_DefaultsToCollidingWithAll()
    {
        var filter = new CollisionFilter(Attack);
        Assert.Equal(Attack, filter.Categories);
        Assert.Equal(CollisionCategories.All, filter.CollidesWith);
    }

    [Fact]
    public void IsDisabled_WhenEitherMaskIsZero()
    {
        Assert.True(new CollisionFilter(0, Hurt).IsDisabled);
        Assert.True(new CollisionFilter(Attack, 0).IsDisabled);
        Assert.False(new CollisionFilter(Attack, Hurt).IsDisabled);
    }

    [Fact]
    public void CanCollide_RequiresMutualAcceptance()
    {
        var attacker = new CollisionFilter(Attack, Hurt);   // is Attack, targets Hurt
        var hurtbox = new CollisionFilter(Hurt, Attack);    // is Hurt, accepts Attack
        Assert.True(attacker.CanCollideWith(hurtbox));
        Assert.True(hurtbox.CanCollideWith(attacker));      // symmetric

        // A hurtbox that does not accept Attack blocks the pair, even though the
        // attacker's mask still targets the Hurt category.
        var deaf = new CollisionFilter(Hurt, Scenery);
        Assert.False(attacker.CanCollideWith(deaf));
        Assert.False(deaf.CanCollideWith(attacker));
    }

    [Fact]
    public void CanCollide_NeedsCategoryOverlapOnBothMasks()
    {
        // attacker targets Hurt, but the other collider is Scenery (not Hurt).
        var attacker = new CollisionFilter(Attack, Hurt);
        var scenery = new CollisionFilter(Scenery, Attack);
        Assert.False(attacker.CanCollideWith(scenery));
    }

    [Fact]
    public void MultiCategoryMasks_CollideOnAnySharedBit()
    {
        var a = new CollisionFilter(Attack, Hurt | Scenery);
        var b = new CollisionFilter(Scenery, Attack);
        // a targets Scenery (b's category); b targets Attack (a's category).
        Assert.True(a.CanCollideWith(b));
    }

    [Fact]
    public void Allows_IsAliasForCanCollideWith()
    {
        var a = new CollisionFilter(Attack, Hurt);
        var b = new CollisionFilter(Hurt, Attack);
        Assert.Equal(a.CanCollideWith(b), a.Allows(b));
        Assert.Equal(
            a.CanCollideWith(CollisionFilter.Disabled),
            a.Allows(CollisionFilter.Disabled));
    }

    [Fact]
    public void Equality_ComparesBothMasks()
    {
        var a = new CollisionFilter(Attack, Hurt);
        var same = new CollisionFilter(Attack, Hurt);
        var differentCollidesWith = new CollisionFilter(Attack, Scenery);

        Assert.True(a == same);
        Assert.False(a != same);
        Assert.True(a.Equals(same));
        Assert.Equal(a.GetHashCode(), same.GetHashCode());

        Assert.True(a != differentCollidesWith);
        Assert.False(a.Equals(differentCollidesWith));
    }
}
