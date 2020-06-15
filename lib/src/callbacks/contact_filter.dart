part of box2d;

/// Implement this class to provide collision filtering. In other words, you can implement
/// this class if you want finer control over contact creation.
class ContactFilter {
  /// Return true if contact calculations should be performed between these two shapes.
  /// @warning for performance reasons this is only called when the AABBs begin to overlap.
  bool shouldCollide(Fixture fixtureA, Fixture fixtureB) {
    Filter filterA = fixtureA.getFilterData();
    Filter filterB = fixtureB.getFilterData();

    if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0) {
      return filterA.groupIndex > 0;
    }

    bool collide = ((filterA.maskBits & filterB.categoryBits) != 0) &&
        ((filterA.categoryBits & filterB.maskBits) != 0);
    return collide;
  }
}
