import 'package:forge2d/forge2d.dart';

/// Implement this class to provide collision filtering. In other words, you can
/// implement this class if you want finer control over contact creation.
class ContactFilter {
  /// Return true if contact calculations should be performed between these two
  /// shapes.
  ///
  /// Warning: for performance reasons this is only called when the AABBs begin
  /// to overlap.
  bool shouldCollide(Fixture fixtureA, Fixture fixtureB) {
    final filterA = fixtureA.filterData;
    final filterB = fixtureB.filterData;

    if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0) {
      return filterA.groupIndex > 0;
    }

    return ((filterA.maskBits & filterB.categoryBits) != 0) &&
        ((filterA.categoryBits & filterB.maskBits) != 0);
  }
}
