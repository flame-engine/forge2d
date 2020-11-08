part of racer;

class GroundArea {
  GroundArea(this.frictionModifier, this.outOfCourse) : hashCode = _lastHash++;

  // Hashable implementation.
  @override
  final int hashCode;

  @override
  bool operator ==(Object other) {
    return other is GroundArea && other.hashCode == hashCode;
  }

  final double frictionModifier;
  final bool outOfCourse;

  static int _lastHash = 0;
}
