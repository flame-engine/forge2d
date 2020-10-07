part of racer;

class GroundArea {
  GroundArea(this.frictionModifier, this.outOfCourse) : hashCode = _lastHash++;

  // Hashable implementation.
  final int hashCode;

  final double frictionModifier;
  final bool outOfCourse;

  static int _lastHash = 0;
}
