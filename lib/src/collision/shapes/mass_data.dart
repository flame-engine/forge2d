part of box2d;

/// This holds the mass data computed for a shape.
class MassData {
  /// The mass of the shape, usually in kilograms.
  double mass = 0.0;

  /// The position of the shape's centroid relative to the shape's origin.
  final Vector2 center;

  /// The rotational inertia of the shape about the local origin.
  double I = 0.0;

  /// Blank mass data
  MassData() : this.center = Vector2.zero();

  /// Copies from the given mass data
  ///
  /// @param md mass data to copy from
  MassData.copy(MassData md)
      : mass = md.mass,
        center = md.center.clone(),
        I = md.I;

  void set(MassData md) {
    mass = md.mass;
    I = md.I;
    center.setFrom(md.center);
  }

  /// Return a copy of this object.
  MassData clone() {
    return MassData.copy(this);
  }
}
