part of box2d;

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// <ul><li>e_circles: the local center of circleB</li>
/// <li>e_faceA: the local center of cirlceB or the clip point of polygonB</li>
/// <li>e_faceB: the clip point of polygonA</li></ul>
/// This structure is stored across time steps, so we keep it small.<br/>
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
class ManifoldPoint {
  /// usage depends on manifold type
  final Vector2 localPoint;

  /// the non-penetration impulse
  double normalImpulse = 0.0;

  /// the friction impulse
  double tangentImpulse = 0.0;

  /// uniquely identifies a contact point between two shapes
  final ContactID id;

  /// Blank manifold point with everything zeroed out.
  ManifoldPoint()
      : localPoint = Vector2.zero(),
        id = ContactID();

  /// Creates a manifold point as a copy of the given point
  /// @param cp point to copy from
  ManifoldPoint.copy(final ManifoldPoint cp)
      : localPoint = cp.localPoint.clone(),
        normalImpulse = cp.normalImpulse,
        tangentImpulse = cp.tangentImpulse,
        id = ContactID.copy(cp.id);

  /// Sets this manifold point form the given one
  /// @param cp the point to copy from
  void set(final ManifoldPoint cp) {
    localPoint.setFrom(cp.localPoint);
    normalImpulse = cp.normalImpulse;
    tangentImpulse = cp.tangentImpulse;
    id.set(cp.id);
  }
}
