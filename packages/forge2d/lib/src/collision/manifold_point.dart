import 'package:forge2d/forge2d.dart';

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
///  - e_circles: the local center of circleB.
///  - e_faceA: the local center of cirlceB or the clip point of polygonB.
///  - e_faceB: the clip point of polygonA.
///
/// This structure is stored across time steps, so we keep it small.
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
  ManifoldPoint() : localPoint = Vector2.zero(), id = ContactID();

  /// Creates a manifold point as a copy of the given point.
  ManifoldPoint.copy(ManifoldPoint manifoldPoint)
    : localPoint = manifoldPoint.localPoint.clone(),
      normalImpulse = manifoldPoint.normalImpulse,
      tangentImpulse = manifoldPoint.tangentImpulse,
      id = ContactID.copy(manifoldPoint.id);

  /// Sets this manifold point form the given one.
  void set(ManifoldPoint manifoldPoint) {
    localPoint.setFrom(manifoldPoint.localPoint);
    normalImpulse = manifoldPoint.normalImpulse;
    tangentImpulse = manifoldPoint.tangentImpulse;
    id.set(manifoldPoint.id);
  }
}
