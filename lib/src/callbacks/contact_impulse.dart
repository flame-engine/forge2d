part of box2d;

/// Contact impulses for reporting. Impulses are used instead of forces because sub-step forces may
/// approach infinity for rigid body collisions. These match up one-to-one with the contact points in
/// b2Manifold.
class ContactImpulse {
  Float64List normalImpulses = Float64List(Settings.maxManifoldPoints);
  Float64List tangentImpulses = Float64List(Settings.maxManifoldPoints);
  int count = 0;
}
