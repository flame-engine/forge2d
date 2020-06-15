part of box2d;

/// Input for Distance.
/// You have to option to use the shape radii in the computation.
class DistanceInput {
  DistanceProxy proxyA = new DistanceProxy();
  DistanceProxy proxyB = new DistanceProxy();
  Transform transformA = new Transform.zero();
  Transform transformB = new Transform.zero();
  bool useRadii = false;
}
