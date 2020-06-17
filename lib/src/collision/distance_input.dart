part of box2d;

/// Input for Distance.
/// You have to option to use the shape radii in the computation.
class DistanceInput {
  DistanceProxy proxyA = DistanceProxy();
  DistanceProxy proxyB = DistanceProxy();
  Transform transformA = Transform.zero();
  Transform transformB = Transform.zero();
  bool useRadii = false;
}
