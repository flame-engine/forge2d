import 'package:forge2d/forge2d.dart';

/// Input for Distance.
/// You have to option to use the shape radii in the computation.
class DistanceInput {
  DistanceProxy proxyA = DistanceProxy();
  DistanceProxy proxyB = DistanceProxy();
  Transform transformA = Transform.zero();
  Transform transformB = Transform.zero();
  bool useRadii = false;
}
