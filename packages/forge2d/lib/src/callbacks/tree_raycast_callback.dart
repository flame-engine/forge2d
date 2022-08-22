import 'package:forge2d/forge2d.dart';

/// Callback for {@link DynamicTree}
abstract class TreeRayCastCallback {
  /// retruns the fraction to the node
  double raycastCallback(RayCastInput input, int nodeId);
}
