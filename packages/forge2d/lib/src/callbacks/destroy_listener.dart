import 'package:forge2d/forge2d.dart';

/// Joints and fixtures are destroyed when their associated
/// body is destroyed. Implement this listener so that you
/// may remove references to these joints and shapes.
abstract class DestroyListener {
  /// Called when any joint is about to be destroyed due
  /// to the destruction of one of its attached bodies.
  void onDestroyJoint(Joint joint);

  /// Called when any fixture is about to be destroyed due
  /// to the destruction of its parent body.
  void onDestroyFixture(Fixture fixture);
}
