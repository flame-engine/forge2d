part of forge2d;

/// Joints and fixtures are destroyed when their associated
/// body is destroyed. Implement this listener so that you
/// may remove references to these joints and shapes.
abstract class DestructionListener {
  /// Called when any joint is about to be destroyed due
  /// to the destruction of one of its attached bodies.
  /// @param joint
  void sayGoodbyeJoint(Joint joint);

  /// Called when any fixture is about to be destroyed due
  /// to the destruction of its parent body.
  /// @param fixture
  void sayGoodbyeFixture(Fixture fixture);
}
