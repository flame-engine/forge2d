part of box2d;

class VelocityConstraintPoint {
  final Vector2 rA = Vector2.zero();
  final Vector2 rB = Vector2.zero();
  double normalImpulse = 0.0;
  double tangentImpulse = 0.0;
  double normalMass = 0.0;
  double tangentMass = 0.0;
  double velocityBias = 0.0;
}

class ContactVelocityConstraint {
  List<VelocityConstraintPoint> points =
      List<VelocityConstraintPoint>(Settings.maxManifoldPoints);
  final Vector2 normal = Vector2.zero();
  final Matrix2 normalMass = Matrix2.zero();
  final Matrix2 K = Matrix2.zero();
  int indexA = 0;
  int indexB = 0;
  double invMassA = 0.0, invMassB = 0.0;
  double invIA = 0.0, invIB = 0.0;
  double friction = 0.0;
  double restitution = 0.0;
  double tangentSpeed = 0.0;
  int pointCount = 0;
  int contactIndex = 0;

  ContactVelocityConstraint() {
    for (int i = 0; i < points.length; i++) {
      points[i] = VelocityConstraintPoint();
    }
  }
}
