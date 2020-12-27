part of forge2d;

class ContactPositionConstraint {
  List<Vector2> localPoints = List<Vector2>.generate(
    settings.maxManifoldPoints,
    (_) => Vector2.zero(),
  );
  final Vector2 localNormal = Vector2.zero();
  final Vector2 localPoint = Vector2.zero();
  int indexA = 0;
  int indexB = 0;
  double invMassA = 0.0, invMassB = 0.0;
  final Vector2 localCenterA = Vector2.zero();
  final Vector2 localCenterB = Vector2.zero();
  double invIA = 0.0, invIB = 0.0;
  ManifoldType type;
  double radiusA = 0.0, radiusB = 0.0;
  int pointCount = 0;

  ContactPositionConstraint();
}
