part of forge2d;

class EdgeAndCircleContact extends Contact {
  EdgeAndCircleContact(Fixture fA, int indexA, Fixture fB, int indexB)
      : super(fA, indexA, fB, indexB) {
    assert(fixtureA.getType() == ShapeType.EDGE);
    assert(fixtureB.getType() == ShapeType.CIRCLE);
  }

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    World.collision.collideEdgeAndCircle(
      manifold,
      fixtureA.shape as EdgeShape,
      xfA,
      fixtureB.shape as CircleShape,
      xfB,
    );
  }
}
