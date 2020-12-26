part of forge2d;

class EdgeAndCircleContact extends Contact {
  EdgeAndCircleContact(
      Fixture fixtureA, int indexA, Fixture fixtureB, int indexB)
      : assert(fixtureA.getType() == ShapeType.EDGE),
        assert(fixtureB.getType() == ShapeType.CIRCLE),
        super(fixtureA, indexA, fixtureB, indexB);

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
