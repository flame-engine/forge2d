part of forge2d;

class EdgeAndPolygonContact extends Contact {
  EdgeAndPolygonContact(
      Fixture fixtureA, int indexA, Fixture fixtureB, int indexB)
      : assert(fixtureA.getType() == ShapeType.EDGE),
        assert(fixtureB.getType() == ShapeType.POLYGON),
        super(fixtureA, indexA, fixtureB, indexB);

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    World.collision.collideEdgeAndPolygon(
      manifold,
      fixtureA.shape as EdgeShape,
      xfA,
      fixtureB.shape as PolygonShape,
      xfB,
    );
  }
}
