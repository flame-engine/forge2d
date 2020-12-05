part of forge2d;

class EdgeAndPolygonContact extends Contact {
  EdgeAndPolygonContact(Fixture fA, int indexA, Fixture fB, int indexB)
      : super(fA, indexA, fB, indexB) {
    assert(fixtureA.getType() == ShapeType.EDGE);
    assert(fixtureB.getType() == ShapeType.POLYGON);
  }

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
