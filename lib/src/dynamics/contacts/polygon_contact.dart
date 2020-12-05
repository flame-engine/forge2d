part of forge2d;

class PolygonContact extends Contact {
  PolygonContact(Fixture fixtureA, Fixture fixtureB)
      : super(fixtureA, 0, fixtureB, 0) {
    assert(fixtureA.getType() == ShapeType.POLYGON);
    assert(fixtureB.getType() == ShapeType.POLYGON);
  }

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    World.collision.collidePolygons(
      manifold,
      fixtureA.shape as PolygonShape,
      xfA,
      fixtureB.shape as PolygonShape,
      xfB,
    );
  }
}
