part of forge2d;

class PolygonAndCircleContact extends Contact {
  PolygonAndCircleContact(Fixture fixtureA, Fixture fixtureB)
      : assert(fixtureA.getType() == ShapeType.POLYGON),
        assert(fixtureB.getType() == ShapeType.CIRCLE),
        super(fixtureA, 0, fixtureB, 0);

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    World.collision.collidePolygonAndCircle(
      manifold,
      fixtureA.shape as PolygonShape,
      xfA,
      fixtureB.shape as CircleShape,
      xfB,
    );
  }
}
