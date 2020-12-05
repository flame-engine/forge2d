part of forge2d;

class CircleContact extends Contact {
  CircleContact(Fixture fixtureA, Fixture fixtureB)
      : super(fixtureA, 0, fixtureB, 0) {
    assert(fixtureA.getType() == ShapeType.CIRCLE);
    assert(fixtureB.getType() == ShapeType.CIRCLE);
  }

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    World.collision.collideCircles(
      manifold,
      fixtureA.shape as CircleShape,
      xfA,
      fixtureB.shape as CircleShape,
      xfB,
    );
  }
}
