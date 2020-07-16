part of box2d;

class CircleContact extends Contact {
  CircleContact(Fixture fixtureA, Fixture fixtureB) : super(fixtureA, 0, fixtureB, 0) {
    assert(_fixtureA.getType() == ShapeType.CIRCLE);
    assert(_fixtureB.getType() == ShapeType.CIRCLE);
  }

  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    World.collision.collideCircles(manifold, _fixtureA.getShape() as CircleShape,
        xfA, _fixtureB.getShape() as CircleShape, xfB);
  }
}
