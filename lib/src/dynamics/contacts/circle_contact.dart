part of box2d;

class CircleContact extends Contact {
  CircleContact(IWorldPool argPool) : super(argPool);

  void init0(Fixture fixtureA, Fixture fixtureB) {
    init(fixtureA, 0, fixtureB, 0);
    assert(_fixtureA.getType() == ShapeType.CIRCLE);
    assert(_fixtureB.getType() == ShapeType.CIRCLE);
  }

  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    Collision().collideCircles(
        manifold,
        _fixtureA.getShape() as CircleShape,
        xfA,
        _fixtureB.getShape() as CircleShape,
        xfB);
  }
}
