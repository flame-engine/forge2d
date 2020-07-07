part of box2d;

class PolygonContact extends Contact {
  PolygonContact(IWorldPool argPool) : super(argPool) {
    assert(_pool != null);
  }

  void init0(Fixture fixtureA, Fixture fixtureB) {
    init(fixtureA, 0, fixtureB, 0);
    assert(_fixtureA.getType() == ShapeType.POLYGON);
    assert(_fixtureB.getType() == ShapeType.POLYGON);
  }

  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    Collision().collidePolygons(
        manifold,
        _fixtureA.getShape() as PolygonShape,
        xfA,
        _fixtureB.getShape() as PolygonShape,
        xfB);
  }
}
