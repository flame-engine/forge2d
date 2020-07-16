part of box2d;

class PolygonContact extends Contact {
  PolygonContact(Fixture fixtureA, Fixture fixtureB) : super(fixtureA, 0, fixtureB, 0) {
    assert(_fixtureA.getType() == ShapeType.POLYGON);
    assert(_fixtureB.getType() == ShapeType.POLYGON);
  }

  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    World.collision.collidePolygons(
        manifold,
        _fixtureA.getShape() as PolygonShape,
        xfA,
        _fixtureB.getShape() as PolygonShape,
        xfB);
  }
}
