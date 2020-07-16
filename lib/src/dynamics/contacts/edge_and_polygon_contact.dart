part of box2d;

class EdgeAndPolygonContact extends Contact {
  EdgeAndPolygonContact(Fixture fA, int indexA, Fixture fB, int indexB) :
    super(fA, indexA, fB, indexB) {
    assert(_fixtureA.getType() == ShapeType.EDGE);
    assert(_fixtureB.getType() == ShapeType.POLYGON);
  }

  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    World.collision.collideEdgeAndPolygon(
        manifold,
        _fixtureA.getShape() as EdgeShape,
        xfA,
        _fixtureB.getShape() as PolygonShape,
        xfB);
  }
}
