part of forge2d;

class ChainAndPolygonContact extends Contact {
  ChainAndPolygonContact(Fixture fA, int indexA, Fixture fB, int indexB)
      : super(fA, indexA, fB, indexB) {
    assert(_fixtureA.getType() == ShapeType.CHAIN);
    assert(_fixtureB.getType() == ShapeType.POLYGON);
  }

  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    final ChainShape chain = _fixtureA.getShape() as ChainShape;
    final EdgeShape edge = chain.getChildEdge(_indexA);
    World.collision.collideEdgeAndPolygon(
        manifold,
        edge,
        xfA,
        _fixtureB.getShape() as PolygonShape,
        xfB,
    );
  }
}
