part of forge2d;

class ChainAndPolygonContact extends Contact {
  ChainAndPolygonContact(Fixture fA, int indexA, Fixture fB, int indexB)
      : super(fA, indexA, fB, indexB) {
    assert(fixtureA.getType() == ShapeType.CHAIN);
    assert(fixtureB.getType() == ShapeType.POLYGON);
  }

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    final ChainShape chain = fixtureA.shape as ChainShape;
    final EdgeShape edge = chain.getChildEdge(indexA);
    World.collision.collideEdgeAndPolygon(
      manifold,
      edge,
      xfA,
      fixtureB.shape as PolygonShape,
      xfB,
    );
  }
}
