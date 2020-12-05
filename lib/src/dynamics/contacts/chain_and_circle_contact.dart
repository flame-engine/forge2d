part of forge2d;

class ChainAndCircleContact extends Contact {
  ChainAndCircleContact(Fixture fA, int indexA, Fixture fB, int indexB)
      : super(fA, indexA, fB, indexB) {
    assert(fixtureA.getType() == ShapeType.CHAIN);
    assert(fixtureB.getType() == ShapeType.CIRCLE);
  }

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    final ChainShape chain = fixtureA.shape as ChainShape;
    final EdgeShape edge = chain.getChildEdge(indexA);
    World.collision.collideEdgeAndCircle(
      manifold,
      edge,
      xfA,
      fixtureB.shape as CircleShape,
      xfB,
    );
  }
}
