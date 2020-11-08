part of forge2d;

class ChainAndCircleContact extends Contact {
  ChainAndCircleContact(Fixture fA, int indexA, Fixture fB, int indexB)
      : super(fA, indexA, fB, indexB) {
    assert(_fixtureA.getType() == ShapeType.CHAIN);
    assert(_fixtureB.getType() == ShapeType.CIRCLE);
  }

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    final ChainShape chain = _fixtureA.getShape() as ChainShape;
    final EdgeShape edge = chain.getChildEdge(_indexA);
    World.collision.collideEdgeAndCircle(
      manifold,
      edge,
      xfA,
      _fixtureB.getShape() as CircleShape,
      xfB,
    );
  }
}
