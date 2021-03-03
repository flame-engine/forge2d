import '../../../forge2d.dart';

class ChainAndCircleContact extends Contact {
  ChainAndCircleContact(
    Fixture fixtureA,
    int indexA,
    Fixture fixtureB,
    int indexB,
  )   : assert(fixtureA.getType() == ShapeType.chain),
        assert(fixtureB.getType() == ShapeType.circle),
        super(fixtureA, indexA, fixtureB, indexB);

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    final chain = fixtureA.shape as ChainShape;
    final edge = chain.getChildEdge(indexA);
    World.collision.collideEdgeAndCircle(
      manifold,
      edge,
      xfA,
      fixtureB.shape as CircleShape,
      xfB,
    );
  }
}
