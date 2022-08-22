import 'package:forge2d/forge2d.dart';

class ChainAndPolygonContact extends Contact {
  ChainAndPolygonContact(
    super.fixtureA,
    super.indexA,
    super.fixtureB,
    super.indexB,
  )   : assert(fixtureA.type == ShapeType.chain),
        assert(fixtureB.type == ShapeType.polygon);

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    final chain = fixtureA.shape as ChainShape;
    final edge = chain.childEdge(indexA);
    World.collision.collideEdgeAndPolygon(
      manifold,
      edge,
      xfA,
      fixtureB.shape as PolygonShape,
      xfB,
    );
  }
}
