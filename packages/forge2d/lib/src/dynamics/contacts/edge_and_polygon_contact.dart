import 'package:forge2d/forge2d.dart';

class EdgeAndPolygonContact extends Contact {
  EdgeAndPolygonContact(
    super.fixtureA,
    super.indexA,
    super.fixtureB,
    super.indexB,
    super.collision,
    super.distance,
  ) : assert(fixtureA.type == ShapeType.edge),
      assert(fixtureB.type == ShapeType.polygon);

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    collision.collideEdgeAndPolygon(
      manifold,
      fixtureA.shape as EdgeShape,
      xfA,
      fixtureB.shape as PolygonShape,
      xfB,
    );
  }
}
