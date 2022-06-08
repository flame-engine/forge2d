import 'package:forge2d/forge2d.dart';

class EdgeAndCircleContact extends Contact {
  EdgeAndCircleContact(
    super.fixtureA,
    super.indexA,
    super.fixtureB,
    super.indexB,
  )   : assert(fixtureA.type == ShapeType.edge),
        assert(fixtureB.type == ShapeType.circle);

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    World.collision.collideEdgeAndCircle(
      manifold,
      fixtureA.shape as EdgeShape,
      xfA,
      fixtureB.shape as CircleShape,
      xfB,
    );
  }
}
