import 'package:forge2d/forge2d.dart';

class PolygonAndCircleContact extends Contact {
  PolygonAndCircleContact(Fixture fixtureA, Fixture fixtureB)
      : assert(fixtureA.type == ShapeType.polygon),
        assert(fixtureB.type == ShapeType.circle),
        super(fixtureA, 0, fixtureB, 0);

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    World.collision.collidePolygonAndCircle(
      manifold,
      fixtureA.shape as PolygonShape,
      xfA,
      fixtureB.shape as CircleShape,
      xfB,
    );
  }
}
