import 'package:forge2d/forge2d.dart';

class PolygonAndCircleContact extends Contact {
  PolygonAndCircleContact(
    Fixture fixtureA,
    Fixture fixtureB,
    Collision collision,
    Distance distance,
  ) : assert(fixtureA.type == ShapeType.polygon),
      assert(fixtureB.type == ShapeType.circle),
      super(fixtureA, 0, fixtureB, 0, collision, distance);

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    collision.collidePolygonAndCircle(
      manifold,
      fixtureA.shape as PolygonShape,
      xfA,
      fixtureB.shape as CircleShape,
      xfB,
    );
  }
}
