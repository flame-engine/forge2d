import '../../../forge2d.dart';

class PolygonAndCircleContact extends Contact {
  PolygonAndCircleContact(Fixture fixtureA, Fixture fixtureB)
      : assert(fixtureA.getType() == ShapeType.polygon),
        assert(fixtureB.getType() == ShapeType.circle),
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
