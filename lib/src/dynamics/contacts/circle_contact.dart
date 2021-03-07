import '../../../forge2d.dart';

class CircleContact extends Contact {
  CircleContact(Fixture fixtureA, Fixture fixtureB)
      : assert(fixtureA.type == ShapeType.circle),
        assert(fixtureB.type == ShapeType.circle),
        super(fixtureA, 0, fixtureB, 0);

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    World.collision.collideCircles(
      manifold,
      fixtureA.shape as CircleShape,
      xfA,
      fixtureB.shape as CircleShape,
      xfB,
    );
  }
}
