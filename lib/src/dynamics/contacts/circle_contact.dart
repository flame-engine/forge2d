import '../../../forge2d.dart';

class CircleContact extends Contact {
  CircleContact(Fixture fixtureA, Fixture fixtureB)
      : assert(fixtureA.getType() == ShapeType.circle),
        assert(fixtureB.getType() == ShapeType.circle),
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
