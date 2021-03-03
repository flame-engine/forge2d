import '../../../forge2d.dart';

class PolygonContact extends Contact {
  PolygonContact(Fixture fixtureA, Fixture fixtureB)
      : assert(fixtureA.getType() == ShapeType.polygon),
        assert(fixtureB.getType() == ShapeType.polygon),
        super(fixtureA, 0, fixtureB, 0);

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    World.collision.collidePolygons(
      manifold,
      fixtureA.shape as PolygonShape,
      xfA,
      fixtureB.shape as PolygonShape,
      xfB,
    );
  }
}
