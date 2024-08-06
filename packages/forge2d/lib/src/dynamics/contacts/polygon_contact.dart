import 'package:forge2d/forge2d.dart';

class PolygonContact extends Contact {
  PolygonContact(
    Fixture fixtureA,
    Fixture fixtureB,
    Collision collision,
    Distance distance,
  )   : assert(fixtureA.type == ShapeType.polygon),
        assert(fixtureB.type == ShapeType.polygon),
        super(fixtureA, 0, fixtureB, 0, collision, distance);

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    collision.collidePolygons(
      manifold,
      fixtureA.shape as PolygonShape,
      xfA,
      fixtureB.shape as PolygonShape,
      xfB,
    );
  }
}
