import '../../../forge2d.dart';

class EdgeAndPolygonContact extends Contact {
  EdgeAndPolygonContact(
    Fixture fixtureA,
    int indexA,
    Fixture fixtureB,
    int indexB,
  )   : assert(fixtureA.type == ShapeType.edge),
        assert(fixtureB.type == ShapeType.polygon),
        super(fixtureA, indexA, fixtureB, indexB);

  @override
  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    World.collision.collideEdgeAndPolygon(
      manifold,
      fixtureA.shape as EdgeShape,
      xfA,
      fixtureB.shape as PolygonShape,
      xfB,
    );
  }
}
