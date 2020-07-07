part of box2d;

class EdgeAndCircleContact extends Contact {
  EdgeAndCircleContact(IWorldPool argPool) : super(argPool);

  void init(Fixture fA, int indexA, Fixture fB, int indexB) {
    super.init(fA, indexA, fB, indexB);
    assert(_fixtureA.getType() == ShapeType.EDGE);
    assert(_fixtureB.getType() == ShapeType.CIRCLE);
  }

  void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
    Collision().collideEdgeAndCircle(
        manifold,
        _fixtureA.getShape() as EdgeShape,
        xfA,
        _fixtureB.getShape() as CircleShape,
        xfB);
  }
}
