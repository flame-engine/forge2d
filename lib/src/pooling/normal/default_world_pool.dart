part of box2d;

/// Provides object pooling for all objects used in the engine. Objects retrieved from here should
/// only be used temporarily, and then pushed back (with the exception of arrays).

class Pool {
  static final Distance distance = Distance();
  static final Collision collision = Collision();
  static final TimeOfImpact toi = TimeOfImpact();
  static final MutableStack<PolygonContact> polygonContactStack =
      PolygonContactStack(Settings.CONTACT_STACK_INIT_SIZE);
  static final MutableStack<CircleContact> circleContactStack =
      CircleContactStack(Settings.CONTACT_STACK_INIT_SIZE);
  static final MutableStack<PolygonAndCircleContact> polygonCircleContactStack =
      PolygonCircleContactStack(Settings.CONTACT_STACK_INIT_SIZE);
  static final MutableStack<EdgeAndCircleContact> edgeCircleContactStack =
      EdgeCircleContactStack(Settings.CONTACT_STACK_INIT_SIZE);
  static final MutableStack<EdgeAndPolygonContact> edgePolygonContactStack =
      EdgePolygonContactStack(Settings.CONTACT_STACK_INIT_SIZE);
  static final MutableStack<ChainAndCircleContact> chainCircleContactStack =
      ChainCircleContactStack(Settings.CONTACT_STACK_INIT_SIZE);
  static final MutableStack<ChainAndPolygonContact> chainPolygonContactStack =
      ChainPolygonContactStack(Settings.CONTACT_STACK_INIT_SIZE);
}

class PolygonContactStack extends MutableStack<PolygonContact> {
  PolygonContactStack(int size) : super(size);
  PolygonContact newInstance() => PolygonContact();
}

class CircleContactStack extends MutableStack<CircleContact> {
  CircleContactStack(int size) : super(size);
  CircleContact newInstance() => CircleContact();
}

class PolygonCircleContactStack
// TODO(spydon) refactor to remove all "And"
    extends MutableStack<PolygonAndCircleContact> {
  PolygonCircleContactStack(int size) : super(size);
  PolygonAndCircleContact newInstance() => PolygonAndCircleContact();
}

class EdgeCircleContactStack extends MutableStack<EdgeAndCircleContact> {
  EdgeCircleContactStack(int size) : super(size);
  EdgeAndCircleContact newInstance() => EdgeAndCircleContact();
}

class EdgePolygonContactStack extends MutableStack<EdgeAndPolygonContact> {
  EdgePolygonContactStack(int size) : super(size);
  EdgeAndPolygonContact newInstance() => EdgeAndPolygonContact();
}

class ChainCircleContactStack extends MutableStack<ChainAndCircleContact> {
  ChainCircleContactStack(int size) : super(size);
  ChainAndCircleContact newInstance() => ChainAndCircleContact();
}

class ChainPolygonContactStack extends MutableStack<ChainAndPolygonContact> {
  ChainPolygonContactStack(int size) : super(size);
  ChainAndPolygonContact newInstance() => ChainAndPolygonContact();
}
