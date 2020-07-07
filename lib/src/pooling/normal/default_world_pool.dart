part of box2d;

/// Provides object pooling for all objects used in the engine. Objects retrieved from here should
/// only be used temporarily, and then pushed back (with the exception of arrays).

class OrderedStackVec2 extends OrderedStack<Vector2> {
  OrderedStackVec2(int argStackSize, int argContainerSize)
      : super(argStackSize, argContainerSize);
  Vector2 newInstance() => Vector2.zero();
}

class OrderedStackVec3 extends OrderedStack<Vector3> {
  OrderedStackVec3(int argStackSize, int argContainerSize)
      : super(argStackSize, argContainerSize);
  Vector3 newInstance() => Vector3.zero();
}

class OrderedStackMat22 extends OrderedStack<Matrix2> {
  OrderedStackMat22(int argStackSize, int argContainerSize)
      : super(argStackSize, argContainerSize);
  Matrix2 newInstance() => Matrix2.zero();
}

class OrderedStackMat33 extends OrderedStack<Matrix3> {
  OrderedStackMat33(int argStackSize, int argContainerSize)
      : super(argStackSize, argContainerSize);
  Matrix3 newInstance() => Matrix3.zero();
}

class OrderedStackAABB extends OrderedStack<AABB> {
  OrderedStackAABB(int argStackSize, int argContainerSize)
      : super(argStackSize, argContainerSize);
  AABB newInstance() => AABB();
}

class OrderedStackRot extends OrderedStack<Rot> {
  OrderedStackRot(int argStackSize, int argContainerSize)
      : super(argStackSize, argContainerSize);
  Rot newInstance() => Rot();
}

abstract class MutableStackWithPool<T> extends MutableStack<T> {
  IWorldPool _pool;
  MutableStackWithPool(this._pool, int argInitSize) : super(argInitSize);
}

class MutableStackPolygonContact extends MutableStackWithPool<PolygonContact> {
  MutableStackPolygonContact(IWorldPool pool, int argInitSize)
      : super(pool, argInitSize);
  PolygonContact newInstance() => PolygonContact(_pool);
}

class MutableStackCircleContact extends MutableStackWithPool<CircleContact> {
  MutableStackCircleContact(IWorldPool pool, int argInitSize)
      : super(pool, argInitSize);
  CircleContact newInstance() => CircleContact(_pool);
}

class MutableStackPolygonAndCircleContact
    extends MutableStackWithPool<PolygonAndCircleContact> {
  MutableStackPolygonAndCircleContact(IWorldPool pool, int argInitSize)
      : super(pool, argInitSize);
  PolygonAndCircleContact newInstance() => PolygonAndCircleContact(_pool);
}

class MutableStackEdgeAndCircleContact
    extends MutableStackWithPool<EdgeAndCircleContact> {
  MutableStackEdgeAndCircleContact(IWorldPool pool, int argInitSize)
      : super(pool, argInitSize);
  EdgeAndCircleContact newInstance() => EdgeAndCircleContact(_pool);
}

class MutableStackEdgeAndPolygonContact
    extends MutableStackWithPool<EdgeAndPolygonContact> {
  MutableStackEdgeAndPolygonContact(IWorldPool pool, int argInitSize)
      : super(pool, argInitSize);
  EdgeAndPolygonContact newInstance() => EdgeAndPolygonContact(_pool);
}

class MutableStackChainAndCircleContact
    extends MutableStackWithPool<ChainAndCircleContact> {
  MutableStackChainAndCircleContact(IWorldPool pool, int argInitSize)
      : super(pool, argInitSize);
  ChainAndCircleContact newInstance() => ChainAndCircleContact(_pool);
}

class MutableStackChainAndPolygonContact
    extends MutableStackWithPool<ChainAndPolygonContact> {
  MutableStackChainAndPolygonContact(IWorldPool pool, int argInitSize)
      : super(pool, argInitSize);
  ChainAndPolygonContact newInstance() => ChainAndPolygonContact(_pool);
}

class DefaultWorldPool implements IWorldPool {
  final OrderedStack<Vector2> _vecs;
  final OrderedStack<Vector3> _vec3s;
  final OrderedStack<Matrix2> _mats;
  final OrderedStack<Matrix3> _mat33s;
  final OrderedStack<AABB> _aabbs;
  final OrderedStack<Rot> _rots;

  final HashMap<int, Float64List> _afloats = HashMap<int, Float64List>();
  final HashMap<int, List<int>> _aints = HashMap<int, List<int>>();
  final HashMap<int, List<Vector2>> _avecs = HashMap<int, List<Vector2>>();

  IWorldPool _world;

  IWorldPool get world => _world;

  MutableStackWithPool<PolygonContact> _pcstack;
  MutableStackWithPool<CircleContact> _ccstack;
  MutableStackWithPool<PolygonAndCircleContact> _cpstack;
  MutableStackWithPool<EdgeAndCircleContact> _ecstack;
  MutableStackWithPool<EdgeAndPolygonContact> _epstack;
  MutableStackWithPool<ChainAndCircleContact> _chcstack;
  MutableStackWithPool<ChainAndPolygonContact> _chpstack;

  Collision _collision;

  DefaultWorldPool(int argSize, int argContainerSize)
      : _vecs = OrderedStackVec2(argSize, argContainerSize),
        _vec3s = OrderedStackVec3(argSize, argContainerSize),
        _mats = OrderedStackMat22(argSize, argContainerSize),
        _aabbs = OrderedStackAABB(argSize, argContainerSize),
        _rots = OrderedStackRot(argSize, argContainerSize),
        _mat33s = OrderedStackMat33(argSize, argContainerSize) {
    _pcstack =
        MutableStackPolygonContact(this, Settings.CONTACT_STACK_INIT_SIZE);
    _ccstack =
        MutableStackCircleContact(this, Settings.CONTACT_STACK_INIT_SIZE);
    _cpstack = MutableStackPolygonAndCircleContact(
        this, Settings.CONTACT_STACK_INIT_SIZE);
    _ecstack = MutableStackEdgeAndCircleContact(
        this, Settings.CONTACT_STACK_INIT_SIZE);
    _epstack = MutableStackEdgeAndPolygonContact(
        this, Settings.CONTACT_STACK_INIT_SIZE);
    _chcstack = MutableStackChainAndCircleContact(
        this, Settings.CONTACT_STACK_INIT_SIZE);
    _chpstack = MutableStackChainAndPolygonContact(
        this, Settings.CONTACT_STACK_INIT_SIZE);
    _world = this;
  }

  IDynamicStack<Contact> getPolyContactStack() {
    return _pcstack;
  }

  IDynamicStack<Contact> getCircleContactStack() {
    return _ccstack;
  }

  IDynamicStack<Contact> getPolyCircleContactStack() {
    return _cpstack;
  }

  IDynamicStack<Contact> getEdgeCircleContactStack() {
    return _ecstack;
  }

  IDynamicStack<Contact> getEdgePolyContactStack() {
    return _epstack;
  }

  IDynamicStack<Contact> getChainCircleContactStack() {
    return _chcstack;
  }

  IDynamicStack<Contact> getChainPolyContactStack() {
    return _chpstack;
  }

  Vector2 popVec2() {
    return _vecs.pop();
  }

  List<Vector2> popVec2Some(int argNum) {
    return _vecs.popSome(argNum);
  }

  void pushVec2(int argNum) {
    _vecs.push(argNum);
  }

  Vector3 popVec3() {
    return _vec3s.pop();
  }

  List<Vector3> popVec3Some(int argNum) {
    return _vec3s.popSome(argNum);
  }

  void pushVec3(int argNum) {
    _vec3s.push(argNum);
  }

  Matrix2 popMat22() {
    return _mats.pop();
  }

  List<Matrix2> popMat22Some(int argNum) {
    return _mats.popSome(argNum);
  }

  void pushMat22(int argNum) {
    _mats.push(argNum);
  }

  Matrix3 popMat33() {
    return _mat33s.pop();
  }

  void pushMat33(int argNum) {
    _mat33s.push(argNum);
  }

  AABB popAABB() {
    return _aabbs.pop();
  }

  List<AABB> popAABBSome(int argNum) {
    return _aabbs.popSome(argNum);
  }

  void pushAABB(int argNum) {
    _aabbs.push(argNum);
  }

  Rot popRot() {
    return _rots.pop();
  }

  void pushRot(int num) {
    _rots.push(num);
  }

  Collision getCollision() {
    return _collision;
  }

  Float64List getFloatArray(int argLength) {
    if (!_afloats.containsKey(argLength)) {
      _afloats[argLength] = Float64List(argLength);
    }

    // : "Array not built with correct length";
    assert(_afloats[argLength].length == argLength);
    return _afloats[argLength];
  }

  List<int> getIntArray(int argLength) {
    if (!_aints.containsKey(argLength)) {
      _aints[argLength] = BufferUtils.intList(argLength);
    }

    // : "Array not built with correct length";
    assert(_aints[argLength].length == argLength);
    return _aints[argLength];
  }

  List<Vector2> getVec2Array(int argLength) {
    if (!_avecs.containsKey(argLength)) {
      _avecs[argLength] = List<Vector2>.filled(argLength, Vector2.zero());
    }

    // : "Array not built with correct length";
    assert(_avecs[argLength].length == argLength);
    return _avecs[argLength];
  }
}
