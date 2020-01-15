/*******************************************************************************
 * Copyright (c) 2015, Daniel Murphy, Google
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

part of box2d;

/// Provides object pooling for all objects used in the engine. Objects retrieved from here should
/// only be used temporarily, and then pushed back (with the exception of arrays).

class OrderedStackVec2 extends OrderedStack<Vector2> {
  OrderedStackVec2(int argStackSize, int argContainerSize)
      : super(argStackSize, argContainerSize);
  Vector2 newInstance() => new Vector2.zero();
}

class OrderedStackVec3 extends OrderedStack<Vector3> {
  OrderedStackVec3(int argStackSize, int argContainerSize)
      : super(argStackSize, argContainerSize);
  Vector3 newInstance() => new Vector3.zero();
}

class OrderedStackMat22 extends OrderedStack<Matrix2> {
  OrderedStackMat22(int argStackSize, int argContainerSize)
      : super(argStackSize, argContainerSize);
  Matrix2 newInstance() => new Matrix2.zero();
}

class OrderedStackMat33 extends OrderedStack<Matrix3> {
  OrderedStackMat33(int argStackSize, int argContainerSize)
      : super(argStackSize, argContainerSize);
  Matrix3 newInstance() => new Matrix3.zero();
}

class OrderedStackAABB extends OrderedStack<AABB> {
  OrderedStackAABB(int argStackSize, int argContainerSize)
      : super(argStackSize, argContainerSize);
  AABB newInstance() => new AABB();
}

class OrderedStackRot extends OrderedStack<Rot> {
  OrderedStackRot(int argStackSize, int argContainerSize)
      : super(argStackSize, argContainerSize);
  Rot newInstance() => new Rot();
}

abstract class MutableStackWithPool<T> extends MutableStack<T> {
  IWorldPool _pool;
  MutableStackWithPool(this._pool, int argInitSize) : super(argInitSize);
}

class MutableStackPolygonContact extends MutableStackWithPool<PolygonContact> {
  MutableStackPolygonContact(IWorldPool pool, int argInitSize)
      : super(pool, argInitSize);
  PolygonContact newInstance() => new PolygonContact(_pool);
}

class MutableStackCircleContact extends MutableStackWithPool<CircleContact> {
  MutableStackCircleContact(IWorldPool pool, int argInitSize)
      : super(pool, argInitSize);
  CircleContact newInstance() => new CircleContact(_pool);
}

class MutableStackPolygonAndCircleContact
    extends MutableStackWithPool<PolygonAndCircleContact> {
  MutableStackPolygonAndCircleContact(IWorldPool pool, int argInitSize)
      : super(pool, argInitSize);
  PolygonAndCircleContact newInstance() => new PolygonAndCircleContact(_pool);
}

class MutableStackEdgeAndCircleContact
    extends MutableStackWithPool<EdgeAndCircleContact> {
  MutableStackEdgeAndCircleContact(IWorldPool pool, int argInitSize)
      : super(pool, argInitSize);
  EdgeAndCircleContact newInstance() => new EdgeAndCircleContact(_pool);
}

class MutableStackEdgeAndPolygonContact
    extends MutableStackWithPool<EdgeAndPolygonContact> {
  MutableStackEdgeAndPolygonContact(IWorldPool pool, int argInitSize)
      : super(pool, argInitSize);
  EdgeAndPolygonContact newInstance() => new EdgeAndPolygonContact(_pool);
}

class MutableStackChainAndCircleContact
    extends MutableStackWithPool<ChainAndCircleContact> {
  MutableStackChainAndCircleContact(IWorldPool pool, int argInitSize)
      : super(pool, argInitSize);
  ChainAndCircleContact newInstance() => new ChainAndCircleContact(_pool);
}

class MutableStackChainAndPolygonContact
    extends MutableStackWithPool<ChainAndPolygonContact> {
  MutableStackChainAndPolygonContact(IWorldPool pool, int argInitSize)
      : super(pool, argInitSize);
  ChainAndPolygonContact newInstance() => new ChainAndPolygonContact(_pool);
}

class DefaultWorldPool implements IWorldPool {
  final OrderedStack<Vector2> _vecs;
  final OrderedStack<Vector3> _vec3s;
  final OrderedStack<Matrix2> _mats;
  final OrderedStack<Matrix3> _mat33s;
  final OrderedStack<AABB> _aabbs;
  final OrderedStack<Rot> _rots;

  final HashMap<int, Float64List> _afloats = new HashMap<int, Float64List>();
  final HashMap<int, List<int>> _aints = new HashMap<int, List<int>>();
  final HashMap<int, List<Vector2>> _avecs = new HashMap<int, List<Vector2>>();

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
  TimeOfImpact _toi;
  final Distance _dist;

  DefaultWorldPool(int argSize, int argContainerSize)
      : _vecs = new OrderedStackVec2(argSize, argContainerSize),
        _vec3s = new OrderedStackVec3(argSize, argContainerSize),
        _mats = new OrderedStackMat22(argSize, argContainerSize),
        _aabbs = new OrderedStackAABB(argSize, argContainerSize),
        _rots = new OrderedStackRot(argSize, argContainerSize),
        _mat33s = new OrderedStackMat33(argSize, argContainerSize),
        _dist = new Distance() {
    _pcstack =
        new MutableStackPolygonContact(this, Settings.CONTACT_STACK_INIT_SIZE);
    _ccstack =
        new MutableStackCircleContact(this, Settings.CONTACT_STACK_INIT_SIZE);
    _cpstack = new MutableStackPolygonAndCircleContact(
        this, Settings.CONTACT_STACK_INIT_SIZE);
    _ecstack = new MutableStackEdgeAndCircleContact(
        this, Settings.CONTACT_STACK_INIT_SIZE);
    _epstack = new MutableStackEdgeAndPolygonContact(
        this, Settings.CONTACT_STACK_INIT_SIZE);
    _chcstack = new MutableStackChainAndCircleContact(
        this, Settings.CONTACT_STACK_INIT_SIZE);
    _chpstack = new MutableStackChainAndPolygonContact(
        this, Settings.CONTACT_STACK_INIT_SIZE);
    _collision = new Collision(this);
    _toi = new TimeOfImpact(this);
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

  TimeOfImpact getTimeOfImpact() {
    return _toi;
  }

  Distance getDistance() {
    return _dist;
  }

  Float64List getFloatArray(int argLength) {
    if (!_afloats.containsKey(argLength)) {
      _afloats[argLength] = new Float64List(argLength);
    }

    assert(_afloats[argLength].length ==
        argLength); // : "Array not built with correct length";
    return _afloats[argLength];
  }

  List<int> getIntArray(int argLength) {
    if (!_aints.containsKey(argLength)) {
      _aints[argLength] = BufferUtils.allocClearIntList(argLength);
    }

    assert(_aints[argLength].length ==
        argLength); // : "Array not built with correct length";
    return _aints[argLength];
  }

  List<Vector2> getVec2Array(int argLength) {
    if (!_avecs.containsKey(argLength)) {
      List<Vector2> ray = new List<Vector2>(argLength);
      for (int i = 0; i < argLength; i++) {
        ray[i] = new Vector2.zero();
      }
      _avecs[argLength] = ray;
    }

    assert(_avecs[argLength].length ==
        argLength); // : "Array not built with correct length";
    return _avecs[argLength];
  }
}
