part of box2d;

class ParticleGroup {
  ParticleSystem _system;
  int _firstIndex = 0;
  int _lastIndex = 0;
  int _groupFlags = 0;
  double _strength = 0.0;
  ParticleGroup _prev;
  ParticleGroup _next;

  int _timestamp = 0;
  double _mass = 0.0;
  double _inertia = 0.0;
  final Vector2 _center = Vector2.zero();
  final Vector2 _linearVelocity = Vector2.zero();
  double _angularVelocity = 0.0;
  final Transform _transform = Transform.zero();

  bool _destroyAutomatically = false;
  bool _toBeDestroyed = false;
  bool _toBeSplit = false;

  Object _userData;

  ParticleGroup() {
    // _system = null;
    _firstIndex = 0;
    _lastIndex = 0;
    _groupFlags = 0;
    _strength = 1.0;

    _timestamp = -1;
    _mass = 0.0;
    _inertia = 0.0;
    _angularVelocity = 0.0;
    _transform.setIdentity();

    _destroyAutomatically = true;
    _toBeDestroyed = false;
    _toBeSplit = false;
  }

  ParticleGroup getNext() {
    return _next;
  }

  int getParticleCount() {
    return _lastIndex - _firstIndex;
  }

  int getBufferIndex() {
    return _firstIndex;
  }

  int getGroupFlags() {
    return _groupFlags;
  }

  void setGroupFlags(int flags) {
    _groupFlags = flags;
  }

  double getMass() {
    updateStatistics();
    return _mass;
  }

  double getInertia() {
    updateStatistics();
    return _inertia;
  }

  Vector2 getCenter() {
    updateStatistics();
    return _center;
  }

  Vector2 getLinearVelocity() {
    updateStatistics();
    return _linearVelocity;
  }

  double getAngularVelocity() {
    updateStatistics();
    return _angularVelocity;
  }

  Transform getTransform() {
    return _transform;
  }

  Vector2 getPosition() {
    return _transform.p;
  }

  double getAngle() {
    return _transform.q.getAngle();
  }

  Object getUserData() {
    return _userData;
  }

  void setUserData(Object data) {
    _userData = data;
  }

  void updateStatistics() {
    if (_timestamp != _system.timestamp) {
      double m = _system.getParticleMass();
      _mass = 0.0;
      _center.setZero();
      _linearVelocity.setZero();
      for (int i = _firstIndex; i < _lastIndex; i++) {
        _mass += m;
        Vector2 pos = _system.positionBuffer.data[i];
        _center.x += m * pos.x;
        _center.y += m * pos.y;
        Vector2 vel = _system.velocityBuffer.data[i];
        _linearVelocity.x += m * vel.x;
        _linearVelocity.y += m * vel.y;
      }
      if (_mass > 0) {
        _center.x *= 1 / _mass;
        _center.y *= 1 / _mass;
        _linearVelocity.x *= 1 / _mass;
        _linearVelocity.y *= 1 / _mass;
      }
      _inertia = 0.0;
      _angularVelocity = 0.0;
      for (int i = _firstIndex; i < _lastIndex; i++) {
        Vector2 pos = _system.positionBuffer.data[i];
        Vector2 vel = _system.velocityBuffer.data[i];
        double px = pos.x - _center.x;
        double py = pos.y - _center.y;
        double vx = vel.x - _linearVelocity.x;
        double vy = vel.y - _linearVelocity.y;
        _inertia += m * (px * px + py * py);
        _angularVelocity += m * (px * vy - py * vx);
      }
      if (_inertia > 0) {
        _angularVelocity *= 1 / _inertia;
      }
      _timestamp = _system.timestamp;
    }
  }
}
