part of forge2d;

class ParticleGroup {
  ParticleSystem _system;
  int _firstIndex = 0;
  int _lastIndex = 0;
  int _groupFlags = 0;
  double _strength = 0.0;

  int _timestamp = 0;
  double _mass = 0.0;
  double _inertia = 0.0;
  final Vector2 _center = Vector2.zero();
  final Vector2 linearVelocity = Vector2.zero();
  double _angularVelocity = 0.0;
  final Transform _transform = Transform.zero();

  bool _destroyAutomatically = false;
  bool _toBeDestroyed = false;
  bool _toBeSplit = false;

  Object _userData;

  ParticleGroup() {
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
    return linearVelocity;
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
      final double m = _system.getParticleMass();
      _mass = 0.0;
      _center.setZero();
      linearVelocity.setZero();
      for (int i = _firstIndex; i < _lastIndex; i++) {
        _mass += m;
        final Vector2 pos = _system.positionBuffer[i];
        _center.x += m * pos.x;
        _center.y += m * pos.y;
        final Vector2 vel = _system.velocityBuffer[i];
        linearVelocity.x += m * vel.x;
        linearVelocity.y += m * vel.y;
      }
      if (_mass > 0) {
        _center.x *= 1 / _mass;
        _center.y *= 1 / _mass;
        linearVelocity.x *= 1 / _mass;
        linearVelocity.y *= 1 / _mass;
      }
      _inertia = 0.0;
      _angularVelocity = 0.0;
      for (int i = _firstIndex; i < _lastIndex; i++) {
        final Vector2 pos = _system.positionBuffer[i];
        final Vector2 vel = _system.velocityBuffer[i];
        final double px = pos.x - _center.x;
        final double py = pos.y - _center.y;
        final double vx = vel.x - linearVelocity.x;
        final double vy = vel.y - linearVelocity.y;
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
