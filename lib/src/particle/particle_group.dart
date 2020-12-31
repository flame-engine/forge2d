part of forge2d;

class ParticleGroup {
  ParticleSystem _system;
  final List<Particle> particles = [];
  int groupFlags = 0;
  double _strength = 1.0;

  int _timestamp = -1;
  double _mass = 0.0;
  double _inertia = 0.0;
  final Vector2 _center = Vector2.zero();
  final Vector2 _linearVelocity = Vector2.zero();
  double _angularVelocity = 0.0;
  final Transform _transform = Transform.zero().setIdentity();

  bool _destroyAutomatically = true;
  bool _toBeDestroyed = false;
  bool _toBeSplit = false;

  Object userData;

  ParticleGroup();

  void add(Particle particle) {
    particles.add(particle);
  }

  bool contains(Particle particle) => particles.contains(particle);

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

  void updateStatistics() {
    if (_timestamp != _system.timestamp) {
      final double m = _system.getParticleMass();
      final _mass = m * particles.length;
      _center.setZero();
      _linearVelocity.setZero();
      for (Particle particle in particles) {
        _center.setFrom(_center + (particle.position * m));
        _linearVelocity.setFrom(_linearVelocity + (particle.velocity * m));
      }
      if (_mass > 0) {
        _center.x *= 1 / _mass;
        _center.y *= 1 / _mass;
        _linearVelocity.x *= 1 / _mass;
        _linearVelocity.y *= 1 / _mass;
      }
      _inertia = 0.0;
      _angularVelocity = 0.0;
      for (Particle particle in particles) {
        final Vector2 position = particle.position;
        final Vector2 velocity = particle.velocity;
        final double px = position.x - _center.x;
        final double py = position.y - _center.y;
        final double vx = velocity.x - _linearVelocity.x;
        final double vy = velocity.y - _linearVelocity.y;
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
