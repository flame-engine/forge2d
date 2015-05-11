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
  final Vector2 _center = new Vector2.zero();
  final Vector2 _linearVelocity = new Vector2.zero();
  double _angularVelocity = 0.0;
  final Transform _transform = new Transform.zero();

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
