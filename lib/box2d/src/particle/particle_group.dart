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

  ParticleSystem m_system;
  int m_firstIndex = 0;
  int m_lastIndex = 0;
  int m_groupFlags = 0;
  double m_strength = 0.0;
  ParticleGroup m_prev;
  ParticleGroup m_next;

  int m_timestamp = 0;
  double m_mass = 0.0;
  double m_inertia = 0.0;
  final Vec2 m_center = new Vec2.zero();
  final Vec2 m_linearVelocity = new Vec2.zero();
  double m_angularVelocity = 0.0;
  final Transform m_transform = new Transform.zero();

  bool m_destroyAutomatically = false;
  bool m_toBeDestroyed = false;
  bool m_toBeSplit = false;

  Object m_userData;

  ParticleGroup() {
    // m_system = null;
    m_firstIndex = 0;
    m_lastIndex = 0;
    m_groupFlags = 0;
    m_strength = 1.0;

    m_timestamp = -1;
    m_mass = 0.0;
    m_inertia = 0.0;
    m_angularVelocity = 0.0;
    m_transform.setIdentity();

    m_destroyAutomatically = true;
    m_toBeDestroyed = false;
    m_toBeSplit = false;
  }

  ParticleGroup getNext() {
    return m_next;
  }

  int getParticleCount() {
    return m_lastIndex - m_firstIndex;
  }

  int getBufferIndex() {
    return m_firstIndex;
  }

  int getGroupFlags() {
    return m_groupFlags;
  }

  void setGroupFlags(int flags) {
    m_groupFlags = flags;
  }

  double getMass() {
    updateStatistics();
    return m_mass;
  }

  double getInertia() {
    updateStatistics();
    return m_inertia;
  }

  Vec2 getCenter() {
    updateStatistics();
    return m_center;
  }

  Vec2 getLinearVelocity() {
    updateStatistics();
    return m_linearVelocity;
  }

  double getAngularVelocity() {
    updateStatistics();
    return m_angularVelocity;
  }

  Transform getTransform() {
    return m_transform;
  }

  Vec2 getPosition() {
    return m_transform.p;
  }

  double getAngle() {
    return m_transform.q.getAngle();
  }

  Object getUserData() {
    return m_userData;
  }

  void setUserData(Object data) {
    m_userData = data;
  }

  void updateStatistics() {
    if (m_timestamp != m_system.m_timestamp) {
      double m = m_system.getParticleMass();
      m_mass = 0.0;
      m_center.setZero();
      m_linearVelocity.setZero();
      for (int i = m_firstIndex; i < m_lastIndex; i++) {
        m_mass += m;
        Vec2 pos = m_system.m_positionBuffer.data[i];
        m_center.x += m * pos.x;
        m_center.y += m * pos.y;
        Vec2 vel = m_system.m_velocityBuffer.data[i];
        m_linearVelocity.x += m * vel.x;
        m_linearVelocity.y += m * vel.y;
      }
      if (m_mass > 0) {
        m_center.x *= 1 / m_mass;
        m_center.y *= 1 / m_mass;
        m_linearVelocity.x *= 1 / m_mass;
        m_linearVelocity.y *= 1 / m_mass;
      }
      m_inertia = 0.0;
      m_angularVelocity = 0.0;
      for (int i = m_firstIndex; i < m_lastIndex; i++) {
        Vec2 pos = m_system.m_positionBuffer.data[i];
        Vec2 vel = m_system.m_velocityBuffer.data[i];
        double px = pos.x - m_center.x;
        double py = pos.y - m_center.y;
        double vx = vel.x - m_linearVelocity.x;
        double vy = vel.y - m_linearVelocity.y;
        m_inertia += m * (px * px + py * py);
        m_angularVelocity += m * (px * vy - py * vx);
      }
      if (m_inertia > 0) {
        m_angularVelocity *= 1 / m_inertia;
      }
      m_timestamp = m_system.m_timestamp;
    }
  }
}
