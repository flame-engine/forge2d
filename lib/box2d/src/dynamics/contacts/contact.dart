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

/**
 * The class manages contact between two shapes. A contact exists for each overlapping AABB in the
 * broad-phase (except if filtered). Therefore a contact object may exist that has no contact
 * points.
 * 
 */
abstract class Contact {

  // Flags stored in m_flags
  // Used when crawling contact graph when forming islands.
  static final int ISLAND_FLAG = 0x0001;
  // Set when the shapes are touching.
  static final int TOUCHING_FLAG = 0x0002;
  // This contact can be disabled (by user)
  static final int ENABLED_FLAG = 0x0004;
  // This contact needs filtering because a fixture filter was changed.
  static final int FILTER_FLAG = 0x0008;
  // This bullet contact had a TOI event
  static final int BULLET_HIT_FLAG = 0x0010;

  static final int TOI_FLAG = 0x0020;

  int m_flags = 0;

  // World pool and list pointers.
  Contact m_prev;
  Contact m_next;

  // Nodes for connecting bodies.
  ContactEdge m_nodeA = new ContactEdge();
  ContactEdge m_nodeB = new ContactEdge();

  Fixture m_fixtureA;
  Fixture m_fixtureB;

  int m_indexA = 0;
  int m_indexB = 0;

  final Manifold m_manifold = new Manifold();

  double m_toiCount = 0.0;
  double m_toi = 0.0;

  double m_friction = 0.0;
  double m_restitution = 0.0;

  double m_tangentSpeed = 0.0;

  final IWorldPool pool;

  Contact(this.pool);

  /** initialization for pooling */
  void init(Fixture fA, int indexA, Fixture fB, int indexB) {
    m_flags = ENABLED_FLAG;

    m_fixtureA = fA;
    m_fixtureB = fB;

    m_indexA = indexA;
    m_indexB = indexB;

    m_manifold.pointCount = 0;

    m_prev = null;
    m_next = null;

    m_nodeA.contact = null;
    m_nodeA.prev = null;
    m_nodeA.next = null;
    m_nodeA.other = null;

    m_nodeB.contact = null;
    m_nodeB.prev = null;
    m_nodeB.next = null;
    m_nodeB.other = null;

    m_toiCount = 0.0;
    m_friction = Contact.mixFriction(fA.m_friction, fB.m_friction);
    m_restitution = Contact.mixRestitution(fA.m_restitution, fB.m_restitution);

    m_tangentSpeed = 0.0;
  }

  /**
   * Get the contact manifold. Do not set the point count to zero. Instead call Disable.
   */
  Manifold getManifold() {
    return m_manifold;
  }

  /**
   * Get the world manifold.
   */
  void getWorldManifold(WorldManifold worldManifold) {
    final Body bodyA = m_fixtureA.getBody();
    final Body bodyB = m_fixtureB.getBody();
    final Shape shapeA = m_fixtureA.getShape();
    final Shape shapeB = m_fixtureB.getShape();

    worldManifold.initialize(m_manifold, bodyA.getTransform(), shapeA.m_radius, bodyB.getTransform(), shapeB.m_radius);
  }

  /**
   * Is this contact touching
   * 
   * @return
   */
  bool isTouching() {
    return (m_flags & TOUCHING_FLAG) == TOUCHING_FLAG;
  }

  /**
   * Enable/disable this contact. This can be used inside the pre-solve contact listener. The
   * contact is only disabled for the current time step (or sub-step in continuous collisions).
   * 
   * @param flag
   */
  void setEnabled(bool flag) {
    if (flag) {
      m_flags |= ENABLED_FLAG;
    } else {
      m_flags &= ~ENABLED_FLAG;
    }
  }

  /**
   * Has this contact been disabled?
   * 
   * @return
   */
  bool isEnabled() {
    return (m_flags & ENABLED_FLAG) == ENABLED_FLAG;
  }

  /**
   * Get the next contact in the world's contact list.
   * 
   * @return
   */
  Contact getNext() {
    return m_next;
  }

  /**
   * Get the first fixture in this contact.
   * 
   * @return
   */
  Fixture getFixtureA() {
    return m_fixtureA;
  }

  int getChildIndexA() {
    return m_indexA;
  }

  /**
   * Get the second fixture in this contact.
   * 
   * @return
   */
  Fixture getFixtureB() {
    return m_fixtureB;
  }

  int getChildIndexB() {
    return m_indexB;
  }

  void setFriction(double friction) {
    m_friction = friction;
  }

  double getFriction() {
    return m_friction;
  }

  void resetFriction() {
    m_friction = Contact.mixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
  }

  void setRestitution(double restitution) {
    m_restitution = restitution;
  }

  double getRestitution() {
    return m_restitution;
  }

  void resetRestitution() {
    m_restitution = Contact.mixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);
  }

  void setTangentSpeed(double speed) {
    m_tangentSpeed = speed;
  }

  double getTangentSpeed() {
    return m_tangentSpeed;
  }

  void evaluate(Manifold manifold, Transform xfA, Transform xfB);

  /**
   * Flag this contact for filtering. Filtering will occur the next time step.
   */
  void flagForFiltering() {
    m_flags |= FILTER_FLAG;
  }

  // djm pooling
  final Manifold _oldManifold = new Manifold();

  void update(ContactListener listener) {

    _oldManifold.set(m_manifold);

    // Re-enable this contact.
    m_flags |= ENABLED_FLAG;

    bool touching = false;
    bool wasTouching = (m_flags & TOUCHING_FLAG) == TOUCHING_FLAG;

    bool sensorA = m_fixtureA.isSensor();
    bool sensorB = m_fixtureB.isSensor();
    bool sensor = sensorA || sensorB;

    Body bodyA = m_fixtureA.getBody();
    Body bodyB = m_fixtureB.getBody();
    Transform xfA = bodyA.getTransform();
    Transform xfB = bodyB.getTransform();
    // log.debug("TransformA: "+xfA);
    // log.debug("TransformB: "+xfB);

    if (sensor) {
      Shape shapeA = m_fixtureA.getShape();
      Shape shapeB = m_fixtureB.getShape();
      touching = pool.getCollision().testOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);

      // Sensors don't generate manifolds.
      m_manifold.pointCount = 0;
    } else {
      evaluate(m_manifold, xfA, xfB);
      touching = m_manifold.pointCount > 0;

      // Match old contact ids to new contact ids and copy the
      // stored impulses to warm start the solver.
      for (int i = 0; i < m_manifold.pointCount; ++i) {
        ManifoldPoint mp2 = m_manifold.points[i];
        mp2.normalImpulse = 0.0;
        mp2.tangentImpulse = 0.0;
        ContactID id2 = mp2.id;

        for (int j = 0; j < _oldManifold.pointCount; ++j) {
          ManifoldPoint mp1 = _oldManifold.points[j];

          if (mp1.id.isEqual(id2)) {
            mp2.normalImpulse = mp1.normalImpulse;
            mp2.tangentImpulse = mp1.tangentImpulse;
            break;
          }
        }
      }

      if (touching != wasTouching) {
        bodyA.setAwake(true);
        bodyB.setAwake(true);
      }
    }

    if (touching) {
      m_flags |= TOUCHING_FLAG;
    } else {
      m_flags &= ~TOUCHING_FLAG;
    }

    if (listener == null) {
      return;
    }

    if (wasTouching == false && touching == true) {
      listener.beginContact(this);
    }

    if (wasTouching == true && touching == false) {
      listener.endContact(this);
    }

    if (sensor == false && touching) {
      listener.preSolve(this, _oldManifold);
    }
  }

  /**
   * Friction mixing law. The idea is to allow either fixture to drive the restitution to zero. For
   * example, anything slides on ice.
   * 
   * @param friction1
   * @param friction2
   * @return
   */
  static double mixFriction(double friction1, double friction2) {
    return Math.sqrt(friction1 * friction2);
  }

  /**
   * Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface. For
   * example, a superball bounces on anything.
   * 
   * @param restitution1
   * @param restitution2
   * @return
   */
  static double mixRestitution(double restitution1, double restitution2) {
    return restitution1 > restitution2 ? restitution1 : restitution2;
  }

}
