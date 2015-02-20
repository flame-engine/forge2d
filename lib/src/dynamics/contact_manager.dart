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
 * Delegate of World.
 * 
 * @author Daniel Murphy
 */

class ContactManager implements PairCallback {
  BroadPhase m_broadPhase;
  Contact m_contactList;
  int m_contactCount = 0;
  ContactFilter m_contactFilter;
  ContactListener m_contactListener;

  final World _pool;

  ContactManager(this._pool, BroadPhase broadPhase) {
    m_contactList = null;
    m_contactFilter = new ContactFilter();
    m_contactListener = null;
    m_broadPhase = broadPhase;
  }

  /**
   * Broad-phase callback.
   * 
   * @param proxyUserDataA
   * @param proxyUserDataB
   */
  void addPair(Object proxyUserDataA, Object proxyUserDataB) {
    FixtureProxy proxyA = proxyUserDataA;
    FixtureProxy proxyB = proxyUserDataB;

    Fixture fixtureA = proxyA.fixture;
    Fixture fixtureB = proxyB.fixture;

    int indexA = proxyA.childIndex;
    int indexB = proxyB.childIndex;

    Body bodyA = fixtureA.getBody();
    Body bodyB = fixtureB.getBody();

    // Are the fixtures on the same body?
    if (bodyA == bodyB) {
      return;
    }

    // TODO_ERIN use a hash table to remove a potential bottleneck when both
    // bodies have a lot of contacts.
    // Does a contact already exist?
    ContactEdge edge = bodyB.getContactList();
    while (edge != null) {
      if (edge.other == bodyA) {
        Fixture fA = edge.contact.getFixtureA();
        Fixture fB = edge.contact.getFixtureB();
        int iA = edge.contact.getChildIndexA();
        int iB = edge.contact.getChildIndexB();

        if (fA == fixtureA && iA == indexA && fB == fixtureB && iB == indexB) {
          // A contact already exists.
          return;
        }

        if (fA == fixtureB && iA == indexB && fB == fixtureA && iB == indexA) {
          // A contact already exists.
          return;
        }
      }

      edge = edge.next;
    }

    // Does a joint override collision? is at least one body dynamic?
    if (bodyB.shouldCollide(bodyA) == false) {
      return;
    }

    // Check user filtering.
    if (m_contactFilter != null &&
        m_contactFilter.shouldCollide(fixtureA, fixtureB) == false) {
      return;
    }

    // Call the factory.
    Contact c = _pool.popContact(fixtureA, indexA, fixtureB, indexB);
    if (c == null) {
      return;
    }

    // Contact creation may swap fixtures.
    fixtureA = c.getFixtureA();
    fixtureB = c.getFixtureB();
    indexA = c.getChildIndexA();
    indexB = c.getChildIndexB();
    bodyA = fixtureA.getBody();
    bodyB = fixtureB.getBody();

    // Insert into the world.
    c.m_prev = null;
    c.m_next = m_contactList;
    if (m_contactList != null) {
      m_contactList.m_prev = c;
    }
    m_contactList = c;

    // Connect to island graph.

    // Connect to body A
    c.m_nodeA.contact = c;
    c.m_nodeA.other = bodyB;

    c.m_nodeA.prev = null;
    c.m_nodeA.next = bodyA.m_contactList;
    if (bodyA.m_contactList != null) {
      bodyA.m_contactList.prev = c.m_nodeA;
    }
    bodyA.m_contactList = c.m_nodeA;

    // Connect to body B
    c.m_nodeB.contact = c;
    c.m_nodeB.other = bodyA;

    c.m_nodeB.prev = null;
    c.m_nodeB.next = bodyB.m_contactList;
    if (bodyB.m_contactList != null) {
      bodyB.m_contactList.prev = c.m_nodeB;
    }
    bodyB.m_contactList = c.m_nodeB;

    // wake up the bodies
    if (!fixtureA.isSensor() && !fixtureB.isSensor()) {
      bodyA.setAwake(true);
      bodyB.setAwake(true);
    }

    ++m_contactCount;
  }

  void findNewContacts() {
    m_broadPhase.updatePairs(this);
  }

  void destroy(Contact c) {
    Fixture fixtureA = c.getFixtureA();
    Fixture fixtureB = c.getFixtureB();
    Body bodyA = fixtureA.getBody();
    Body bodyB = fixtureB.getBody();

    if (m_contactListener != null && c.isTouching()) {
      m_contactListener.endContact(c);
    }

    // Remove from the world.
    if (c.m_prev != null) {
      c.m_prev.m_next = c.m_next;
    }

    if (c.m_next != null) {
      c.m_next.m_prev = c.m_prev;
    }

    if (c == m_contactList) {
      m_contactList = c.m_next;
    }

    // Remove from body 1
    if (c.m_nodeA.prev != null) {
      c.m_nodeA.prev.next = c.m_nodeA.next;
    }

    if (c.m_nodeA.next != null) {
      c.m_nodeA.next.prev = c.m_nodeA.prev;
    }

    if (c.m_nodeA == bodyA.m_contactList) {
      bodyA.m_contactList = c.m_nodeA.next;
    }

    // Remove from body 2
    if (c.m_nodeB.prev != null) {
      c.m_nodeB.prev.next = c.m_nodeB.next;
    }

    if (c.m_nodeB.next != null) {
      c.m_nodeB.next.prev = c.m_nodeB.prev;
    }

    if (c.m_nodeB == bodyB.m_contactList) {
      bodyB.m_contactList = c.m_nodeB.next;
    }

    // Call the factory.
    _pool.pushContact(c);
    --m_contactCount;
  }

  /**
   * This is the top level collision call for the time step. Here all the narrow phase collision is
   * processed for the world contact list.
   */
  void collide() {
    // Update awake contacts.
    Contact c = m_contactList;
    while (c != null) {
      Fixture fixtureA = c.getFixtureA();
      Fixture fixtureB = c.getFixtureB();
      int indexA = c.getChildIndexA();
      int indexB = c.getChildIndexB();
      Body bodyA = fixtureA.getBody();
      Body bodyB = fixtureB.getBody();

      // is this contact flagged for filtering?
      if ((c.m_flags & Contact.FILTER_FLAG) == Contact.FILTER_FLAG) {
        // Should these bodies collide?
        if (bodyB.shouldCollide(bodyA) == false) {
          Contact cNuke = c;
          c = cNuke.getNext();
          destroy(cNuke);
          continue;
        }

        // Check user filtering.
        if (m_contactFilter != null &&
            m_contactFilter.shouldCollide(fixtureA, fixtureB) == false) {
          Contact cNuke = c;
          c = cNuke.getNext();
          destroy(cNuke);
          continue;
        }

        // Clear the filtering flag.
        c.m_flags &= ~Contact.FILTER_FLAG;
      }

      bool activeA = bodyA.isAwake() && bodyA.m_type != BodyType.STATIC;
      bool activeB = bodyB.isAwake() && bodyB.m_type != BodyType.STATIC;

      // At least one body must be awake and it must be dynamic or kinematic.
      if (activeA == false && activeB == false) {
        c = c.getNext();
        continue;
      }

      int proxyIdA = fixtureA.m_proxies[indexA].proxyId;
      int proxyIdB = fixtureB.m_proxies[indexB].proxyId;
      bool overlap = m_broadPhase.testOverlap(proxyIdA, proxyIdB);

      // Here we destroy contacts that cease to overlap in the broad-phase.
      if (overlap == false) {
        Contact cNuke = c;
        c = cNuke.getNext();
        destroy(cNuke);
        continue;
      }

      // The contact persists.
      c.update(m_contactListener);
      c = c.getNext();
    }
  }
}
