part of box2d;

/// Delegate of World.
class ContactManager implements PairCallback {
  BroadPhase broadPhase;
  Contact contactList;
  int contactCount = 0;
  ContactFilter contactFilter;
  ContactListener contactListener;

  final World _world;

  ContactManager(this._world, BroadPhase broadPhase_) {
    contactList = null;
    contactFilter = ContactFilter();
    contactListener = null;
    broadPhase = broadPhase_;
  }

  /// Broad-phase callback.
  void addPair(FixtureProxy proxyUserDataA, FixtureProxy proxyUserDataB) {
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
        Fixture fA = edge.contact.fixtureA;
        Fixture fB = edge.contact.fixtureB;
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
    if (contactFilter != null &&
        contactFilter.shouldCollide(fixtureA, fixtureB) == false) {
      return;
    }

    // Call the factory.
    Contact c = _world.popContact(fixtureA, indexA, fixtureB, indexB);
    if (c == null) {
      return;
    }

    // Contact creation may swap fixtures.
    fixtureA = c.fixtureA;
    fixtureB = c.fixtureB;
    indexA = c.getChildIndexA();
    indexB = c.getChildIndexB();
    bodyA = fixtureA.getBody();
    bodyB = fixtureB.getBody();

    // Insert into the world.
    c._prev = null;
    c._next = contactList;
    if (contactList != null) {
      contactList._prev = c;
    }
    contactList = c;

    // Connect to island graph.

    // Connect to body A
    c._nodeA.contact = c;
    c._nodeA.other = bodyB;

    c._nodeA.prev = null;
    c._nodeA.next = bodyA._contactList;
    if (bodyA._contactList != null) {
      bodyA._contactList.prev = c._nodeA;
    }
    bodyA._contactList = c._nodeA;

    // Connect to body B
    c._nodeB.contact = c;
    c._nodeB.other = bodyA;

    c._nodeB.prev = null;
    c._nodeB.next = bodyB._contactList;
    if (bodyB._contactList != null) {
      bodyB._contactList.prev = c._nodeB;
    }
    bodyB._contactList = c._nodeB;

    // wake up the bodies
    if (!fixtureA.isSensor() && !fixtureB.isSensor()) {
      bodyA.setAwake(true);
      bodyB.setAwake(true);
    }

    ++contactCount;
  }

  void findNewContacts() {
    broadPhase.updatePairs(this);
  }

  void destroy(Contact c) {
    Fixture fixtureA = c.fixtureA;
    Fixture fixtureB = c.fixtureB;
    Body bodyA = fixtureA.getBody();
    Body bodyB = fixtureB.getBody();

    if (contactListener != null && c.isTouching()) {
      contactListener.endContact(c);
    }

    // Remove from the world.
    if (c._prev != null) {
      c._prev._next = c._next;
    }

    if (c._next != null) {
      c._next._prev = c._prev;
    }

    if (c == contactList) {
      contactList = c._next;
    }

    // Remove from body 1
    if (c._nodeA.prev != null) {
      c._nodeA.prev.next = c._nodeA.next;
    }

    if (c._nodeA.next != null) {
      c._nodeA.next.prev = c._nodeA.prev;
    }

    if (c._nodeA == bodyA._contactList) {
      bodyA._contactList = c._nodeA.next;
    }

    // Remove from body 2
    if (c._nodeB.prev != null) {
      c._nodeB.prev.next = c._nodeB.next;
    }

    if (c._nodeB.next != null) {
      c._nodeB.next.prev = c._nodeB.prev;
    }

    if (c._nodeB == bodyB._contactList) {
      bodyB._contactList = c._nodeB.next;
    }

    // Call the factory.
    _world.pushContact(c);
    --contactCount;
  }

  /// This is the top level collision call for the time step. Here all the narrow phase collision is
  /// processed for the world contact list.
  void collide() {
    // Update awake contacts.
    Contact c = contactList;
    while (c != null) {
      Fixture fixtureA = c.fixtureA;
      Fixture fixtureB = c.fixtureB;
      int indexA = c.getChildIndexA();
      int indexB = c.getChildIndexB();
      Body bodyA = fixtureA.getBody();
      Body bodyB = fixtureB.getBody();

      // is this contact flagged for filtering?
      if ((c._flags & Contact.FILTER_FLAG) == Contact.FILTER_FLAG) {
        // Should these bodies collide?
        if (bodyB.shouldCollide(bodyA) == false) {
          Contact cNuke = c;
          c = cNuke.getNext();
          destroy(cNuke);
          continue;
        }

        // Check user filtering.
        if (contactFilter != null &&
            contactFilter.shouldCollide(fixtureA, fixtureB) == false) {
          Contact cNuke = c;
          c = cNuke.getNext();
          destroy(cNuke);
          continue;
        }

        // Clear the filtering flag.
        c._flags &= ~Contact.FILTER_FLAG;
      }

      bool activeA = bodyA.isAwake() && bodyA._bodyType != BodyType.STATIC;
      bool activeB = bodyB.isAwake() && bodyB._bodyType != BodyType.STATIC;

      // At least one body must be awake and it must be dynamic or kinematic.
      if (activeA == false && activeB == false) {
        c = c.getNext();
        continue;
      }

      int proxyIdA = fixtureA._proxies[indexA].proxyId;
      int proxyIdB = fixtureB._proxies[indexB].proxyId;
      bool overlap = broadPhase.testOverlap(proxyIdA, proxyIdB);

      // Here we destroy contacts that cease to overlap in the broad-phase.
      if (overlap == false) {
        Contact cNuke = c;
        c = cNuke.getNext();
        destroy(cNuke);
        continue;
      }

      // The contact persists.
      c.update(contactListener);
      c = c.getNext();
    }
  }
}
