part of forge2d;

/// Delegate of World.
class ContactManager implements PairCallback {
  BroadPhase broadPhase;
  Contact contactList;
  int contactCount = 0;
  ContactFilter contactFilter;
  ContactListener contactListener;

  ContactManager(this.broadPhase) {
    contactFilter = ContactFilter();
  }

  /// Broad-phase callback.
  @override
  void addPair(FixtureProxy proxyUserDataA, FixtureProxy proxyUserDataB) {
    final FixtureProxy proxyA = proxyUserDataA;
    final FixtureProxy proxyB = proxyUserDataB;

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
        final Fixture fA = edge.contact.fixtureA;
        final Fixture fB = edge.contact.fixtureB;
        final int iA = edge.contact.getChildIndexA();
        final int iB = edge.contact.getChildIndexB();

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

    final Contact c = Contact.init(fixtureA, indexA, fixtureB, indexB);
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
    contactList?._prev = c;
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
    bodyB._contactList?.prev = c._nodeB;
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
    final Fixture fixtureA = c.fixtureA;
    final Fixture fixtureB = c.fixtureB;
    final Body bodyA = fixtureA.getBody();
    final Body bodyB = fixtureB.getBody();

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

    if (c._manifold.pointCount > 0 &&
        !fixtureA.isSensor() &&
        !fixtureB.isSensor()) {
      fixtureA.getBody().setAwake(true);
      fixtureB.getBody().setAwake(true);
    }
    --contactCount;
  }

  /// This is the top level collision call for the time step. Here all the narrow phase collision is
  /// processed for the world contact list.
  void collide() {
    // Update awake contacts.
    Contact c = contactList;
    while (c != null) {
      final Fixture fixtureA = c.fixtureA;
      final Fixture fixtureB = c.fixtureB;
      final int indexA = c._indexA;
      final int indexB = c._indexB;
      final Body bodyA = fixtureA.getBody();
      final Body bodyB = fixtureB.getBody();

      // is this contact flagged for filtering?
      if ((c.flags & Contact.FILTER_FLAG) == Contact.FILTER_FLAG) {
        // Should these bodies collide?
        if (bodyB.shouldCollide(bodyA) == false) {
          final Contact cNuke = c;
          c = cNuke.getNext();
          destroy(cNuke);
          continue;
        }

        // Check user filtering.
        if (contactFilter != null &&
            contactFilter.shouldCollide(fixtureA, fixtureB) == false) {
          final Contact cNuke = c;
          c = cNuke.getNext();
          destroy(cNuke);
          continue;
        }

        // Clear the filtering flag.
        c.flags &= ~Contact.FILTER_FLAG;
      }

      final bool activeA =
          bodyA.isAwake() && bodyA._bodyType != BodyType.STATIC;
      final bool activeB =
          bodyB.isAwake() && bodyB._bodyType != BodyType.STATIC;

      // At least one body must be awake and it must be dynamic or kinematic.
      if (activeA == false && activeB == false) {
        c = c.getNext();
        continue;
      }

      final int proxyIdA = fixtureA._proxies[indexA].proxyId;
      final int proxyIdB = fixtureB._proxies[indexB].proxyId;
      final bool overlap = broadPhase.testOverlap(proxyIdA, proxyIdB);

      // Here we destroy contacts that cease to overlap in the broad-phase.
      if (overlap == false) {
        final Contact cNuke = c;
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
