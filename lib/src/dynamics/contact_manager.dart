part of forge2d;

/// Delegate of World.
class ContactManager implements PairCallback {
  BroadPhase broadPhase;
  final List<Contact> contacts = [];
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

    Body bodyA = fixtureA.body;
    Body bodyB = fixtureB.body;

    // Are the fixtures on the same body?
    if (bodyA == bodyB) {
      return;
    }

    // TODO_ERIN use a hash table to remove a potential bottleneck when both
    // bodies have a lot of contacts.
    // Does a contact already exist?
    for (Contact contact in bodyB.contacts) {
      if (contact.containsBody(bodyA)) {
        final Fixture currentFixtureA = contact.fixtureA;
        final Fixture currentFixtureB = contact.fixtureB;
        final int currentIndexA = contact.indexA;
        final int currentIndexB = contact.indexB;

        if (currentFixtureA == fixtureA &&
            currentIndexA == indexA &&
            currentFixtureB == fixtureB &&
            currentIndexB == indexB) {
          // A contact already exists.
          return;
        }

        if (currentFixtureA == fixtureB &&
            currentIndexA == indexB &&
            currentFixtureB == fixtureA &&
            currentIndexB == indexA) {
          // A contact already exists.
          return;
        }
      }
    }

    // Does a joint override collision? is at least one body dynamic?
    if (!bodyB.shouldCollide(bodyA)) {
      return;
    }

    // Check user filtering.
    if (contactFilter != null &&
        contactFilter.shouldCollide(fixtureA, fixtureB) == false) {
      return;
    }

    final Contact contact = Contact.init(fixtureA, indexA, fixtureB, indexB);

    // Contact creation may swap fixtures.
    fixtureA = contact.fixtureA;
    fixtureB = contact.fixtureB;
    indexA = contact.indexA;
    indexB = contact.indexB;
    bodyA = fixtureA.body;
    bodyB = fixtureB.body;

    // Insert into the world.
    contacts.add(contact);

    // Connect to island graph.

    // Connect to body A
    bodyA.contacts.add(contact);
    bodyB.contacts.add(contact);

    // Wake up the bodies
    if (!fixtureA.isSensor() && !fixtureB.isSensor()) {
      bodyA.setAwake(true);
      bodyB.setAwake(true);
    }
  }

  void findNewContacts() {
    broadPhase.updatePairs(this);
  }

  void destroy(Contact c) {
    final Fixture fixtureA = c.fixtureA;
    final Fixture fixtureB = c.fixtureB;

    if (contactListener != null && c.isTouching()) {
      contactListener.endContact(c);
    }

    contacts.remove(c);
    c.bodyA.contacts.remove(c);
    c.bodyB.contacts.remove(c);

    if (c._manifold.pointCount > 0 &&
        !fixtureA.isSensor() &&
        !fixtureB.isSensor()) {
      fixtureA.body.setAwake(true);
      fixtureB.body.setAwake(true);
    }
  }

  /// This is the top level collision call for the time step. Here all the narrow phase collision is
  /// processed for the world contact list.
  void collide() {
    final List<Contact> contactRemovals = [];
    // Update awake contacts.
    for (Contact c in contacts) {
      final Fixture fixtureA = c.fixtureA;
      final Fixture fixtureB = c.fixtureB;
      final int indexA = c.indexA;
      final int indexB = c.indexB;
      final Body bodyA = fixtureA.body;
      final Body bodyB = fixtureB.body;

      // is this contact flagged for filtering?
      if ((c.flags & Contact.FILTER_FLAG) == Contact.FILTER_FLAG) {
        // Should these bodies collide?
        if (bodyB.shouldCollide(bodyA) == false) {
          contactRemovals.add(c);
          continue;
        }

        // Check user filtering.
        if (contactFilter != null &&
            contactFilter.shouldCollide(fixtureA, fixtureB) == false) {
          contactRemovals.add(c);
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
        continue;
      }

      final int proxyIdA = fixtureA._proxies[indexA].proxyId;
      final int proxyIdB = fixtureB._proxies[indexB].proxyId;
      final bool overlap = broadPhase.testOverlap(proxyIdA, proxyIdB);

      // Here we destroy contacts that cease to overlap in the broad-phase.
      if (overlap == false) {
        contactRemovals.add(c);
        continue;
      }

      // The contact persists.
      c.update(contactListener);
    }
    contactRemovals.forEach(destroy);
  }
}
