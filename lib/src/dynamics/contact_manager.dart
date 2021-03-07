import '../../forge2d.dart';
import '../callbacks/contact_filter.dart';
import '../callbacks/contact_listener.dart';
import '../callbacks/pair_callback.dart';

/// Delegate of World.
class ContactManager implements PairCallback {
  BroadPhase broadPhase;
  final List<Contact> contacts = [];
  ContactFilter? contactFilter;
  ContactListener? contactListener;

  ContactManager(this.broadPhase) {
    contactFilter = ContactFilter();
  }

  /// Broad-phase callback.
  @override
  void addPair(FixtureProxy proxyUserDataA, FixtureProxy proxyUserDataB) {
    final proxyA = proxyUserDataA;
    final proxyB = proxyUserDataB;

    final fixtureA = proxyA.fixture;
    final fixtureB = proxyB.fixture;

    final indexA = proxyA.childIndex;
    final indexB = proxyB.childIndex;

    final bodyA = fixtureA.body;
    final bodyB = fixtureB.body;

    // Are the fixtures on the same body?
    if (bodyA == bodyB) {
      return;
    }

    // Check whether a contact already exists
    for (final contact in bodyB.contacts) {
      if (contact.containsBody(bodyA)) {
        if (contact.representsArguments(fixtureA, indexA, fixtureB, indexB)) {
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
    if (contactFilter?.shouldCollide(fixtureA, fixtureB) == false) {
      return;
    }

    final contact = Contact.init(fixtureA, indexA, fixtureB, indexB);

    // Insert into the world.
    contacts.add(contact);

    // Connect to island graph.

    // Connect to ths bodies
    bodyA.contacts.add(contact);
    bodyB.contacts.add(contact);

    // Wake up the bodies
    if (!fixtureA.isSensor && !fixtureB.isSensor) {
      bodyA.setAwake(true);
      bodyB.setAwake(true);
    }
  }

  void findNewContacts() {
    broadPhase.updatePairs(this);
  }

  void destroy(Contact c) {
    final fixtureA = c.fixtureA;
    final fixtureB = c.fixtureB;

    if (c.isTouching()) {
      contactListener?.endContact(c);
    }

    contacts.remove(c);
    c.bodyA.contacts.remove(c);
    c.bodyB.contacts.remove(c);

    if (c.manifold.pointCount > 0 && !fixtureA.isSensor && !fixtureB.isSensor) {
      fixtureA.body.setAwake(true);
      fixtureB.body.setAwake(true);
    }
  }

  /// This is the top level collision call for the time step. Here all the narrow phase collision is
  /// processed for the world contact list.
  void collide() {
    final contactRemovals = <Contact>[];
    // Update awake contacts.
    for (final c in contacts) {
      final fixtureA = c.fixtureA;
      final fixtureB = c.fixtureB;
      final indexA = c.indexA;
      final indexB = c.indexB;
      final bodyA = fixtureA.body;
      final bodyB = fixtureB.body;

      // is this contact flagged for filtering?
      if ((c.flags & Contact.filterFlag) == Contact.filterFlag) {
        // Should these bodies collide?
        if (!bodyB.shouldCollide(bodyA)) {
          contactRemovals.add(c);
          continue;
        }

        // Check user filtering.
        if (contactFilter?.shouldCollide(fixtureA, fixtureB) == false) {
          contactRemovals.add(c);
          continue;
        }

        // Clear the filtering flag.
        c.flags &= ~Contact.filterFlag;
      }

      final activeA = bodyA.isAwake() && bodyA.bodyType != BodyType.static;
      final activeB = bodyB.isAwake() && bodyB.bodyType != BodyType.static;

      // At least one body must be awake and it must be dynamic or kinematic.
      if (activeA == false && activeB == false) {
        continue;
      }

      final proxyIdA = fixtureA.proxies[indexA].proxyId;
      final proxyIdB = fixtureB.proxies[indexB].proxyId;

      // Here we destroy contacts that cease to overlap in the broad-phase.
      if (!broadPhase.testOverlap(proxyIdA, proxyIdB)) {
        contactRemovals.add(c);
        continue;
      }

      // The contact persists.
      c.update(contactListener);
    }
    contactRemovals.forEach(destroy);
  }
}
