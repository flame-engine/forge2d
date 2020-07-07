part of box2d;

/// The class manages contact between two shapes. A contact exists for each overlapping AABB in the
/// broad-phase (except if filtered). Therefore a contact object may exist that has no contact
/// points.
abstract class Contact {
  // Flags stored in _flags
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

  int _flags = 0;

  // World pool and list pointers.
  Contact _prev;
  Contact _next;

  // Nodes for connecting bodies.
  ContactEdge _nodeA = ContactEdge();
  ContactEdge _nodeB = ContactEdge();

  Fixture _fixtureA;
  Fixture _fixtureB;

  int _indexA = 0;
  int _indexB = 0;

  final Manifold _manifold = Manifold();

  int _toiCount = 0;
  double _toi = 0.0;

  double _friction = 0.0;
  double _restitution = 0.0;

  double _tangentSpeed = 0.0;

  final IWorldPool _pool;

  Contact(this._pool);

  /// initialization for pooling
  void init(Fixture fA, int indexA, Fixture fB, int indexB) {
    _flags = ENABLED_FLAG;

    _fixtureA = fA;
    _fixtureB = fB;

    _indexA = indexA;
    _indexB = indexB;

    _manifold.pointCount = 0;

    _prev = null;
    _next = null;

    _nodeA.contact = null;
    _nodeA.prev = null;
    _nodeA.next = null;
    _nodeA.other = null;

    _nodeB.contact = null;
    _nodeB.prev = null;
    _nodeB.next = null;
    _nodeB.other = null;

    _toiCount = 0;
    _friction = Contact.mixFriction(fA._friction, fB._friction);
    _restitution = Contact.mixRestitution(fA._restitution, fB._restitution);

    _tangentSpeed = 0.0;
  }

  /// Get the world manifold.
  void getWorldManifold(WorldManifold worldManifold) {
    final Body bodyA = _fixtureA.getBody();
    final Body bodyB = _fixtureB.getBody();
    final Shape shapeA = _fixtureA.getShape();
    final Shape shapeB = _fixtureB.getShape();

    worldManifold.initialize(_manifold, bodyA._transform, shapeA.radius,
        bodyB._transform, shapeB.radius);
  }

  /// Is this contact touching
  bool isTouching() {
    return (_flags & TOUCHING_FLAG) == TOUCHING_FLAG;
  }

  /// Enable/disable this contact. This can be used inside the pre-solve contact listener. The
  /// contact is only disabled for the current time step (or sub-step in continuous collisions).
  void setEnabled(bool flag) {
    if (flag) {
      _flags |= ENABLED_FLAG;
    } else {
      _flags &= ~ENABLED_FLAG;
    }
  }

  /// Has this contact been disabled?
  bool isEnabled() {
    return (_flags & ENABLED_FLAG) == ENABLED_FLAG;
  }

  /// Get the next contact in the world's contact list.
  Contact getNext() {
    return _next;
  }

  /// Get the first fixture in this contact.
  Fixture get fixtureA => _fixtureA;

  int getChildIndexA() {
    return _indexA;
  }

  /// Get the second fixture in this contact.
  Fixture get fixtureB => _fixtureB;

  int getChildIndexB() {
    return _indexB;
  }

  void resetFriction() {
    _friction = Contact.mixFriction(_fixtureA._friction, _fixtureB._friction);
  }

  void resetRestitution() {
    _restitution =
        Contact.mixRestitution(_fixtureA._restitution, _fixtureB._restitution);
  }

  void evaluate(Manifold manifold, Transform xfA, Transform xfB);

  /// Flag this contact for filtering. Filtering will occur the next time step.
  void flagForFiltering() {
    _flags |= FILTER_FLAG;
  }

  // djm pooling
  final Manifold _oldManifold = Manifold();

  void update(ContactListener listener) {
    _oldManifold.set(_manifold);

    // Re-enable this contact.
    _flags |= ENABLED_FLAG;

    bool touching = false;
    bool wasTouching = (_flags & TOUCHING_FLAG) == TOUCHING_FLAG;

    bool sensorA = _fixtureA.isSensor();
    bool sensorB = _fixtureB.isSensor();
    bool sensor = sensorA || sensorB;

    Body bodyA = _fixtureA.getBody();
    Body bodyB = _fixtureB.getBody();
    Transform xfA = bodyA._transform;
    Transform xfB = bodyB._transform;

    if (sensor) {
      Shape shapeA = _fixtureA.getShape();
      Shape shapeB = _fixtureB.getShape();
      touching =
          Collision().testOverlap(shapeA, _indexA, shapeB, _indexB, xfA, xfB);

      // Sensors don't generate manifolds.
      _manifold.pointCount = 0;
    } else {
      evaluate(_manifold, xfA, xfB);
      touching = _manifold.pointCount > 0;

      // Match old contact ids to new contact ids and copy the
      // stored impulses to warm start the solver.
      for (int i = 0; i < _manifold.pointCount; ++i) {
        ManifoldPoint mp2 = _manifold.points[i];
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
      _flags |= TOUCHING_FLAG;
    } else {
      _flags &= ~TOUCHING_FLAG;
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

  /// Friction mixing law. The idea is to allow either fixture to drive the restitution to zero. For
  /// example, anything slides on ice.
  static double mixFriction(double friction1, double friction2) {
    return Math.sqrt(friction1 * friction2);
  }

  /// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface. For
  /// example, a superball bounces on anything.
  static double mixRestitution(double restitution1, double restitution2) {
    return restitution1 > restitution2 ? restitution1 : restitution2;
  }
}
