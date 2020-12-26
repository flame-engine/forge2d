part of forge2d;

/// A fixture is used to attach a shape to a body for collision detection. A fixture inherits its
/// transform from its parent. Fixtures hold additional non-geometric data such as friction,
/// collision filters, etc. Fixtures are created via Body::CreateFixture.
///
/// @warning you cannot reuse fixtures.
class Fixture {
  double _density = 0.0;

  Body body;

  Shape shape;

  double _friction = 0.0;
  double _restitution = 0.0;

  List<FixtureProxy> _proxies = List.empty();
  int _proxyCount = 0;

  final Filter _filter = Filter();

  bool _isSensor = false;

  /// Use this to store your application specific data.
  Object userData;

  /// Get the type of the child shape. You can use this to down cast to the concrete shape.
  ///
  /// @return the shape type.
  ShapeType getType() => shape.shapeType;

  /// Is this fixture a sensor (non-solid)?
  ///
  /// @return the true if the shape is a sensor.
  /// @return
  bool isSensor() {
    return _isSensor;
  }

  /// Set if this fixture is a sensor.
  ///
  /// @param sensor
  void setSensor(bool sensor) {
    if (sensor != _isSensor) {
      body.setAwake(true);
      _isSensor = sensor;
    }
  }

  /// Set the contact filtering data. This is an expensive operation and should not be called
  /// frequently. This will not update contacts until the next time step when either parent body is
  /// awake. This automatically calls refilter.
  ///
  /// @param filter
  void setFilterData(final Filter filter) {
    _filter.set(filter);

    refilter();
  }

  /// Get the contact filtering data.
  ///
  /// @return
  Filter getFilterData() {
    return _filter;
  }

  /// Call this if you want to establish collision that was previously disabled by
  /// ContactFilter::ShouldCollide.
  void refilter() {
    if (body == null) {
      return;
    }

    // Flag associated contacts for filtering.
    for (Contact contact in body.contacts) {
      final Fixture fixtureA = contact.fixtureA;
      final Fixture fixtureB = contact.fixtureB;
      if (fixtureA == this || fixtureB == this) {
        contact.flagForFiltering();
      }
    }

    final World world = body.world;

    if (world == null) {
      return;
    }

    // Touch each proxy so that new pairs may be created
    final BroadPhase broadPhase = world._contactManager.broadPhase;
    for (int i = 0; i < _proxyCount; ++i) {
      broadPhase.touchProxy(_proxies[i].proxyId);
    }
  }

  void setDensity(double density) {
    assert(density >= 0.9);
    _density = density;
  }

  double getDensity() {
    return _density;
  }

  /// Test a point for containment in this fixture. This only works for convex shapes.
  ///
  /// @param p a point in world coordinates.
  /// @return
  bool testPoint(final Vector2 p) {
    return shape.testPoint(body._transform, p);
  }

  /// Cast a ray against this shape.
  ///
  /// @param input the ray-cast input parameters.
  /// @param output the ray-cast results.
  bool raycast(RayCastOutput output, RayCastInput input, int childIndex) {
    return shape.raycast(output, input, body._transform, childIndex);
  }

  /// Get the mass data for this fixture. The mass data is based on the density and the shape. The
  /// rotational inertia is about the shape's origin.
  ///
  /// @return
  void getMassData(MassData massData) {
    shape.computeMass(massData, _density);
  }

  /// Get the coefficient of friction.
  ///
  /// @return
  double getFriction() {
    return _friction;
  }

  /// Set the coefficient of friction. This will _not_ change the friction of existing contacts.
  ///
  /// @param friction
  void setFriction(double friction) {
    _friction = friction;
  }

  /// Get the coefficient of restitution.
  ///
  /// @return
  double getRestitution() {
    return _restitution;
  }

  /// Set the coefficient of restitution. This will _not_ change the restitution of existing
  /// contacts.
  ///
  /// @param restitution
  void setRestitution(double restitution) {
    _restitution = restitution;
  }

  /// Get the fixture's AABB. This AABB may be enlarge and/or stale. If you need a more accurate
  /// AABB, compute it using the shape and the body transform.
  ///
  /// @return
  AABB getAABB(int childIndex) {
    assert(childIndex >= 0 && childIndex < _proxyCount);
    return _proxies[childIndex].aabb;
  }

  /// Compute the distance from this fixture.
  ///
  /// @param p a point in world coordinates.
  /// @return distance
  double computeDistance(Vector2 p, int childIndex, Vector2 normalOut) {
    return shape.computeDistanceToOut(
        body._transform, p, childIndex, normalOut);
  }

  // We need separation create/destroy functions from the constructor/destructor because
  // the destructor cannot access the allocator (no destructor arguments allowed by C++).

  void create(Body body, FixtureDef def) {
    userData = def.userData;
    _friction = def.friction;
    _restitution = def.restitution;

    this.body = body;

    _filter.set(def.filter);

    _isSensor = def.isSensor;

    shape = def.shape.clone();

    // Reserve proxy space
    final int childCount = shape.getChildCount();
    if (_proxies.length < childCount) {
      final List<FixtureProxy> old = _proxies;
      final int newLength = math.max(old.length * 2, childCount);
      _proxies = List<FixtureProxy>.generate(
        newLength,
        (_) => FixtureProxy()..proxyId = BroadPhase.NULL_PROXY,
      );
    }
    _proxyCount = 0;
    _density = def.density;
  }

  // These support body activation/deactivation.
  void createProxies(BroadPhase broadPhase, final Transform xf) {
    assert(_proxyCount == 0);

    // Create proxies in the broad-phase.
    _proxyCount = shape.getChildCount();

    for (int i = 0; i < _proxyCount; ++i) {
      final FixtureProxy proxy = _proxies[i];
      shape.computeAABB(proxy.aabb, xf, i);
      proxy.proxyId = broadPhase.createProxy(proxy.aabb, proxy);
      proxy.fixture = this;
      proxy.childIndex = i;
    }
  }

  /// Internal method
  ///
  /// @param broadPhase
  void destroyProxies(BroadPhase broadPhase) {
    // Destroy proxies in the broad-phase.
    for (int i = 0; i < _proxyCount; ++i) {
      final FixtureProxy proxy = _proxies[i];
      broadPhase.destroyProxy(proxy.proxyId);
      proxy.proxyId = BroadPhase.NULL_PROXY;
    }

    _proxyCount = 0;
  }

  final AABB _pool1 = AABB();
  final AABB _pool2 = AABB();
  final Vector2 _displacement = Vector2.zero();

  /// Internal method
  ///
  /// @param broadPhase
  /// @param xf1
  /// @param xf2
  void synchronize(BroadPhase broadPhase, final Transform transform1,
      final Transform transform2) {
    if (_proxyCount == 0) {
      return;
    }

    for (int i = 0; i < _proxyCount; ++i) {
      final FixtureProxy proxy = _proxies[i];

      // Compute an AABB that covers the swept shape (may miss some rotation effect).
      final AABB aabb1 = _pool1;
      final AABB aab = _pool2;
      shape.computeAABB(aabb1, transform1, proxy.childIndex);
      shape.computeAABB(aab, transform2, proxy.childIndex);

      proxy.aabb.lowerBound.x = aabb1.lowerBound.x < aab.lowerBound.x
          ? aabb1.lowerBound.x
          : aab.lowerBound.x;
      proxy.aabb.lowerBound.y = aabb1.lowerBound.y < aab.lowerBound.y
          ? aabb1.lowerBound.y
          : aab.lowerBound.y;
      proxy.aabb.upperBound.x = aabb1.upperBound.x > aab.upperBound.x
          ? aabb1.upperBound.x
          : aab.upperBound.x;
      proxy.aabb.upperBound.y = aabb1.upperBound.y > aab.upperBound.y
          ? aabb1.upperBound.y
          : aab.upperBound.y;
      _displacement.x = transform2.p.x - transform1.p.x;
      _displacement.y = transform2.p.y - transform1.p.y;

      broadPhase.moveProxy(proxy.proxyId, proxy.aabb, _displacement);
    }
  }
}
