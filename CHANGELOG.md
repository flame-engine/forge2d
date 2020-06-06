### 0.4.6
* Fix bug with torque not being modified from applyTorque

### 0.4.5
* Fixing warnings and formatting

### 0.4.1 - 0.4.4
* Updates related to Flame interop + bugfixes

### 0.4.0

* Breaking: Made package strong-mode compliant 
  (with `implicit-casts` and `implicit-dynamic` off). This means many API points
  are now more strongly typed (`num` is now `double`). Fix errors by providing
  doubles where they are expected (`1` will become `1.0` and 
  `num fraction = .5` will become `double fraction = .5`), and explicitly 
  casting joints (`world.createJoint(jointDef) as RevoluteJoint`).
* Pulled in optimizations and bug fixes from the original jbox2d project.
* Made `velocityThreshold` mutable (as per jbox2d and other box2d 
  implementations).

### 0.3.0

* Updated to vector\_math 2.0.0

### 0.2.0

* Complete re-write. Many APIs have changed.
