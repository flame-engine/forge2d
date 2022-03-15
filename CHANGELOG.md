# CHANGELOG

## 0.8.2
 - Fix `ConcurrentModificationError` when destroying a body with joints

## 0.8.1
 - Export `settings.dart`
 - Make fields in `settings.dart` mutable

## 0.8.0
 - Move `velocityIterations` and `positionIterations` to settings instead of `stepDt`
 - Make `settings.velocityThreshold` mutable

## 0.7.3
 - Fix bug with the hull creation of `PolygonShape`

## 0.7.2
 - Use field getters on getter methods where computation is O(1)

## 0.7.1
 - Fix area for polygon that is defined counter-clockwise

## 0.7.0
 - Refactored the particle system
 - Cleaned up the code and remove buffer_utils
 - Fixed y-flip to only be used on getScreenToWorld and getWorldToScreen
 - Removed the get prefix from getScreenToWorld and getWorldToScreen
 - Move debug draw methods to respective classes
 - Move web dependencies to example
 - Stable null-safety release

## 0.6.5
 - Consider y-flip for all translation in the viewport
 
## 0.6.4
 - Consider y-flip in ViewportTransform

## 0.6.3
 - Remove faulty assert in world.destroyJoint

## 0.6.2
 - One more bug fix with deleting contacts when called Body.removeFixture() when it's in contact

## 0.6.1
 - Fix bug that throws error during World.destroyBody(), Body.setType() and Body.setActive() if body has some contacts

## 0.6.0
 - Refactor joints, fixtures and contacts not to be linked lists
 - Make force/impulses default to the center of a body

## 0.5.0
 - Rebranding to forge2d
 - Make code more like the dart standard and less C++

## Old box2d.dart

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
