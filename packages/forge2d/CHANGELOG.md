## 0.12.1

 - **FIX**: Lower meta bounds. ([39987d8d](https://github.com/flame-engine/forge2d/commit/39987d8d1a6b88f286d0e0cae109bba55be1159e))

## 0.12.0

> Note: This release has breaking changes.

 - **REFACTOR**: ignored invariant boolean warning ([#47](https://github.com/flame-engine/forge2d/issues/47)). ([c032f9c8](https://github.com/flame-engine/forge2d/commit/c032f9c8b14521cf5fde87974696dd741a4cbb36))
 - **FIX**: Fixed issue where particles created were centred around (0,0) rather than their parent group's position. ([#69](https://github.com/flame-engine/forge2d/issues/69)). ([a36e88e5](https://github.com/flame-engine/forge2d/commit/a36e88e5d611c9b63ceb9aed7881e6f46e92a483))
 - **FIX**: Properly use the `getX/YAxis` out vector ([#61](https://github.com/flame-engine/forge2d/issues/61)). ([c1aba67d](https://github.com/flame-engine/forge2d/commit/c1aba67d950d38838155fba5abee8b2df477c7b8))
 - **FEAT**: made ContactListener methods optional overrides ([#55](https://github.com/flame-engine/forge2d/issues/55)). ([47037a14](https://github.com/flame-engine/forge2d/commit/47037a140d2c2dd3cfa64aeb7c9299a134afcb42))
 - **BREAKING** **REFACTOR**: gravity getter and setter ([#66](https://github.com/flame-engine/forge2d/issues/66)). ([5108e302](https://github.com/flame-engine/forge2d/commit/5108e30217a55bd04c2a63ea053975c42fef5ad6))
 - **BREAKING** **REFACTOR**: isBullet getter and setter ([#64](https://github.com/flame-engine/forge2d/issues/64)). ([ee5d4bf0](https://github.com/flame-engine/forge2d/commit/ee5d4bf056e566eae81324971e2306a5ed970f45))
 - **BREAKING** **REFACTOR**: `isEnabled` getter and setter ([#65](https://github.com/flame-engine/forge2d/issues/65)). ([839e53a1](https://github.com/flame-engine/forge2d/commit/839e53a1e73ceb4826cd774df89da7b22025652b))
 - **BREAKING** **REFACTOR**: removed wrapper get and set methods for GearJoint.ratio ([#63](https://github.com/flame-engine/forge2d/issues/63)). ([58b51159](https://github.com/flame-engine/forge2d/commit/58b511599f83ec767faf3913a12612872c961c0f))
 - **BREAKING** **REFACTOR**: used getter and setters for maxMotorForce ([#62](https://github.com/flame-engine/forge2d/issues/62)). ([4ab2eb27](https://github.com/flame-engine/forge2d/commit/4ab2eb2738e2be62f0ee411a51742c5ce65686b3))
 - **BREAKING** **REFACTOR**: used setters and getters for motorSpeed ([#58](https://github.com/flame-engine/forge2d/issues/58)). ([96103f18](https://github.com/flame-engine/forge2d/commit/96103f180df30d17b3ab3c9309927e35df67bb96))
 - **BREAKING** **FIX**: Add friction and restitution arguments to createFixtureFromShape ([#56](https://github.com/flame-engine/forge2d/issues/56)). ([a91b08b6](https://github.com/flame-engine/forge2d/commit/a91b08b6e7d6083eedf270a1c0d77a29b75c9263))
 - **BREAKING** **FIX**: Set the RopeJoint's local anchors to null vectors by default ([#67](https://github.com/flame-engine/forge2d/issues/67)). ([2dfe1e8e](https://github.com/flame-engine/forge2d/commit/2dfe1e8e82a25b5dd352638fcc7d284190cd25d1))
 - **BREAKING** **FEAT**: Move to dart 2.17 and clean up comment. ([#59](https://github.com/flame-engine/forge2d/issues/59)). ([8abd859f](https://github.com/flame-engine/forge2d/commit/8abd859fc7b0fe721e8941cdc1f2dcd1ba60204f))

## 0.11.0

> Note: This release has breaking changes.

 - **FEAT**: allowed specifying `BodyDef` properties via constructor (#51). ([aa3f5f7c](https://github.com/flame-engine/forge2d/commit/aa3f5f7cd604f1c7e3a8d90622dea812e3c63566))
 - **FEAT**: allowed specifying `FixtureDef` properties via constructor (#52). ([1c09a139](https://github.com/flame-engine/forge2d/commit/1c09a1395deb97b53638791238ab094870a201b8))
 - **BREAKING** **FEAT**: allow modifying gravity in the x-axis and y-axis (#53). ([c14b4245](https://github.com/flame-engine/forge2d/commit/c14b4245bd86f78418eb38a20ba0f0a62061af4f))
 - **BREAKING** **FEAT**: made default friciton 0 (#54). ([e943a567](https://github.com/flame-engine/forge2d/commit/e943a567483510dcebd08bc8e5d4b53366670c67))

## 0.10.0

> Note: This release has breaking changes.

 - **REFACTOR**: moved Joint render method implementation (#49). ([b9128e5a](https://github.com/flame-engine/forge2d/commit/b9128e5a589140b596cf377faeaaf1140ada3c9e))
 - **REFACTOR**: removed mock instances (#48). ([6fdf5646](https://github.com/flame-engine/forge2d/commit/6fdf5646a36b2749481269bfc56a0b88f35516c3))
 - **BREAKING** **REFACTOR**: made `createJoint` use `Joint` over `JointDef` (#50). ([0e8f3655](https://github.com/flame-engine/forge2d/commit/0e8f3655b75d1c7c516b0ac79c7b6bde54fd3fcf))
 - **BREAKING** **FEAT**: remove `JointType` (#45). ([42a39524](https://github.com/flame-engine/forge2d/commit/42a395241bb9c70932a1bbf6e84f936b52965d35))

## 0.9.0

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
