# Change Log

All notable changes to this project will be documented in this file.
See [Conventional Commits](https://conventionalcommits.org) for commit guidelines.

## 2025-10-26

### Changes

---

Packages with breaking changes:

 - There are no breaking changes in this release.

Packages with other changes:

 - [`forge2d` - `v0.14.2+1`](#forge2d---v01421)

---

#### `forge2d` - `v0.14.2+1`

 - **FIX**: Create and destroy joints after world has been locked ([#110](https://github.com/flame-engine/forge2d/issues/110)). ([0704959f](https://github.com/flame-engine/forge2d/commit/0704959fa0ef904a056228846c006498d9361520))


## 2025-09-07

### Changes

---

Packages with breaking changes:

 - There are no breaking changes in this release.

Packages with other changes:

 - [`forge2d` - `v0.14.2`](#forge2d---v0142)

---

#### `forge2d` - `v0.14.2`

 - **FIX**: Loosen constraints on vector_math to ^2.1.4 ([#108](https://github.com/flame-engine/forge2d/issues/108)). ([c2ca6157](https://github.com/flame-engine/forge2d/commit/c2ca61574a3701cc4cd53372124c773b6b1d9333))


## 2025-09-07

### Changes

---

Packages with breaking changes:

 - There are no breaking changes in this release.

Packages with other changes:

 - [`forge2d` - `v0.14.1`](#forge2d---v0141)

---

#### `forge2d` - `v0.14.1`

 - **FIX**: Destroy and create bodies after step is done ([#106](https://github.com/flame-engine/forge2d/issues/106)). ([92c7ccb8](https://github.com/flame-engine/forge2d/commit/92c7ccb85774a86a1f0656226e06a2504cdd47a6))
 - **FIX**: Fix CircleShape.computeDistanceToOut and add some tests. ([#100](https://github.com/flame-engine/forge2d/issues/100)). ([584a07e2](https://github.com/flame-engine/forge2d/commit/584a07e261573fbc2d9e39590b0d20eea74d733a))


## 2024-12-14

### Changes

---

Packages with breaking changes:

 - [`forge2d` - `v0.14.0`](#forge2d---v0140)

Packages with other changes:

 - There are no other changes in this release.

---

#### `forge2d` - `v0.14.0`

 - **REFACTOR**: remove dead code ([#97](https://github.com/flame-engine/forge2d/issues/97)). ([6a75e0d2](https://github.com/flame-engine/forge2d/commit/6a75e0d2d4452bed3bfe6dbe35f84af9acebff76))
 - **BREAKING** **REFACTOR**: Use vector_math 32 instead of 64 ([#96](https://github.com/flame-engine/forge2d/issues/96)). ([2d726279](https://github.com/flame-engine/forge2d/commit/2d72627937e6b8dca672ed4dbc2de16577400623))


## 2024-08-06

### Changes

---

Packages with breaking changes:

 - There are no breaking changes in this release.

Packages with other changes:

 - [`forge2d` - `v0.13.1`](#forge2d---v0131)

---

#### `forge2d` - `v0.13.1`

 - **REFACTOR**: Don't use static for Distance and Collision  ([#92](https://github.com/flame-engine/forge2d/issues/92)). ([130994e1](https://github.com/flame-engine/forge2d/commit/130994e1f109071c690d4a8a970b28e106d9625e))


## 2024-04-05

### Changes

---

Packages with breaking changes:

 - [`forge2d` - `v0.13.0`](#forge2d---v0130)

Packages with other changes:

 - There are no other changes in this release.

---

#### `forge2d` - `v0.13.0`

 - **BREAKING** **REFACTOR**: Un-export `Timer` ([#90](https://github.com/flame-engine/forge2d/issues/90)). ([abf23783](https://github.com/flame-engine/forge2d/commit/abf23783c1d5dc3de6db2751f0d080816f76df20))


## 2023-11-03

### Changes

---

Packages with breaking changes:

 - There are no breaking changes in this release.

Packages with other changes:

 - [`forge2d` - `v0.12.2`](#forge2d---v0122)

---

#### `forge2d` - `v0.12.2`

 - **FIX**: Depend on any test version. ([1f80bf35](https://github.com/flame-engine/forge2d/commit/1f80bf35c84cec53a35bbf308c29810086597771))


## 2023-11-03

### Changes

---

Packages with breaking changes:

 - There are no breaking changes in this release.

Packages with other changes:

 - [`forge2d` - `v0.12.1`](#forge2d---v0121)

---

#### `forge2d` - `v0.12.1`

 - **FIX**: Lower meta bounds. ([39987d8d](https://github.com/flame-engine/forge2d/commit/39987d8d1a6b88f286d0e0cae109bba55be1159e))


## 2023-11-03

### Changes

---

Packages with breaking changes:

 - [`forge2d` - `v0.12.0`](#forge2d---v0120)

Packages with other changes:

 - There are no other changes in this release.

---

#### `forge2d` - `v0.12.0`

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


## 2022-04-11

### Changes

---

Packages with breaking changes:

 - [`forge2d` - `v0.11.0`](#forge2d---v0110)

Packages with other changes:

 - There are no other changes in this release.

---

#### `forge2d` - `v0.11.0`

 - **FEAT**: allowed specifying `BodyDef` properties via constructor (#51). ([aa3f5f7c](https://github.com/flame-engine/forge2d/commit/aa3f5f7cd604f1c7e3a8d90622dea812e3c63566))
 - **FEAT**: allowed specifying `FixtureDef` properties via constructor (#52). ([1c09a139](https://github.com/flame-engine/forge2d/commit/1c09a1395deb97b53638791238ab094870a201b8))
 - **BREAKING** **FEAT**: allow modifying gravity in the x-axis and y-axis (#53). ([c14b4245](https://github.com/flame-engine/forge2d/commit/c14b4245bd86f78418eb38a20ba0f0a62061af4f))
 - **BREAKING** **FEAT**: made default friciton 0 (#54). ([e943a567](https://github.com/flame-engine/forge2d/commit/e943a567483510dcebd08bc8e5d4b53366670c67))


## 2022-03-27

### Changes

---

Packages with breaking changes:

- [`forge2d` - `v0.10.0`](#forge2d---v0100)

Packages with other changes:

- There are no other changes in this release.

---

#### `forge2d` - `v0.10.0`

 - **REFACTOR**: moved Joint render method implementation (#49).
 - **REFACTOR**: removed mock instances (#48).
 - **BREAKING** **REFACTOR**: made `createJoint` use `Joint` over `JointDef` (#50).
 - **BREAKING** **FEAT**: remove `JointType` (#45).


## 2022-03-22

### Changes

---

Packages with breaking changes:

 - There are no breaking changes in this release.

Packages with other changes:

 - [`forge2d` - `v0.9.0`](#forge2d---v090)

---

#### `forge2d` - `v0.9.0`

