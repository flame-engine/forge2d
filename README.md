#

<p align="center">
  <a href="https://flame-engine.org">
    <img alt="flame" width="200px" src="https://raw.githubusercontent.com/flame-engine/forge2d/master/design/with-text.png">
  </a>
</p>

<p align="center">
  Forge2D - Dart bindings for the Box2D physics engine
</p>

<p align="center">
  <a title="Pub" href="https://pub.dartlang.org/packages/forge2d" ><img src="https://img.shields.io/pub/v/forge2d.svg?style=popout" /></a> <img src="https://github.com/flame-engine/forge2d/workflows/cicd/badge.svg?branch=main&event=push" alt="Test" /> <a title="Discord" href="https://discord.gg/pxrBmy4" ><img src="https://img.shields.io/discord/509714518008528896.svg" /></a>
</p>

Forge2D provides an idiomatic Dart API for the native
[Box2D](https://box2d.org) v3 physics engine. The C library is bundled,
compiled by the Dart/Flutter build through
[native assets](https://dart.dev/tools/hooks), and called over `dart:ffi`,
so simulations run at native speed with the real, actively maintained
Box2D.

You can use it independently in Dart or in your
[Flame](https://github.com/flame-engine/flame) project with the help of
[flame_forge2d](https://github.com/flame-engine/flame/tree/main/packages/flame_forge2d).

## Requirements

- Dart 3.12+ (Flutter 3.38+), which builds the bundled C library
  automatically through build hooks.
- A C toolchain on native platforms: Xcode on macOS/iOS, Visual Studio
  Build Tools on Windows, clang or gcc on Linux, and the NDK on Android.
- Supported platforms: Android, iOS, macOS, Windows, Linux, and the web.

On the web the same API runs against a bundled WebAssembly build of Box2D
(about 220 KB). No setup is needed: `initializeForge2D()` finds the module
at the package asset path in Dart web apps, and at the bundled package
asset in Flutter web apps. For custom hosting setups the location can be
overridden with `initializeForge2D(wasmUri: ...)`, and
`dart run forge2d:setup_web` copies the module into a `web/` directory.

## Getting started

```dart
import 'package:forge2d/forge2d.dart';

Future<void> main() async {
  await initializeForge2D();

  final world = World();

  // Static ground: a wide box whose top surface is at y = 0.
  world
      .createBody(BodyDef(position: Vector2(0, -1)))
      .createShape(Polygon.box(50, 1));

  // A dynamic box dropped from above.
  final box = world.createBody(
    BodyDef(type: BodyType.dynamic, position: Vector2(0, 10)),
  )..createShape(Polygon.square(0.5));

  for (var i = 0; i < 90; i++) {
    world.step(1 / 60);
  }
  print(box.position); // The box has landed on the ground.

  world.destroy();
}
```

Highlights of the API:

- `World`, `Body`, `Shape`, `Chain`, and the joints are cheap value-like
  handles over native ids; destroy things explicitly with `destroy()`.
- Contact, sensor, and body-move events are polled from the world after
  each step (`world.contactEvents`, `world.sensorEvents`,
  `world.bodyMoveEvents`).
- Ray casts (`castRayClosest`, `castRayAll`, `castRay`), AABB overlap
  queries, and explosions are available on `World`.
- `DebugDraw` can be implemented to render the physics world for
  debugging.

## Performance

The standard [bench2d](https://github.com/joelgwebber/bench2d) benchmark
(a 40-high pyramid of boxes, 256 frames), on the same machine:

| Engine | ms/frame (mean) |
|---|---|
| forge2d 0.14 (pure Dart port) | 9.37 |
| forge2d with native Box2D v3 | 0.51 |

## Migrating from forge2d 0.14

Forge2D 0.15 is a ground-up rewrite on the Box2D v3 API. For the full
walkthrough, see the [Forge2D migration
guide](https://docs.flame-engine.org/main/other_modules/forge2d/migration.html).
The high-level concepts map as follows:

- Call `await initializeForge2D()` once before creating a world.
- `Fixture` is gone: bodies now carry `Shape`s directly, created with
  `body.createShape(geometry, ShapeDef(...))` where the geometry is a
  `Circle`, `Capsule`, `Segment`, `Polygon`, or a `Chain` via
  `body.createChain`.
- `EdgeShape` is replaced by `Segment` (standalone) and one-sided chain
  shapes for level geometry.
- `ContactListener` callbacks are replaced by polled events:
  `world.contactEvents` after each step, opted in per shape with
  `ShapeDef(enableContactEvents: true)`.
- Query and ray-cast callback classes are replaced by methods on `World`
  returning results directly.
- The joints are now distance, filter, motor, mouse, prismatic, revolute,
  weld, and wheel. Gear, pulley, rope, friction, and constant-volume
  joints do not exist in Box2D v3.
- The particle system (LiquidFun) is not part of Box2D v3 and has been
  removed; stay on forge2d 0.14 if you depend on it.
- Worlds default to `subStepCount: 4` in `step` instead of velocity and
  position iterations.
- A bare `World()` now has the Box2D default gravity of `(0, -10)`; the old
  API defaulted to zero gravity. Top-down games should pass
  `World(gravity: Vector2.zero())`.
- Destroying bodies, shapes, chains, or joints while the world is stepping
  (from a collision callback) is deferred until the step ends; creating
  them mid-step throws a `StateError` instead of the old silent queueing.

## Timeline

Box2D was first written in C++ and released by
[Erin Catto](https://github.com/erincatto) in 2007, and it is still
actively maintained.

It was ported to Java (jbox2d) by [Daniel Murphy](https://github.com/dmurph)
around 2015, then from that Java port it was ported to Dart by
[Dominic Hamon](https://github.com/dominichamon) and
[Kevin Moore](https://github.com/kevmoo).

A few years after that [Lukas Klingsbo](https://github.com/spydon)
refactored the code to better follow the Dart standard and the project was
renamed to Forge2D.

Since Box2D v3 rewrote the engine in C with a first-class embedding API,
Forge2D moved from being a port to being bindings: the same idiomatic Dart
surface, powered by the real engine.

There have also been countless other contributors which we are very
thankful to!

## Credits

 * The [Flame engine team](https://github.com/orgs/flame-engine/people) who
   is continuously working on maintaining and improving Forge2D.
 * [Erin Catto](https://github.com/erincatto) for Box2D itself, which this
   package embeds.
 * Special thanks to [Lukas Klingsbo](https://github.com/spydon), a Flame
   team member who took this project under his wing and greatly improved
   the project!
 * The Dart port of [Box2D](https://github.com/google/box2d.dart) that
   earlier versions of Forge2D were built on.
