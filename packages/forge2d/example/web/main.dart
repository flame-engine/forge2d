import 'dart:js_interop';

import 'package:forge2d/forge2d.dart';
import 'package:web/web.dart';

import 'canvas_debug_draw.dart';
import 'scenes.dart';

Future<void> main() async {
  await initializeForge2D();
  document.getElementById('loading')?.remove();
  _App().start();
}

/// The DOM events the app listens to.
enum DomEvent {
  keyDown,
  keyUp,
  pointerDown,
  pointerMove,
  pointerUp,
  pointerCancel,
  click;

  /// The DOM event type name.
  String get type => name.toLowerCase();
}

void _listen<T extends Event>(
  DomEvent event,
  EventTarget target,
  void Function(T) handler,
) => target.addEventListener(
  event.type,
  ((Event domEvent) => handler(domEvent as T)).toJS,
);

class _App {
  _App()
    : _canvas = document.getElementById('canvas')! as HTMLCanvasElement,
      _hint = document.getElementById('hint')! as HTMLDivElement,
      _nav = document.getElementById('nav')! as HTMLDivElement {
    _draw = CanvasDebugDraw(
      _canvas.getContext('2d')! as CanvasRenderingContext2D,
    );
  }

  final HTMLCanvasElement _canvas;
  final HTMLDivElement _hint;
  final HTMLDivElement _nav;
  late final CanvasDebugDraw _draw;

  World? _world;
  SceneHooks _hooks = SceneHooks();
  Scene _currentScene = scenes.first;
  MouseJoint? _mouseJoint;
  double _lastTimestamp = 0;
  double _accumulator = 0;

  static const _timeStep = 1 / 60;

  void start() {
    for (final scene in scenes) {
      final button = HTMLButtonElement()
        ..textContent = scene.name
        ..onClick.listen((_) => _select(scene));
      _nav.append(button);
    }

    _listen<KeyboardEvent>(
      DomEvent.keyDown,
      document,
      (event) => _hooks.onKey?.call(event.key, down: true),
    );
    _listen<KeyboardEvent>(
      DomEvent.keyUp,
      document,
      (event) => _hooks.onKey?.call(event.key, down: false),
    );
    _listen<Event>(
      DomEvent.click,
      document.getElementById('reset')!,
      (_) => _select(_currentScene),
    );
    _listen<PointerEvent>(DomEvent.pointerDown, _canvas, _onPointerDown);
    _listen<PointerEvent>(DomEvent.pointerMove, _canvas, _onPointerMove);
    _listen<PointerEvent>(
      DomEvent.pointerUp,
      _canvas,
      (_) => _releaseMouseJoint(),
    );
    _listen<PointerEvent>(
      DomEvent.pointerCancel,
      _canvas,
      (_) => _releaseMouseJoint(),
    );

    final initial = Uri.base.fragment;
    _select(
      scenes.firstWhere(
        (scene) => _slug(scene.name) == initial,
        orElse: () => scenes.first,
      ),
    );
    window.requestAnimationFrame(_frame.toJS);
  }

  String _slug(String name) => name.toLowerCase().replaceAll(' ', '-');

  void _select(Scene scene) {
    _releaseMouseJoint();
    _world?.destroy();
    _currentScene = scene;
    _hooks = SceneHooks();
    _world = World()..let(scene.build, _hooks);
    _draw
      ..center = Vector2(0, scene.centerY)
      ..viewHeight = scene.viewHeight;
    _hint.textContent = scene.hint;
    window.location.hash = _slug(scene.name);

    final buttons = _nav.querySelectorAll('button');
    for (var i = 0; i < buttons.length; i++) {
      final button = buttons.item(i)! as HTMLButtonElement;
      button.className = button.textContent == scene.name ? 'active' : '';
    }
  }

  void _frame(JSNumber timestamp) {
    final now = timestamp.toDartDouble / 1000;
    final elapsed = _lastTimestamp == 0
        ? _timeStep
        : (now - _lastTimestamp).clamp(0.0, 0.1);
    _lastTimestamp = now;
    _accumulator += elapsed;

    final world = _world;
    if (world != null) {
      while (_accumulator >= _timeStep) {
        _hooks.onTick?.call(_timeStep);
        world.step(_timeStep);
        _accumulator -= _timeStep;
      }
      if (_hooks.cameraFollow case final follow?) {
        _draw.center.setFrom(follow());
      }
      _resizeCanvas();
      _draw.beginFrame(_canvas.width.toDouble(), _canvas.height.toDouble());
      _renderShapes(world);
      // Joints still come from the debug draw; shape drawing is off because
      // _renderShapes draws them from their geometry.
      world.draw(_draw..drawShapes = false);
    }
    window.requestAnimationFrame(_frame.toJS);
  }

  /// Renders every shape in view from its read-back geometry and its
  /// body's transform, the same pattern a game renderer uses. The camera
  /// rectangle doubles as a culling query.
  void _renderShapes(World world) {
    final viewHalfHeight = _draw.viewHeight / 2;
    final viewHalfWidth =
        viewHalfHeight * (_canvas.width / _canvas.height.clamp(1, 1 << 16));
    final margin = Vector2(viewHalfWidth + 5, viewHalfHeight + 5);
    final view = Aabb(_draw.center - margin, _draw.center + margin);

    for (final shape in world.overlapAabb(view)) {
      final transform = shape.body.transform;
      final color = shape.userData is int
          ? shape.userData! as int
          : Palette.slate;
      switch (shape.geometry) {
        case Circle(:final center, :final radius):
          _draw.drawSolidCircle(
            Transform(transform.apply(center), transform.rotation),
            radius,
            color,
          );
        case Capsule(:final center1, :final center2, :final radius):
          _draw.drawSolidCapsule(
            transform.apply(center1),
            transform.apply(center2),
            radius,
            color,
          );
        case Segment(:final point1, :final point2):
          _draw.drawSegment(
            transform.apply(point1),
            transform.apply(point2),
            color,
          );
        case Polygon(:final points, :final radius):
          _draw.drawSolidPolygon(transform, points!, radius, color);
      }
    }
  }

  void _resizeCanvas() {
    final ratio = window.devicePixelRatio;
    final width = (_canvas.clientWidth * ratio).round();
    final height = (_canvas.clientHeight * ratio).round();
    if (_canvas.width != width || _canvas.height != height) {
      _canvas
        ..width = width
        ..height = height;
    }
  }

  Vector2 _worldPoint(PointerEvent event) {
    final rectangle = _canvas.getBoundingClientRect();
    final ratio = window.devicePixelRatio;
    return _draw.toWorld(
      (event.clientX - rectangle.left) * ratio,
      (event.clientY - rectangle.top) * ratio,
    );
  }

  void _onPointerDown(PointerEvent event) {
    final world = _world;
    final anchor = _hooks.anchor;
    if (world == null || anchor == null) {
      return;
    }
    final point = _worldPoint(event);
    final nearby = world.overlapAabb(
      Aabb(point - Vector2(0.1, 0.1), point + Vector2(0.1, 0.1)),
    );
    for (final shape in nearby) {
      final body = shape.body;
      if (body.type == BodyType.dynamic && shape.testPoint(point)) {
        _mouseJoint = world.createMouseJoint(
          MouseJointDef(
            bodyA: anchor,
            bodyB: body,
            target: point,
            hertz: 6,
            dampingRatio: 0.8,
            maxForce: 1500 * body.mass,
          ),
        );
        body.isAwake = true;
        _canvas.setPointerCapture(event.pointerId);
        return;
      }
    }
  }

  void _onPointerMove(PointerEvent event) {
    final joint = _mouseJoint;
    if (joint != null && joint.isValid) {
      joint.target = _worldPoint(event);
      joint.bodyB.isAwake = true;
    }
  }

  void _releaseMouseJoint() {
    if (_mouseJoint case final joint? when joint.isValid) {
      joint.destroy();
    }
    _mouseJoint = null;
  }
}

extension<T> on T {
  /// Calls [function] with this value and [argument], enabling cascades.
  void let<A>(void Function(T, A) function, A argument) =>
      function(this, argument);
}
