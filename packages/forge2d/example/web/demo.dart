import 'dart:async';
import 'dart:html' hide Body;

import 'package:forge2d/forge2d_browser.dart';

/// An abstract class for any Demo of the Forge2D library.
abstract class Demo {
  /// All of the bodies in a simulation.
  List<Body> bodies = <Body>[];

  /// The default canvas width and height.
  static const int canvasWidth = 900;
  static const int canvasHeight = 600;

  /// Scale of the viewport.
  static const double viewportScale = 10.0;

  /// The gravity vector's y value.
  static const double _gravity = -10.0;

  /// The timestep and iteration numbers.
  static const double timeStep = 1 / 60;

  /// The physics world.
  final World world;

  // For timing the world.step call. It is kept running but reset and polled
  // every frame to minimize overhead.
  final Stopwatch _stopwatch;

  final double _viewportScale;

  /// The drawing canvas.
  late final CanvasElement canvas;

  /// The canvas rendering context.
  late final CanvasRenderingContext2D ctx;

  /// The transform abstraction layer between the world and drawing canvas.
  late final ViewportTransform viewport;

  /// The debug drawing tool.
  late final DebugDraw debugDraw;

  /// Frame count for fps
  int frameCount = 0;

  /// HTML element used to display the FPS counter
  late final Element fpsCounter;

  /// Microseconds for world step update
  int? elapsedUs;

  /// HTML element used to display the world step time
  late final Element worldStepTime;

  Demo(String name, [Vector2? gravity, this._viewportScale = viewportScale])
      : world = World(gravity ?? Vector2(0.0, _gravity)),
        _stopwatch = Stopwatch()..start() {
    world.setAllowSleep(true);
    querySelector('#title')?.innerHtml = name;
  }

  /// Advances the world forward by timestep seconds.
  void step(num timestamp) {
    _stopwatch.reset();
    world.stepDt(timeStep);
    elapsedUs = _stopwatch.elapsedMicroseconds;

    // Clear the animation panel and draw new frame.
    ctx.clearRect(0, 0, canvasWidth, canvasHeight);
    world.drawDebugData();
    frameCount++;

    window.requestAnimationFrame(step);
  }

  /// Creates the canvas and readies the demo for animation. Must be called
  /// before calling runAnimation.
  void initializeAnimation() {
    // Setup the canvas.
    canvas = (Element.tag('canvas') as CanvasElement)
      ..width = canvasWidth
      ..height = canvasHeight;
    document.body?.nodes.add(canvas);
    ctx = canvas.context2D;

    // Create the viewport transform with the center at extents.
    final extents = Vector2(canvasWidth / 2, canvasHeight / 2);
    viewport = CanvasViewportTransform(extents, extents)
      ..scale = _viewportScale;

    // Create our canvas drawing tool to give to the world.
    debugDraw = CanvasDraw(viewport, ctx);

    // Have the world draw itself for debugging purposes.
    world.debugDraw = debugDraw;

    fpsCounter = querySelector('#fps-counter')!;
    worldStepTime = querySelector('#world-step-time')!;
    Timer.periodic(const Duration(seconds: 1), (Timer t) {
      fpsCounter.innerHtml = frameCount.toString();
      frameCount = 0;
    });
    Timer.periodic(const Duration(milliseconds: 200), (Timer t) {
      if (elapsedUs == null) {
        return;
      }
      worldStepTime.innerHtml = '${elapsedUs! / 1000} ms';
    });
  }

  void initialize();

  /// Starts running the demo as an animation using an animation scheduler.
  void runAnimation() {
    window.requestAnimationFrame(step);
  }
}
