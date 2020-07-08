library demo;

import 'dart:async';
import 'dart:html' hide Body;
import 'package:box2d_flame/box2d_browser.dart' hide Timer;

/// An abstract class for any Demo of the Box2D library.
abstract class Demo {
  static const int WORLD_POOL_SIZE = 100;
  static const int WORLD_POOL_CONTAINER_SIZE = 10;

  /// All of the bodies in a simulation.
  List<Body> bodies = List<Body>();

  /// The default canvas width and height.
  static const int CANVAS_WIDTH = 900;
  static const int CANVAS_HEIGHT = 600;

  /// Scale of the viewport.
  static const double _VIEWPORT_SCALE = 10.0;

  /// The gravity vector's y value.
  static const double GRAVITY = -10.0;

  /// The timestep and iteration numbers.
  static const double TIME_STEP = 1 / 60;
  static const int VELOCITY_ITERATIONS = 10;
  static const int POSITION_ITERATIONS = 10;

  /// The physics world.
  final World world;

  // For timing the world.step call. It is kept running but reset and polled
  // every frame to minimize overhead.
  final Stopwatch _stopwatch;

  final double _viewportScale;

  /// The drawing canvas.
  CanvasElement canvas;

  /// The canvas rendering context.
  CanvasRenderingContext2D ctx;

  /// The transform abstraction layer between the world and drawing canvas.
  ViewportTransform viewport;

  /// The debug drawing tool.
  DebugDraw debugDraw;

  /// Frame count for fps
  int frameCount;

  /// HTML element used to display the FPS counter
  Element fpsCounter;

  /// Microseconds for world step update
  int elapsedUs;

  /// HTML element used to display the world step time
  Element worldStepTime;

  Demo(String name, [Vector2 gravity, this._viewportScale = _VIEWPORT_SCALE])
      : this.world = World.withGravity(gravity ??= Vector2(0.0, GRAVITY)),
        _stopwatch = Stopwatch()..start() {
    querySelector("#title").innerHtml = name;
  }

  /// Advances the world forward by timestep seconds.
  void step(num timestamp) {
    _stopwatch.reset();
    world.stepDt(TIME_STEP, VELOCITY_ITERATIONS, POSITION_ITERATIONS);
    elapsedUs = _stopwatch.elapsedMicroseconds;

    // Clear the animation panel and draw new frame.
    ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
    world.drawDebugData();
    frameCount++;

    window.requestAnimationFrame(step);
  }

  /// Creates the canvas and readies the demo for animation. Must be called
  /// before calling runAnimation.
  void initializeAnimation() {
    // Setup the canvas.
    canvas = (Element.tag('canvas') as CanvasElement)
      ..width = CANVAS_WIDTH
      ..height = CANVAS_HEIGHT;
    document.body.nodes.add(canvas);
    ctx = canvas.context2D;

    // Create the viewport transform with the center at extents.
    var extents = Vector2(CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2);
    viewport = CanvasViewportTransform(extents, extents)
      ..scale = _viewportScale;

    // Create our canvas drawing tool to give to the world.
    debugDraw = CanvasDraw(viewport, ctx);

    // Have the world draw itself for debugging purposes.
    world.debugDraw = debugDraw;

    frameCount = 0;
    fpsCounter = querySelector("#fps-counter");
    worldStepTime = querySelector("#world-step-time");
    Timer.periodic(Duration(seconds: 1), (Timer t) {
      fpsCounter.innerHtml = frameCount.toString();
      frameCount = 0;
    });
    Timer.periodic(Duration(milliseconds: 200), (Timer t) {
      if (elapsedUs == null) return;
      worldStepTime.innerHtml = "${elapsedUs / 1000} ms";
    });
  }

  void initialize();

  /// Starts running the demo as an animation using an animation scheduler.
  void runAnimation() {
    window.requestAnimationFrame(step);
  }
}
