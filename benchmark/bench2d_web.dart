import 'dart:html';

import 'package:box2d_flame/box2d_browser.dart';

import 'bench2d.dart';

void main() {
  // Render version
  Bench2dWeb()
    ..initializeAnimation()
    ..runAnimation();
}

class Bench2dWeb extends Bench2d {
  static const int CANVAS_WIDTH = 900;
  static const int CANVAS_HEIGHT = 600;
  static const double _VIEWPORT_SCALE = 10.0;

  CanvasElement canvas;
  CanvasRenderingContext2D ctx;
  ViewportTransform viewport;
  DebugDraw debugDraw;

  /// Creates the canvas and readies the demo for animation. Must be called
  /// before calling runAnimation.
  void initializeAnimation() {
    // Setup the canvas.
    canvas = CanvasElement()
      ..width = CANVAS_WIDTH
      ..height = CANVAS_HEIGHT;

    ctx = canvas.getContext("2d") as CanvasRenderingContext2D;
    document.body.append(canvas);

    // Create the viewport transform with the center at extents.
    final extents = Vector2(CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2);
    viewport = CanvasViewportTransform(extents, extents)
      ..scale = _VIEWPORT_SCALE;

    // Create our canvas drawing tool to give to the world.
    debugDraw = CanvasDraw(viewport, ctx);

    // Have the world draw itself for debugging purposes.
    world.debugDraw = debugDraw;

    initialize();
  }

  void render(num delta) {
    super.step();

    ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
    world.drawDebugData();
    window.animationFrame.then(render);
  }

  void runAnimation() {
    window.animationFrame.then(render);
  }
}
