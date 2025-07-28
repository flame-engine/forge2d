import 'dart:js_interop';

import 'package:forge2d/forge2d_browser.dart';
import 'package:web/web.dart';

import 'bench2d.dart';

void main() {
  // Render version
  Bench2dWeb()
    ..initializeAnimation()
    ..runAnimation();
}

class Bench2dWeb extends Bench2d {
  static const int canvasWidth = 900;
  static const int canvasHeight = 600;
  static const double _viewportScale = 10.0;

  late HTMLCanvasElement canvas;
  late CanvasRenderingContext2D ctx;
  late ViewportTransform viewport;
  late DebugDraw debugDraw;

  /// Creates the canvas and readies the demo for animation. Must be called
  /// before calling runAnimation.
  void initializeAnimation() {
    // Setup the canvas.
    canvas = HTMLCanvasElement()
      ..width = canvasWidth
      ..height = canvasHeight;

    ctx = canvas.getContext('2d')! as CanvasRenderingContext2D;
    document.body?.append(canvas);

    // Create the viewport transform with the center at extents.
    final extents = Vector2(canvasWidth / 2, canvasHeight / 2);
    viewport = CanvasViewportTransform(extents, extents)
      ..scale = _viewportScale;

    // Create our canvas drawing tool to give to the world.
    debugDraw = CanvasDraw(viewport, ctx);

    // Have the world draw itself for debugging purposes.
    world.debugDraw = debugDraw;

    initialize();
  }

  void render(num delta) {
    super.step();

    ctx.clearRect(0, 0, canvasWidth, canvasHeight);
    world.drawDebugData();
    window.requestAnimationFrame(render.toJS);
  }

  void runAnimation() {
    window.requestAnimationFrame(render.toJS);
  }
}
