/// *****************************************************************************
/// Copyright (c) 2015, Daniel Murphy, Google
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without modification,
/// are permitted provided that the following conditions are met:
///  * Redistributions of source code must retain the above copyright notice,
///    this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above copyright notice,
///    this list of conditions and the following disclaimer in the documentation
///    and/or other materials provided with the distribution.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
/// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
/// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
/// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
/// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
/// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
/// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
/// *****************************************************************************

import 'dart:html';

import 'package:box2d_flame/box2d_browser.dart';

import 'bench2d.dart';

void main() {
  // Render version
  new Bench2dWeb()
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
    canvas = new CanvasElement()
      ..width = CANVAS_WIDTH
      ..height = CANVAS_HEIGHT;

    ctx = canvas.getContext("2d") as CanvasRenderingContext2D;
    document.body.append(canvas);

    // Create the viewport transform with the center at extents.
    final extents = new Vector2(CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2);
    viewport = new CanvasViewportTransform(extents, extents)
      ..scale = _VIEWPORT_SCALE;

    // Create our canvas drawing tool to give to the world.
    debugDraw = new CanvasDraw(viewport, ctx);

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
