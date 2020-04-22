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

library box2d.callbacks.canvas_draw;

import 'dart:html';

import 'package:box2d_flame/box2d.dart';
import 'package:box2d_flame/src/math_utils.dart' as MathUtils;

class CanvasDraw extends DebugDraw {
  /// The canvas rendering context with which to draw.
  final CanvasRenderingContext2D ctx;

  CanvasDraw(ViewportTransform viewport, this.ctx) : super(viewport) {
    assert(null != viewport && null != ctx);
  }

  /// Draw a closed polygon provided in CCW order. WARNING: This mutates
  /// [vertices].
  void drawPolygon(List<Vector2> vertices, int vertexCount, Color3i color) {
    _pathPolygon(vertices, vertexCount, color);
    ctx.stroke();
  }

  /// Draw a solid closed polygon provided in CCW order. WARNING: This mutates
  /// [vertices].
  void drawSolidPolygon(
      List<Vector2> vertices, int vertexCount, Color3i color) {
    _pathPolygon(vertices, vertexCount, color);
    ctx.fill();
  }

  void _pathPolygon(List<Vector2> vertices, int vertexCount, Color3i color) {
    // Set the color and convert to screen coordinates.
    _setColor(color);
    // TODO(gregbglw): Do a single ctx transform rather than convert all of
    // these vectors.
    for (int i = 0; i < vertexCount; ++i) {
      vertices[i] = getWorldToScreenToOut(vertices[i]);
    }

    ctx.beginPath();
    ctx.moveTo(vertices[0].x, vertices[0].y);

    // Draw lines to all of the remaining points.
    for (int i = 1; i < vertexCount; ++i) {
      ctx.lineTo(vertices[i].x, vertices[i].y);
    }

    // Draw a line back to the starting point.
    ctx.lineTo(vertices[0].x, vertices[0].y);

    // Close the drawn polygon ready for fill/stroke
    ctx.closePath();
  }

  /// Draw a line segment. WARNING: This mutates [p1] and [p2].
  void drawSegment(Vector2 p1, Vector2 p2, Color3i color) {
    _setColor(color);
    p1 = getWorldToScreenToOut(p1);
    p2 = getWorldToScreenToOut(p2);

    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.closePath();
    ctx.stroke();
  }

  /// Draw a circle. WARNING: This mutates [center].
  void drawCircle(Vector2 center, num radius, Color3i color, [Vector2 axis]) {
    radius *= viewportTransform.scale;
    _pathCircle(center, radius, color);
    ctx.stroke();
  }

  /// Draw a solid circle. WARNING: This mutates [center].
  void drawSolidCircle(
      Vector2 center, num radius, Vector2 axis, Color3i color) {
    radius *= viewportTransform.scale;
    drawPoint(center, radius, color);
  }

  /// Draws the given point with the given *unscaled* radius, in the given [color].
  /// WARNING: This mutates [point].
  void drawPoint(Vector2 point, num radiusOnScreen, Color3i color) {
    _pathCircle(point, radiusOnScreen, color);
    ctx.fill();
  }

  void _pathCircle(Vector2 center, num radius, Color3i color) {
    _setColor(color);
    center = getWorldToScreenToOut(center);

    ctx.beginPath();
    ctx.arc(center.x, center.y, radius, 0, MathUtils.TWOPI, true);
    ctx.closePath();
  }

  /// Draw a transform. Choose your own length scale. WARNING: This mutates
  /// [xf.position].
  void drawTransform(Transform xf, Color3i color) {
    drawCircle(xf.p, 0.1, color);
    // TODO(rupertk): Draw rotation representation (drawCircle axis parameter?)
  }

  /// Draw a string.
  void drawStringXY(num x, num y, String s, Color3i color) {
    _setColor(color);
    ctx.strokeText(s, x, y);
  }

  /// Sets the rendering context stroke and fill color to [color].
  void _setColor(Color3i color) {
    ctx.setStrokeColorRgb(color.x, color.y, color.z, 0.9);
    ctx.setFillColorRgb(color.x, color.y, color.z, 0.8);
  }

  void drawParticles(List<Vector2> centers, double radius,
      List<ParticleColor> colors, int count) {
    throw "Unimplemented";
  }

  void drawParticlesWireframe(List<Vector2> centers, double radius,
      List<ParticleColor> colors, int count) {
    throw "Unimplemented";
  }
}
