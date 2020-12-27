library forge2d.callbacks.canvas_draw;

import 'dart:html';
import 'dart:math';

import '../../forge2d.dart';

class CanvasDraw extends DebugDraw {
  /// The canvas rendering context with which to draw.
  final CanvasRenderingContext2D ctx;

  CanvasDraw(ViewportTransform viewport, this.ctx)
      : assert(null != viewport && null != ctx),
        super(viewport);

  /// Draw a closed polygon provided in CCW order. WARNING: This mutates
  /// [vertices].
  @override
  void drawPolygon(List<Vector2> vertices, Color3i color) {
    _pathPolygon(vertices, color);
    ctx.stroke();
  }

  /// Draw a solid closed polygon provided in CCW order. WARNING: This mutates
  /// [vertices].
  @override
  void drawSolidPolygon(List<Vector2> vertices, Color3i color) {
    _pathPolygon(vertices, color);
    ctx.fill();
  }

  void _pathPolygon(List<Vector2> vertices, Color3i color) {
    // Set the color and convert to screen coordinates.
    _setColor(color);
    // TODO(gregbglw): Do a single ctx transform rather than convert all of
    // these vectors.
    vertices = vertices.map(getWorldToScreen).toList();

    ctx.beginPath();
    ctx.moveTo(vertices[0].x, vertices[0].y);

    // Draw lines to all of the remaining points.
    for (int i = 1; i < vertices.length; ++i) {
      ctx.lineTo(vertices[i].x, vertices[i].y);
    }

    // Draw a line back to the starting point.
    ctx.lineTo(vertices[0].x, vertices[0].y);

    // Close the drawn polygon ready for fill/stroke
    ctx.closePath();
  }

  /// Draw a line segment. WARNING: This mutates [p1] and [p2].
  @override
  void drawSegment(Vector2 p1, Vector2 p2, Color3i color) {
    _setColor(color);
    p1 = getWorldToScreen(p1);
    p2 = getWorldToScreen(p2);

    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.closePath();
    ctx.stroke();
  }

  /// Draw a circle. WARNING: This mutates [center].
  @override
  void drawCircle(Vector2 center, num radius, Color3i color, [Vector2 axis]) {
    radius *= viewportTransform.scale;
    _pathCircle(center, radius, color);
    ctx.stroke();
  }

  /// Draw a solid circle. WARNING: This mutates [center].
  @override
  void drawSolidCircle(
      Vector2 center, num radius, Vector2 axis, Color3i color) {
    radius *= viewportTransform.scale;
    drawPoint(center, radius, color);
  }

  /// Draws the given point with the given *unscaled* radius, in the given [color].
  /// WARNING: This mutates [point].
  @override
  void drawPoint(Vector2 point, num radiusOnScreen, Color3i color) {
    _pathCircle(point, radiusOnScreen, color);
    ctx.fill();
  }

  void _pathCircle(Vector2 center, num radius, Color3i color) {
    _setColor(color);
    center = getWorldToScreen(center);

    ctx.beginPath();
    ctx.arc(center.x, center.y, radius, 0, pi * 2, true);
    ctx.closePath();
  }

  /// Draw a transform. Choose your own length scale. WARNING: This mutates
  /// [xf.position].
  @override
  void drawTransform(Transform xf, Color3i color) {
    drawCircle(xf.p, 0.1, color);
    // TODO(rupertk): Draw rotation representation (drawCircle axis parameter?)
  }

  /// Draw a string.
  @override
  void drawStringXY(num x, num y, String s, Color3i color) {
    _setColor(color);
    ctx.strokeText(s, x, y);
  }

  /// Sets the rendering context stroke and fill color to [color].
  void _setColor(Color3i color) {
    ctx.setStrokeColorRgb(color.x, color.y, color.z, 0.9);
    ctx.setFillColorRgb(color.x, color.y, color.z, 0.8);
  }

  @override
  void drawParticles(List<Vector2> centers, double radius,
      List<ParticleColor> colors, int count) {
    throw "Unimplemented";
  }

  @override
  void drawParticlesWireframe(List<Vector2> centers, double radius,
      List<ParticleColor> colors, int count) {
    throw "Unimplemented";
  }
}
