import 'dart:html';
import 'dart:math';

import 'package:forge2d/forge2d.dart';
import 'package:forge2d/forge2d_browser.dart';

class CanvasDraw extends DebugDraw {
  /// The canvas rendering context with which to draw.
  final CanvasRenderingContext2D ctx;

  CanvasDraw(super.viewport, this.ctx);

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
    final screenVertices = vertices.map(worldToScreen).toList();

    ctx.beginPath();
    ctx.moveTo(screenVertices[0].x, screenVertices[0].y);

    // Draw lines to all of the remaining points.
    for (final vertex in screenVertices) {
      ctx.lineTo(vertex.x, vertex.y);
    }

    // Draw a line back to the starting point.
    ctx.lineTo(screenVertices[0].x, screenVertices[0].y);

    // Close the drawn polygon ready for fill/stroke
    ctx.closePath();
  }

  /// Draw a line segment. WARNING: This mutates [p1] and [p2].
  @override
  void drawSegment(Vector2 p1, Vector2 p2, Color3i color) {
    _setColor(color);
    final screenP1 = worldToScreen(p1);
    final screenP2 = worldToScreen(p2);

    ctx.beginPath();
    ctx.moveTo(screenP1.x, screenP1.y);
    ctx.lineTo(screenP2.x, screenP2.y);
    ctx.closePath();
    ctx.stroke();
  }

  /// Draw a circle.
  @override
  void drawCircle(Vector2 center, num radius, Color3i color) {
    final screenRadius = radius * viewport.scale;
    _pathCircle(center, screenRadius, color);
    ctx.stroke();
  }

  /// Draw a solid circle.
  @override
  void drawSolidCircle(Vector2 center, num radius, Color3i color) {
    final screenRadius = radius * viewport.scale;
    drawPoint(center, screenRadius, color);
  }

  /// Draws the given point with the given *unscaled* radius, in the given
  /// [color].
  @override
  void drawPoint(
    Vector2 point,
    num radiusOnScreen,
    Color3i color,
  ) {
    _pathCircle(point, radiusOnScreen, color);
    ctx.fill();
  }

  void _pathCircle(Vector2 center, num radius, Color3i color) {
    _setColor(color);
    final screenCenter = worldToScreen(center);

    ctx.beginPath();
    ctx.arc(screenCenter.x, screenCenter.y, radius, 0, pi * 2, true);
    ctx.closePath();
  }

  /// Draw a transform. Choose your own length scale.
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
    ctx.setStrokeColorRgb(color.r, color.g, color.b, color.a);
    ctx.setFillColorRgb(color.r, color.g, color.b, color.a);
  }

  @override
  void drawParticles(List<Particle> particles, double radius) {
    particles.forEach((p) {
      drawSolidCircle(p.position, radius, p.color);
    });
  }

  @override
  void drawParticlesWireframe(List<Particle> particles, double radius) {
    throw 'Unimplemented';
  }
}
