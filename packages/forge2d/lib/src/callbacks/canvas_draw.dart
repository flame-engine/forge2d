import 'dart:js_interop';
import 'dart:math';

import 'package:forge2d/forge2d.dart';
import 'package:forge2d/forge2d_browser.dart';
import 'package:web/web.dart';

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
    _setColor(color);

    // Save the current canvas state
    ctx.save();

    // Apply the transformation matrix for world-to-screen conversion
    ctx.transform(
      viewport.scale,
      0,
      0,
      -viewport.scale,
      viewport.extents.x + viewport.translation.x,
      viewport.extents.y - viewport.translation.y,
    );

    // Begin the path and move to the first vertex
    ctx.beginPath();
    ctx.moveTo(vertices[0].x, vertices[0].y);

    // Draw lines to the remaining vertices
    for (final vertex in vertices) {
      ctx.lineTo(vertex.x, vertex.y);
    }

    // Close the polygon
    ctx.closePath();

    // Restore the canvas state
    ctx.restore();
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
    ctx.strokeStyle = color.toHex().toJS;
    ctx.fillStyle = color.toHex().toJS;
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
