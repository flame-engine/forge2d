import 'dart:js_interop';
import 'dart:math' as math;

import 'package:forge2d/forge2d.dart';
import 'package:web/web.dart';

/// Renders the world onto a canvas through the [DebugDraw] interface, with
/// a soft glow-on-dark styling.
class CanvasDebugDraw extends DebugDraw {
  CanvasDebugDraw(this._context);

  final CanvasRenderingContext2D _context;

  /// The world point at the center of the view.
  Vector2 center = Vector2.zero();

  /// How many world meters are visible vertically.
  double viewHeight = 40;

  double _scale = 20;
  double _canvasWidth = 0;
  double _canvasHeight = 0;

  /// Prepares the transform for a frame and clears the canvas.
  void beginFrame(double canvasWidth, double canvasHeight) {
    _canvasWidth = canvasWidth;
    _canvasHeight = canvasHeight;
    _scale = canvasHeight / viewHeight;
    _context.clearRect(0, 0, canvasWidth, canvasHeight);
  }

  double _x(double worldX) => _canvasWidth / 2 + (worldX - center.x) * _scale;
  double _y(double worldY) => _canvasHeight / 2 - (worldY - center.y) * _scale;

  /// Converts a canvas point to world coordinates.
  Vector2 toWorld(double canvasX, double canvasY) => Vector2(
    center.x + (canvasX - _canvasWidth / 2) / _scale,
    center.y - (canvasY - _canvasHeight / 2) / _scale,
  );

  String _rgba(int color, double alpha) {
    final r = (color >> 16) & 0xFF;
    final g = (color >> 8) & 0xFF;
    final b = color & 0xFF;
    return 'rgba($r, $g, $b, $alpha)';
  }

  void _stroke(int color) {
    _context
      ..strokeStyle = _rgba(color, 0.95).toJS
      ..lineWidth = 1.6
      ..stroke();
  }

  void _fill(int color) {
    _context
      ..fillStyle = _rgba(color, 0.28).toJS
      ..fill();
  }

  void _polygonPath(List<Vector2> vertices) {
    _context.beginPath();
    for (var i = 0; i < vertices.length; i++) {
      final vertex = vertices[i];
      if (i == 0) {
        _context.moveTo(_x(vertex.x), _y(vertex.y));
      } else {
        _context.lineTo(_x(vertex.x), _y(vertex.y));
      }
    }
    _context.closePath();
  }

  @override
  void drawPolygon(List<Vector2> vertices, int color) {
    _polygonPath(vertices);
    _stroke(color);
  }

  @override
  void drawSolidPolygon(
    Transform transform,
    List<Vector2> vertices,
    double radius,
    int color,
  ) {
    _polygonPath([for (final vertex in vertices) transform.apply(vertex)]);
    _fill(color);
    _stroke(color);
  }

  @override
  void drawCircle(Vector2 center, double radius, int color) {
    _context.beginPath();
    _context.arc(_x(center.x), _y(center.y), radius * _scale, 0, 2 * math.pi);
    _stroke(color);
  }

  @override
  void drawSolidCircle(Transform transform, double radius, int color) {
    final position = transform.position;
    _context.beginPath();
    _context.arc(
      _x(position.x),
      _y(position.y),
      radius * _scale,
      0,
      2 * math.pi,
    );
    _fill(color);
    _stroke(color);
    // A radius line makes rotation visible.
    final edge = transform.apply(Vector2(radius, 0));
    _context.beginPath();
    _context.moveTo(_x(position.x), _y(position.y));
    _context.lineTo(_x(edge.x), _y(edge.y));
    _stroke(color);
  }

  @override
  void drawSolidCapsule(
    Vector2 point1,
    Vector2 point2,
    double radius,
    int color,
  ) {
    final axis = point2 - point1;
    final angle = math.atan2(axis.y, axis.x);
    _context.beginPath();
    _context.arc(
      _x(point1.x),
      _y(point1.y),
      radius * _scale,
      -angle + math.pi / 2,
      -angle + 3 * math.pi / 2,
    );
    _context.arc(
      _x(point2.x),
      _y(point2.y),
      radius * _scale,
      -angle - math.pi / 2,
      -angle + math.pi / 2,
    );
    _context.closePath();
    _fill(color);
    _stroke(color);
  }

  @override
  void drawSegment(Vector2 point1, Vector2 point2, int color) {
    _context.beginPath();
    _context.moveTo(_x(point1.x), _y(point1.y));
    _context.lineTo(_x(point2.x), _y(point2.y));
    _stroke(color);
  }

  @override
  void drawPoint(Vector2 point, double size, int color) {
    _context.beginPath();
    _context.arc(_x(point.x), _y(point.y), size / 2, 0, 2 * math.pi);
    _context.fillStyle = _rgba(color, 0.9).toJS;
    _context.fill();
  }

  @override
  void drawString(Vector2 point, String text, int color) {
    _context
      ..fillStyle = _rgba(0xE5E9F5, 0.8).toJS
      ..font = '12px ui-sans-serif, system-ui'
      ..fillText(text, _x(point.x), _y(point.y));
  }
}
