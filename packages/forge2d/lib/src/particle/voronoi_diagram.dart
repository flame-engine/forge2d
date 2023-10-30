import 'dart:collection';
import 'dart:math';

import 'package:forge2d/forge2d.dart';

abstract class VoronoiDiagramCallback {
  void call(Particle particleA, Particle particleB, Particle particleC);
}

class VoronoiGenerator {
  final Vector2 center;
  final Particle particle;

  VoronoiGenerator(this.center, this.particle);
}

class VoronoiDiagramTask {
  final int x;
  final int y;
  final int i;
  final VoronoiGenerator generator;

  VoronoiDiagramTask(this.x, this.y, this.i, this.generator);
}

class VoronoiDiagram {
  final List<VoronoiGenerator> generators = [];
  int _countX = 0;
  int _countY = 0;
  final Map<int, VoronoiGenerator> _diagram = {};

  void nodes(VoronoiDiagramCallback callback) {
    for (var y = 0; y < _countY - 1; y++) {
      for (var x = 0; x < _countX - 1; x++) {
        final i = x + y * _countX;
        final a = _diagram[i]!;
        final b = _diagram[i + 1]!;
        final c = _diagram[i + _countX]!;
        final d = _diagram[i + 1 + _countX]!;
        if (b != c) {
          if (a != b && a != c) {
            callback.call(a.particle, b.particle, c.particle);
          }
          if (d != b && d != c) {
            callback.call(b.particle, d.particle, c.particle);
          }
        }
      }
    }
  }

  void addGenerator(Vector2 center, Particle particle) {
    generators.add(VoronoiGenerator(center, particle));
  }

  final Vector2 _lower = Vector2.zero();
  final Vector2 _upper = Vector2.zero();

  final ListQueue<VoronoiDiagramTask> _queue = ListQueue<VoronoiDiagramTask>();

  void generate(double radius) {
    if (generators.isEmpty) {
      return;
    }
    _diagram.clear();
    _queue.clear();
    final inverseRadius = 1 / radius;
    final firstGenerator = generators.first;
    _lower.setFrom(firstGenerator.center);
    _upper.setFrom(firstGenerator.center);
    for (final g in generators) {
      Vector2.min(_lower, g.center, _lower);
      Vector2.max(_upper, g.center, _upper);
    }
    _countX = 1 + (inverseRadius * (_upper.x - _lower.x)).toInt();
    _countY = 1 + (inverseRadius * (_upper.y - _lower.y)).toInt();
    for (final g in generators) {
      g.center.setFrom((g.center - _lower)..scale(inverseRadius));
      final x = max(0, min(g.center.x.toInt(), _countX - 1));
      final y = max(0, min(g.center.y.toInt(), _countY - 1));
      _queue.addFirst(VoronoiDiagramTask(x, y, x + y * _countX, g));
    }
    while (_queue.isNotEmpty) {
      final front = _queue.removeFirst();
      final x = front.x;
      final y = front.y;
      final i = front.i;
      final g = front.generator;
      if (_diagram.containsKey(i)) {
        _diagram[i] = g;
        if (x > 0) {
          _queue.addFirst(VoronoiDiagramTask(x - 1, y, i - 1, g));
        }
        if (y > 0) {
          _queue.addFirst(VoronoiDiagramTask(x, y - 1, i - _countX, g));
        }
        if (x < _countX - 1) {
          _queue.addFirst(VoronoiDiagramTask(x + 1, y, i + 1, g));
        }
        if (y < _countY - 1) {
          _queue.addFirst(VoronoiDiagramTask(x, y + 1, i + _countX, g));
        }
      }
    }
    final maxIteration = _countX + _countY;
    for (var iteration = 0; iteration < maxIteration; iteration++) {
      for (var y = 0; y < _countY; y++) {
        for (var x = 0; x < _countX - 1; x++) {
          final i = x + y * _countX;
          final a = _diagram[i]!;
          final b = _diagram[i + 1]!;
          if (a != b) {
            _queue.addFirst(VoronoiDiagramTask(x, y, i, b));
            _queue.addFirst(VoronoiDiagramTask(x + 1, y, i + 1, a));
          }
        }
      }
      for (var y = 0; y < _countY - 1; y++) {
        for (var x = 0; x < _countX; x++) {
          final i = x + y * _countX;
          final a = _diagram[i]!;
          final b = _diagram[i + _countX]!;
          if (a != b) {
            _queue.addFirst(VoronoiDiagramTask(x, y, i, b));
            _queue.addFirst(VoronoiDiagramTask(x, y + 1, i + _countX, a));
          }
        }
      }
      var updated = false;
      while (_queue.isNotEmpty) {
        final front = _queue.removeFirst();
        final x = front.x;
        final y = front.y;
        final i = front.i;
        final k = front.generator;
        final a = _diagram[i]!;
        final b = k;
        if (a != b) {
          final ax = a.center.x - x;
          final ay = a.center.y - y;
          final bx = b.center.x - x;
          final by = b.center.y - y;
          final a2 = ax * ax + ay * ay;
          final b2 = bx * bx + by * by;
          if (a2 > b2) {
            _diagram[i] = b;
            if (x > 0) {
              _queue.addFirst(VoronoiDiagramTask(x - 1, y, i - 1, b));
            }
            if (y > 0) {
              _queue.addFirst(VoronoiDiagramTask(x, y - 1, i - _countX, b));
            }
            if (x < _countX - 1) {
              _queue.addFirst(VoronoiDiagramTask(x + 1, y, i + 1, b));
            }
            if (y < _countY - 1) {
              _queue.addFirst(VoronoiDiagramTask(x, y + 1, i + _countX, b));
            }
            updated = true;
          }
        }
      }
      if (!updated) {
        break;
      }
    }
  }
}
