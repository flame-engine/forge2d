part of box2d;

abstract class VoronoiDiagramCallback {
  void call(int aTag, int bTag, int cTag);
}

class VoronoiGenerator {
  final Vector2 center = Vector2.zero();
  int tag = 0;
}

class VoronoiDiagramTask {
  int _x = 0, _y = 0, _i = 0;
  VoronoiGenerator _generator;

  VoronoiDiagramTask.zero() {}

  VoronoiDiagramTask(int x, int y, int i, VoronoiGenerator g) {
    _x = x;
    _y = y;
    _i = i;
    _generator = g;
  }

  VoronoiDiagramTask set(int x, int y, int i, VoronoiGenerator g) {
    _x = x;
    _y = y;
    _i = i;
    _generator = g;
    return this;
  }
}

class VoronoiDiagramTaskMutableStack extends MutableStack<VoronoiDiagramTask> {
  VoronoiDiagramTaskMutableStack(int size) : super(size);

  VoronoiDiagramTask newInstance() => VoronoiDiagramTask.zero();
}

class VoronoiDiagram {
  List<VoronoiGenerator> _generatorBuffer;
  int _generatorCount = 0;
  int _countX = 0, _countY = 0;
  // The diagram is an array of "pointers".
  List<VoronoiGenerator> _diagram;

  VoronoiDiagram(int generatorCapacity) {
    _generatorBuffer =
        List<VoronoiGenerator>.filled(generatorCapacity, VoronoiGenerator());
    _generatorCount = 0;
    _countX = 0;
    _countY = 0;
    _diagram = null;
  }

  void getNodes(VoronoiDiagramCallback callback) {
    for (int y = 0; y < _countY - 1; y++) {
      for (int x = 0; x < _countX - 1; x++) {
        int i = x + y * _countX;
        VoronoiGenerator a = _diagram[i];
        VoronoiGenerator b = _diagram[i + 1];
        VoronoiGenerator c = _diagram[i + _countX];
        VoronoiGenerator d = _diagram[i + 1 + _countX];
        if (b != c) {
          if (a != b && a != c) {
            callback.call(a.tag, b.tag, c.tag);
          }
          if (d != b && d != c) {
            callback.call(b.tag, d.tag, c.tag);
          }
        }
      }
    }
  }

  void addGenerator(Vector2 center, int tag) {
    VoronoiGenerator g = _generatorBuffer[_generatorCount++];
    g.center.x = center.x;
    g.center.y = center.y;
    g.tag = tag;
  }

  final Vector2 _lower = Vector2.zero();
  final Vector2 _upper = Vector2.zero();
  MutableStack<VoronoiDiagramTask> _taskPool =
      VoronoiDiagramTaskMutableStack(50);

  final ListQueue<VoronoiDiagramTask> _queue =
      ListQueue<VoronoiDiagramTask>();

  void generate(double radius) {
    assert(_diagram == null);
    double inverseRadius = 1 / radius;
    _lower.x = double.maxFinite;
    _lower.y = double.maxFinite;
    _upper.x = -double.maxFinite;
    _upper.y = -double.maxFinite;
    for (int k = 0; k < _generatorCount; k++) {
      VoronoiGenerator g = _generatorBuffer[k];
      Vector2.min(_lower, g.center, _lower);
      Vector2.max(_upper, g.center, _upper);
    }
    _countX = 1 + (inverseRadius * (_upper.x - _lower.x)).toInt();
    _countY = 1 + (inverseRadius * (_upper.y - _lower.y)).toInt();
    _diagram = List<VoronoiGenerator>(_countX * _countY);
    _queue.clear();
    for (int k = 0; k < _generatorCount; k++) {
      VoronoiGenerator g = _generatorBuffer[k];
      g.center.x = inverseRadius * (g.center.x - _lower.x);
      g.center.y = inverseRadius * (g.center.y - _lower.y);
      int x = Math.max(0, Math.min(g.center.x.toInt(), _countX - 1));
      int y = Math.max(0, Math.min(g.center.y.toInt(), _countY - 1));
      _queue.addFirst(_taskPool.pop().set(x, y, x + y * _countX, g));
    }
    while (_queue.isNotEmpty) {
      VoronoiDiagramTask front = _queue.removeFirst();
      int x = front._x;
      int y = front._y;
      int i = front._i;
      VoronoiGenerator g = front._generator;
      if (_diagram[i] == null) {
        _diagram[i] = g;
        if (x > 0) {
          _queue.addFirst(_taskPool.pop().set(x - 1, y, i - 1, g));
        }
        if (y > 0) {
          _queue.addFirst(_taskPool.pop().set(x, y - 1, i - _countX, g));
        }
        if (x < _countX - 1) {
          _queue.addFirst(_taskPool.pop().set(x + 1, y, i + 1, g));
        }
        if (y < _countY - 1) {
          _queue.addFirst(_taskPool.pop().set(x, y + 1, i + _countX, g));
        }
      }
      _taskPool.push(front);
    }
    int maxIteration = _countX + _countY;
    for (int iteration = 0; iteration < maxIteration; iteration++) {
      for (int y = 0; y < _countY; y++) {
        for (int x = 0; x < _countX - 1; x++) {
          int i = x + y * _countX;
          VoronoiGenerator a = _diagram[i];
          VoronoiGenerator b = _diagram[i + 1];
          if (a != b) {
            _queue.addFirst(_taskPool.pop().set(x, y, i, b));
            _queue.addFirst(_taskPool.pop().set(x + 1, y, i + 1, a));
          }
        }
      }
      for (int y = 0; y < _countY - 1; y++) {
        for (int x = 0; x < _countX; x++) {
          int i = x + y * _countX;
          VoronoiGenerator a = _diagram[i];
          VoronoiGenerator b = _diagram[i + _countX];
          if (a != b) {
            _queue.addFirst(_taskPool.pop().set(x, y, i, b));
            _queue.addFirst(_taskPool.pop().set(x, y + 1, i + _countX, a));
          }
        }
      }
      bool updated = false;
      while (_queue.isNotEmpty) {
        VoronoiDiagramTask front = _queue.removeFirst();
        int x = front._x;
        int y = front._y;
        int i = front._i;
        VoronoiGenerator k = front._generator;
        VoronoiGenerator a = _diagram[i];
        VoronoiGenerator b = k;
        if (a != b) {
          double ax = a.center.x - x;
          double ay = a.center.y - y;
          double bx = b.center.x - x;
          double by = b.center.y - y;
          double a2 = ax * ax + ay * ay;
          double b2 = bx * bx + by * by;
          if (a2 > b2) {
            _diagram[i] = b;
            if (x > 0) {
              _queue.addFirst(_taskPool.pop().set(x - 1, y, i - 1, b));
            }
            if (y > 0) {
              _queue.addFirst(_taskPool.pop().set(x, y - 1, i - _countX, b));
            }
            if (x < _countX - 1) {
              _queue.addFirst(_taskPool.pop().set(x + 1, y, i + 1, b));
            }
            if (y < _countY - 1) {
              _queue.addFirst(_taskPool.pop().set(x, y + 1, i + _countX, b));
            }
            updated = true;
          }
        }
        _taskPool.push(front);
      }
      if (!updated) {
        break;
      }
    }
  }
}
