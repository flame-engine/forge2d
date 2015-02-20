/*******************************************************************************
 * Copyright (c) 2015, Daniel Murphy, Google
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

part of box2d;

abstract class VoronoiDiagramCallback {
  void callback(int aTag, int bTag, int cTag);
}

class VoronoiGenerator {
  final Vec2 center = new Vec2.zero();
  int tag = 0;
}

class VoronoiDiagramTask {
  int m_x = 0,
      m_y = 0,
      m_i = 0;
  VoronoiGenerator m_generator;

  VoronoiDiagramTask.zero() {}

  VoronoiDiagramTask(int x, int y, int i, VoronoiGenerator g) {
    m_x = x;
    m_y = y;
    m_i = i;
    m_generator = g;
  }

  VoronoiDiagramTask set(int x, int y, int i, VoronoiGenerator g) {
    m_x = x;
    m_y = y;
    m_i = i;
    m_generator = g;
    return this;
  }
}

class VoronoiDiagramTaskMutableStack extends MutableStack<VoronoiDiagramTask> {
  VoronoiDiagramTaskMutableStack(int size) : super(size);
  VoronoiDiagramTask newInstance() {
    return new VoronoiDiagramTask.zero();
  }
}

class VoronoiDiagram {
  List<VoronoiGenerator> _m_generatorBuffer;
  int _m_generatorCount = 0;
  int _m_countX = 0,
      _m_countY = 0;
  // The diagram is an array of "pointers".
  List<VoronoiGenerator> _m_diagram;

  VoronoiDiagram(int generatorCapacity) {
    _m_generatorBuffer = new List<VoronoiGenerator>(generatorCapacity);
    for (int i = 0; i < generatorCapacity; i++) {
      _m_generatorBuffer[i] = new VoronoiGenerator();
    }
    _m_generatorCount = 0;
    _m_countX = 0;
    _m_countY = 0;
    _m_diagram = null;
  }

  void getNodes(VoronoiDiagramCallback callback) {
    for (int y = 0; y < _m_countY - 1; y++) {
      for (int x = 0; x < _m_countX - 1; x++) {
        int i = x + y * _m_countX;
        VoronoiGenerator a = _m_diagram[i];
        VoronoiGenerator b = _m_diagram[i + 1];
        VoronoiGenerator c = _m_diagram[i + _m_countX];
        VoronoiGenerator d = _m_diagram[i + 1 + _m_countX];
        if (b != c) {
          if (a != b && a != c) {
            callback.callback(a.tag, b.tag, c.tag);
          }
          if (d != b && d != c) {
            callback.callback(b.tag, d.tag, c.tag);
          }
        }
      }
    }
  }

  void addGenerator(Vec2 center, int tag) {
    VoronoiGenerator g = _m_generatorBuffer[_m_generatorCount++];
    g.center.x = center.x;
    g.center.y = center.y;
    g.tag = tag;
  }

  final Vec2 _lower = new Vec2.zero();
  final Vec2 _upper = new Vec2.zero();
  MutableStack<VoronoiDiagramTask> _taskPool =
      new VoronoiDiagramTaskMutableStack(50);

  final StackQueue<VoronoiDiagramTask> _queue =
      new StackQueue<VoronoiDiagramTask>();

  void generate(double radius) {
    assert(_m_diagram == null);
    double inverseRadius = 1 / radius;
    _lower.x = double.MAX_FINITE;
    _lower.y = double.MAX_FINITE;
    _upper.x = -double.MAX_FINITE;
    _upper.y = -double.MAX_FINITE;
    for (int k = 0; k < _m_generatorCount; k++) {
      VoronoiGenerator g = _m_generatorBuffer[k];
      Vec2.minToOut(_lower, g.center, _lower);
      Vec2.maxToOut(_upper, g.center, _upper);
    }
    _m_countX = 1 + (inverseRadius * (_upper.x - _lower.x)).toInt();
    _m_countY = 1 + (inverseRadius * (_upper.y - _lower.y)).toInt();
    _m_diagram = new List<VoronoiGenerator>(_m_countX * _m_countY);
    _queue.reset(new List<VoronoiDiagramTask>(4 * _m_countX * _m_countX));
    for (int k = 0; k < _m_generatorCount; k++) {
      VoronoiGenerator g = _m_generatorBuffer[k];
      g.center.x = inverseRadius * (g.center.x - _lower.x);
      g.center.y = inverseRadius * (g.center.y - _lower.y);
      int x = Math.max(0, Math.min(g.center.x.toInt(), _m_countX - 1));
      int y = Math.max(0, Math.min(g.center.y.toInt(), _m_countY - 1));
      _queue.push(_taskPool.pop().set(x, y, x + y * _m_countX, g));
    }
    while (!_queue.empty()) {
      VoronoiDiagramTask front = _queue.pop();
      int x = front.m_x;
      int y = front.m_y;
      int i = front.m_i;
      VoronoiGenerator g = front.m_generator;
      if (_m_diagram[i] == null) {
        _m_diagram[i] = g;
        if (x > 0) {
          _queue.push(_taskPool.pop().set(x - 1, y, i - 1, g));
        }
        if (y > 0) {
          _queue.push(_taskPool.pop().set(x, y - 1, i - _m_countX, g));
        }
        if (x < _m_countX - 1) {
          _queue.push(_taskPool.pop().set(x + 1, y, i + 1, g));
        }
        if (y < _m_countY - 1) {
          _queue.push(_taskPool.pop().set(x, y + 1, i + _m_countX, g));
        }
      }
      _taskPool.push(front);
    }
    int maxIteration = _m_countX + _m_countY;
    for (int iteration = 0; iteration < maxIteration; iteration++) {
      for (int y = 0; y < _m_countY; y++) {
        for (int x = 0; x < _m_countX - 1; x++) {
          int i = x + y * _m_countX;
          VoronoiGenerator a = _m_diagram[i];
          VoronoiGenerator b = _m_diagram[i + 1];
          if (a != b) {
            _queue.push(_taskPool.pop().set(x, y, i, b));
            _queue.push(_taskPool.pop().set(x + 1, y, i + 1, a));
          }
        }
      }
      for (int y = 0; y < _m_countY - 1; y++) {
        for (int x = 0; x < _m_countX; x++) {
          int i = x + y * _m_countX;
          VoronoiGenerator a = _m_diagram[i];
          VoronoiGenerator b = _m_diagram[i + _m_countX];
          if (a != b) {
            _queue.push(_taskPool.pop().set(x, y, i, b));
            _queue.push(_taskPool.pop().set(x, y + 1, i + _m_countX, a));
          }
        }
      }
      bool updated = false;
      while (!_queue.empty()) {
        VoronoiDiagramTask front = _queue.pop();
        int x = front.m_x;
        int y = front.m_y;
        int i = front.m_i;
        VoronoiGenerator k = front.m_generator;
        VoronoiGenerator a = _m_diagram[i];
        VoronoiGenerator b = k;
        if (a != b) {
          double ax = a.center.x - x;
          double ay = a.center.y - y;
          double bx = b.center.x - x;
          double by = b.center.y - y;
          double a2 = ax * ax + ay * ay;
          double b2 = bx * bx + by * by;
          if (a2 > b2) {
            _m_diagram[i] = b;
            if (x > 0) {
              _queue.push(_taskPool.pop().set(x - 1, y, i - 1, b));
            }
            if (y > 0) {
              _queue.push(_taskPool.pop().set(x, y - 1, i - _m_countX, b));
            }
            if (x < _m_countX - 1) {
              _queue.push(_taskPool.pop().set(x + 1, y, i + 1, b));
            }
            if (y < _m_countY - 1) {
              _queue.push(_taskPool.pop().set(x, y + 1, i + _m_countX, b));
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
