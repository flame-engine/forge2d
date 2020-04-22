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

import 'package:box2d_flame/box2d.dart';

void main() {
  new Bench2d()
    ..initialize()
    ..warmup()
    ..bench();
}

void log(String msg) {
  if (DEBUG) {
    print(msg);
  }
}

const bool DEBUG = const bool.fromEnvironment('debug', defaultValue: false);

const bool CHECKSUM =
    const bool.fromEnvironment('checksum', defaultValue: false);

const int FRAMES = 256;
const int PYRAMID_SIZE = 40;

const double _timeStep = 1.0 / 60.0;

double meanF(List<double> values) {
  double total = 0.0;
  for (int i = 0; i < FRAMES; ++i) {
    total += values[i];
  }
  return total / FRAMES;
}

// Simple nearest-rank %ile (on sorted array). We should have enough samples to make this reasonable.
double percentile(List<double> values, double pc) {
  int rank = (pc * values.length) ~/ 100;
  return values[rank];
}

class Bench2d {
  final World world;

  Bench2d() : world = new World.withGravity(new Vector2(0.0, -10.0));

  void initialize() {
    BodyDef bd = new BodyDef();
    Body ground = world.createBody(bd);

    EdgeShape groundShape = new EdgeShape()
      ..set(new Vector2(-40.0, 0.0), new Vector2(40.0, 0.0));
    ground.createFixtureFromShape(groundShape, 0.0);

    // add boxes
    const boxSize = .5;
    PolygonShape shape = new PolygonShape()..setAsBoxXY(boxSize, boxSize);

    Vector2 x = new Vector2(-7.0, 0.75);
    Vector2 y = new Vector2.zero();
    Vector2 deltaX = new Vector2(0.5625, 1.0);
    Vector2 deltaY = new Vector2(1.125, 0.0);

    for (int i = 0; i < PYRAMID_SIZE; ++i) {
      y.setFrom(x);

      for (int j = i; j < PYRAMID_SIZE; ++j) {
        BodyDef bd = new BodyDef()
          ..type = BodyType.DYNAMIC
          ..position.setFrom(y);
        world.createBody(bd)..createFixtureFromShape(shape, 5.0);
        y.add(deltaY);
      }

      x.add(deltaX);
    }
  }

  List<double> bench() {
    List<double> times = new List<double>(FRAMES);
    Stopwatch stopwatch = new Stopwatch()..start();
    for (int i = 0; i < FRAMES; ++i) {
      int begin = stopwatch.elapsedMilliseconds;
      step();
      int end = stopwatch.elapsedMilliseconds;
      times[i] = (end - begin).toDouble();
      log("${times[i]}");
    }

    times.sort();
    double mean = meanF(times);
    double fifth = percentile(times, 5.0);
    double ninetyFifth = percentile(times, 95.0);
    print("Benchmark complete.\n"
        "ms/frame: $mean 5th %ile: ${percentile(times, 5.0)} 95th %ile: ${percentile(times, 95.0)}");
    if (CHECKSUM) {
      checksum(world);
    }
    return <double>[mean, fifth, ninetyFifth];
  }

  void warmup() {
    for (int i = 0; i < FRAMES; ++i) {
      step();
    }
  }

  void step() {
    world.stepDt(_timeStep, 3, 3);
  }

  void checksum(World world) {
    Vector2 positionSum = new Vector2.zero();
    Vector2 linearVelocitySum = new Vector2.zero();
    double angularVelocitySum = 0.0;
    var checksum = (Body b) {
      positionSum = positionSum + b.position;
      linearVelocitySum = linearVelocitySum + b.linearVelocity;
      angularVelocitySum += b.angularVelocity;
    };
    Body firstBody = world.bodyList;
    print(firstBody);
    world.forEachBody(checksum);
    print(
        "pos: $positionSum linVel $linearVelocitySum angVel $angularVelocitySum");
  }
}
