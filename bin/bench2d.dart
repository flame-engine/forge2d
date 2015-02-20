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

import 'package:box2d/box2d.dart';

main() {
  Bench2D bench = new Bench2D();
  bench.warmup();
  bench.bench();
}

class Bench2D {
  static const bool DEBUG = false;
  static const bool CHECKSUM = false;
  
  static const int FRAMES = 256;
  static const int PYRAMID_SIZE = 40;

  World world;
  Body groundBody;

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
    print("Benchmark complete.\nms/frame: $mean 5th %ile: ${percentile(times, 5.0)} 95th %ile: ${percentile(times, 95.0)}");
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

  Body topBody = null;

  Bench2D() {
    Vec2 gravity = new Vec2(0.0, -10.0);
    world = new World.withGravity(gravity);

    {
      BodyDef bd = new BodyDef();
      Body ground = world.createBody(bd);

      EdgeShape shape = new EdgeShape();
      shape.set(new Vec2(-40.0, 0.0), new Vec2(40.0, 0.0));
      ground.createFixtureFromShape(shape, 0.0);
    }

    {
      double a = .5;
      PolygonShape shape = new PolygonShape();
      shape.setAsBoxXY(a, a);

      Vec2 x = new Vec2(-7.0, 0.75);
      Vec2 y = new Vec2.zero();
      Vec2 deltaX = new Vec2(0.5625, 1.0);
      Vec2 deltaY = new Vec2(1.125, 0.0);

      for (int i = 0; i < PYRAMID_SIZE; ++i) {
        y.set(x);

        for (int j = i; j < PYRAMID_SIZE; ++j) {
          BodyDef bd = new BodyDef();
          bd.type = BodyType.DYNAMIC;
          bd.position.set(y);
          Body body = world.createBody(bd);
          body.createFixtureFromShape(shape, 5.0);
          topBody = body;
          y.addLocal(deltaY);
        }

        x.addLocal(deltaX);
      }
    }
  }

  static int stepCount = 0;

  void step() {
    double timeStep = 1.0 / 60.0;
    world.stepDt(timeStep, 3, 3);
    stepCount++;
  }

  void checksum(World world) {
    Vec2 positionSum = new Vec2.zero();
    Vec2 linearVelocitySum = new Vec2.zero();
    double angularVelocitySum = 0.0;
    var checksum = (Body b) {
      positionSum = positionSum.add(b.getPosition());
      linearVelocitySum = linearVelocitySum.add(b.getLinearVelocity());
      angularVelocitySum += b.getAngularVelocity();
    };
    Body firstBody = world.m_bodyList;
    print(firstBody);
    world.forEachBody(checksum);
    print("pos: $positionSum linVel $linearVelocitySum angVel $angularVelocitySum");
  }

  void log(String msg) {
    if (DEBUG) {
      print(msg);
    }
  }
}

