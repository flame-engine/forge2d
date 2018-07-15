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

library DominoTest;

import 'package:box2d_flame/box2d.dart';

import 'demo.dart';

/** Demonstration of dominoes being knocked over. */
class DominoTest extends Demo {
  DominoTest() : super("Domino test");

  void initialize() {
    {
      // Floor
      FixtureDef fd = new FixtureDef();
      PolygonShape sd = new PolygonShape();
      sd.setAsBoxXY(50.0, 10.0);
      fd.shape = sd;

      BodyDef bd = new BodyDef();
      bd.position = new Vector2(0.0, -10.0);
      final body = world.createBody(bd);
      body.createFixtureFromFixtureDef(fd);
      bodies.add(body);
    }

    {
      // Platforms
      for (int i = 0; i < 4; i++) {
        FixtureDef fd = new FixtureDef();
        PolygonShape sd = new PolygonShape();
        sd.setAsBoxXY(15.0, 0.125);
        fd.shape = sd;

        BodyDef bd = new BodyDef();
        bd.position = new Vector2(0.0, 5.0 + 5 * i);
        final body = world.createBody(bd);
        body.createFixtureFromFixtureDef(fd);
        bodies.add(body);
      }
    }

    // Dominoes
    {
      FixtureDef fd = new FixtureDef();
      PolygonShape sd = new PolygonShape();
      sd.setAsBoxXY(0.125, 2.0);
      fd.shape = sd;
      fd.density = 25.0;

      BodyDef bd = new BodyDef();
      bd.type = BodyType.DYNAMIC;

      double friction = .5;
      int numPerRow = 25;

      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < numPerRow; j++) {
          fd.friction = friction;
          bd.position =
              new Vector2(-14.75 + j * (29.5 / (numPerRow - 1)), 7.3 + 5 * i);
          if (i == 2 && j == 0) {
            bd.angle = -.1;
            bd.position.x += .1;
          } else if (i == 3 && j == numPerRow - 1) {
            bd.angle = .1;
            bd.position.x -= .1;
          } else {
            bd.angle = 0.0;
          }
          Body myBody = world.createBody(bd);
          myBody.createFixtureFromFixtureDef(fd);
          bodies.add(myBody);
        }
      }
    }
  }
}

void main() {
  final domino = new DominoTest();
  domino.initialize();
  domino.initializeAnimation();
  domino.runAnimation();
}
