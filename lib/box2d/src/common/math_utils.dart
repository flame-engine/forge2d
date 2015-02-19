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

class MathUtils {
  static final double TWOPI = Math.PI * 2.0;

  static double atan2(double a, double b) {
    return Math.atan2(a, b);
  }

  static double distanceSquared(Vec2 v1, Vec2 v2) {
    double dx = (v1.x - v2.x);
    double dy = (v1.y - v2.y);
    return dx * dx + dy * dy;
  }

  static double distance(Vec2 v1, Vec2 v2) {
    return Math.sqrt(distanceSquared(v1, v2));
  }

  /** Returns the closest value to 'a' that is in between 'low' and 'high' */
  static double clampDouble(final double a, final double low, final double high) {
    return Math.max(low, Math.min(a, high));
  }

  static Vec2 clampVec2(final Vec2 a, final Vec2 low, final Vec2 high) {
    final Vec2 min = new Vec2.zero();
    min.x = a.x < high.x ? a.x : high.x;
    min.y = a.y < high.y ? a.y : high.y;
    min.x = low.x > min.x ? low.x : min.x;
    min.y = low.y > min.y ? low.y : min.y;
    return min;
  }

  static bool approxEquals(num expected, num actual, [num tolerance = null]) {
    if (tolerance == null) {
      tolerance = (expected / 1e4).abs();
    }
    return ((expected - actual).abs() <= tolerance);
  }

  // Optimize sin and cos. Note: using LUT worsens performance.
  static final Float64List sinLUT = initSinLUT();

  static initSinLUT() {
    var table = new Float64List(Settings.SINCOS_LUT_LENGTH);
    for (int i = 0; i < Settings.SINCOS_LUT_LENGTH; i++) {
      table[i] = Math.sin(i * Settings.SINCOS_LUT_PRECISION);
    }
    return table;
  }

  static double sin(double x) {
    if (Settings.SINCOS_LUT_ENABLED) {
      return sinLUT_(x);
    } else {
      return Math.sin(x);
    }
  }

  static double sinLUT_(double x) {
    x %= TWOPI;

    if (x < 0) {
      x += TWOPI;
    }

    if (Settings.SINCOS_LUT_LERP) {

      x /= Settings.SINCOS_LUT_PRECISION;

      final int index = x.toInt();

      if (index != 0) {
        x %= index;
      }

      // the next index is 0
      if (index == Settings.SINCOS_LUT_LENGTH - 1) {
        return ((1 - x) * sinLUT[index] + x * sinLUT[0]);
      } else {
        return ((1 - x) * sinLUT[index] + x * sinLUT[index + 1]);
      }

    } else {
      return sinLUT[(x / Settings.SINCOS_LUT_PRECISION).round() % Settings.SINCOS_LUT_LENGTH];
    }
  }

  static double cos(double x) {
    return Math.cos(x);
  }

}
