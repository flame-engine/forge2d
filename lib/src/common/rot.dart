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

part of box2d.common;

class Rot {
  double s = 0.0, c = 1.0; // sin and cos

  Rot();

  Rot.withAngle(double angle)
      : s = Math.sin(angle),
        c = Math.cos(angle);

  void setAngle(double angle) {
    s = Math.sin(angle);
    c = Math.cos(angle);
  }

  void set(Rot other) {
    s = other.s;
    c = other.c;
  }

  void setIdentity() {
    s = 0.0;
    c = 1.0;
  }

  double getSin() => s;

  String toString() => "Rot(s:$s, c:$c)";

  double getCos() => c;

  double getAngle() => Math.atan2(s, c);

  Vector2 getXAxis(Vector2 xAxis) => Vector2(c, s);

  Vector2 getYAxis(Vector2 yAxis) => Vector2(-s, c);

  Rot clone() {
    return Rot()
      ..s = s
      ..c = c;
  }

  static Rot mul(Rot q, Rot r) {
    return Rot()
        ..s = q.s * r.c + q.c * r.s
        ..c = q.c * r.c - q.s * r.s;
  }

  static Rot mulTrans(Rot q, Rot r) {
    return Rot()
      ..s = q.c * r.s - q.s * r.c
      ..c = q.c * r.c + q.s * r.s;
  }

  static Vector2 mulVec2(Rot q, Vector2 v) {
    return Vector2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
  }

  static Vector2 mulTransVec2(Rot q, Vector2 v) {
    return Vector2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
  }
}
