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

  Rot setAngle(double angle) {
    s = Math.sin(angle);
    c = Math.cos(angle);
    return this;
  }

  Rot set(Rot other) {
    s = other.s;
    c = other.c;
    return this;
  }

  Rot setIdentity() {
    s = 0.0;
    c = 1.0;
    return this;
  }

  double getSin() => s;

  String toString() {
    return "Rot(s:$s, c:$c)";
  }

  double getCos() => c;

  double getAngle() => Math.atan2(s, c);

  void getXAxis(Vector2 xAxis) {
    xAxis.setValues(c, s);
  }

  void getYAxis(Vector2 yAxis) {
    yAxis.setValues(-s, c);
  }

  Rot clone() {
    Rot copy = new Rot();
    copy.s = s;
    copy.c = c;
    return copy;
  }

  static void mul(Rot q, Rot r, Rot out) {
    double tempc = q.c * r.c - q.s * r.s;
    out.s = q.s * r.c + q.c * r.s;
    out.c = tempc;
  }

  static void mulUnsafe(Rot q, Rot r, Rot out) {
    assert(r != out);
    assert(q != out);
    // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
    // [qs qc] [rs rc] [qs*rc+qc*rs -qs*rs+qc*rc]
    // s = qs * rc + qc * rs
    // c = qc * rc - qs * rs
    out.s = q.s * r.c + q.c * r.s;
    out.c = q.c * r.c - q.s * r.s;
  }

  static void mulTrans(Rot q, Rot r, Rot out) {
    final double tempc = q.c * r.c + q.s * r.s;
    out.s = q.c * r.s - q.s * r.c;
    out.c = tempc;
  }

  static void mulTransUnsafe(Rot q, Rot r, Rot out) {
    // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
    // [-qs qc] [rs rc] [-qs*rc+qc*rs qs*rs+qc*rc]
    // s = qc * rs - qs * rc
    // c = qc * rc + qs * rs
    out.s = q.c * r.s - q.s * r.c;
    out.c = q.c * r.c + q.s * r.s;
  }

  static void mulToOut(Rot q, Vector2 v, Vector2 out) {
    double tempy = q.s * v.x + q.c * v.y;
    out.x = q.c * v.x - q.s * v.y;
    out.y = tempy;
  }

  static void mulToOutUnsafe(Rot q, Vector2 v, Vector2 out) {
    out.x = q.c * v.x - q.s * v.y;
    out.y = q.s * v.x + q.c * v.y;
  }

  static void mulTransVec2(Rot q, Vector2 v, Vector2 out) {
    final double tempy = -q.s * v.x + q.c * v.y;
    out.x = q.c * v.x + q.s * v.y;
    out.y = tempy;
  }

  static void mulTransUnsafeVec2(Rot q, Vector2 v, Vector2 out) {
    out.x = q.c * v.x + q.s * v.y;
    out.y = -q.s * v.x + q.c * v.y;
  }
}
