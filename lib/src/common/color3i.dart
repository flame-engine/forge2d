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

class Color3i {
  static final Color3i WHITE = new Color3i(255, 255, 255);
  static final Color3i BLACK = new Color3i(0, 0, 0);
  static final Color3i BLUE = new Color3i(0, 0, 255);
  static final Color3i GREEN = new Color3i(0, 255, 0);
  static final Color3i RED = new Color3i(255, 0, 0);

  int x = 0;
  int y = 0;
  int z = 0;

  Color3i.zero();

  Color3i(this.x, this.y, this.z);

  Color3i.fromRGBd(double r, double g, double b)
      : x = (r * 255).floor().toInt(),
        y = (g * 255).floor().toInt(),
        z = (b * 255).floor().toInt();

  void setRGB(int r, int g, int b) {
    x = r;
    y = g;
    z = b;
  }

  void setFromRGBd(double r, double g, double b) {
    x = (r * 255).floor().toInt();
    y = (g * 255).floor().toInt();
    z = (b * 255).floor().toInt();
  }

  void setColor3i(Color3i argColor) {
    x = argColor.x;
    y = argColor.y;
    z = argColor.z;
  }
}
