part of box2d;

/// Small color object for each particle
class ParticleColor {
  Int8List _data = Int8List(4);

  void set r(int v) {
    _data[0] = v;
  }

  void set g(int v) {
    _data[1] = v;
  }

  void set b(int v) {
    _data[2] = v;
  }

  void set a(int v) {
    _data[3] = v;
  }

  int get r => _data[0];
  int get g => _data[1];
  int get b => _data[2];
  int get a => _data[3];

  ParticleColor() {
    _data[0] = 127;
    _data[1] = 127;
    _data[2] = 127;
    _data[3] = 50;
  }

  ParticleColor.RGBA(int r, int g, int b, int a) {
    setRGBA(r, g, b, a);
  }

  ParticleColor.color3i(Color3i color) {
    setColor3i(color);
  }

  void setColor3i(Color3i color) {
    r = color.x;
    g = color.y;
    b = color.z;
    a = 255;
  }

  void setParticleColor(ParticleColor color) {
    r = color.r;
    g = color.g;
    b = color.b;
    a = color.a;
  }

  bool isZero() {
    return r == 0 && g == 0 && b == 0 && a == 0;
  }

  void setRGBA(int r, int g, int b, int a) {
    this.r = r;
    this.g = g;
    this.b = b;
    this.a = a;
  }
}
