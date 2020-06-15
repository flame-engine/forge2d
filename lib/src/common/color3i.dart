part of box2d.common;

class Color3i {
  static final Color3i WHITE = Color3i(255, 255, 255);
  static final Color3i BLACK = Color3i(0, 0, 0);
  static final Color3i BLUE = Color3i(0, 0, 255);
  static final Color3i GREEN = Color3i(0, 255, 0);
  static final Color3i RED = Color3i(255, 0, 0);

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
