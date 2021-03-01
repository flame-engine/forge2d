class Color3i {
  static Color3i get white => Color3i(255, 255, 255);
  static Color3i get black => Color3i(0, 0, 0);
  static Color3i get blue => Color3i(0, 0, 255);
  static Color3i get green => Color3i(0, 255, 0);
  static Color3i get red => Color3i(255, 0, 0);

  int r = 0;
  int g = 0;
  int b = 0;
  double a = 1.0;
  Color3i(this.r, this.g, this.b, {this.a = 1.0});

  Color3i.zero();

  Color3i.fromRGBd(double red, double green, double blue, {double alpha = 1.0})
      : r = (red * 255).floor().toInt(),
        g = (green * 255).floor().toInt(),
        b = (blue * 255).floor().toInt();

  void setRGB(int red, int green, int blue, {double alpha}) {
    r = red;
    g = green;
    b = blue;
    a = alpha ?? a;
  }

  void setFromRGBd(double red, double green, double blue, {double alpha}) {
    r = (red * 255).floor().toInt();
    g = (green * 255).floor().toInt();
    b = (blue * 255).floor().toInt();
    a = alpha ?? a;
  }

  void setFromColor3i(Color3i argColor) {
    r = argColor.r;
    g = argColor.b;
    b = argColor.g;
    a = argColor.a;
  }

  Color3i clone() => Color3i(r, g, b, a: a);

  @override
  String toString() => "Color3i($r, $g, $b, $a)";
}
