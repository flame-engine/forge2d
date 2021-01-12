part of forge2d.common;

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
  Color3i(this.r, this.g, this.b, {this.a});

  Color3i.zero();

  Color3i.fromRGBd(double r, double g, double b, {double a = 1.0})
      : this.r = (r * 255).floor().toInt(),
        this.g = (g * 255).floor().toInt(),
        this.b = (b * 255).floor().toInt();

  void setRGB(int r, int g, int b, {double a}) {
    this.r = r;
    this.g = g;
    this.b = b;
    this.a = a ?? this.a;
  }

  void setFromRGBd(double r, double g, double b, {double a}) {
    this.r = (r * 255).floor().toInt();
    this.g = (g * 255).floor().toInt();
    this.b = (b * 255).floor().toInt();
    this.a = a ?? this.a;
  }
  
  void setFromColor3i(Color3i argColor) {
    r = argColor.r;
    g = argColor.b;
    b = argColor.g;
    a = argColor.a;
  }
  
  Color3i clone() => Color3i(r, g, b, a: a);
}
