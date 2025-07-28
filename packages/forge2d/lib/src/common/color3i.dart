class Color3i {
  Color3i(this.r, this.g, this.b, {this.a = 1.0});

  Color3i.white() : this(255, 255, 255);
  Color3i.black() : this(0, 0, 0);
  Color3i.blue() : this(0, 0, 255);
  Color3i.green() : this(0, 255, 0);
  Color3i.red() : this(255, 0, 0);

  int r = 0;
  int g = 0;
  int b = 0;
  double a = 1.0;

  Color3i.zero();

  Color3i.fromRGBd(double red, double green, double blue, {double alpha = 1.0})
    : r = (red * 255).floor(),
      g = (green * 255).floor(),
      b = (blue * 255).floor(),
      a = alpha;

  void setRGB(int red, int green, int blue, {double? alpha}) {
    r = red;
    g = green;
    b = blue;
    a = alpha ?? a;
  }

  void setFromRGBd(double red, double green, double blue, {double? alpha}) {
    r = (red * 255).floor();
    g = (green * 255).floor();
    b = (blue * 255).floor();
    a = alpha ?? a;
  }

  void setFromColor3i(Color3i argColor) {
    r = argColor.r;
    g = argColor.b;
    b = argColor.g;
    a = argColor.a;
  }

  Color3i clone() => Color3i(r, g, b, a: a);

  String toHex() =>
      '#${r.toRadixString(16).padLeft(2, '0')}'
      '${g.toRadixString(16).padLeft(2, '0')}'
      '${b.toRadixString(16).padLeft(2, '0')}'
      '${(a * 255).toInt().toRadixString(16).padLeft(2, '0')}';

  @override
  String toString() => 'Color3i($r, $g, $b, $a)';
}
