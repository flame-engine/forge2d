part of box2d;

/// This holds contact filtering data.
class Filter {
  /// The collision category bits. Normally you would just set one bit.
  int categoryBits = 0x0001;

  /// The collision mask bits. This states the categories that this
  /// shape would accept for collision.
  int maskBits = 0xFFFF;

  /// Collision groups allow a certain group of objects to never collide (negative)
  /// or always collide (positive). Zero means no collision group. Non-zero group
  /// filtering always wins against the mask bits.
  int groupIndex = 0;

  void set(Filter argOther) {
    categoryBits = argOther.categoryBits;
    maskBits = argOther.maskBits;
    groupIndex = argOther.groupIndex;
  }
}
