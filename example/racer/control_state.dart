/// Used to track keyboard state. Entries are masked bitwise.

class ControlState {
  static const int UP = 1;
  static const int DOWN = 2;
  static const int LEFT = 4;
  static const int RIGHT = 8;
}
