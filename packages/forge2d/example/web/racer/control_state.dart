/// Used to track keyboard state. Entries are masked bitwise.
class ControlState {
  static const int up = 1;
  static const int down = 2;
  static const int left = 4;
  static const int right = 8;
}
