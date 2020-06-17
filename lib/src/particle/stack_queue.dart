part of box2d;

class StackQueue<T> {
  List<T> _buffer;
  int _front = 0;
  int _back = 0;
  int _end = 0;

  StackQueue() {}

  void reset(List<T> buffer) {
    _buffer = buffer;
    _front = 0;
    _back = 0;
    _end = buffer.length;
  }

  void push(T task) {
    if (_back >= _end) {
      BufferUtils.arrayCopy(_buffer, _front, _buffer, 0, _back - _front);
      _back -= _front;
      _front = 0;
      if (_back >= _end) {
        return;
      }
    }
    _buffer[_back++] = task;
  }

  T pop() {
    assert(_front < _back);
    return _buffer[_front++];
  }

  bool empty() {
    return _front >= _back;
  }

  T front() {
    return _buffer[_front];
  }
}
