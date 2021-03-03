import 'dart:typed_data';

/// Reallocate a buffer.
List<T> reallocateBufferWithAlloc<T>(
  List<T> oldBuffer,
  int newCapacity,
  T Function() alloc,
) {
  assert(newCapacity > oldBuffer.length);
  return oldBuffer +
      List.generate(
        newCapacity - oldBuffer.length,
        (_) => alloc(),
      );
}

/// Reallocate a buffer.
List<int> reallocateBufferInt(
  List<int> oldBuffer,
  int newCapacity,
) {
  assert(newCapacity > oldBuffer.length);
  return oldBuffer + List.filled(newCapacity - oldBuffer.length, 0);
}

/// Reallocate a buffer.
Float64List reallocateBuffer(
  Float64List oldBuffer,
  int newCapacity,
) {
  assert(newCapacity > oldBuffer.length);
  return Float64List(newCapacity)..setRange(0, oldBuffer.length, oldBuffer);
}

/// Rotate an array
void rotate<T>(List<T> ray, int first, int newFirst, int last) {
  var next = newFirst;
  while (next != first) {
    final temp = ray[first];
    ray[first] = ray[next];
    ray[next] = temp;
    first++;
    next++;
    if (next == last) {
      next = newFirst;
    } else if (first == newFirst) {
      newFirst = next;
    }
  }
}

// Replace Java's Arrays::sort.
// TODO(srdjan): Make a version that does not require copying.
void sort<T>(List<T> list, int fromPos, int toPos) {
  final temp = List<T>.from(list.getRange(fromPos, toPos));
  temp.sort();
  list.setRange(fromPos, toPos, temp);
}
