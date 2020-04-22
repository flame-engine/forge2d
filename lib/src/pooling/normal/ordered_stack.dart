/*******************************************************************************
 * Copyright (c) 2015, Daniel Murphy, Google
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

part of box2d;

abstract class OrderedStack<E> {
  final List<E> _pool;
  int _index;
  final int _size;
  final List<E> _container;

  OrderedStack(int argStackSize, int argContainerSize)
      : _pool = new List<E>(argStackSize),
        _index = 0,
        _size = argStackSize,
        _container = new List<E>(argContainerSize) {
    // pool = new List(argStackSize);
    for (int i = 0; i < argStackSize; i++) {
      _pool[i] = newInstance();
    }
  }

  E pop() {
    assert(_index <
        _size); // "End of stack reached, there is probably a leak somewhere";
    return _pool[_index++];
  }

  List<E> popSome(int argNum) {
    assert(_index + argNum <
        _size); // "End of stack reached, there is probably a leak somewhere";
    assert(argNum <= _container.length); // "Container array is too small";
    BufferUtils.arraycopy(_pool, _index, _container, 0, argNum);
    _index += argNum;
    return _container;
  }

  void push(int argNum) {
    _index -= argNum;
    assert(_index >= 0);
  }

  /// Creates a new instance of the object contained by this stack.
  E newInstance();
}
