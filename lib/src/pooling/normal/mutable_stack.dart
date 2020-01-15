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

abstract class MutableStack<E> implements IDynamicStack<E> {
  List<E> _stack;
  int _index;
  int _size;

  MutableStack(int argInitSize) {
    _index = 0;
    _stack = null;
    _index = 0;
    _size = 0;
    extendStack(argInitSize);
  }

  void extendStack(int argSize) {
    List<E> newStack = newArray(argSize);
    if (_stack != null) {
      BufferUtils.arraycopy(_stack, 0, newStack, 0, _size);
    }
    for (int i = 0; i < newStack.length; i++) {
      newStack[i] = newInstance();
    }
    _stack = newStack;
    _size = newStack.length;
  }

  E pop() {
    if (_index >= _size) {
      extendStack(_size * 2);
    }
    return _stack[_index++];
  }

  void push(E argObject) {
    assert(_index > 0);
    _stack[--_index] = argObject;
  }

  /// Creates a new instance of the object contained by this stack.
  E newInstance();

  List<E> newArray(int size) {
    return new List<E>(size);
  }
}
