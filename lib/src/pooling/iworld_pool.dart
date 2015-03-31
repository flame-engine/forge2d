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

abstract class IWorldPool {
  IDynamicStack<Contact> getPolyContactStack();

  IDynamicStack<Contact> getCircleContactStack();

  IDynamicStack<Contact> getPolyCircleContactStack();

  IDynamicStack<Contact> getEdgeCircleContactStack();

  IDynamicStack<Contact> getEdgePolyContactStack();

  IDynamicStack<Contact> getChainCircleContactStack();

  IDynamicStack<Contact> getChainPolyContactStack();

  Vector2 popVec2();

  List<Vector2> popVec2Some(int num);

  void pushVec2(int num);

  Vector3 popVec3();

  List<Vector3> popVec3Some(int num);

  void pushVec3(int num);

  Matrix2 popMat22();

  List<Matrix2> popMat22Some(int num);

  void pushMat22(int num);

  Matrix3 popMat33();

  void pushMat33(int num);

  AABB popAABB();

  List<AABB> popAABBSome(int num);

  void pushAABB(int num);

  Rot popRot();

  void pushRot(int num);

  Collision getCollision();

  TimeOfImpact getTimeOfImpact();

  Distance getDistance();

  Float64List getFloatArray(int argLength);

  List<int> getIntArray(int argLength);

  List<Vector2> getVec2Array(int argLength);
}
