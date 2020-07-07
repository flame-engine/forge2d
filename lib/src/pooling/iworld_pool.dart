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

  Float64List getFloatArray(int argLength);

  List<int> getIntArray(int argLength);

  List<Vector2> getVec2Array(int argLength);
}
