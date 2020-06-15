part of box2d;

class DynamicTreeNode {
  /// Enlarged AABB
  final AABB aabb = AABB();

  Object userData;

  DynamicTreeNode parent;

  DynamicTreeNode child1;
  DynamicTreeNode child2;
  final int id;
  int height = 0;

  DynamicTreeNode(this.id);
}
