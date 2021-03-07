import '../../../forge2d.dart';

class DynamicTreeNode {
  DynamicTreeNode(this.id);
  final int id;

  /// Enlarged AABB
  final AABB aabb = AABB();

  Object? userData;

  DynamicTreeNode? parent;
  DynamicTreeNode? child1;
  DynamicTreeNode? child2;
  int height = 0;
}
