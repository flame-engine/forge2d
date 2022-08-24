import 'package:forge2d/forge2d.dart';

class DynamicTreeNode {
  final int id;
  DynamicTreeNode(this.id);

  /// Enlarged AABB
  final AABB aabb = AABB();

  Object? userData;

  DynamicTreeNode? parent;
  DynamicTreeNode? child1;
  DynamicTreeNode? child2;
  int height = 0;
}
