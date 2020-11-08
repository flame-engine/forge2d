part of forge2d;

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
class JointEdge {
  /// Provides quick access to the other body attached
  Body other;

  /// the joint
  Joint joint;

  /// the previous joint edge in the body's joint list
  JointEdge prev;

  /// the next joint edge in the body's joint list
  JointEdge next;
}
