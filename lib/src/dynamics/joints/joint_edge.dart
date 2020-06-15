part of box2d;

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
class JointEdge {
  /// Provides quick access to the other body attached
  Body other = null;

  /// the joint
  Joint joint = null;

  /// the previous joint edge in the body's joint list
  JointEdge prev = null;

  /// the next joint edge in the body's joint list
  JointEdge next = null;
}
