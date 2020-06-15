part of box2d;

/// A contact edge is used to connect bodies and contacts together in a contact graph where each body
/// is a node and each contact is an edge. A contact edge belongs to a doubly linked list maintained
/// in each attached body. Each contact has two contact nodes, one for each attached body.
///
class ContactEdge {
  /// provides quick access to the other body attached.
  Body other;

  /// the contact
  Contact contact;

  /// the previous contact edge in the body's contact list
  ContactEdge prev;

  /// the next contact edge in the body's contact list
  ContactEdge next;
}
