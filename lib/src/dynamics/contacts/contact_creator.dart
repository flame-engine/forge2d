part of box2d;

abstract class ContactCreator {
  Contact contactCreateFcn(Fixture fixtureA, Fixture fixtureB);

  void contactDestroyFcn(Contact contact);
}
