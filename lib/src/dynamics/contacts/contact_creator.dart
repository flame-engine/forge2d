part of box2d;

abstract class ContactCreator {
  Contact contactCreateFcn(
      IWorldPool argPool, Fixture fixtureA, Fixture fixtureB);

  void contactDestroyFcn(IWorldPool argPool, Contact contact);
}
