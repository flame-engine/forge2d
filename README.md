## Forge2D - A Dart port of the Box2D physics engine
This is Box2D for Dart (and Flame).
Box2D physics engine is a famous physics engine and this is our port of it.
You can use it independently in Dart or in your [Flame](https://github.com/flame-engine/flame) project with the help of [flame_forge2d](https://github.com/flame-engine/flame_forge2d).
Some documentation of how to use it together with flame can be found [here](https://github.com/flame-engine/flame/blob/master/doc/box2d.md).

### Timeline
Box2D was first written in C++ and released by [Erin Catto](https://github.com/erincatto) in 2007, but it is still maintained.
It was then ported to Java (jbox2d) by [Daniel Murphy](https://github.com/dmurph) around 2015.
Then from that Java port it was ported to Dart by [Dominic Hamon](https://github.com/dominichamon) and [Kevin Moore](https://github.com/kevmoo).
A few years after that [Lukas Klingsbo](https://github.com/spydon) refactored the code to follow the dart standard more, since it still had a lot of reminiscence from C++.
After this refactor we renamed it to Forge2D since the upstream wasn't maintained to take in our PRs.
There has also been countless other contributors which we are very thankful to!