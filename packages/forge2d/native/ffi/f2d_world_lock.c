// Box2D's world table is process-wide and b2CreateWorld and b2DestroyWorld
// are not thread safe (https://github.com/erincatto/box2d/issues/904), but
// Dart isolates are threads over one copy of this library.

#include "atomic.h"
#include "box2d/box2d.h"

static b2AtomicInt f2d_world_lock;

static void f2d_lock(void) {
  while (b2AtomicLoadInt(&f2d_world_lock) != 0 ||
         !b2AtomicCompareExchangeInt(&f2d_world_lock, 0, 1)) {
  }
}

static void f2d_unlock(void) { b2AtomicStoreInt(&f2d_world_lock, 0); }

BOX2D_EXPORT b2WorldId f2d_create_world_locked(const b2WorldDef* definition) {
  f2d_lock();
  b2WorldId id = b2CreateWorld(definition);
  f2d_unlock();
  return id;
}

BOX2D_EXPORT void f2d_destroy_world_locked(b2WorldId id) {
  f2d_lock();
  b2DestroyWorld(id);
  f2d_unlock();
}
