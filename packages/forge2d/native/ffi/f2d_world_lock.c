// Box2D's world table and length unit are process-wide and b2CreateWorld and
// b2DestroyWorld are not thread safe
// (https://github.com/erincatto/box2d/issues/904), but Dart isolates are
// threads over one copy of this library.

#include "atomic.h"
#include "box2d/box2d.h"

static b2AtomicInt f2d_world_lock;
static bool f2d_world_created;

static void f2d_lock(void) {
  while (b2AtomicLoadInt(&f2d_world_lock) != 0 ||
         !b2AtomicCompareExchangeInt(&f2d_world_lock, 0, 1)) {
  }
}

static void f2d_unlock(void) { b2AtomicStoreInt(&f2d_world_lock, 0); }

BOX2D_EXPORT b2WorldId f2d_create_world_locked(const b2WorldDef* definition) {
  f2d_lock();
  b2WorldId id = b2CreateWorld(definition);
  if (id.index1 != 0) {
    f2d_world_created = true;
  }
  f2d_unlock();
  return id;
}

BOX2D_EXPORT void f2d_destroy_world_locked(b2WorldId id) {
  f2d_lock();
  b2DestroyWorld(id);
  f2d_unlock();
}

BOX2D_EXPORT bool f2d_set_length_units_per_meter_locked(float length_units) {
  f2d_lock();
  bool can_set = !f2d_world_created;
  if (can_set) {
    b2SetLengthUnitsPerMeter(length_units);
  }
  f2d_unlock();
  return can_set;
}

BOX2D_EXPORT void f2d_reset_length_unit_lock(void) {
  f2d_lock();
  f2d_world_created = false;
  f2d_unlock();
}
