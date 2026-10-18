// Creating and destroying worlds under a process-wide lock.
//
// Box2D keeps every world in one process-wide table, and b2CreateWorld and
// b2DestroyWorld claim and free its slots without synchronization. Box2D
// leaves the guard to the caller: "You will get a race condition if you
// create or destroy Box2D worlds from multiple threads. You should use a mutex
// to guard this." (manual, Multithreading Multiple Worlds; see also
// https://github.com/erincatto/box2d/issues/904).
//
// Dart isolates run on threads of their own over one copy of this library, so
// the native backend creates and destroys worlds through these two functions
// instead. Everything else a world does only touches that world, which must be
// used from one isolate at a time.
//
// Only the native (FFI) build compiles this file. The WebAssembly module runs
// on a single thread, and each instance has its own memory and world table.

#include "box2d/box2d.h"

#if defined(_WIN32)
#include <windows.h>
static SRWLOCK f2d_world_lock = SRWLOCK_INIT;
static void f2d_lock(void) { AcquireSRWLockExclusive(&f2d_world_lock); }
static void f2d_unlock(void) { ReleaseSRWLockExclusive(&f2d_world_lock); }
#else
#include <pthread.h>
static pthread_mutex_t f2d_world_lock = PTHREAD_MUTEX_INITIALIZER;
static void f2d_lock(void) { pthread_mutex_lock(&f2d_world_lock); }
static void f2d_unlock(void) { pthread_mutex_unlock(&f2d_world_lock); }
#endif

BOX2D_EXPORT b2WorldId f2d_create_world_locked(const b2WorldDef* def) {
  f2d_lock();
  b2WorldId id = b2CreateWorld(def);
  f2d_unlock();
  return id;
}

BOX2D_EXPORT void f2d_destroy_world_locked(b2WorldId id) {
  f2d_lock();
  b2DestroyWorld(id);
  f2d_unlock();
}
