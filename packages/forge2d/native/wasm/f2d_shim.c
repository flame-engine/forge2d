// The forge2d WebAssembly shim.
//
// The wasm C ABI passes structs indirectly, which makes the raw Box2D
// exports impractical to call from JavaScript. This shim wraps every
// function the Dart backend needs with a flat scalar/pointer signature:
//
// - World ids cross as one uint32: index1 | (generation << 16).
// - Body/shape/chain/joint ids cross as (int32 index1, uint32 wg) where
//   wg packs world0 | (generation << 16), matching the Dart seam.
// - Vectors and tuples are written into caller-provided buffers.
// - 64-bit filter bits cross as two uint32 halves.
// - Events are copied into caller-provided float64 buffers (float64
//   represents every int32 exactly, which keeps one buffer per event kind).
// - Synchronous callbacks (queries, custom filter, pre-solve, debug draw)
//   call fixed host imports provided at instantiation time.
//
// Keep this file in sync with lib/src/backend/raw_box2d_wasm.dart.

#include <emscripten.h>
#include <string.h>

#include "box2d/box2d.h"

#define F2D_EXPORT EMSCRIPTEN_KEEPALIVE

// Host imports, provided by the Dart side at instantiation.
extern float f2d_host_cast_ray(int32_t shape_index1, uint32_t shape_wg,
                               float px, float py, float nx, float ny,
                               float fraction);
extern int f2d_host_overlap(int32_t shape_index1, uint32_t shape_wg);
extern int f2d_host_custom_filter(int32_t a_index1, uint32_t a_wg,
                                  int32_t b_index1, uint32_t b_wg);
extern int f2d_host_pre_solve(int32_t a_index1, uint32_t a_wg,
                              int32_t b_index1, uint32_t b_wg, float nx,
                              float ny);
extern void f2d_host_draw_polygon(const float* vertices, int count,
                                  uint32_t color);
extern void f2d_host_draw_solid_polygon(float px, float py, float qc, float qs,
                                        const float* vertices, int count,
                                        float radius, uint32_t color);
extern void f2d_host_draw_circle(float x, float y, float radius,
                                 uint32_t color);
extern void f2d_host_draw_solid_circle(float px, float py, float qc, float qs,
                                       float radius, uint32_t color);
extern void f2d_host_draw_solid_capsule(float p1x, float p1y, float p2x,
                                        float p2y, float radius,
                                        uint32_t color);
extern void f2d_host_draw_segment(float p1x, float p1y, float p2x, float p2y,
                                  uint32_t color);
extern void f2d_host_draw_transform(float px, float py, float qc, float qs);
extern void f2d_host_draw_point(float x, float y, float size, uint32_t color);
extern void f2d_host_draw_string(float x, float y, const char* text,
                                 uint32_t color);

// Id conversions.

static b2WorldId f2d_world(uint32_t id) {
  return (b2WorldId){(uint16_t)(id & 0xFFFF), (uint16_t)(id >> 16)};
}

static uint32_t f2d_pack_world(b2WorldId id) {
  return (uint32_t)id.index1 | ((uint32_t)id.generation << 16);
}

static b2BodyId f2d_body(int32_t index1, uint32_t wg) {
  return (b2BodyId){index1, (uint16_t)(wg & 0xFFFF), (uint16_t)(wg >> 16)};
}

static b2ShapeId f2d_shape(int32_t index1, uint32_t wg) {
  return (b2ShapeId){index1, (uint16_t)(wg & 0xFFFF), (uint16_t)(wg >> 16)};
}

static b2ChainId f2d_chain(int32_t index1, uint32_t wg) {
  return (b2ChainId){index1, (uint16_t)(wg & 0xFFFF), (uint16_t)(wg >> 16)};
}

static b2JointId f2d_joint(int32_t index1, uint32_t wg) {
  return (b2JointId){index1, (uint16_t)(wg & 0xFFFF), (uint16_t)(wg >> 16)};
}

static uint32_t f2d_wg_body(b2BodyId id) {
  return (uint32_t)id.world0 | ((uint32_t)id.generation << 16);
}

static uint32_t f2d_wg_shape(b2ShapeId id) {
  return (uint32_t)id.world0 | ((uint32_t)id.generation << 16);
}

static uint32_t f2d_wg_chain(b2ChainId id) {
  return (uint32_t)id.world0 | ((uint32_t)id.generation << 16);
}

static uint32_t f2d_wg_joint(b2JointId id) {
  return (uint32_t)id.world0 | ((uint32_t)id.generation << 16);
}

static uint64_t f2d_bits(uint32_t lo, uint32_t hi) {
  return (uint64_t)lo | ((uint64_t)hi << 32);
}

static b2Filter f2d_filter(uint32_t category_lo, uint32_t category_hi,
                           uint32_t mask_lo, uint32_t mask_hi,
                           int32_t group_index) {
  b2Filter filter = b2DefaultFilter();
  filter.categoryBits = f2d_bits(category_lo, category_hi);
  filter.maskBits = f2d_bits(mask_lo, mask_hi);
  filter.groupIndex = group_index;
  return filter;
}

static b2QueryFilter f2d_query_filter(uint32_t category_lo,
                                      uint32_t category_hi, uint32_t mask_lo,
                                      uint32_t mask_hi) {
  b2QueryFilter filter = b2DefaultQueryFilter();
  filter.categoryBits = f2d_bits(category_lo, category_hi);
  filter.maskBits = f2d_bits(mask_lo, mask_hi);
  return filter;
}

// World.

F2D_EXPORT uint32_t f2d_create_world(
    float gravity_x, float gravity_y, float restitution_threshold,
    float hit_event_threshold, float contact_hertz,
    float contact_damping_ratio, float max_contact_push_speed,
    float maximum_linear_speed, int enable_sleep, int enable_continuous) {
  b2WorldDef def = b2DefaultWorldDef();
  def.gravity = (b2Vec2){gravity_x, gravity_y};
  def.restitutionThreshold = restitution_threshold;
  def.hitEventThreshold = hit_event_threshold;
  def.contactHertz = contact_hertz;
  def.contactDampingRatio = contact_damping_ratio;
  def.maxContactPushSpeed = max_contact_push_speed;
  def.maximumLinearSpeed = maximum_linear_speed;
  def.enableSleep = enable_sleep;
  def.enableContinuous = enable_continuous;
  return f2d_pack_world(b2CreateWorld(&def));
}

F2D_EXPORT void f2d_destroy_world(uint32_t w) { b2DestroyWorld(f2d_world(w)); }

F2D_EXPORT int f2d_world_is_valid(uint32_t w) {
  return b2World_IsValid(f2d_world(w));
}

F2D_EXPORT void f2d_world_step(uint32_t w, float time_step, int sub_steps) {
  b2World_Step(f2d_world(w), time_step, sub_steps);
}

F2D_EXPORT void f2d_world_set_gravity(uint32_t w, float x, float y) {
  b2World_SetGravity(f2d_world(w), (b2Vec2){x, y});
}

F2D_EXPORT void f2d_world_get_gravity(uint32_t w, float* out) {
  b2Vec2 gravity = b2World_GetGravity(f2d_world(w));
  out[0] = gravity.x;
  out[1] = gravity.y;
}

F2D_EXPORT void f2d_world_enable_sleeping(uint32_t w, int enabled) {
  b2World_EnableSleeping(f2d_world(w), enabled);
}

F2D_EXPORT int f2d_world_is_sleeping_enabled(uint32_t w) {
  return b2World_IsSleepingEnabled(f2d_world(w));
}

F2D_EXPORT void f2d_world_enable_continuous(uint32_t w, int enabled) {
  b2World_EnableContinuous(f2d_world(w), enabled);
}

F2D_EXPORT int f2d_world_is_continuous_enabled(uint32_t w) {
  return b2World_IsContinuousEnabled(f2d_world(w));
}

F2D_EXPORT void f2d_world_explode(uint32_t w, uint32_t mask_lo,
                                  uint32_t mask_hi, float x, float y,
                                  float radius, float falloff,
                                  float impulse_per_length) {
  b2ExplosionDef def = b2DefaultExplosionDef();
  def.maskBits = f2d_bits(mask_lo, mask_hi);
  def.position = (b2Vec2){x, y};
  def.radius = radius;
  def.falloff = falloff;
  def.impulsePerLength = impulse_per_length;
  b2World_Explode(f2d_world(w), &def);
}

// Bodies.

F2D_EXPORT void f2d_create_body(
    uint32_t w, int type, float px, float py, float qc, float qs, float lvx,
    float lvy, float angular_velocity, float linear_damping,
    float angular_damping, float gravity_scale, float sleep_threshold,
    const char* name, int enable_sleep, int is_awake, int fixed_rotation,
    int is_bullet, int is_enabled, int allow_fast_rotation, int32_t* out) {
  b2BodyDef def = b2DefaultBodyDef();
  def.type = (b2BodyType)type;
  def.position = (b2Vec2){px, py};
  def.rotation = (b2Rot){qc, qs};
  def.linearVelocity = (b2Vec2){lvx, lvy};
  def.angularVelocity = angular_velocity;
  def.linearDamping = linear_damping;
  def.angularDamping = angular_damping;
  def.gravityScale = gravity_scale;
  def.sleepThreshold = sleep_threshold;
  def.name = name;
  def.enableSleep = enable_sleep;
  def.isAwake = is_awake;
  def.fixedRotation = fixed_rotation;
  def.isBullet = is_bullet;
  def.isEnabled = is_enabled;
  def.allowFastRotation = allow_fast_rotation;
  b2BodyId id = b2CreateBody(f2d_world(w), &def);
  out[0] = id.index1;
  out[1] = (int32_t)f2d_wg_body(id);
}

F2D_EXPORT void f2d_destroy_body(int32_t i1, uint32_t wg) {
  b2DestroyBody(f2d_body(i1, wg));
}

F2D_EXPORT int f2d_body_is_valid(int32_t i1, uint32_t wg) {
  return b2Body_IsValid(f2d_body(i1, wg));
}

F2D_EXPORT void f2d_body_get_position(int32_t i1, uint32_t wg, float* out) {
  b2Vec2 position = b2Body_GetPosition(f2d_body(i1, wg));
  out[0] = position.x;
  out[1] = position.y;
}

F2D_EXPORT void f2d_body_get_rotation(int32_t i1, uint32_t wg, float* out) {
  b2Rot rotation = b2Body_GetRotation(f2d_body(i1, wg));
  out[0] = rotation.c;
  out[1] = rotation.s;
}

F2D_EXPORT void f2d_body_set_transform(int32_t i1, uint32_t wg, float px,
                                       float py, float qc, float qs) {
  b2Body_SetTransform(f2d_body(i1, wg), (b2Vec2){px, py}, (b2Rot){qc, qs});
}

F2D_EXPORT void f2d_body_get_linear_velocity(int32_t i1, uint32_t wg,
                                             float* out) {
  b2Vec2 velocity = b2Body_GetLinearVelocity(f2d_body(i1, wg));
  out[0] = velocity.x;
  out[1] = velocity.y;
}

F2D_EXPORT void f2d_body_set_linear_velocity(int32_t i1, uint32_t wg, float x,
                                             float y) {
  b2Body_SetLinearVelocity(f2d_body(i1, wg), (b2Vec2){x, y});
}

F2D_EXPORT void f2d_body_apply_force(int32_t i1, uint32_t wg, float fx,
                                     float fy, float px, float py, int wake) {
  b2Body_ApplyForce(f2d_body(i1, wg), (b2Vec2){fx, fy}, (b2Vec2){px, py},
                    wake);
}

F2D_EXPORT void f2d_body_apply_force_to_center(int32_t i1, uint32_t wg,
                                               float fx, float fy, int wake) {
  b2Body_ApplyForceToCenter(f2d_body(i1, wg), (b2Vec2){fx, fy}, wake);
}

F2D_EXPORT void f2d_body_apply_torque(int32_t i1, uint32_t wg, float torque,
                                      int wake) {
  b2Body_ApplyTorque(f2d_body(i1, wg), torque, wake);
}

F2D_EXPORT void f2d_body_apply_linear_impulse(int32_t i1, uint32_t wg,
                                              float ix, float iy, float px,
                                              float py, int wake) {
  b2Body_ApplyLinearImpulse(f2d_body(i1, wg), (b2Vec2){ix, iy},
                            (b2Vec2){px, py}, wake);
}

F2D_EXPORT void f2d_body_apply_linear_impulse_to_center(int32_t i1,
                                                        uint32_t wg, float ix,
                                                        float iy, int wake) {
  b2Body_ApplyLinearImpulseToCenter(f2d_body(i1, wg), (b2Vec2){ix, iy}, wake);
}

F2D_EXPORT void f2d_body_apply_angular_impulse(int32_t i1, uint32_t wg,
                                               float impulse, int wake) {
  b2Body_ApplyAngularImpulse(f2d_body(i1, wg), impulse, wake);
}

F2D_EXPORT void f2d_body_get_local_center(int32_t i1, uint32_t wg,
                                          float* out) {
  b2Vec2 center = b2Body_GetLocalCenterOfMass(f2d_body(i1, wg));
  out[0] = center.x;
  out[1] = center.y;
}

F2D_EXPORT void f2d_body_get_world_center(int32_t i1, uint32_t wg,
                                          float* out) {
  b2Vec2 center = b2Body_GetWorldCenterOfMass(f2d_body(i1, wg));
  out[0] = center.x;
  out[1] = center.y;
}

F2D_EXPORT void f2d_body_set_mass_data(int32_t i1, uint32_t wg, float mass,
                                       float rotational_inertia, float cx,
                                       float cy) {
  b2MassData mass_data = {mass, {cx, cy}, rotational_inertia};
  b2Body_SetMassData(f2d_body(i1, wg), mass_data);
}

F2D_EXPORT const char* f2d_body_get_name(int32_t i1, uint32_t wg) {
  return b2Body_GetName(f2d_body(i1, wg));
}

F2D_EXPORT void f2d_body_set_name(int32_t i1, uint32_t wg, const char* name) {
  b2Body_SetName(f2d_body(i1, wg), name);
}

F2D_EXPORT void f2d_body_get_world_point(int32_t i1, uint32_t wg, float x,
                                         float y, float* out) {
  b2Vec2 point = b2Body_GetWorldPoint(f2d_body(i1, wg), (b2Vec2){x, y});
  out[0] = point.x;
  out[1] = point.y;
}

F2D_EXPORT void f2d_body_get_local_point(int32_t i1, uint32_t wg, float x,
                                         float y, float* out) {
  b2Vec2 point = b2Body_GetLocalPoint(f2d_body(i1, wg), (b2Vec2){x, y});
  out[0] = point.x;
  out[1] = point.y;
}

F2D_EXPORT int f2d_body_get_shape_count(int32_t i1, uint32_t wg) {
  return b2Body_GetShapeCount(f2d_body(i1, wg));
}

F2D_EXPORT int f2d_body_get_shapes(int32_t i1, uint32_t wg, int32_t* out,
                                   int capacity) {
  b2ShapeId shapes[capacity];
  int count = b2Body_GetShapes(f2d_body(i1, wg), shapes, capacity);
  for (int i = 0; i < count; ++i) {
    out[2 * i] = shapes[i].index1;
    out[2 * i + 1] = (int32_t)f2d_wg_shape(shapes[i]);
  }
  return count;
}

F2D_EXPORT int f2d_body_get_joint_count(int32_t i1, uint32_t wg) {
  return b2Body_GetJointCount(f2d_body(i1, wg));
}

F2D_EXPORT int f2d_body_get_joints(int32_t i1, uint32_t wg, int32_t* out,
                                   int capacity) {
  b2JointId joints[capacity];
  int count = b2Body_GetJoints(f2d_body(i1, wg), joints, capacity);
  for (int i = 0; i < count; ++i) {
    out[2 * i] = joints[i].index1;
    out[2 * i + 1] = (int32_t)f2d_wg_joint(joints[i]);
  }
  return count;
}

// Scalar body accessors, generated by macro.
#define F2D_BODY_GET_FLOAT(name, fn) \
  F2D_EXPORT float name(int32_t i1, uint32_t wg) { return fn(f2d_body(i1, wg)); }
#define F2D_BODY_SET_FLOAT(name, fn) \
  F2D_EXPORT void name(int32_t i1, uint32_t wg, float v) { fn(f2d_body(i1, wg), v); }
#define F2D_BODY_GET_INT(name, fn) \
  F2D_EXPORT int name(int32_t i1, uint32_t wg) { return (int)fn(f2d_body(i1, wg)); }
#define F2D_BODY_SET_BOOL(name, fn) \
  F2D_EXPORT void name(int32_t i1, uint32_t wg, int v) { fn(f2d_body(i1, wg), v); }
#define F2D_BODY_CALL(name, fn) \
  F2D_EXPORT void name(int32_t i1, uint32_t wg) { fn(f2d_body(i1, wg)); }

F2D_BODY_GET_FLOAT(f2d_body_get_angular_velocity, b2Body_GetAngularVelocity)
F2D_BODY_SET_FLOAT(f2d_body_set_angular_velocity, b2Body_SetAngularVelocity)
F2D_BODY_GET_FLOAT(f2d_body_get_mass, b2Body_GetMass)
F2D_BODY_GET_FLOAT(f2d_body_get_rotational_inertia, b2Body_GetRotationalInertia)
F2D_BODY_CALL(f2d_body_apply_mass_from_shapes, b2Body_ApplyMassFromShapes)
F2D_BODY_GET_INT(f2d_body_get_type, b2Body_GetType)
F2D_EXPORT void f2d_body_set_type(int32_t i1, uint32_t wg, int type) {
  b2Body_SetType(f2d_body(i1, wg), (b2BodyType)type);
}
F2D_BODY_GET_INT(f2d_body_is_awake, b2Body_IsAwake)
F2D_BODY_SET_BOOL(f2d_body_set_awake, b2Body_SetAwake)
F2D_BODY_GET_INT(f2d_body_is_sleep_enabled, b2Body_IsSleepEnabled)
F2D_BODY_SET_BOOL(f2d_body_enable_sleep, b2Body_EnableSleep)
F2D_BODY_GET_FLOAT(f2d_body_get_sleep_threshold, b2Body_GetSleepThreshold)
F2D_BODY_SET_FLOAT(f2d_body_set_sleep_threshold, b2Body_SetSleepThreshold)
F2D_BODY_GET_INT(f2d_body_is_enabled, b2Body_IsEnabled)
F2D_BODY_CALL(f2d_body_disable, b2Body_Disable)
F2D_BODY_CALL(f2d_body_enable, b2Body_Enable)
F2D_BODY_GET_INT(f2d_body_is_fixed_rotation, b2Body_IsFixedRotation)
F2D_BODY_SET_BOOL(f2d_body_set_fixed_rotation, b2Body_SetFixedRotation)
F2D_BODY_GET_INT(f2d_body_is_bullet, b2Body_IsBullet)
F2D_BODY_SET_BOOL(f2d_body_set_bullet, b2Body_SetBullet)
F2D_BODY_GET_FLOAT(f2d_body_get_gravity_scale, b2Body_GetGravityScale)
F2D_BODY_SET_FLOAT(f2d_body_set_gravity_scale, b2Body_SetGravityScale)
F2D_BODY_GET_FLOAT(f2d_body_get_linear_damping, b2Body_GetLinearDamping)
F2D_BODY_SET_FLOAT(f2d_body_set_linear_damping, b2Body_SetLinearDamping)
F2D_BODY_GET_FLOAT(f2d_body_get_angular_damping, b2Body_GetAngularDamping)
F2D_BODY_SET_FLOAT(f2d_body_set_angular_damping, b2Body_SetAngularDamping)

// Shapes.

typedef struct f2dShapeDef {
  float friction;
  float restitution;
  float rolling_resistance;
  float tangent_speed;
  int32_t user_material_id;
  uint32_t custom_color;
  float density;
  uint32_t category_lo;
  uint32_t category_hi;
  uint32_t mask_lo;
  uint32_t mask_hi;
  int32_t group_index;
  int32_t is_sensor;
  int32_t enable_sensor_events;
  int32_t enable_contact_events;
  int32_t enable_hit_events;
  int32_t enable_pre_solve_events;
  int32_t invoke_contact_creation;
  int32_t update_body_mass;
} f2dShapeDef;

static b2ShapeDef f2d_shape_def(const f2dShapeDef* in) {
  b2ShapeDef def = b2DefaultShapeDef();
  def.material.friction = in->friction;
  def.material.restitution = in->restitution;
  def.material.rollingResistance = in->rolling_resistance;
  def.material.tangentSpeed = in->tangent_speed;
  def.material.userMaterialId = in->user_material_id;
  def.material.customColor = in->custom_color;
  def.density = in->density;
  def.filter = f2d_filter(in->category_lo, in->category_hi, in->mask_lo,
                          in->mask_hi, in->group_index);
  def.isSensor = in->is_sensor;
  def.enableSensorEvents = in->enable_sensor_events;
  def.enableContactEvents = in->enable_contact_events;
  def.enableHitEvents = in->enable_hit_events;
  def.enablePreSolveEvents = in->enable_pre_solve_events;
  def.invokeContactCreation = in->invoke_contact_creation;
  def.updateBodyMass = in->update_body_mass;
  return def;
}

static void f2d_out_shape(b2ShapeId id, int32_t* out) {
  out[0] = id.index1;
  out[1] = (int32_t)f2d_wg_shape(id);
}

F2D_EXPORT void f2d_create_circle_shape(int32_t i1, uint32_t wg,
                                        const f2dShapeDef* def, float cx,
                                        float cy, float radius, int32_t* out) {
  b2ShapeDef shape_def = f2d_shape_def(def);
  b2Circle circle = {{cx, cy}, radius};
  f2d_out_shape(b2CreateCircleShape(f2d_body(i1, wg), &shape_def, &circle),
                out);
}

F2D_EXPORT void f2d_create_capsule_shape(int32_t i1, uint32_t wg,
                                         const f2dShapeDef* def, float c1x,
                                         float c1y, float c2x, float c2y,
                                         float radius, int32_t* out) {
  b2ShapeDef shape_def = f2d_shape_def(def);
  b2Capsule capsule = {{c1x, c1y}, {c2x, c2y}, radius};
  f2d_out_shape(b2CreateCapsuleShape(f2d_body(i1, wg), &shape_def, &capsule),
                out);
}

F2D_EXPORT void f2d_create_segment_shape(int32_t i1, uint32_t wg,
                                         const f2dShapeDef* def, float p1x,
                                         float p1y, float p2x, float p2y,
                                         int32_t* out) {
  b2ShapeDef shape_def = f2d_shape_def(def);
  b2Segment segment = {{p1x, p1y}, {p2x, p2y}};
  f2d_out_shape(b2CreateSegmentShape(f2d_body(i1, wg), &shape_def, &segment),
                out);
}

F2D_EXPORT void f2d_create_box_shape(int32_t i1, uint32_t wg,
                                     const f2dShapeDef* def, float half_width,
                                     float half_height, float cx, float cy,
                                     float qc, float qs, float radius,
                                     int32_t* out) {
  b2ShapeDef shape_def = f2d_shape_def(def);
  b2Polygon polygon = b2MakeOffsetRoundedBox(half_width, half_height,
                                             (b2Vec2){cx, cy}, (b2Rot){qc, qs},
                                             radius);
  f2d_out_shape(b2CreatePolygonShape(f2d_body(i1, wg), &shape_def, &polygon),
                out);
}

// Returns false when the hull is degenerate.
F2D_EXPORT int f2d_create_polygon_shape(int32_t i1, uint32_t wg,
                                        const f2dShapeDef* def,
                                        const float* points, int count,
                                        float radius, int32_t* out) {
  b2Hull hull = b2ComputeHull((const b2Vec2*)points, count);
  if (hull.count == 0) {
    return 0;
  }
  b2ShapeDef shape_def = f2d_shape_def(def);
  b2Polygon polygon = b2MakePolygon(&hull, radius);
  f2d_out_shape(b2CreatePolygonShape(f2d_body(i1, wg), &shape_def, &polygon),
                out);
  return 1;
}

F2D_EXPORT void f2d_destroy_shape(int32_t i1, uint32_t wg,
                                  int update_body_mass) {
  b2DestroyShape(f2d_shape(i1, wg), update_body_mass);
}

F2D_EXPORT void f2d_shape_get_body(int32_t i1, uint32_t wg, int32_t* out) {
  b2BodyId id = b2Shape_GetBody(f2d_shape(i1, wg));
  out[0] = id.index1;
  out[1] = (int32_t)f2d_wg_body(id);
}

F2D_EXPORT void f2d_shape_set_density(int32_t i1, uint32_t wg, float density,
                                      int update_body_mass) {
  b2Shape_SetDensity(f2d_shape(i1, wg), density, update_body_mass);
}

F2D_EXPORT void f2d_shape_get_filter(int32_t i1, uint32_t wg, int32_t* out) {
  b2Filter filter = b2Shape_GetFilter(f2d_shape(i1, wg));
  out[0] = (int32_t)(filter.categoryBits & 0xFFFFFFFF);
  out[1] = (int32_t)(filter.categoryBits >> 32);
  out[2] = (int32_t)(filter.maskBits & 0xFFFFFFFF);
  out[3] = (int32_t)(filter.maskBits >> 32);
  out[4] = filter.groupIndex;
}

F2D_EXPORT void f2d_shape_set_filter(int32_t i1, uint32_t wg,
                                     uint32_t category_lo, uint32_t category_hi,
                                     uint32_t mask_lo, uint32_t mask_hi,
                                     int32_t group_index) {
  b2Shape_SetFilter(f2d_shape(i1, wg), f2d_filter(category_lo, category_hi,
                                                  mask_lo, mask_hi,
                                                  group_index));
}

F2D_EXPORT int f2d_shape_test_point(int32_t i1, uint32_t wg, float x,
                                    float y) {
  return b2Shape_TestPoint(f2d_shape(i1, wg), (b2Vec2){x, y});
}

F2D_EXPORT void f2d_shape_get_aabb(int32_t i1, uint32_t wg, float* out) {
  b2AABB aabb = b2Shape_GetAABB(f2d_shape(i1, wg));
  out[0] = aabb.lowerBound.x;
  out[1] = aabb.lowerBound.y;
  out[2] = aabb.upperBound.x;
  out[3] = aabb.upperBound.y;
}

#define F2D_SHAPE_GET_FLOAT(name, fn) \
  F2D_EXPORT float name(int32_t i1, uint32_t wg) { return fn(f2d_shape(i1, wg)); }
#define F2D_SHAPE_SET_FLOAT(name, fn) \
  F2D_EXPORT void name(int32_t i1, uint32_t wg, float v) { fn(f2d_shape(i1, wg), v); }
#define F2D_SHAPE_GET_INT(name, fn) \
  F2D_EXPORT int name(int32_t i1, uint32_t wg) { return (int)fn(f2d_shape(i1, wg)); }
#define F2D_SHAPE_SET_BOOL(name, fn) \
  F2D_EXPORT void name(int32_t i1, uint32_t wg, int v) { fn(f2d_shape(i1, wg), v); }

F2D_SHAPE_GET_INT(f2d_shape_is_valid, b2Shape_IsValid)
F2D_SHAPE_GET_INT(f2d_shape_get_type, b2Shape_GetType)
F2D_SHAPE_GET_INT(f2d_shape_is_sensor, b2Shape_IsSensor)
F2D_SHAPE_GET_FLOAT(f2d_shape_get_density, b2Shape_GetDensity)
F2D_SHAPE_GET_FLOAT(f2d_shape_get_friction, b2Shape_GetFriction)
F2D_SHAPE_SET_FLOAT(f2d_shape_set_friction, b2Shape_SetFriction)
F2D_SHAPE_GET_FLOAT(f2d_shape_get_restitution, b2Shape_GetRestitution)
F2D_SHAPE_SET_FLOAT(f2d_shape_set_restitution, b2Shape_SetRestitution)
F2D_SHAPE_GET_INT(f2d_shape_are_sensor_events_enabled,
                  b2Shape_AreSensorEventsEnabled)
F2D_SHAPE_SET_BOOL(f2d_shape_enable_sensor_events, b2Shape_EnableSensorEvents)
F2D_SHAPE_GET_INT(f2d_shape_are_contact_events_enabled,
                  b2Shape_AreContactEventsEnabled)
F2D_SHAPE_SET_BOOL(f2d_shape_enable_contact_events, b2Shape_EnableContactEvents)
F2D_SHAPE_GET_INT(f2d_shape_are_hit_events_enabled, b2Shape_AreHitEventsEnabled)
F2D_SHAPE_SET_BOOL(f2d_shape_enable_hit_events, b2Shape_EnableHitEvents)
F2D_SHAPE_GET_INT(f2d_shape_are_pre_solve_events_enabled,
                  b2Shape_ArePreSolveEventsEnabled)
F2D_SHAPE_SET_BOOL(f2d_shape_enable_pre_solve_events,
                   b2Shape_EnablePreSolveEvents)

// Chains.

F2D_EXPORT void f2d_create_chain(int32_t i1, uint32_t wg, const float* points,
                                 int point_count, const float* materials,
                                 int material_count, uint32_t category_lo,
                                 uint32_t category_hi, uint32_t mask_lo,
                                 uint32_t mask_hi, int32_t group_index,
                                 int is_loop, int enable_sensor_events,
                                 int32_t* out) {
  b2SurfaceMaterial surface_materials[material_count];
  for (int i = 0; i < material_count; ++i) {
    const float* m = materials + i * 6;
    surface_materials[i] = (b2SurfaceMaterial){
        m[0], m[1], m[2], m[3], (int)m[4], (uint32_t)m[5]};
  }
  b2ChainDef def = b2DefaultChainDef();
  def.points = (const b2Vec2*)points;
  def.count = point_count;
  def.materials = surface_materials;
  def.materialCount = material_count;
  def.filter = f2d_filter(category_lo, category_hi, mask_lo, mask_hi,
                          group_index);
  def.isLoop = is_loop;
  def.enableSensorEvents = enable_sensor_events;
  b2ChainId id = b2CreateChain(f2d_body(i1, wg), &def);
  out[0] = id.index1;
  out[1] = (int32_t)f2d_wg_chain(id);
}

F2D_EXPORT void f2d_destroy_chain(int32_t i1, uint32_t wg) {
  b2DestroyChain(f2d_chain(i1, wg));
}

F2D_EXPORT int f2d_chain_is_valid(int32_t i1, uint32_t wg) {
  return b2Chain_IsValid(f2d_chain(i1, wg));
}

F2D_EXPORT void f2d_chain_set_friction(int32_t i1, uint32_t wg, float v) {
  b2Chain_SetFriction(f2d_chain(i1, wg), v);
}

F2D_EXPORT float f2d_chain_get_friction(int32_t i1, uint32_t wg) {
  return b2Chain_GetFriction(f2d_chain(i1, wg));
}

F2D_EXPORT void f2d_chain_set_restitution(int32_t i1, uint32_t wg, float v) {
  b2Chain_SetRestitution(f2d_chain(i1, wg), v);
}

F2D_EXPORT float f2d_chain_get_restitution(int32_t i1, uint32_t wg) {
  return b2Chain_GetRestitution(f2d_chain(i1, wg));
}

F2D_EXPORT int f2d_chain_get_segment_count(int32_t i1, uint32_t wg) {
  return b2Chain_GetSegmentCount(f2d_chain(i1, wg));
}

F2D_EXPORT int f2d_chain_get_segments(int32_t i1, uint32_t wg, int32_t* out,
                                      int capacity) {
  b2ShapeId segments[capacity];
  int count = b2Chain_GetSegments(f2d_chain(i1, wg), segments, capacity);
  for (int i = 0; i < count; ++i) {
    out[2 * i] = segments[i].index1;
    out[2 * i + 1] = (int32_t)f2d_wg_shape(segments[i]);
  }
  return count;
}

// Joints.

static void f2d_out_joint(b2JointId id, int32_t* out) {
  out[0] = id.index1;
  out[1] = (int32_t)f2d_wg_joint(id);
}

F2D_EXPORT void f2d_create_distance_joint(
    uint32_t w, int32_t ai1, uint32_t awg, int32_t bi1, uint32_t bwg,
    float anchor_ax, float anchor_ay, float anchor_bx, float anchor_by,
    float length, int enable_spring, float hertz, float damping_ratio,
    int enable_limit, float min_length, float max_length, int enable_motor,
    float max_motor_force, float motor_speed, int collide_connected,
    int32_t* out) {
  b2DistanceJointDef def = b2DefaultDistanceJointDef();
  def.bodyIdA = f2d_body(ai1, awg);
  def.bodyIdB = f2d_body(bi1, bwg);
  def.localAnchorA = (b2Vec2){anchor_ax, anchor_ay};
  def.localAnchorB = (b2Vec2){anchor_bx, anchor_by};
  def.length = length;
  def.enableSpring = enable_spring;
  def.hertz = hertz;
  def.dampingRatio = damping_ratio;
  def.enableLimit = enable_limit;
  def.minLength = min_length;
  def.maxLength = max_length;
  def.enableMotor = enable_motor;
  def.maxMotorForce = max_motor_force;
  def.motorSpeed = motor_speed;
  def.collideConnected = collide_connected;
  f2d_out_joint(b2CreateDistanceJoint(f2d_world(w), &def), out);
}

F2D_EXPORT void f2d_create_filter_joint(uint32_t w, int32_t ai1, uint32_t awg,
                                        int32_t bi1, uint32_t bwg,
                                        int32_t* out) {
  b2FilterJointDef def = b2DefaultFilterJointDef();
  def.bodyIdA = f2d_body(ai1, awg);
  def.bodyIdB = f2d_body(bi1, bwg);
  f2d_out_joint(b2CreateFilterJoint(f2d_world(w), &def), out);
}

F2D_EXPORT void f2d_create_motor_joint(uint32_t w, int32_t ai1, uint32_t awg,
                                       int32_t bi1, uint32_t bwg,
                                       float offset_x, float offset_y,
                                       float angular_offset, float max_force,
                                       float max_torque,
                                       float correction_factor,
                                       int collide_connected, int32_t* out) {
  b2MotorJointDef def = b2DefaultMotorJointDef();
  def.bodyIdA = f2d_body(ai1, awg);
  def.bodyIdB = f2d_body(bi1, bwg);
  def.linearOffset = (b2Vec2){offset_x, offset_y};
  def.angularOffset = angular_offset;
  def.maxForce = max_force;
  def.maxTorque = max_torque;
  def.correctionFactor = correction_factor;
  def.collideConnected = collide_connected;
  f2d_out_joint(b2CreateMotorJoint(f2d_world(w), &def), out);
}

F2D_EXPORT void f2d_create_mouse_joint(uint32_t w, int32_t ai1, uint32_t awg,
                                       int32_t bi1, uint32_t bwg,
                                       float target_x, float target_y,
                                       float hertz, float damping_ratio,
                                       float max_force, int collide_connected,
                                       int32_t* out) {
  b2MouseJointDef def = b2DefaultMouseJointDef();
  def.bodyIdA = f2d_body(ai1, awg);
  def.bodyIdB = f2d_body(bi1, bwg);
  def.target = (b2Vec2){target_x, target_y};
  def.hertz = hertz;
  def.dampingRatio = damping_ratio;
  def.maxForce = max_force;
  def.collideConnected = collide_connected;
  f2d_out_joint(b2CreateMouseJoint(f2d_world(w), &def), out);
}

F2D_EXPORT void f2d_create_prismatic_joint(
    uint32_t w, int32_t ai1, uint32_t awg, int32_t bi1, uint32_t bwg,
    float anchor_ax, float anchor_ay, float anchor_bx, float anchor_by,
    float axis_x, float axis_y, float reference_angle,
    float target_translation, int enable_spring, float hertz,
    float damping_ratio, int enable_limit, float lower_translation,
    float upper_translation, int enable_motor, float max_motor_force,
    float motor_speed, int collide_connected, int32_t* out) {
  b2PrismaticJointDef def = b2DefaultPrismaticJointDef();
  def.bodyIdA = f2d_body(ai1, awg);
  def.bodyIdB = f2d_body(bi1, bwg);
  def.localAnchorA = (b2Vec2){anchor_ax, anchor_ay};
  def.localAnchorB = (b2Vec2){anchor_bx, anchor_by};
  def.localAxisA = (b2Vec2){axis_x, axis_y};
  def.referenceAngle = reference_angle;
  def.targetTranslation = target_translation;
  def.enableSpring = enable_spring;
  def.hertz = hertz;
  def.dampingRatio = damping_ratio;
  def.enableLimit = enable_limit;
  def.lowerTranslation = lower_translation;
  def.upperTranslation = upper_translation;
  def.enableMotor = enable_motor;
  def.maxMotorForce = max_motor_force;
  def.motorSpeed = motor_speed;
  def.collideConnected = collide_connected;
  f2d_out_joint(b2CreatePrismaticJoint(f2d_world(w), &def), out);
}

F2D_EXPORT void f2d_create_revolute_joint(
    uint32_t w, int32_t ai1, uint32_t awg, int32_t bi1, uint32_t bwg,
    float anchor_ax, float anchor_ay, float anchor_bx, float anchor_by,
    float reference_angle, float target_angle, int enable_spring, float hertz,
    float damping_ratio, int enable_limit, float lower_angle,
    float upper_angle, int enable_motor, float max_motor_torque,
    float motor_speed, float draw_size, int collide_connected, int32_t* out) {
  b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
  def.bodyIdA = f2d_body(ai1, awg);
  def.bodyIdB = f2d_body(bi1, bwg);
  def.localAnchorA = (b2Vec2){anchor_ax, anchor_ay};
  def.localAnchorB = (b2Vec2){anchor_bx, anchor_by};
  def.referenceAngle = reference_angle;
  def.targetAngle = target_angle;
  def.enableSpring = enable_spring;
  def.hertz = hertz;
  def.dampingRatio = damping_ratio;
  def.enableLimit = enable_limit;
  def.lowerAngle = lower_angle;
  def.upperAngle = upper_angle;
  def.enableMotor = enable_motor;
  def.maxMotorTorque = max_motor_torque;
  def.motorSpeed = motor_speed;
  def.drawSize = draw_size;
  def.collideConnected = collide_connected;
  f2d_out_joint(b2CreateRevoluteJoint(f2d_world(w), &def), out);
}

F2D_EXPORT void f2d_create_weld_joint(
    uint32_t w, int32_t ai1, uint32_t awg, int32_t bi1, uint32_t bwg,
    float anchor_ax, float anchor_ay, float anchor_bx, float anchor_by,
    float reference_angle, float linear_hertz, float angular_hertz,
    float linear_damping_ratio, float angular_damping_ratio,
    int collide_connected, int32_t* out) {
  b2WeldJointDef def = b2DefaultWeldJointDef();
  def.bodyIdA = f2d_body(ai1, awg);
  def.bodyIdB = f2d_body(bi1, bwg);
  def.localAnchorA = (b2Vec2){anchor_ax, anchor_ay};
  def.localAnchorB = (b2Vec2){anchor_bx, anchor_by};
  def.referenceAngle = reference_angle;
  def.linearHertz = linear_hertz;
  def.angularHertz = angular_hertz;
  def.linearDampingRatio = linear_damping_ratio;
  def.angularDampingRatio = angular_damping_ratio;
  def.collideConnected = collide_connected;
  f2d_out_joint(b2CreateWeldJoint(f2d_world(w), &def), out);
}

F2D_EXPORT void f2d_create_wheel_joint(
    uint32_t w, int32_t ai1, uint32_t awg, int32_t bi1, uint32_t bwg,
    float anchor_ax, float anchor_ay, float anchor_bx, float anchor_by,
    float axis_x, float axis_y, int enable_spring, float hertz,
    float damping_ratio, int enable_limit, float lower_translation,
    float upper_translation, int enable_motor, float max_motor_torque,
    float motor_speed, int collide_connected, int32_t* out) {
  b2WheelJointDef def = b2DefaultWheelJointDef();
  def.bodyIdA = f2d_body(ai1, awg);
  def.bodyIdB = f2d_body(bi1, bwg);
  def.localAnchorA = (b2Vec2){anchor_ax, anchor_ay};
  def.localAnchorB = (b2Vec2){anchor_bx, anchor_by};
  def.localAxisA = (b2Vec2){axis_x, axis_y};
  def.enableSpring = enable_spring;
  def.hertz = hertz;
  def.dampingRatio = damping_ratio;
  def.enableLimit = enable_limit;
  def.lowerTranslation = lower_translation;
  def.upperTranslation = upper_translation;
  def.enableMotor = enable_motor;
  def.maxMotorTorque = max_motor_torque;
  def.motorSpeed = motor_speed;
  def.collideConnected = collide_connected;
  f2d_out_joint(b2CreateWheelJoint(f2d_world(w), &def), out);
}

F2D_EXPORT void f2d_destroy_joint(int32_t i1, uint32_t wg) {
  b2DestroyJoint(f2d_joint(i1, wg));
}

F2D_EXPORT int f2d_joint_is_valid(int32_t i1, uint32_t wg) {
  return b2Joint_IsValid(f2d_joint(i1, wg));
}

F2D_EXPORT int f2d_joint_get_type(int32_t i1, uint32_t wg) {
  return (int)b2Joint_GetType(f2d_joint(i1, wg));
}

F2D_EXPORT void f2d_joint_get_body_a(int32_t i1, uint32_t wg, int32_t* out) {
  b2BodyId id = b2Joint_GetBodyA(f2d_joint(i1, wg));
  out[0] = id.index1;
  out[1] = (int32_t)f2d_wg_body(id);
}

F2D_EXPORT void f2d_joint_get_body_b(int32_t i1, uint32_t wg, int32_t* out) {
  b2BodyId id = b2Joint_GetBodyB(f2d_joint(i1, wg));
  out[0] = id.index1;
  out[1] = (int32_t)f2d_wg_body(id);
}

F2D_EXPORT void f2d_joint_get_local_anchor_a(int32_t i1, uint32_t wg,
                                             float* out) {
  b2Vec2 anchor = b2Joint_GetLocalAnchorA(f2d_joint(i1, wg));
  out[0] = anchor.x;
  out[1] = anchor.y;
}

F2D_EXPORT void f2d_joint_get_local_anchor_b(int32_t i1, uint32_t wg,
                                             float* out) {
  b2Vec2 anchor = b2Joint_GetLocalAnchorB(f2d_joint(i1, wg));
  out[0] = anchor.x;
  out[1] = anchor.y;
}

F2D_EXPORT int f2d_joint_get_collide_connected(int32_t i1, uint32_t wg) {
  return b2Joint_GetCollideConnected(f2d_joint(i1, wg));
}

F2D_EXPORT void f2d_joint_set_collide_connected(int32_t i1, uint32_t wg,
                                                int v) {
  b2Joint_SetCollideConnected(f2d_joint(i1, wg), v);
}

F2D_EXPORT void f2d_joint_wake_bodies(int32_t i1, uint32_t wg) {
  b2Joint_WakeBodies(f2d_joint(i1, wg));
}

F2D_EXPORT void f2d_joint_get_constraint_force(int32_t i1, uint32_t wg,
                                               float* out) {
  b2Vec2 force = b2Joint_GetConstraintForce(f2d_joint(i1, wg));
  out[0] = force.x;
  out[1] = force.y;
}

F2D_EXPORT float f2d_joint_get_constraint_torque(int32_t i1, uint32_t wg) {
  return b2Joint_GetConstraintTorque(f2d_joint(i1, wg));
}

#define F2D_JOINT_GET_FLOAT(name, fn) \
  F2D_EXPORT float name(int32_t i1, uint32_t wg) { return fn(f2d_joint(i1, wg)); }
#define F2D_JOINT_SET_FLOAT(name, fn) \
  F2D_EXPORT void name(int32_t i1, uint32_t wg, float v) { fn(f2d_joint(i1, wg), v); }
#define F2D_JOINT_GET_INT(name, fn) \
  F2D_EXPORT int name(int32_t i1, uint32_t wg) { return (int)fn(f2d_joint(i1, wg)); }
#define F2D_JOINT_SET_BOOL(name, fn) \
  F2D_EXPORT void name(int32_t i1, uint32_t wg, int v) { fn(f2d_joint(i1, wg), v); }
#define F2D_JOINT_SET_RANGE(name, fn) \
  F2D_EXPORT void name(int32_t i1, uint32_t wg, float a, float b) { \
    fn(f2d_joint(i1, wg), a, b); \
  }

F2D_JOINT_GET_FLOAT(f2d_distance_joint_get_length, b2DistanceJoint_GetLength)
F2D_JOINT_SET_FLOAT(f2d_distance_joint_set_length, b2DistanceJoint_SetLength)
F2D_JOINT_GET_FLOAT(f2d_distance_joint_get_current_length,
                    b2DistanceJoint_GetCurrentLength)
F2D_JOINT_GET_INT(f2d_distance_joint_is_spring_enabled,
                  b2DistanceJoint_IsSpringEnabled)
F2D_JOINT_SET_BOOL(f2d_distance_joint_enable_spring,
                   b2DistanceJoint_EnableSpring)
F2D_JOINT_GET_FLOAT(f2d_distance_joint_get_spring_hertz,
                    b2DistanceJoint_GetSpringHertz)
F2D_JOINT_SET_FLOAT(f2d_distance_joint_set_spring_hertz,
                    b2DistanceJoint_SetSpringHertz)
F2D_JOINT_GET_FLOAT(f2d_distance_joint_get_spring_damping_ratio,
                    b2DistanceJoint_GetSpringDampingRatio)
F2D_JOINT_SET_FLOAT(f2d_distance_joint_set_spring_damping_ratio,
                    b2DistanceJoint_SetSpringDampingRatio)
F2D_JOINT_GET_INT(f2d_distance_joint_is_limit_enabled,
                  b2DistanceJoint_IsLimitEnabled)
F2D_JOINT_SET_BOOL(f2d_distance_joint_enable_limit,
                   b2DistanceJoint_EnableLimit)
F2D_JOINT_GET_FLOAT(f2d_distance_joint_get_min_length,
                    b2DistanceJoint_GetMinLength)
F2D_JOINT_GET_FLOAT(f2d_distance_joint_get_max_length,
                    b2DistanceJoint_GetMaxLength)
F2D_JOINT_SET_RANGE(f2d_distance_joint_set_length_range,
                    b2DistanceJoint_SetLengthRange)
F2D_JOINT_GET_INT(f2d_distance_joint_is_motor_enabled,
                  b2DistanceJoint_IsMotorEnabled)
F2D_JOINT_SET_BOOL(f2d_distance_joint_enable_motor,
                   b2DistanceJoint_EnableMotor)
F2D_JOINT_GET_FLOAT(f2d_distance_joint_get_motor_speed,
                    b2DistanceJoint_GetMotorSpeed)
F2D_JOINT_SET_FLOAT(f2d_distance_joint_set_motor_speed,
                    b2DistanceJoint_SetMotorSpeed)
F2D_JOINT_GET_FLOAT(f2d_distance_joint_get_max_motor_force,
                    b2DistanceJoint_GetMaxMotorForce)
F2D_JOINT_SET_FLOAT(f2d_distance_joint_set_max_motor_force,
                    b2DistanceJoint_SetMaxMotorForce)
F2D_JOINT_GET_FLOAT(f2d_distance_joint_get_motor_force,
                    b2DistanceJoint_GetMotorForce)

F2D_EXPORT void f2d_motor_joint_get_linear_offset(int32_t i1, uint32_t wg,
                                                  float* out) {
  b2Vec2 offset = b2MotorJoint_GetLinearOffset(f2d_joint(i1, wg));
  out[0] = offset.x;
  out[1] = offset.y;
}

F2D_EXPORT void f2d_motor_joint_set_linear_offset(int32_t i1, uint32_t wg,
                                                  float x, float y) {
  b2MotorJoint_SetLinearOffset(f2d_joint(i1, wg), (b2Vec2){x, y});
}

F2D_JOINT_GET_FLOAT(f2d_motor_joint_get_angular_offset,
                    b2MotorJoint_GetAngularOffset)
F2D_JOINT_SET_FLOAT(f2d_motor_joint_set_angular_offset,
                    b2MotorJoint_SetAngularOffset)
F2D_JOINT_GET_FLOAT(f2d_motor_joint_get_max_force, b2MotorJoint_GetMaxForce)
F2D_JOINT_SET_FLOAT(f2d_motor_joint_set_max_force, b2MotorJoint_SetMaxForce)
F2D_JOINT_GET_FLOAT(f2d_motor_joint_get_max_torque, b2MotorJoint_GetMaxTorque)
F2D_JOINT_SET_FLOAT(f2d_motor_joint_set_max_torque, b2MotorJoint_SetMaxTorque)
F2D_JOINT_GET_FLOAT(f2d_motor_joint_get_correction_factor,
                    b2MotorJoint_GetCorrectionFactor)
F2D_JOINT_SET_FLOAT(f2d_motor_joint_set_correction_factor,
                    b2MotorJoint_SetCorrectionFactor)

F2D_EXPORT void f2d_mouse_joint_get_target(int32_t i1, uint32_t wg,
                                           float* out) {
  b2Vec2 target = b2MouseJoint_GetTarget(f2d_joint(i1, wg));
  out[0] = target.x;
  out[1] = target.y;
}

F2D_EXPORT void f2d_mouse_joint_set_target(int32_t i1, uint32_t wg, float x,
                                           float y) {
  b2MouseJoint_SetTarget(f2d_joint(i1, wg), (b2Vec2){x, y});
}

F2D_JOINT_GET_FLOAT(f2d_mouse_joint_get_spring_hertz,
                    b2MouseJoint_GetSpringHertz)
F2D_JOINT_SET_FLOAT(f2d_mouse_joint_set_spring_hertz,
                    b2MouseJoint_SetSpringHertz)
F2D_JOINT_GET_FLOAT(f2d_mouse_joint_get_spring_damping_ratio,
                    b2MouseJoint_GetSpringDampingRatio)
F2D_JOINT_SET_FLOAT(f2d_mouse_joint_set_spring_damping_ratio,
                    b2MouseJoint_SetSpringDampingRatio)
F2D_JOINT_GET_FLOAT(f2d_mouse_joint_get_max_force, b2MouseJoint_GetMaxForce)
F2D_JOINT_SET_FLOAT(f2d_mouse_joint_set_max_force, b2MouseJoint_SetMaxForce)

F2D_JOINT_GET_INT(f2d_prismatic_joint_is_spring_enabled,
                  b2PrismaticJoint_IsSpringEnabled)
F2D_JOINT_SET_BOOL(f2d_prismatic_joint_enable_spring,
                   b2PrismaticJoint_EnableSpring)
F2D_JOINT_GET_FLOAT(f2d_prismatic_joint_get_spring_hertz,
                    b2PrismaticJoint_GetSpringHertz)
F2D_JOINT_SET_FLOAT(f2d_prismatic_joint_set_spring_hertz,
                    b2PrismaticJoint_SetSpringHertz)
F2D_JOINT_GET_FLOAT(f2d_prismatic_joint_get_spring_damping_ratio,
                    b2PrismaticJoint_GetSpringDampingRatio)
F2D_JOINT_SET_FLOAT(f2d_prismatic_joint_set_spring_damping_ratio,
                    b2PrismaticJoint_SetSpringDampingRatio)
F2D_JOINT_GET_FLOAT(f2d_prismatic_joint_get_target_translation,
                    b2PrismaticJoint_GetTargetTranslation)
F2D_JOINT_SET_FLOAT(f2d_prismatic_joint_set_target_translation,
                    b2PrismaticJoint_SetTargetTranslation)
F2D_JOINT_GET_INT(f2d_prismatic_joint_is_limit_enabled,
                  b2PrismaticJoint_IsLimitEnabled)
F2D_JOINT_SET_BOOL(f2d_prismatic_joint_enable_limit,
                   b2PrismaticJoint_EnableLimit)
F2D_JOINT_GET_FLOAT(f2d_prismatic_joint_get_lower_limit,
                    b2PrismaticJoint_GetLowerLimit)
F2D_JOINT_GET_FLOAT(f2d_prismatic_joint_get_upper_limit,
                    b2PrismaticJoint_GetUpperLimit)
F2D_JOINT_SET_RANGE(f2d_prismatic_joint_set_limits, b2PrismaticJoint_SetLimits)
F2D_JOINT_GET_INT(f2d_prismatic_joint_is_motor_enabled,
                  b2PrismaticJoint_IsMotorEnabled)
F2D_JOINT_SET_BOOL(f2d_prismatic_joint_enable_motor,
                   b2PrismaticJoint_EnableMotor)
F2D_JOINT_GET_FLOAT(f2d_prismatic_joint_get_motor_speed,
                    b2PrismaticJoint_GetMotorSpeed)
F2D_JOINT_SET_FLOAT(f2d_prismatic_joint_set_motor_speed,
                    b2PrismaticJoint_SetMotorSpeed)
F2D_JOINT_GET_FLOAT(f2d_prismatic_joint_get_max_motor_force,
                    b2PrismaticJoint_GetMaxMotorForce)
F2D_JOINT_SET_FLOAT(f2d_prismatic_joint_set_max_motor_force,
                    b2PrismaticJoint_SetMaxMotorForce)
F2D_JOINT_GET_FLOAT(f2d_prismatic_joint_get_motor_force,
                    b2PrismaticJoint_GetMotorForce)
F2D_JOINT_GET_FLOAT(f2d_prismatic_joint_get_translation,
                    b2PrismaticJoint_GetTranslation)
F2D_JOINT_GET_FLOAT(f2d_prismatic_joint_get_speed, b2PrismaticJoint_GetSpeed)

F2D_JOINT_GET_INT(f2d_revolute_joint_is_spring_enabled,
                  b2RevoluteJoint_IsSpringEnabled)
F2D_JOINT_SET_BOOL(f2d_revolute_joint_enable_spring,
                   b2RevoluteJoint_EnableSpring)
F2D_JOINT_GET_FLOAT(f2d_revolute_joint_get_spring_hertz,
                    b2RevoluteJoint_GetSpringHertz)
F2D_JOINT_SET_FLOAT(f2d_revolute_joint_set_spring_hertz,
                    b2RevoluteJoint_SetSpringHertz)
F2D_JOINT_GET_FLOAT(f2d_revolute_joint_get_spring_damping_ratio,
                    b2RevoluteJoint_GetSpringDampingRatio)
F2D_JOINT_SET_FLOAT(f2d_revolute_joint_set_spring_damping_ratio,
                    b2RevoluteJoint_SetSpringDampingRatio)
F2D_JOINT_GET_FLOAT(f2d_revolute_joint_get_target_angle,
                    b2RevoluteJoint_GetTargetAngle)
F2D_JOINT_SET_FLOAT(f2d_revolute_joint_set_target_angle,
                    b2RevoluteJoint_SetTargetAngle)
F2D_JOINT_GET_FLOAT(f2d_revolute_joint_get_angle, b2RevoluteJoint_GetAngle)
F2D_JOINT_GET_INT(f2d_revolute_joint_is_limit_enabled,
                  b2RevoluteJoint_IsLimitEnabled)
F2D_JOINT_SET_BOOL(f2d_revolute_joint_enable_limit,
                   b2RevoluteJoint_EnableLimit)
F2D_JOINT_GET_FLOAT(f2d_revolute_joint_get_lower_limit,
                    b2RevoluteJoint_GetLowerLimit)
F2D_JOINT_GET_FLOAT(f2d_revolute_joint_get_upper_limit,
                    b2RevoluteJoint_GetUpperLimit)
F2D_JOINT_SET_RANGE(f2d_revolute_joint_set_limits, b2RevoluteJoint_SetLimits)
F2D_JOINT_GET_INT(f2d_revolute_joint_is_motor_enabled,
                  b2RevoluteJoint_IsMotorEnabled)
F2D_JOINT_SET_BOOL(f2d_revolute_joint_enable_motor,
                   b2RevoluteJoint_EnableMotor)
F2D_JOINT_GET_FLOAT(f2d_revolute_joint_get_motor_speed,
                    b2RevoluteJoint_GetMotorSpeed)
F2D_JOINT_SET_FLOAT(f2d_revolute_joint_set_motor_speed,
                    b2RevoluteJoint_SetMotorSpeed)
F2D_JOINT_GET_FLOAT(f2d_revolute_joint_get_max_motor_torque,
                    b2RevoluteJoint_GetMaxMotorTorque)
F2D_JOINT_SET_FLOAT(f2d_revolute_joint_set_max_motor_torque,
                    b2RevoluteJoint_SetMaxMotorTorque)
F2D_JOINT_GET_FLOAT(f2d_revolute_joint_get_motor_torque,
                    b2RevoluteJoint_GetMotorTorque)

F2D_JOINT_GET_FLOAT(f2d_weld_joint_get_linear_hertz, b2WeldJoint_GetLinearHertz)
F2D_JOINT_SET_FLOAT(f2d_weld_joint_set_linear_hertz, b2WeldJoint_SetLinearHertz)
F2D_JOINT_GET_FLOAT(f2d_weld_joint_get_angular_hertz,
                    b2WeldJoint_GetAngularHertz)
F2D_JOINT_SET_FLOAT(f2d_weld_joint_set_angular_hertz,
                    b2WeldJoint_SetAngularHertz)
F2D_JOINT_GET_FLOAT(f2d_weld_joint_get_linear_damping_ratio,
                    b2WeldJoint_GetLinearDampingRatio)
F2D_JOINT_SET_FLOAT(f2d_weld_joint_set_linear_damping_ratio,
                    b2WeldJoint_SetLinearDampingRatio)
F2D_JOINT_GET_FLOAT(f2d_weld_joint_get_angular_damping_ratio,
                    b2WeldJoint_GetAngularDampingRatio)
F2D_JOINT_SET_FLOAT(f2d_weld_joint_set_angular_damping_ratio,
                    b2WeldJoint_SetAngularDampingRatio)

F2D_JOINT_GET_INT(f2d_wheel_joint_is_spring_enabled,
                  b2WheelJoint_IsSpringEnabled)
F2D_JOINT_SET_BOOL(f2d_wheel_joint_enable_spring, b2WheelJoint_EnableSpring)
F2D_JOINT_GET_FLOAT(f2d_wheel_joint_get_spring_hertz,
                    b2WheelJoint_GetSpringHertz)
F2D_JOINT_SET_FLOAT(f2d_wheel_joint_set_spring_hertz,
                    b2WheelJoint_SetSpringHertz)
F2D_JOINT_GET_FLOAT(f2d_wheel_joint_get_spring_damping_ratio,
                    b2WheelJoint_GetSpringDampingRatio)
F2D_JOINT_SET_FLOAT(f2d_wheel_joint_set_spring_damping_ratio,
                    b2WheelJoint_SetSpringDampingRatio)
F2D_JOINT_GET_INT(f2d_wheel_joint_is_limit_enabled,
                  b2WheelJoint_IsLimitEnabled)
F2D_JOINT_SET_BOOL(f2d_wheel_joint_enable_limit, b2WheelJoint_EnableLimit)
F2D_JOINT_GET_FLOAT(f2d_wheel_joint_get_lower_limit,
                    b2WheelJoint_GetLowerLimit)
F2D_JOINT_GET_FLOAT(f2d_wheel_joint_get_upper_limit,
                    b2WheelJoint_GetUpperLimit)
F2D_JOINT_SET_RANGE(f2d_wheel_joint_set_limits, b2WheelJoint_SetLimits)
F2D_JOINT_GET_INT(f2d_wheel_joint_is_motor_enabled,
                  b2WheelJoint_IsMotorEnabled)
F2D_JOINT_SET_BOOL(f2d_wheel_joint_enable_motor, b2WheelJoint_EnableMotor)
F2D_JOINT_GET_FLOAT(f2d_wheel_joint_get_motor_speed,
                    b2WheelJoint_GetMotorSpeed)
F2D_JOINT_SET_FLOAT(f2d_wheel_joint_set_motor_speed,
                    b2WheelJoint_SetMotorSpeed)
F2D_JOINT_GET_FLOAT(f2d_wheel_joint_get_max_motor_torque,
                    b2WheelJoint_GetMaxMotorTorque)
F2D_JOINT_SET_FLOAT(f2d_wheel_joint_set_max_motor_torque,
                    b2WheelJoint_SetMaxMotorTorque)
F2D_JOINT_GET_FLOAT(f2d_wheel_joint_get_motor_torque,
                    b2WheelJoint_GetMotorTorque)

// Events. Counts are queried first; the fill functions then write float64
// records into a caller-provided buffer and return the record count.
//
// Strides (in float64 elements):
// - begin contact: 13 [aI1, aWg, bI1, bWg, nx, ny, pointCount,
//   p0x, p0y, sep0, p1x, p1y, sep1]
// - end contact: 4 [aI1, aWg, bI1, bWg]
// - hit contact: 9 [aI1, aWg, bI1, bWg, px, py, nx, ny, approachSpeed]
// - sensor: 4 [sensorI1, sensorWg, visitorI1, visitorWg]
// - body move: 7 [i1, wg, x, y, qc, qs, fellAsleep]

F2D_EXPORT void f2d_world_get_contact_event_counts(uint32_t w, int32_t* out) {
  b2ContactEvents events = b2World_GetContactEvents(f2d_world(w));
  out[0] = events.beginCount;
  out[1] = events.endCount;
  out[2] = events.hitCount;
}

F2D_EXPORT int f2d_world_get_contact_begin_events(uint32_t w, double* out) {
  b2ContactEvents events = b2World_GetContactEvents(f2d_world(w));
  for (int i = 0; i < events.beginCount; ++i) {
    const b2ContactBeginTouchEvent* event = events.beginEvents + i;
    double* record = out + i * 13;
    record[0] = event->shapeIdA.index1;
    record[1] = f2d_wg_shape(event->shapeIdA);
    record[2] = event->shapeIdB.index1;
    record[3] = f2d_wg_shape(event->shapeIdB);
    record[4] = event->manifold.normal.x;
    record[5] = event->manifold.normal.y;
    record[6] = event->manifold.pointCount;
    for (int p = 0; p < 2; ++p) {
      record[7 + 3 * p] = event->manifold.points[p].point.x;
      record[8 + 3 * p] = event->manifold.points[p].point.y;
      record[9 + 3 * p] = event->manifold.points[p].separation;
    }
  }
  return events.beginCount;
}

F2D_EXPORT int f2d_world_get_contact_end_events(uint32_t w, double* out) {
  b2ContactEvents events = b2World_GetContactEvents(f2d_world(w));
  for (int i = 0; i < events.endCount; ++i) {
    const b2ContactEndTouchEvent* event = events.endEvents + i;
    double* record = out + i * 4;
    record[0] = event->shapeIdA.index1;
    record[1] = f2d_wg_shape(event->shapeIdA);
    record[2] = event->shapeIdB.index1;
    record[3] = f2d_wg_shape(event->shapeIdB);
  }
  return events.endCount;
}

F2D_EXPORT int f2d_world_get_contact_hit_events(uint32_t w, double* out) {
  b2ContactEvents events = b2World_GetContactEvents(f2d_world(w));
  for (int i = 0; i < events.hitCount; ++i) {
    const b2ContactHitEvent* event = events.hitEvents + i;
    double* record = out + i * 9;
    record[0] = event->shapeIdA.index1;
    record[1] = f2d_wg_shape(event->shapeIdA);
    record[2] = event->shapeIdB.index1;
    record[3] = f2d_wg_shape(event->shapeIdB);
    record[4] = event->point.x;
    record[5] = event->point.y;
    record[6] = event->normal.x;
    record[7] = event->normal.y;
    record[8] = event->approachSpeed;
  }
  return events.hitCount;
}

F2D_EXPORT void f2d_world_get_sensor_event_counts(uint32_t w, int32_t* out) {
  b2SensorEvents events = b2World_GetSensorEvents(f2d_world(w));
  out[0] = events.beginCount;
  out[1] = events.endCount;
}

F2D_EXPORT int f2d_world_get_sensor_begin_events(uint32_t w, double* out) {
  b2SensorEvents events = b2World_GetSensorEvents(f2d_world(w));
  for (int i = 0; i < events.beginCount; ++i) {
    const b2SensorBeginTouchEvent* event = events.beginEvents + i;
    double* record = out + i * 4;
    record[0] = event->sensorShapeId.index1;
    record[1] = f2d_wg_shape(event->sensorShapeId);
    record[2] = event->visitorShapeId.index1;
    record[3] = f2d_wg_shape(event->visitorShapeId);
  }
  return events.beginCount;
}

F2D_EXPORT int f2d_world_get_sensor_end_events(uint32_t w, double* out) {
  b2SensorEvents events = b2World_GetSensorEvents(f2d_world(w));
  for (int i = 0; i < events.endCount; ++i) {
    const b2SensorEndTouchEvent* event = events.endEvents + i;
    double* record = out + i * 4;
    record[0] = event->sensorShapeId.index1;
    record[1] = f2d_wg_shape(event->sensorShapeId);
    record[2] = event->visitorShapeId.index1;
    record[3] = f2d_wg_shape(event->visitorShapeId);
  }
  return events.endCount;
}

F2D_EXPORT int f2d_world_get_body_event_count(uint32_t w) {
  return b2World_GetBodyEvents(f2d_world(w)).moveCount;
}

F2D_EXPORT int f2d_world_get_body_events(uint32_t w, double* out) {
  b2BodyEvents events = b2World_GetBodyEvents(f2d_world(w));
  for (int i = 0; i < events.moveCount; ++i) {
    const b2BodyMoveEvent* event = events.moveEvents + i;
    double* record = out + i * 7;
    record[0] = event->bodyId.index1;
    record[1] = f2d_wg_body(event->bodyId);
    record[2] = event->transform.p.x;
    record[3] = event->transform.p.y;
    record[4] = event->transform.q.c;
    record[5] = event->transform.q.s;
    record[6] = event->fellAsleep;
  }
  return events.moveCount;
}

// Queries.

// Returns whether something was hit; the hit is written as
// [i1, wg, px, py, nx, ny, fraction] float32s with ids stored via the
// int32 view of the same buffer slots 0 and 1.
F2D_EXPORT int f2d_world_cast_ray_closest(uint32_t w, float ox, float oy,
                                          float tx, float ty,
                                          uint32_t category_lo,
                                          uint32_t category_hi,
                                          uint32_t mask_lo, uint32_t mask_hi,
                                          double* out) {
  b2RayResult result = b2World_CastRayClosest(
      f2d_world(w), (b2Vec2){ox, oy}, (b2Vec2){tx, ty},
      f2d_query_filter(category_lo, category_hi, mask_lo, mask_hi));
  if (!result.hit) {
    return 0;
  }
  out[0] = result.shapeId.index1;
  out[1] = f2d_wg_shape(result.shapeId);
  out[2] = result.point.x;
  out[3] = result.point.y;
  out[4] = result.normal.x;
  out[5] = result.normal.y;
  out[6] = result.fraction;
  return 1;
}

static float f2d_cast_ray_trampoline(b2ShapeId shape, b2Vec2 point,
                                     b2Vec2 normal, float fraction,
                                     void* context) {
  (void)context;
  return f2d_host_cast_ray(shape.index1, f2d_wg_shape(shape), point.x, point.y,
                           normal.x, normal.y, fraction);
}

F2D_EXPORT void f2d_world_cast_ray(uint32_t w, float ox, float oy, float tx,
                                   float ty, uint32_t category_lo,
                                   uint32_t category_hi, uint32_t mask_lo,
                                   uint32_t mask_hi) {
  b2World_CastRay(f2d_world(w), (b2Vec2){ox, oy}, (b2Vec2){tx, ty},
                  f2d_query_filter(category_lo, category_hi, mask_lo, mask_hi),
                  f2d_cast_ray_trampoline, NULL);
}

static bool f2d_overlap_trampoline(b2ShapeId shape, void* context) {
  (void)context;
  return f2d_host_overlap(shape.index1, f2d_wg_shape(shape));
}

F2D_EXPORT void f2d_world_overlap_aabb(uint32_t w, float lx, float ly,
                                       float ux, float uy,
                                       uint32_t category_lo,
                                       uint32_t category_hi, uint32_t mask_lo,
                                       uint32_t mask_hi) {
  b2AABB aabb = {{lx, ly}, {ux, uy}};
  b2World_OverlapAABB(
      f2d_world(w), aabb,
      f2d_query_filter(category_lo, category_hi, mask_lo, mask_hi),
      f2d_overlap_trampoline, NULL);
}

// Simulation callbacks.

static bool f2d_custom_filter_trampoline(b2ShapeId a, b2ShapeId b,
                                         void* context) {
  (void)context;
  return f2d_host_custom_filter(a.index1, f2d_wg_shape(a), b.index1,
                                f2d_wg_shape(b));
}

F2D_EXPORT void f2d_world_set_custom_filter(uint32_t w, int enabled) {
  b2World_SetCustomFilterCallback(
      f2d_world(w), enabled ? f2d_custom_filter_trampoline : NULL, NULL);
}

static bool f2d_pre_solve_trampoline(b2ShapeId a, b2ShapeId b,
                                     b2Manifold* manifold, void* context) {
  (void)context;
  return f2d_host_pre_solve(a.index1, f2d_wg_shape(a), b.index1,
                            f2d_wg_shape(b), manifold->normal.x,
                            manifold->normal.y);
}

F2D_EXPORT void f2d_world_set_pre_solve(uint32_t w, int enabled) {
  b2World_SetPreSolveCallback(f2d_world(w),
                              enabled ? f2d_pre_solve_trampoline : NULL, NULL);
}

// Debug drawing.

static void f2d_draw_polygon(const b2Vec2* vertices, int count,
                             b2HexColor color, void* context) {
  (void)context;
  f2d_host_draw_polygon((const float*)vertices, count, (uint32_t)color);
}

static void f2d_draw_solid_polygon(b2Transform transform,
                                   const b2Vec2* vertices, int count,
                                   float radius, b2HexColor color,
                                   void* context) {
  (void)context;
  f2d_host_draw_solid_polygon(transform.p.x, transform.p.y, transform.q.c,
                              transform.q.s, (const float*)vertices, count,
                              radius, (uint32_t)color);
}

static void f2d_draw_circle(b2Vec2 center, float radius, b2HexColor color,
                            void* context) {
  (void)context;
  f2d_host_draw_circle(center.x, center.y, radius, (uint32_t)color);
}

static void f2d_draw_solid_circle(b2Transform transform, float radius,
                                  b2HexColor color, void* context) {
  (void)context;
  f2d_host_draw_solid_circle(transform.p.x, transform.p.y, transform.q.c,
                             transform.q.s, radius, (uint32_t)color);
}

static void f2d_draw_solid_capsule(b2Vec2 p1, b2Vec2 p2, float radius,
                                   b2HexColor color, void* context) {
  (void)context;
  f2d_host_draw_solid_capsule(p1.x, p1.y, p2.x, p2.y, radius,
                              (uint32_t)color);
}

static void f2d_draw_segment(b2Vec2 p1, b2Vec2 p2, b2HexColor color,
                             void* context) {
  (void)context;
  f2d_host_draw_segment(p1.x, p1.y, p2.x, p2.y, (uint32_t)color);
}

static void f2d_draw_transform(b2Transform transform, void* context) {
  (void)context;
  f2d_host_draw_transform(transform.p.x, transform.p.y, transform.q.c,
                          transform.q.s);
}

static void f2d_draw_point(b2Vec2 point, float size, b2HexColor color,
                           void* context) {
  (void)context;
  f2d_host_draw_point(point.x, point.y, size, (uint32_t)color);
}

static void f2d_draw_string(b2Vec2 point, const char* text, b2HexColor color,
                            void* context) {
  (void)context;
  f2d_host_draw_string(point.x, point.y, text, (uint32_t)color);
}

F2D_EXPORT void f2d_world_draw(uint32_t w, int use_bounds, float bounds_lx,
                               float bounds_ly, float bounds_ux,
                               float bounds_uy, int draw_shapes,
                               int draw_joints, int draw_joint_extras,
                               int draw_bounds, int draw_mass,
                               int draw_body_names, int draw_contacts,
                               int draw_graph_colors,
                               int draw_contact_normals,
                               int draw_contact_impulses,
                               int draw_contact_features,
                               int draw_friction_impulses, int draw_islands) {
  b2DebugDraw draw = b2DefaultDebugDraw();
  draw.DrawPolygonFcn = f2d_draw_polygon;
  draw.DrawSolidPolygonFcn = f2d_draw_solid_polygon;
  draw.DrawCircleFcn = f2d_draw_circle;
  draw.DrawSolidCircleFcn = f2d_draw_solid_circle;
  draw.DrawSolidCapsuleFcn = f2d_draw_solid_capsule;
  draw.DrawSegmentFcn = f2d_draw_segment;
  draw.DrawTransformFcn = f2d_draw_transform;
  draw.DrawPointFcn = f2d_draw_point;
  draw.DrawStringFcn = f2d_draw_string;
  draw.useDrawingBounds = use_bounds;
  draw.drawingBounds = (b2AABB){{bounds_lx, bounds_ly}, {bounds_ux, bounds_uy}};
  draw.drawShapes = draw_shapes;
  draw.drawJoints = draw_joints;
  draw.drawJointExtras = draw_joint_extras;
  draw.drawBounds = draw_bounds;
  draw.drawMass = draw_mass;
  draw.drawBodyNames = draw_body_names;
  draw.drawContacts = draw_contacts;
  draw.drawGraphColors = draw_graph_colors;
  draw.drawContactNormals = draw_contact_normals;
  draw.drawContactImpulses = draw_contact_impulses;
  draw.drawContactFeatures = draw_contact_features;
  draw.drawFrictionImpulses = draw_friction_impulses;
  draw.drawIslands = draw_islands;
  b2World_Draw(f2d_world(w), &draw);
}
