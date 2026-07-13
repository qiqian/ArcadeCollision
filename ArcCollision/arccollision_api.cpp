// Library-wide C entry points (ABI version + last-error). The rest of the C ABI is
// implemented alongside its subsystem: collide.cpp, sweep.cpp, distance.cpp,
// shapes.cpp, and world.cpp.
#include "internal.h"

#include <cstddef>

// Lock the arc_shape tagged-union layout: the managed wrapper's NativeShape
// mirrors these exact offsets by hand, so a padding surprise on any compiler
// must fail the build here instead of silently corrupting marshaling.
static_assert(sizeof(arc_shape) == 24, "arc_shape must be 24 bytes (pack 4)");
static_assert(sizeof(arc_status) == sizeof(int32_t), "arc_status must be 32 bits");
static_assert(sizeof(arc_handle) == 8, "arc_handle must be 8 bytes");
static_assert(sizeof(arc_candidate_pair) == 16, "arc_candidate_pair must be 16 bytes");
static_assert(sizeof(arc_sweep_hit) == 24, "arc_sweep_hit must be 24 bytes");
static_assert(sizeof(arc_world_cast_hit) == 32, "arc_world_cast_hit must be 32 bytes");
static_assert(offsetof(arc_world_cast_hit, hit) == 8, "cast hit payload at 8");
static_assert(offsetof(arc_shape, kind) == 0, "kind at 0");
static_assert(offsetof(arc_shape, circle) == 4, "primitive union at 4");
static_assert(offsetof(arc_shape, polygon_rotation) == 4, "polygon_rotation at 4");
static_assert(offsetof(arc_shape, polygon_translation) == 8, "polygon_translation at 8");
static_assert(offsetof(arc_shape, polygon) == 16, "polygon pointer at 16");

extern "C" {

// Bump when the ABI changes; the managed wrapper checks this at world creation.
uint32_t ARC_CALL arc_get_abi_version(void) {
    return ARC_ABI_VERSION;
}

const char* ARC_CALL arc_get_last_error(void) {
    return arc::error_text.c_str();
}

} // extern "C"
