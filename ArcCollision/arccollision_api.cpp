// Library-wide C entry points (ABI version + last-error). The rest of the C ABI is
// implemented alongside its subsystem: collide.cpp, sweep.cpp, distance.cpp,
// shapes.cpp, and world.cpp.
#include "internal.h"

extern "C" {

// Bump when the ABI changes; the managed wrapper checks this at world creation.
uint32_t ARC_CALL arc_get_abi_version(void) {
    return ARC_ABI_VERSION;
}

const char* ARC_CALL arc_get_last_error(void) {
    return arc::error_text.c_str();
}

} // extern "C"
