#include "internal.h"

extern "C" {

uint32_t ARC_CALL arc_get_abi_version(void) {
    return ARC_ABI_VERSION;
}

const char* ARC_CALL arc_get_last_error(void) {
    return arc::error_text.c_str();
}

} // extern "C"
