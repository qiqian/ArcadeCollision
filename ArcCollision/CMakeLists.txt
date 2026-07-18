cmake_minimum_required(VERSION 3.20)
project(ArcCollision VERSION 1.0.0 LANGUAGES C CXX)

option(ARC_BUILD_SHARED_LIBRARY "Build ArcCollision as a shared library" ON)
option(ARC_BUILD_BENCHMARKS "Build ArcCollision internal benchmarks" OFF)

if(IOS)
    # iOS P/Invoke uses __Internal when this static library is linked into the app.
    set(ARC_LIBRARY_KIND STATIC)
elseif(ARC_BUILD_SHARED_LIBRARY)
    set(ARC_LIBRARY_KIND SHARED)
else()
    set(ARC_LIBRARY_KIND STATIC)
endif()

set(ARC_SOURCES
    arccollision_api.cpp
    fixed.cpp
    shapes.cpp
    distance.cpp
    collide.cpp
    sweep.cpp
    dynamic_aabb_tree.cpp
    static_bvh.cpp
    spatial_hash.cpp
    world.cpp)

add_library(ArcCollision ${ARC_LIBRARY_KIND} ${ARC_SOURCES})
target_compile_features(ArcCollision PUBLIC cxx_std_17)
target_include_directories(ArcCollision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# PC builds require SSE4.2. ARM64 always has NEON, and the private SIMD header
# rejects ARMv7/other unsupported architectures. Apple universal builds need the
# x86 feature flag scoped to their x86_64 slice only.
function(arc_enable_integer_simd target)
    if(NOT MSVC AND CMAKE_OSX_ARCHITECTURES MATCHES "x86_64"
            AND CMAKE_OSX_ARCHITECTURES MATCHES "arm64")
        target_compile_options(${target} PRIVATE
            $<$<COMPILE_LANGUAGE:CXX>:-Xarch_x86_64>
            $<$<COMPILE_LANGUAGE:CXX>:-msse4.2>)
    elseif(NOT MSVC AND CMAKE_SYSTEM_PROCESSOR MATCHES
            "^(x86_64|amd64|AMD64|i[3-6]86)$")
        target_compile_options(${target} PRIVATE
            $<$<COMPILE_LANGUAGE:CXX>:-msse4.2>)
    elseif(MSVC AND CMAKE_CXX_COMPILER_ID MATCHES "Clang"
            AND CMAKE_GENERATOR_PLATFORM MATCHES "^(x64|Win32)$")
        target_compile_options(${target} PRIVATE /clang:-msse4.2)
    endif()
endfunction()

arc_enable_integer_simd(ArcCollision)
if(ARC_LIBRARY_KIND STREQUAL "SHARED")
    target_compile_definitions(ArcCollision PRIVATE ARC_BUILD_SHARED=1)
else()
    target_compile_definitions(ArcCollision PRIVATE ARC_BUILD_STATIC=1)
endif()
set_target_properties(ArcCollision PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    POSITION_INDEPENDENT_CODE YES
    OUTPUT_NAME arccollision)

if(MSVC)
    target_compile_options(ArcCollision PRIVATE /W4 /permissive- /Zc:__cplusplus)
else()
    target_compile_options(ArcCollision PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS ArcCollision
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
install(FILES arccollision.h DESTINATION include)

include(CTest)
if(ARC_BUILD_BENCHMARKS AND NOT CMAKE_CROSSCOMPILING)
    add_executable(arc_integer_bounds_benchmark
        benchmarks/integer_bounds_benchmark.cpp
        fixed.cpp)
    target_include_directories(arc_integer_bounds_benchmark PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_features(arc_integer_bounds_benchmark PRIVATE cxx_std_17)
    arc_enable_integer_simd(arc_integer_bounds_benchmark)
endif()

if(BUILD_TESTING AND NOT CMAKE_CROSSCOMPILING)
    add_executable(arc_c_api_smoke tests/c_api_smoke.c)
    target_link_libraries(arc_c_api_smoke PRIVATE ArcCollision)
    target_include_directories(arc_c_api_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    add_test(NAME arc_c_api_smoke COMMAND arc_c_api_smoke)

    # Compile the fixed core directly into this private test executable. Its C++
    # symbols intentionally remain hidden from the shared library's C-only ABI.
    add_executable(arc_simd_integer_tests
        tests/simd_integer_tests.cpp
        fixed.cpp)
    target_link_libraries(arc_simd_integer_tests PRIVATE ArcCollision)
    target_include_directories(arc_simd_integer_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_features(arc_simd_integer_tests PRIVATE cxx_std_17)
    arc_enable_integer_simd(arc_simd_integer_tests)
    add_test(NAME arc_simd_integer_tests COMMAND arc_simd_integer_tests)
endif()
