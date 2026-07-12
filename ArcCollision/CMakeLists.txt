cmake_minimum_required(VERSION 3.20)
project(ArcCollision VERSION 1.0.0 LANGUAGES C CXX)

option(ARC_BUILD_SHARED_LIBRARY "Build ArcCollision as a shared library" ON)

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
if(BUILD_TESTING AND NOT CMAKE_CROSSCOMPILING)
    add_executable(arc_c_api_smoke tests/c_api_smoke.c)
    target_link_libraries(arc_c_api_smoke PRIVATE ArcCollision)
    target_include_directories(arc_c_api_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    add_test(NAME arc_c_api_smoke COMMAND arc_c_api_smoke)
endif()
