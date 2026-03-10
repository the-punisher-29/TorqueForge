# Find and configure raylib
find_path(RAYLIB_INCLUDE_DIR
    NAMES raylib.h
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/raylib/include
    REQUIRED
)
# Find raylib library
find_library(RAYLIB_LIBRARY
    NAMES raylib
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/raylib/lib
    REQUIRED
)
# Set raylib variables for parent scope
set(RAYLIB_INCLUDE_DIR ${RAYLIB_INCLUDE_DIR} PARENT_SCOPE)
set(RAYLIB_LIBRARY ${RAYLIB_LIBRARY} PARENT_SCOPE)


# Find and configure bullet collision
# Bullet headers installed under include/bullet/ by cmake --install
find_path(BULLET_INCLUDE_DIR
    NAMES btBulletCollisionCommon.h
    PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}/bullet/include
        ${CMAKE_CURRENT_SOURCE_DIR}/bullet/include/bullet
    REQUIRED
)

# === Bullet libraries (cross-platform) ===
set(BULLET_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bullet/lib")

if(WIN32)
    # Windows: .lib files with Debug/Release variants
    set(BULLET_COLLISION_LIBRARY_DEBUG "${BULLET_LIB_DIR}/BulletCollision_Debug.lib")
    set(BULLET_COLLISION_LIBRARY_RELEASE "${BULLET_LIB_DIR}/BulletCollision.lib")

    set(BULLET_LINEARMATH_LIBRARY_DEBUG "${BULLET_LIB_DIR}/LinearMath_Debug.lib")
    set(BULLET_LINEARMATH_LIBRARY_RELEASE "${BULLET_LIB_DIR}/LinearMath.lib")

    set(BULLET_COLLISION_LIBRARY
        "$<$<CONFIG:Debug>:${BULLET_COLLISION_LIBRARY_DEBUG}>"
        "$<$<NOT:$<CONFIG:Debug>>:${BULLET_COLLISION_LIBRARY_RELEASE}>"
    )
    set(BULLET_LINEARMATH_LIBRARY
        "$<$<CONFIG:Debug>:${BULLET_LINEARMATH_LIBRARY_DEBUG}>"
        "$<$<NOT:$<CONFIG:Debug>>:${BULLET_LINEARMATH_LIBRARY_RELEASE}>"
    )
else()
    # Linux/macOS: .a static libraries
    find_library(BULLET_COLLISION_LIBRARY
        NAMES BulletCollision
        PATHS ${BULLET_LIB_DIR}
        REQUIRED
    )
    find_library(BULLET_LINEARMATH_LIBRARY
        NAMES LinearMath
        PATHS ${BULLET_LIB_DIR}
        REQUIRED
    )
endif()

set(BULLET_INCLUDE_DIR ${BULLET_INCLUDE_DIR} PARENT_SCOPE)
set(BULLET_COLLISION_LIBRARY ${BULLET_COLLISION_LIBRARY} PARENT_SCOPE)
set(BULLET_LINEARMATH_LIBRARY ${BULLET_LINEARMATH_LIBRARY} PARENT_SCOPE)


# Find and configure tinygltf
find_path(TINYGLTF_INCLUDE_DIR
    NAMES tiny_gltf.h
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/tinygltf
    REQUIRED
)

# Set tinygltf variables for parent scope
set(TINYGLTF_INCLUDE_DIR ${TINYGLTF_INCLUDE_DIR} PARENT_SCOPE)

# Find and configure nlohmann json
find_path(NLOHMANN_JSON_INCLUDE_DIR
    NAMES json.hpp
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/nlohmann_json
    REQUIRED
)

# Set nlohmann json variables for parent scope
set(NLOHMANN_JSON_INCLUDE_DIR ${NLOHMANN_JSON_INCLUDE_DIR} PARENT_SCOPE)