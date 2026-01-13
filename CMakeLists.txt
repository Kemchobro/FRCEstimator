cmake_minimum_required(VERSION 3.10)
project(FRCEstimator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)

    add_compile_options(-Wformat=2 -Wcast-align -Wcast-qual)
    add_compile_options(-Wconversion -Wsign-conversion)
    add_compile_options(-Wundef -Wunused -Wold-style-cast)

    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wdocumentation -Wno-documentation-unknown-command)
    endif()

    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(-O3 -DNDEBUG)
        add_compile_options(-march=native)
    elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(-g -O0)
        add_compile_definitions(DEBUG)
    elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
        add_compile_options(-O2 -g)
    endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    add_compile_options(/W4 /permissive-)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(/O2 /DNDEBUG)
    elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(/Od /Zi)
    endif()
endif()

set(LOCAL_GTSAM_DIR "${CMAKE_SOURCE_DIR}/install/lib/cmake/GTSAM")
if(EXISTS "${LOCAL_GTSAM_DIR}/GTSAMConfig.cmake" OR EXISTS "${LOCAL_GTSAM_DIR}/gtsam-config.cmake")
    set(GTSAM_DIR "${LOCAL_GTSAM_DIR}")
    message(STATUS "Found GTSAM in local install directory: ${GTSAM_DIR}")
endif()

list(APPEND CMAKE_PREFIX_PATH
    "${CMAKE_SOURCE_DIR}/install"
    "/usr/local"
    "/opt/homebrew"
    "$ENV{HOME}/.local"
)

find_package(GTSAM QUIET)

if(NOT GTSAM_FOUND)
    message(FATAL_ERROR
        "GTSAM not found!\n"
        "Please install GTSAM using one of the following methods:\n"
        "  1. Build from source:\n"
        "     - Clone: git clone https://github.com/borglab/gtsam.git\n"
        "     - Run: ./setup_gtsam.sh (or follow README instructions)\n"
        "  2. If GTSAM is installed elsewhere, set GTSAM_DIR:\n"
        "     cmake -DGTSAM_DIR=/path/to/gtsam/lib/cmake/GTSAM -B build\n"
        "  3. Add GTSAM installation prefix to CMAKE_PREFIX_PATH"
    )
endif()

add_executable(optimizer
    src/app/main.cpp
    src/analysis/trajectory_metrics.cpp
    src/ekf/ekf_localization.cpp
    src/odometry/landmark_odometry_solver.cpp
    src/odometry/swerve_kinematics.cpp
    src/pipeline/synthetic_pipeline.cpp
    src/plotting/octave_plotter.cpp
    src/simulation/synthetic_data.cpp
)

target_include_directories(optimizer PRIVATE "${CMAKE_SOURCE_DIR}/src")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_definitions(optimizer PRIVATE
        $<$<CONFIG:Debug>:DEBUG>
    )
endif()

if(TARGET gtsam)
    target_link_libraries(optimizer gtsam)
elseif(TARGET GTSAM::gtsam)
    target_link_libraries(optimizer GTSAM::gtsam)
else()
    find_library(GTSAM_LIBRARY gtsam PATHS ${GTSAM_DIR}/../.. ${CMAKE_PREFIX_PATH}/lib)
    if(GTSAM_LIBRARY)
        target_link_libraries(optimizer ${GTSAM_LIBRARY})
    else()
        target_link_libraries(optimizer gtsam)
    endif()
endif()

if(APPLE)
    if(GTSAM_DIR)
        get_filename_component(GTSAM_LIB_DIR "${GTSAM_DIR}/../.." ABSOLUTE)
        if(EXISTS "${GTSAM_LIB_DIR}")
            set_target_properties(optimizer PROPERTIES
                BUILD_WITH_INSTALL_RPATH FALSE
                INSTALL_RPATH_USE_LINK_PATH TRUE
                BUILD_RPATH "${GTSAM_LIB_DIR}"
                INSTALL_RPATH "${GTSAM_LIB_DIR}"
            )
            target_link_options(optimizer PRIVATE
                "LINKER:-rpath,${GTSAM_LIB_DIR}"
            )
            target_link_directories(optimizer PRIVATE "${GTSAM_LIB_DIR}")

            add_custom_command(TARGET optimizer POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E echo "Fixing GTSAM library path for macOS..."
                COMMAND install_name_tool -change lib/libgtsam.4.dylib @rpath/libgtsam.4.dylib $<TARGET_FILE:optimizer> || true
                COMMAND install_name_tool -add_rpath "${GTSAM_LIB_DIR}" $<TARGET_FILE:optimizer> || true
                COMMENT "Fixing GTSAM library path for macOS"
                VERBATIM
            )
        endif()
    endif()
else()
    if(GTSAM_DIR)
        get_filename_component(GTSAM_LIB_DIR "${GTSAM_DIR}/../lib" ABSOLUTE)
        if(EXISTS "${GTSAM_LIB_DIR}")
            set_target_properties(optimizer PROPERTIES
                INSTALL_RPATH "${GTSAM_LIB_DIR}"
                BUILD_WITH_INSTALL_RPATH TRUE
            )
        endif()
    endif()
endif()
