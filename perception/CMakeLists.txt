cmake_minimum_required(VERSION 3.16)

# DPCBF perception subsystem.
#
# Self-contained: deleting this directory and the guarded block in simulate/CMakeLists.txt
# restores the repository exactly. See perception/docs/architecture.md.
#
# Target layout mirrors the dependency rules - the link sets ARE the enforcement:
#   perception_contracts  INTERFACE  Eigen/STL only. No MuJoCo, no ROS2, no dpcbf, no yaml-cpp.
#   perception_core       STATIC     algorithms; depends only on contracts.
#   perception_config     STATIC     yaml-cpp lives here and nowhere else.
#   perception_mujoco     STATIC     the ONLY target allowed to link MuJoCo.
#   perception_integration STATIC    the facade the application constructs.

option(PERCEPTION_ENABLE "Build the DPCBF perception subsystem" ON)
option(PERCEPTION_WITH_ROS2 "Build the optional ROS2 visualization adapter" OFF)
option(PERCEPTION_WITH_TRAVEL "Build the optional TRAVEL ground-segmentation adapter" OFF)
option(PERCEPTION_BUILD_UPSTREAM_ORACLE
       "Compile upstream obstacle_detector headers as a test-only regression oracle (needs Armadillo)" OFF)
option(PERCEPTION_BUILD_TESTS "Build perception tests" ON)

if(NOT PERCEPTION_ENABLE)
  message(STATUS "perception: PERCEPTION_ENABLE=OFF - subsystem not built")
  return()
endif()

message(STATUS "perception: enabled"
               " (ROS2=${PERCEPTION_WITH_ROS2}"
               " TRAVEL=${PERCEPTION_WITH_TRAVEL}"
               " UPSTREAM_ORACLE=${PERCEPTION_BUILD_UPSTREAM_ORACLE}"
               " TESTS=${PERCEPTION_BUILD_TESTS})")

find_package(Eigen3 QUIET NO_MODULE)
if(TARGET Eigen3::Eigen)
  set(PERCEPTION_EIGEN_TARGET Eigen3::Eigen)
else()
  # The repository's other Eigen consumers use the system include path directly.
  find_path(PERCEPTION_EIGEN_INCLUDE_DIR Eigen/Core PATH_SUFFIXES eigen3)
  if(NOT PERCEPTION_EIGEN_INCLUDE_DIR)
    message(FATAL_ERROR "perception: Eigen3 was not found (looked for Eigen/Core)")
  endif()
  add_library(perception_eigen INTERFACE)
  target_include_directories(perception_eigen INTERFACE ${PERCEPTION_EIGEN_INCLUDE_DIR})
  set(PERCEPTION_EIGEN_TARGET perception_eigen)
endif()

# ---------------------------------------------------------------------------------------
# perception_contracts - header-only value types. Eigen/STL only, by construction.
# ---------------------------------------------------------------------------------------
add_library(perception_contracts INTERFACE)
target_include_directories(perception_contracts INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(perception_contracts INTERFACE ${PERCEPTION_EIGEN_TARGET})
target_compile_features(perception_contracts INTERFACE cxx_std_17)

# ---------------------------------------------------------------------------------------
# perception_dpcbf_headers - the include path for dpcbf's two public headers, and NOTHING
# else. No link: both types this subsystem needs (dpcbf::DynamicObstacle,
# dpcbf::ObstacleState) are plain structs defined in headers, so nothing here pulls in OSQP,
# abseil or yaml-cpp.
#
# THE ACCESS RULE THIS TARGET CANNOT ENFORCE BY ITSELF. An include path cannot distinguish
# the two headers, so the split is stated here and reviewed:
#   perception_mujoco        may include dpcbf/dynamic_obstacles.h ONLY (oracle_provider_mj -
#                            the one deliberate exception in the dependency rules), and must
#                            never name dpcbf::ObstacleState.
#   perception_dpcbf_adapter may include dpcbf/dpcbf_safety_filter.h - it is the only module
#                            allowed to touch dpcbf::ObstacleState fields, and it must never
#                            include a MuJoCo header.
# Nothing else links this target. perception_contracts, perception_core and
# perception_diagnostics deliberately do not, which is what keeps "core must not depend on
# dpcbf" true by link set rather than by intention.
# ---------------------------------------------------------------------------------------
set(PERCEPTION_DPCBF_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../dpcbf"
    CACHE PATH "Root of the DPCBF subsystem, for its public headers")
if(NOT EXISTS "${PERCEPTION_DPCBF_ROOT}/include/dpcbf/dpcbf_safety_filter.h")
  message(FATAL_ERROR
    "perception: dpcbf headers were not found under ${PERCEPTION_DPCBF_ROOT}/include."
    " The obstacle-source selector exists to emit dpcbf::ObstacleState, so unlike MuJoCo,"
    " ROS2 or TRAVEL this is not an optional dependency.")
endif()
add_library(perception_dpcbf_headers INTERFACE)
target_include_directories(perception_dpcbf_headers
  INTERFACE "${PERCEPTION_DPCBF_ROOT}/include"
)

# ---------------------------------------------------------------------------------------
# perception_core - repository-owned algorithms. Grows with each phase; today it carries
# the shared geometry conventions.
# ---------------------------------------------------------------------------------------
add_library(perception_core STATIC
  src/core/geometry/rpy.cpp
  src/core/projection/scan_projector.cpp
  src/core/detection/line_fit.cpp
  src/core/detection/segment_circle_detector.cpp
  src/core/tracking/kf_circle_tracker.cpp
  src/core/safety/safety_state_generator.cpp
)
target_link_libraries(perception_core PUBLIC perception_contracts)
target_compile_features(perception_core PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------------------
# perception_build_info - the git identity that goes into every dump's provenance.
#
# Generated at BUILD time, not configure time: a sha captured when CMake last ran names the
# wrong commit as soon as anyone commits, and a manifest field that names the wrong commit is
# worse than one that says "unknown". The generator script only rewrites the header when its
# contents change, so this does not force a rebuild every time.
# ---------------------------------------------------------------------------------------
set(PERCEPTION_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(PERCEPTION_BUILD_INFO_HEADER
    "${PERCEPTION_GENERATED_DIR}/perception/generated/perception_build_info.h")

add_custom_target(perception_build_info ALL
  BYPRODUCTS "${PERCEPTION_BUILD_INFO_HEADER}"
  COMMAND ${CMAKE_COMMAND}
          -DPERCEPTION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
          -DPERCEPTION_OUTPUT_FILE=${PERCEPTION_BUILD_INFO_HEADER}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateBuildInfo.cmake
  COMMENT "perception: capturing git provenance for dump manifests"
  VERBATIM
)
# Generate once now as well, so the very first configure+build has a header to compile
# against before the custom target has had a chance to run.
execute_process(COMMAND ${CMAKE_COMMAND}
                -DPERCEPTION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
                -DPERCEPTION_OUTPUT_FILE=${PERCEPTION_BUILD_INFO_HEADER}
                -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateBuildInfo.cmake)

# ---------------------------------------------------------------------------------------
# perception_diagnostics - dump writers/readers for every contract, run manifests, replay.
#
# Links perception_contracts ONLY. That link set is the enforcement of the architecture doc's
# rule that core/* must not reach yaml-cpp, MuJoCo, ROS2 or dpcbf: core/diagnostics is the one
# part of core that does file I/O, and it still depends on nothing but the contracts and the
# standard library. perception_replay and the round-trip test link this and nothing else,
# which is what makes a dump replayable on a machine that cannot build the simulator.
# ---------------------------------------------------------------------------------------
add_library(perception_diagnostics STATIC
  src/core/diagnostics/dump_format.cpp
  src/core/diagnostics/json_value.cpp
  src/core/diagnostics/dump_file.cpp
  src/core/diagnostics/run_manifest.cpp
  src/core/diagnostics/replay_report.cpp
)
target_link_libraries(perception_diagnostics PUBLIC perception_contracts)
target_compile_features(perception_diagnostics PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------------------
# perception_config - the only target that knows yaml-cpp.
#
# Also carries integration/dump_recorder.cpp: the config-driven half of the dump subsystem
# (reading `dumps:`, honouring decimation, hashing the resolved config, stamping the git
# provenance). It lives on this side of the boundary precisely because it needs
# PerceptionConfig, which core/diagnostics may not see.
# ---------------------------------------------------------------------------------------
# Prefer the imported target; fall back to the bare library name, which is how the rest of
# this repository (simulate/, dpcbf/) consumes yaml-cpp.
find_package(yaml-cpp QUIET)
if(TARGET yaml-cpp::yaml-cpp)
  set(PERCEPTION_YAMLCPP yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(PERCEPTION_YAMLCPP yaml-cpp)
else()
  set(PERCEPTION_YAMLCPP yaml-cpp)
  message(STATUS "perception: no yaml-cpp CMake target; linking the bare name")
endif()

add_library(perception_config STATIC
  src/integration/perception_config.cpp
  src/integration/dump_recorder.cpp
)
target_link_libraries(perception_config
  PUBLIC perception_contracts perception_diagnostics
  PRIVATE ${PERCEPTION_YAMLCPP}
)
target_include_directories(perception_config PRIVATE "${PERCEPTION_GENERATED_DIR}")
add_dependencies(perception_config perception_build_info)
target_compile_features(perception_config PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------------------
# perception_dpcbf_adapter - contracts -> dpcbf::ObstacleState. The only module that names
# that type. NO MuJoCo link, by rule and by link set: this target must build and be testable
# on a machine that cannot run the simulator, which is also why it sits outside the MuJoCo
# block below.
# ---------------------------------------------------------------------------------------
add_library(perception_dpcbf_adapter STATIC
  src/adapters/dpcbf/oracle_to_dpcbf.cpp
  src/adapters/dpcbf/safety_to_dpcbf.cpp
)
target_link_libraries(perception_dpcbf_adapter
  PUBLIC perception_contracts perception_dpcbf_headers
)
target_compile_features(perception_dpcbf_adapter PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------------------
# perception_dpcbf_probe - the duplicate side-effect-free DpcbfSafetyFilter comparison
# (architecture doc section 16, ladder steps 4-5; P11).
#
# SAME MODULE, SEPARATE TARGET, AND BOTH HALVES OF THAT ARE DELIBERATE.
#
#   Same module: this is the only code in the subsystem that instantiates DpcbfSafetyFilter
#   itself rather than merely naming dpcbf::ObstacleState, and the dependency rules put every
#   dpcbf header inside adapters/dpcbf. Its source therefore lives in src/adapters/dpcbf/
#   alongside the two conversions, and integration/Evaluator calls it through a header that
#   names no dpcbf type at all (paired_filter_probe.h) rather than holding a filter itself.
#
#   Separate target: linking the real dpcbf_safety_filter drags in OSQP and abseil.
#   perception_dpcbf_adapter is on the LIVE path - perception_integration links it, and so
#   therefore does unitree_mujoco - and it must keep building standalone on a machine with no
#   simulator and no dpcbf build. An evaluation-only facility must not add a solver to the
#   live path's link set to buy itself a measurement. Hence the guard: this target exists only
#   where the dpcbf library is actually being built.
# ---------------------------------------------------------------------------------------
if(TARGET dpcbf_safety_filter)
  add_library(perception_dpcbf_probe STATIC
    src/adapters/dpcbf/paired_filter_probe.cpp
  )
  target_link_libraries(perception_dpcbf_probe
    PUBLIC perception_dpcbf_adapter
    PRIVATE dpcbf_safety_filter
  )
  target_compile_features(perception_dpcbf_probe PUBLIC cxx_std_17)
else()
  message(STATUS "perception: dpcbf_safety_filter target absent"
                 " - the paired-filter probe and its evaluator test are skipped")
endif()

# ---------------------------------------------------------------------------------------
# perception_evaluator - oracle-vs-estimate association, error stats, JSON manifests.
#
# THE LINK SET IS THE ENFORCEMENT OF "evaluator -> feeding estimator" (doc section 8). It
# links perception_contracts and perception_diagnostics, and deliberately NOT perception_core:
# SegmentCircleDetector, KfCircleTracker and SafetyStateGenerator are not linkable from here,
# so there is no estimator object in reach for this module to feed. Same principle that keeps
# core off yaml-cpp and MuJoCo. It also does not link perception_dpcbf_probe - it only reads
# the probe's plain-POD outcome type - so it stays buildable with no dpcbf library present.
# ---------------------------------------------------------------------------------------
add_library(perception_evaluator STATIC
  src/integration/evaluator.cpp
)
target_link_libraries(perception_evaluator
  PUBLIC perception_contracts perception_diagnostics
)
target_compile_features(perception_evaluator PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------------------
# MuJoCo. Supplied by the vendored tree under simulate/mujoco. When it is absent, the
# core/config/test targets that do not need it still build.
# ---------------------------------------------------------------------------------------
set(PERCEPTION_MUJOCO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../simulate/mujoco"
    CACHE PATH "Root of the vendored MuJoCo distribution")

set(PERCEPTION_HAVE_MUJOCO OFF)
if(EXISTS "${PERCEPTION_MUJOCO_ROOT}/include/mujoco/mujoco.h")
  find_library(PERCEPTION_MUJOCO_LIBRARY
    NAMES mujoco
    HINTS "${PERCEPTION_MUJOCO_ROOT}/lib"
    NO_DEFAULT_PATH
  )
  if(PERCEPTION_MUJOCO_LIBRARY)
    set(PERCEPTION_HAVE_MUJOCO ON)
  endif()
endif()

# ---------------------------------------------------------------------------------------
# perception_opencv_view - the live scan window. Guarded exactly like dpcbf_visualizer:
# present when OpenCV is, absent otherwise, and never required.
# ---------------------------------------------------------------------------------------
find_package(OpenCV QUIET COMPONENTS core imgproc highgui)
if(OpenCV_FOUND)
  message(STATUS "perception: OpenCV ${OpenCV_VERSION} found - live scan window enabled")
  add_library(perception_opencv_view STATIC
    src/adapters/opencv/live_scan_view.cpp
  )
  target_include_directories(perception_opencv_view PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(perception_opencv_view
    PUBLIC perception_contracts perception_config
    PRIVATE ${OpenCV_LIBS}
  )
  target_compile_features(perception_opencv_view PUBLIC cxx_std_17)
  target_compile_definitions(perception_opencv_view PUBLIC PERCEPTION_HAVE_OPENCV=1)
else()
  message(STATUS "perception: OpenCV not found - the live scan window will be unavailable")
endif()

if(PERCEPTION_HAVE_MUJOCO)
  message(STATUS "perception: MuJoCo found at ${PERCEPTION_MUJOCO_ROOT}")

  add_library(perception_mujoco STATIC
    src/adapters/mujoco/ray_pattern_generator.cpp
    src/adapters/mujoco/frame_provider_mj.cpp
    src/adapters/mujoco/extrinsic_guard.cpp
    src/adapters/mujoco/raycaster_mj.cpp
    src/adapters/mujoco/live_scan_overlay.cpp
    src/adapters/mujoco/oracle_provider_mj.cpp
  )
  target_include_directories(perception_mujoco PUBLIC "${PERCEPTION_MUJOCO_ROOT}/include")
  # perception_dpcbf_headers is here for oracle_provider_mj.cpp and dpcbf/dynamic_obstacles.h
  # ALONE - the one sanctioned MuJoCo-plus-dpcbf module. dpcbf_dynamic_obstacles is linked
  # because Snapshot() is a real symbol, not just a struct declaration.
  target_link_libraries(perception_mujoco
    PUBLIC perception_contracts perception_core perception_config perception_dpcbf_headers
           dpcbf_dynamic_obstacles "${PERCEPTION_MUJOCO_LIBRARY}"
  )
  target_compile_features(perception_mujoco PUBLIC cxx_std_17)

  # perception_integration - the single facade the application constructs.
  add_library(perception_integration STATIC
    src/integration/mid360_bringup.cpp
    src/integration/obstacle_source_selector.cpp
  )
  target_link_libraries(perception_integration PUBLIC perception_mujoco perception_dpcbf_adapter)
  target_compile_features(perception_integration PUBLIC cxx_std_17)
  if(TARGET perception_opencv_view)
    # PUBLIC so PERCEPTION_HAVE_OPENCV reaches mid360_bringup.h in every consumer.
    target_link_libraries(perception_integration PUBLIC perception_opencv_view)
  endif()
  # Lets the application compile the hook regions out entirely when perception is absent.
  target_compile_definitions(perception_integration PUBLIC PERCEPTION_ENABLED=1)

  # Headless raycast validation harness (architecture doc: apps/lidar_bench.cc).
  add_executable(perception_lidar_bench apps/lidar_bench.cc)
  target_link_libraries(perception_lidar_bench PRIVATE perception_integration)
else()
  message(STATUS "perception: MuJoCo NOT found at ${PERCEPTION_MUJOCO_ROOT}"
                 " - adapters/mujoco, the integration facade and lidar_bench are skipped"
                 " (perception_replay still builds; it needs no MuJoCo)")
endif()

# ---------------------------------------------------------------------------------------
# perception_replay - headless dump inspection (architecture doc section 9 and 13).
#
# OUTSIDE the MuJoCo block deliberately: section 9 requires that "core/*, perception_replay,
# unit/contract/regression tests" all build without MuJoCo. Replaying a fixture must work on a
# machine that cannot run the simulator, or the dumps are not really the regression format.
# ---------------------------------------------------------------------------------------
add_executable(perception_replay apps/perception_replay.cc)
target_link_libraries(perception_replay PRIVATE perception_diagnostics)

# ---------------------------------------------------------------------------------------
# Optional adapters. Both default OFF and neither is on any production path.
# ---------------------------------------------------------------------------------------
if(PERCEPTION_WITH_ROS2)
  message(FATAL_ERROR
    "perception: PERCEPTION_WITH_ROS2=ON but the ROS2 adapter has not been implemented yet"
    " (its phase is later than the current Mid-360 bring-up slice)")
endif()

if(PERCEPTION_WITH_TRAVEL)
  message(FATAL_ERROR
    "perception: PERCEPTION_WITH_TRAVEL=ON but the TRAVEL segmentation adapter has not been"
    " implemented yet (its phase is later than the current Mid-360 bring-up slice)")
endif()

# ---------------------------------------------------------------------------------------
# Tests. Plain executables registered with CTest, matching dpcbf_safety_filter_test.
# ---------------------------------------------------------------------------------------
if(PERCEPTION_BUILD_TESTS)
  include(CTest)
  enable_testing()

  # No MuJoCo needed: pure rotation-convention checks.
  add_executable(perception_geometry_rpy_test tests/unit/geometry_rpy_test.cpp)
  target_link_libraries(perception_geometry_rpy_test PRIVATE perception_core)
  add_test(NAME perception_geometry_rpy_test COMMAND perception_geometry_rpy_test)

  # Contract construction/validation/round-trip for every type in core/contracts.
  # Links perception_contracts ONLY - which is itself part of the assertion: if a contract
  # ever grows a MuJoCo, yaml-cpp or dpcbf dependency, this target stops linking.
  add_executable(perception_contracts_test tests/contract/contracts_test.cpp)
  target_link_libraries(perception_contracts_test PRIVATE perception_contracts)
  target_include_directories(perception_contracts_test PRIVATE tests/contract)
  add_test(NAME perception_contracts_test COMMAND perception_contracts_test)

  # The typed config tree. Takes perception/ as argv[1] so it can load the SHIPPED
  # configs/perception.yaml rather than a fixture copy that could drift from it.
  add_executable(perception_config_test tests/contract/config_test.cpp)
  target_link_libraries(perception_config_test PRIVATE perception_config)
  target_include_directories(perception_config_test PRIVATE tests/contract)
  add_test(
    NAME perception_config_test
    COMMAND perception_config_test "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  # Dump round-trip fidelity, versioned-schema rejection, header/manifest fidelity.
  #
  # Links perception_diagnostics ONLY - part of the assertion, exactly as
  # perception_contracts_test links only the contracts: if core/diagnostics ever grows a
  # yaml-cpp, MuJoCo or dpcbf dependency, this target stops linking. argv[1] is a scratch
  # directory under the build tree, so the test never writes into the source tree or /tmp.
  add_executable(perception_dump_roundtrip_test tests/contract/dump_roundtrip_test.cpp)
  target_link_libraries(perception_dump_roundtrip_test PRIVATE perception_diagnostics)
  target_include_directories(perception_dump_roundtrip_test
    PRIVATE tests/contract tests/fixtures
  )
  add_test(
    NAME perception_dump_roundtrip_test
    COMMAND perception_dump_roundtrip_test
            "${CMAKE_CURRENT_BINARY_DIR}/test_scratch/dump_roundtrip"
  )

  # The P7 golden-fixture capture tool. Not a test - it produces
  # tests/fixtures/projection_golden/, which perception_projection_test checks against and
  # which P8's detector port consumes as its input corpus.
  #
  # Deliberately produces its scans through the VERBATIM UPSTREAM REFERENCE rather than
  # through ScanProjector: a golden captured from the implementation under test proves only
  # that the implementation is stable. See tests/regression/p2l_reference.h.
  #
  # Outside the MuJoCo block, unlike P3's oracle capture: this corpus is synthetic by
  # design (P6's segmenter does not exist yet), so nothing here needs a simulator.
  add_executable(perception_projection_golden_capture
                 tests/regression/projection_golden_capture.cc)
  target_link_libraries(perception_projection_golden_capture
    PRIVATE perception_core perception_diagnostics
  )
  target_include_directories(perception_projection_golden_capture
    PRIVATE tests/regression tests/fixtures
  )

  # P7 acceptance: bin correctness exact, p2l equivalence exact, information-loss stats.
  #
  # Links perception_config as well as perception_core so the test can assert that the
  # SHIPPED configs/perception.yaml actually reaches the projector through
  # integration/projection_params.h. That link is on the TEST, not on perception_core -
  # perception_core still links perception_contracts alone, which is what keeps the
  # core-dependency rule enforced by link set rather than by intention.
  # argv[1] is perception/, for the shipped config and the committed fixtures.
  add_executable(perception_projection_test tests/unit/projection_test.cpp)
  target_link_libraries(perception_projection_test
    PRIVATE perception_core perception_config perception_diagnostics
  )
  target_include_directories(perception_projection_test
    PRIVATE tests/contract tests/regression tests/fixtures
  )
  add_test(
    NAME perception_projection_test
    COMMAND perception_projection_test "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  # P8 acceptance: the ported detection core sub-stage by sub-stage, plus the short-arc
  # circle-fitting experiment. Needs no MuJoCo and no Armadillo - the upstream-parity half of
  # P8 is the separate oracle target below, deliberately, so that the claims which do NOT
  # reduce to "same as upstream" are provable on any machine.
  # argv[1] is perception/, for the shipped config.
  add_executable(perception_detection_test tests/unit/detection_test.cpp)
  target_link_libraries(perception_detection_test PRIVATE perception_core perception_config)
  target_include_directories(perception_detection_test
    PRIVATE tests/contract tests/fixtures
  )
  add_test(
    NAME perception_detection_test
    COMMAND perception_detection_test "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  # P9 acceptance: the ported tracking core, the lifecycle, the two P8 findings this phase had to
  # design around, and the section-12 tracking gates. The upstream-parity half is the separate
  # oracle target below.
  #
  # Links perception_config for the same reason perception_detection_test does - the test asserts
  # that the SHIPPED configs/perception.yaml actually reaches the tracker through
  # integration/tracking_params.h. It also includes tests/fixtures/detection_scenes.h and runs the
  # REAL detector, because the +/-pi seam scenario's subject is a detector behaviour and a
  # synthesised version of it would assume the answer.
  # argv[1] is perception/, for the shipped config.
  add_executable(perception_tracking_test tests/unit/tracking_test.cpp)
  target_link_libraries(perception_tracking_test PRIVATE perception_core perception_config)
  target_include_directories(perception_tracking_test
    PRIVATE tests/contract tests/fixtures
  )
  add_test(
    NAME perception_tracking_test
    COMMAND perception_tracking_test "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  # P10 acceptance: validity gating, conservative inflation, the velocity spike clamp, and the
  # section-12 safety gates (containment, radius under-estimation, uncertainty coverage, the
  # analytic latency check).
  #
  # Links perception_config for the same reason the detection and tracking tests do - it asserts
  # that the SHIPPED configs/perception.yaml actually reaches the stage through
  # integration/safety_params.h. It runs the REAL detector and the REAL tracker over ray-cast
  # scenes as well as over P9's synthetic streams, because a containment claim about the shipped
  # pipeline cannot be made from a fixture that skips two of its three stages.
  # argv[1] is perception/, for the shipped config.
  add_executable(perception_safety_test tests/unit/safety_test.cpp)
  target_link_libraries(perception_safety_test PRIVATE perception_core perception_config)
  target_include_directories(perception_safety_test
    PRIVATE tests/contract tests/fixtures
  )
  add_test(
    NAME perception_safety_test
    COMMAND perception_safety_test "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  # P11 acceptance, part 1: the SafetyObstacle -> dpcbf::ObstacleState conversion, the id
  # policy through the tracker's fusion and fission passes, and the measured (not inherited)
  # impact of max_obstacles/drop_invalid on the ESTIMATED path.
  #
  # Needs no dpcbf LIBRARY and no MuJoCo - only the ObstacleState struct from the headers - so
  # the adapter's correctness is provable on a machine that cannot run the simulator, which is
  # the same property perception_dpcbf_adapter itself has.
  # argv[1] is perception/, for the shipped config.
  add_executable(perception_dpcbf_adapter_test tests/adapter/dpcbf_adapter_test.cpp)
  target_link_libraries(perception_dpcbf_adapter_test
    PRIVATE perception_dpcbf_adapter perception_core perception_config
  )
  target_include_directories(perception_dpcbf_adapter_test
    PRIVATE tests/contract tests/fixtures
  )
  add_test(
    NAME perception_dpcbf_adapter_test
    COMMAND perception_dpcbf_adapter_test "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  # P11 acceptance, part 2: the evaluator's association and the duplicate side-effect-free
  # DPCBF comparison. Needs the real dpcbf library for its two filter instances, so it is
  # guarded on the probe target the same way the MuJoCo tests are guarded on MuJoCo.
  # argv[1] is perception/, argv[2] the repository root (for dpcbf/config/dpcbf_config.yaml).
  if(TARGET perception_dpcbf_probe)
    add_executable(perception_paired_evaluator_test tests/integration/evaluator_test.cpp)
    target_link_libraries(perception_paired_evaluator_test
      PRIVATE perception_evaluator perception_dpcbf_probe perception_core perception_config
    )
    target_include_directories(perception_paired_evaluator_test
      PRIVATE tests/contract tests/fixtures
    )
    add_test(
      NAME perception_paired_evaluator_test
      COMMAND perception_paired_evaluator_test
              "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/.."
    )
  endif()

  # ---------------------------------------------------------------------------------------
  # The upstream regression oracle. TEST-ONLY, OFF by default, and never on any production path.
  #
  # Compiles obstacle_detector's ROS-free utility headers (point/point_set/segment/circle/
  # figure_fitting) and links Armadillo, so the ported detector can be compared against the real
  # `arma::pinv` algorithm on identical inputs - the "code used only in tests" row of the
  # architecture doc's adoption table (section 4.2). The pipeline itself is transcribed in
  # tests/regression/upstream_extractor_reference.h; nothing from obstacle_detector/src is compiled,
  # because that half is ROS1.
  #
  # The option is OFF by default because it adds an Armadillo dependency that the rest of the
  # repository does not have. Turn it on to run the parity suite:
  #   cmake -DPERCEPTION_BUILD_UPSTREAM_ORACLE=ON <build-dir>
  # ---------------------------------------------------------------------------------------
  if(PERCEPTION_BUILD_UPSTREAM_ORACLE)
    set(PERCEPTION_OBSTACLE_DETECTOR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../obstacle_detector"
        CACHE PATH "Root of the reference obstacle_detector package, for the test-only oracle")
    if(NOT EXISTS
       "${PERCEPTION_OBSTACLE_DETECTOR_ROOT}/include/obstacle_detector/utilities/figure_fitting.h")
      message(FATAL_ERROR
        "perception: PERCEPTION_BUILD_UPSTREAM_ORACLE=ON but the reference obstacle_detector package"
        " was not found under ${PERCEPTION_OBSTACLE_DETECTOR_ROOT}. Point"
        " PERCEPTION_OBSTACLE_DETECTOR_ROOT at it, or leave the option OFF.")
    endif()

    find_package(Armadillo QUIET)
    if(Armadillo_FOUND OR ARMADILLO_FOUND)
      set(PERCEPTION_ARMADILLO_LIBS ${ARMADILLO_LIBRARIES})
      set(PERCEPTION_ARMADILLO_INCLUDES ${ARMADILLO_INCLUDE_DIRS})
    else()
      # Armadillo's header-only fallback plus the bare library name, matching how this repository
      # consumes yaml-cpp when no CMake package is present.
      set(PERCEPTION_ARMADILLO_LIBS armadillo)
      set(PERCEPTION_ARMADILLO_INCLUDES "")
      message(STATUS "perception: no Armadillo CMake package; linking the bare name")
    endif()

    if(PERCEPTION_BUILD_TESTS)
      add_executable(perception_detection_oracle_test tests/regression/detection_oracle_test.cpp)
      target_link_libraries(perception_detection_oracle_test
        PRIVATE perception_core perception_diagnostics ${PERCEPTION_ARMADILLO_LIBS}
      )
      target_include_directories(perception_detection_oracle_test PRIVATE
        tests/contract tests/fixtures tests/regression
        "${PERCEPTION_OBSTACLE_DETECTOR_ROOT}/include"
        ${PERCEPTION_ARMADILLO_INCLUDES}
      )
      add_test(
        NAME perception_detection_oracle_test
        COMMAND perception_detection_oracle_test "${CMAKE_CURRENT_SOURCE_DIR}"
      )

      # P9 upstream parity. Unlike the detection oracle, this one compiles upstream's KALMAN FILTER
      # ITSELF (utilities/kalman.h, which the audit established is ROS-free) and links Armadillo, so
      # the 1e-12 comparison runs against the real arma::mat arithmetic. Only TrackedObstacle and
      # the cost function are transcribed, because those two reach ROS message and tf headers - see
      # tests/regression/upstream_tracker_reference.h for the split.
      #
      # Needs no argv: the sequences are generated in the test, not loaded from a fixture, because
      # this comparison is about arithmetic rather than about scan geometry.
      add_executable(perception_tracking_oracle_test tests/regression/tracking_oracle_test.cpp)
      target_link_libraries(perception_tracking_oracle_test
        PRIVATE perception_core ${PERCEPTION_ARMADILLO_LIBS}
      )
      target_include_directories(perception_tracking_oracle_test PRIVATE
        tests/contract tests/regression
        "${PERCEPTION_OBSTACLE_DETECTOR_ROOT}/include"
        ${PERCEPTION_ARMADILLO_INCLUDES}
      )
      add_test(NAME perception_tracking_oracle_test COMMAND perception_tracking_oracle_test)
    endif()
  endif()

  if(PERCEPTION_HAVE_MUJOCO)
    # The PRE-CHANGE golden-fixture capture tool. Not a test - it produces
    # tests/fixtures/oracle_golden/, which perception_oracle_equivalence_test then checks
    # against. Kept buildable rather than deleted after use so the fixture can be
    # regenerated deliberately (and so the transcription of the old inline conversion it
    # contains stays under compilation, and therefore under review).
    add_executable(perception_oracle_golden_capture tests/adapter/oracle_golden_capture.cc)
    target_link_libraries(perception_oracle_golden_capture
      PRIVATE perception_diagnostics perception_dpcbf_headers "${PERCEPTION_MUJOCO_LIBRARY}"
              dpcbf_dynamic_obstacles
    )
    target_include_directories(perception_oracle_golden_capture
      PRIVATE tests/adapter "${PERCEPTION_MUJOCO_ROOT}/include"
    )

    # P3 acceptance: the oracle obstacle path is bit-for-bit unchanged by the refactor.
    # Links the real DpcbfSafetyFilter, because "the numbers are identical" is a weaker claim
    # than "the commands the QP produces from them are identical", and this phase promises the
    # second one. argv[1] is the repository root: it needs scene_g1.xml, dpcbf_config.yaml,
    # the shipped perception.yaml and the committed golden fixtures.
    add_executable(perception_oracle_equivalence_test tests/adapter/oracle_equivalence_test.cpp)
    target_link_libraries(perception_oracle_equivalence_test
      PRIVATE perception_integration dpcbf_safety_filter
    )
    target_include_directories(perception_oracle_equivalence_test
      PRIVATE tests/contract tests/adapter
    )
    add_test(
      NAME perception_oracle_equivalence_test
      COMMAND perception_oracle_equivalence_test "${CMAKE_CURRENT_SOURCE_DIR}/.."
    )

    # The end-to-end proof: dump REAL data from the bring-up path, then replay it through the
    # actual perception_replay binary and check the printed summary against what the live scan
    # reported. argv[1] repository root, argv[2] scratch dir, argv[3] the replay executable.
    add_executable(perception_dump_bringup_test tests/adapter/dump_bringup_test.cpp)
    target_link_libraries(perception_dump_bringup_test PRIVATE perception_integration)
    target_include_directories(perception_dump_bringup_test PRIVATE tests/contract)
    add_dependencies(perception_dump_bringup_test perception_replay)
    add_test(
      NAME perception_dump_bringup_test
      COMMAND perception_dump_bringup_test
              "${CMAKE_CURRENT_SOURCE_DIR}/.."
              "${CMAKE_CURRENT_BINARY_DIR}/test_scratch/dump_bringup"
              "$<TARGET_FILE:perception_replay>"
    )

    # The V0-V5 Mid-360 validation suite. Takes the repository root as argv[1] so it can
    # find src/assets/robots/unitree_g1/xmls/scene_g1.xml.
    add_executable(perception_mid360_bringup_test tests/adapter/mid360_bringup_test.cpp)
    target_link_libraries(perception_mid360_bringup_test PRIVATE perception_integration)
    add_test(
      NAME perception_mid360_bringup_test
      COMMAND perception_mid360_bringup_test "${CMAKE_CURRENT_SOURCE_DIR}/.."
    )

    # V6: continuous pose tracking through sustained motion. Separate from the bring-up
    # suite because it answers a different question - not "is the geometry right at rest"
    # but "is the sensor pose genuinely recomputed every scan while the robot moves".
    add_executable(perception_mid360_pose_tracking_test tests/adapter/mid360_pose_tracking_test.cpp)
    target_link_libraries(perception_mid360_pose_tracking_test PRIVATE perception_integration)
    add_test(
      NAME perception_mid360_pose_tracking_test
      COMMAND perception_mid360_pose_tracking_test "${CMAKE_CURRENT_SOURCE_DIR}/.."
    )
  endif()
endif()
