
file(GLOB_RECURSE COMMON_SOURCES ${CMAKE_CURRENT_LIST_DIR}/common/*.cc)

include(add_target.cmake)

include_directories(${CMAKE_CURRENT_LIST_DIR})
include_directories(${CMAKE_CURRENT_LIST_DIR}/targets)

file(GLOB_RECURSE TEST_TARGET_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/test/*.cc)
add_exe_target(test_target "${TEST_TARGET_SOURCES};${COMMON_SOURCES}")

## add targets here...
file(GLOB_RECURSE MECA_HERO_SOURCES ${CMAKE_CURRENT_LIST_DIR}/targets/meca_hero/*.cc)
add_exe_target(meca_hero "${MECA_HERO_SOURCES};${COMMON_SOURCES}")
