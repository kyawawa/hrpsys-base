# cmake_minimum_required(VERSION 3.8)

option(BUILD_BALANCER "Build Balancer RTC" ON)

if(NOT BUILD_BALANCER)
  return()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")

find_package(PkgConfig REQUIRED)
pkg_check_modules(cnoid-body choreonoid-body REQUIRED)
pkg_check_modules(cnoid-util choreonoid-util REQUIRED)
include_directories(${cnoid-body_INCLUDE_DIRS} ${cnoid-util_INCLUDE_DIRS})
link_directories(${cnoid-body_LIBRARY_DIRS} ${cnoid-util_LIBRARY_DIRS})

set(UTILDIR util)
add_library(BalancerUtil SHARED ${UTILDIR}/Interpolator.cpp ${UTILDIR}/Kinematics.cpp)
target_link_libraries(BalancerUtil PRIVATE ${cnoid-body_LIBRARIES})
set_target_properties(BalancerUtil PROPERTIES PREFIX "lib")

add_library(Balancer SHARED Balancer.cpp BalancerService_impl.cpp)
target_link_libraries(Balancer PRIVATE BalancerUtil ${cnoid-body_LIBRARIES} hrpsysBaseStub)
set_target_properties(Balancer PROPERTIES PREFIX "")

add_executable(BalancerComp BalancerComp.cpp)
target_link_libraries(BalancerComp PRIVATE Balancer BalancerUtil hrpsysBaseStub)

add_executable(testWholebodyIK test/testWholebodyIK.cpp)
target_link_libraries(testWholebodyIK PRIVATE BalancerUtil ${cnoid-body_LIBRARIES})
target_compile_definitions(testWholebodyIK PRIVATE CNOID_SHARE_DIR="${cnoid-body_PREFIX}")

set(target BalancerUtil Balancer BalancerComp testWholebodyIK)
install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
