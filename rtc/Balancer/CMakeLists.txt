option(BUILD_BALANCER "Build Balancer RTC" ON)

if(NOT BUILD_BALANCER)
  return()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")

find_package(PkgConfig REQUIRED)
pkg_check_modules(cnoid-body choreonoid-body REQUIRED)
include_directories(${cnoid-body_INCLUDE_DIRS})
link_directories(${cnoid-body_LIBRARY_DIRS})

set(UTILDIR util)
add_library(BalancerUtil SHARED ${UTILDIR}/Interpolator.cpp ${UTILDIR}/Kinematics.cpp)
target_link_libraries(BalancerUtil PRIVATE ${cnoid-body_LIBRARIES})
set_target_properties(BalancerUtil PROPERTIES PREFIX "lib")

add_library(Balancer SHARED Balancer.cpp BalancerService_impl.cpp)
target_link_libraries(Balancer PRIVATE BalancerUtil ${cnoid-body_LIBRARIES} hrpsysBaseStub)
set_target_properties(Balancer PROPERTIES PREFIX "")

add_executable(BalancerComp BalancerComp.cpp)
target_link_libraries(BalancerComp PRIVATE Balancer BalancerUtil hrpsysBaseStub)

set(target BalancerUtil Balancer BalancerComp)
install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
