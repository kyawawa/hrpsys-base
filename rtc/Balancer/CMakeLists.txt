option(BUILD_BALANCER "Build Balancer RTC" ON)

if(NOT BUILD_BALANCER)
  return()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# pkg_check_modules(cnoid-plugin REQUIRED choreonoid-body-plugin)
# include_directories(${cnoid-plugin_INCLUDE_DIRS})
# link_directories(${cnoid-plugin_LIBRARY_DIRS})

set(comp_sources Balancer.cpp BalancerService_impl.cpp
  ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp
  ../SequencePlayer/interpolator.cpp)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)

add_library(Balancer SHARED ${comp_sources})
target_link_libraries(Balancer ${libs})
set_target_properties(Balancer PROPERTIES PREFIX "")

add_executable(BalancerComp BalancerComp.cpp ${comp_sources})
target_link_libraries(BalancerComp ${libs})

set(target Balancer BalancerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
