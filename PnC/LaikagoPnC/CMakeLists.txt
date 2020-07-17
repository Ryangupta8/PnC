file(GLOB_RECURSE sources "*.cpp")
file(GLOB_RECURSE headers "*.hpp")

add_library(LaikagoPnC ${sources})
target_link_libraries(LaikagoPnC ${DART_LIBRARIES}
                                  myRobotSystem
                                  myUtils
                                  myFilters
                                  myPlanner
                                  myWBC
                                  )
