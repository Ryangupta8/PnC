file(GLOB_RECURSE sources "*.cpp")
file(GLOB a1_pnc_headers "*.hpp")
file(GLOB a1_testset_headers "TestSet/*.hpp")
file(GLOB a1_taskset_headers "TaskSet/*.hpp")
file(GLOB a1_ctrlset_headers "CtrlSet/*.hpp")

add_library(A1PnC SHARED ${sources})
target_link_libraries(A1PnC ${DART_LIBRARIES}
                               myRobotSystem
                               myUtils
                               myWBC)

install(TARGETS A1PnC DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${a1_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/A1PnC")
install(FILES ${a1_testset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/A1PnC/TestSet")
install(FILES ${a1_ctrlset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/A1PnC/CtrlSet")
install(FILES ${a1_taskset_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/PnC/A1PnC/TaskSet")
