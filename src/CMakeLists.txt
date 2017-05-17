##############################################################################
# Sources
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# Targets
##############################################################################

add_executable(navigoals ${SOURCES})
add_dependencies(navigoals kobuki_msgs_gencpp)
target_link_libraries(navigoals ${catkin_LIBRARIES})

install(TARGETS navigoals
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
