###################
## add unit tests #
###################

macro(create_unittest test_name)

# create the executable
catkin_add_gtest(${test_name}
  main.cpp
  ${test_name}.cpp
)
if(TARGET ${test_name})
    # link the dependencies to it
    target_link_libraries(${test_name}
        ${catkin_LIBRARIES}
    )
endif()

endmacro(create_unittest test_name)

create_unittest(test_robot_backend)
create_unittest(test_sensor_interface)
create_unittest(test_sensor_logger)
