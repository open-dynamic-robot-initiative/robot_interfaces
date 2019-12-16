###################
## add unit tests #
###################

macro(create_unittest test_name)

# create the executable
catkin_add_gtest(${test_name}_ut
  main.cpp
  test_${test_name}.cpp
)
if(TARGET ${test_name}_ut)
    # link the dependencies to it
    target_link_libraries(${test_name}_ut
        ${catkin_LIBRARIES}
    )
endif()

endmacro(create_unittest test_name)

create_unittest(process_action)
