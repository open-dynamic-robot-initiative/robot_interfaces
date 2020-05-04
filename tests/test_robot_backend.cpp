/**
 * @file
 * @brief Tests for RobotBackend
 * @copyright Copyright (c) 2019, Max Planck Gesellschaft.
 */
#include <gtest/gtest.h>
#include <robot_interfaces/example.hpp>
#include <robot_interfaces/robot_backend.hpp>
#include <robot_interfaces/robot_frontend.hpp>

using namespace robot_interfaces;

/**
 * @brief Fixture for the backend tests.
 */
class TestRobotBackend : public ::testing::Test
{
protected:
    typedef example::Action Action;
    typedef example::Observation Observation;
    typedef robot_interfaces::Status Status;
    typedef robot_interfaces::RobotBackend<Action, Observation> Backend;
    typedef robot_interfaces::SingleProcessRobotData<Action, Observation> Data;
    typedef robot_interfaces::RobotFrontend<Action, Observation> Frontend;

    std::shared_ptr<example::Driver> driver;
    std::shared_ptr<Data> data;

    void SetUp() override
    {
        driver = std::make_shared<example::Driver>(0, 1000);
        data = std::make_shared<Data>();
    }
};

// Test if the "max_number_of_actions" feature is working as expected
TEST_F(TestRobotBackend, max_number_of_actions)
{
    constexpr bool real_time_mode = true;
    constexpr double first_action_timeout =
        std::numeric_limits<double>::infinity();
    constexpr uint32_t max_number_of_actions = 10;

    Backend backend(driver,
                    data,
                    real_time_mode,
                    first_action_timeout,
                    max_number_of_actions);
    backend.initialize();
    Frontend frontend(data);

    Action action;
    action.values[0] = 42;
    action.values[1] = 42;

    robot_interfaces::TimeIndex t;
    for (uint32_t i = 0; i < max_number_of_actions; i++)
    {
        t = frontend.append_desired_action(action);
        Status status = frontend.get_status(t);

        ASSERT_FALSE(status.has_error());
    }

    auto status = frontend.get_status(t + 1);
    ASSERT_TRUE(status.has_error());
    ASSERT_EQ(Status::ErrorStatus::BACKEND_ERROR, status.error_status);
    ASSERT_EQ("Maximum number of actions reached.", status.error_message);
}
