/**
 * @file
 * @brief Tests for RobotLogger
 * @copyright Copyright (c) 2019, Max Planck Gesellschaft.
 */
#include <gtest/gtest.h>
#include <filesystem>

#include <robot_interfaces/example.hpp>
#include <robot_interfaces/robot_backend.hpp>
#include <robot_interfaces/robot_frontend.hpp>
#include <robot_interfaces/robot_log_reader.hpp>
#include <robot_interfaces/robot_logger.hpp>

using namespace robot_interfaces;

/**
 * @brief Fixture for the backend tests.
 */
class TestRobotLogger : public ::testing::Test
{
protected:
    typedef example::Action Action;
    typedef example::Observation Observation;
    typedef robot_interfaces::Status Status;
    typedef robot_interfaces::RobotBackend<Action, Observation> Backend;
    typedef robot_interfaces::SingleProcessRobotData<Action, Observation> Data;
    typedef robot_interfaces::RobotFrontend<Action, Observation> Frontend;
    typedef robot_interfaces::RobotLogger<Action, Observation> Logger;
    typedef robot_interfaces::RobotBinaryLogReader<Action, Observation>
        BinaryLogReader;

    static constexpr bool real_time_mode = false;
    static constexpr double first_action_timeout =
        std::numeric_limits<double>::infinity();
    static constexpr uint32_t max_number_of_actions = 10;

    std::shared_ptr<example::Driver> driver;
    std::shared_ptr<Data> data;
    std::shared_ptr<Backend> backend;
    std::shared_ptr<Frontend> frontend;
    std::shared_ptr<Logger> logger;

    std::filesystem::path logfile;

    void SetUp() override
    {
        driver = std::make_shared<example::Driver>(0, 1000);
        data = std::make_shared<Data>();

        backend = std::make_shared<Backend>(driver,
                                            data,
                                            real_time_mode,
                                            first_action_timeout,
                                            max_number_of_actions);
        frontend = std::make_shared<Frontend>(data);
        logger = std::make_shared<Logger>(data);

        auto tmp_dir = std::filesystem::temp_directory_path();
        logfile = tmp_dir / "test_robot_logger.dat";
    }

    void TearDown() override
    {
        // clean up
        std::filesystem::remove(logfile);
    }
};

TEST_F(TestRobotLogger, write_current_buffer_binary)
{
    backend->initialize();

    Action action;
    action.values[0] = 42;
    action.values[1] = 42;

    robot_interfaces::TimeIndex t;
    for (uint32_t i = 0; i < max_number_of_actions; i++)
    {
        action.values[0] = i;
        t = frontend->append_desired_action(action);
        frontend->get_observation(t);
    }
    logger->write_current_buffer_binary(logfile);

    // read the log for verification
    BinaryLogReader log(logfile);
    // TODO: why is this failing? (last step is missing in log)
    // ASSERT_EQ(log.data.size(), max_number_of_actions);
    for (uint32_t i = 0; i < log.data.size(); i++)
    {
        ASSERT_EQ(log.data[i].desired_action.values[0], i);
        ASSERT_EQ(log.data[i].desired_action.values[1], 42);
    }
}
