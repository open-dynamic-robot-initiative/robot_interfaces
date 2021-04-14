/**
 * @file
 * @brief Tests for RobotLogger
 * @copyright Copyright (c) 2019, Max Planck Gesellschaft.
 */
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>

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

        auto tmp_dir = std::filesystem::temp_directory_path();
        logfile = tmp_dir / "test_robot_logger.log";
    }

    void TearDown() override
    {
        // clean up
        std::filesystem::remove(logfile);
    }

    /**
     * @brief Check whether the given CSV log is valid.
     *
     * @param filename  The log file.
     * @param start_t  Expected first time index in the log.
     * @param end_t  Expected last time index in the log.
     * @param use_gzip  If true, assume the file is gzip-compressed.
     */
    void check_csv_log(const std::string &filename,
                       uint32_t start_t,
                       uint32_t end_t,
                       bool use_gzip)
    {
        std::string line;

        // open file, potentially with gzip decompression
        std::ifstream infile_raw(filename);
        boost::iostreams::filtering_istream infile;
        if (use_gzip)
        {
            infile.push(boost::iostreams::gzip_decompressor());
        }
        infile.push(infile_raw);

        // check header
        {
            ASSERT_TRUE(std::getline(infile, line))
                << "Failed to read header line";

            // remove trailing whitespaces
            line.erase(line.find_last_not_of(" \n") + 1);

            ASSERT_EQ(
                line,
                "#time_index timestamp status_action_repetitions "
                "status_error_status observation_values_0 observation_values_1 "
                "applied_action_values_0 applied_action_values_1 "
                "desired_action_values_0 desired_action_values_1");
        }

        // check data
        for (uint32_t i = start_t; i < end_t; i++)
        {
            int time_index;
            double time_stamp, action_repetitions, error_status, obs_0, obs_1,
                applied_0, applied_1, desired_0, desired_1;

            ASSERT_TRUE(std::getline(infile, line))
                << "Failed to read line " << i;
            std::istringstream iss(line);

            ASSERT_TRUE(iss >> time_index >> time_stamp >> action_repetitions >>
                        error_status >> obs_0 >> obs_1 >> applied_0 >>
                        applied_1 >> desired_0 >> desired_1);

            // check some of the values
            ASSERT_EQ(time_index, i);
            ASSERT_EQ(desired_0, i);
            ASSERT_EQ(desired_1, 42);
        }
    }
};

TEST_F(TestRobotLogger, save_current_robot_data)
{
    Logger logger(data, 0);

    backend->initialize();

    Action action;
    action.values[0] = 42;
    action.values[1] = 42;

    robot_interfaces::TimeIndex t;
    for (uint32_t i = 0; i < max_number_of_actions; i++)
    {
        action.values[0] = i;
        t = frontend->append_desired_action(action);
        // wait for applied action to ensure all data of that time step is there
        frontend->get_applied_action(t);
    }

    // TODO: Why is the sleep needed here?
    // wait a moment to give the logger time to catch up
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    logger.save_current_robot_data(logfile);

    check_csv_log(logfile, 0, max_number_of_actions, false);
}

TEST_F(TestRobotLogger, save_current_robot_data_binary)
{
    Logger logger(data, 0);

    backend->initialize();

    Action action;
    action.values[0] = 42;
    action.values[1] = 42;

    robot_interfaces::TimeIndex t;
    for (uint32_t i = 0; i < max_number_of_actions; i++)
    {
        action.values[0] = i;
        t = frontend->append_desired_action(action);
        // wait for applied action to ensure all data of that time step is there
        frontend->get_applied_action(t);
    }
    logger.save_current_robot_data_binary(logfile);

    // read the log for verification
    BinaryLogReader log(logfile);
    ASSERT_EQ(log.data.size(), max_number_of_actions);
    for (uint32_t i = 0; i < log.data.size(); i++)
    {
        ASSERT_EQ(log.data[i].desired_action.values[0], i);
        ASSERT_EQ(log.data[i].desired_action.values[1], 42);
    }
}

// TODO add another test where time series is smaller than the log
TEST_F(TestRobotLogger, start_stop_binary)
{
    // make sure the logger buffer is large enough
    uint32_t logger_buffer_limit = 2 * max_number_of_actions;
    Logger logger(data, logger_buffer_limit);

    backend->initialize();

    Action action;
    action.values[0] = 42;
    action.values[1] = 42;

    logger.start();

    robot_interfaces::TimeIndex t;
    for (uint32_t i = 0; i < max_number_of_actions; i++)
    {
        action.values[0] = i;
        t = frontend->append_desired_action(action);
        // wait for applied action to ensure all data of that time step is there
        frontend->get_applied_action(t);
    }

    // wait a moment to give the logger time to catch up
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    logger.stop_and_save(logfile, Logger::Format::BINARY);

    // read the log for verification
    BinaryLogReader log(logfile);
    ASSERT_EQ(log.data.size(), max_number_of_actions);
    for (uint32_t i = 0; i < log.data.size(); i++)
    {
        ASSERT_EQ(log.data[i].desired_action.values[0], i);
        ASSERT_EQ(log.data[i].desired_action.values[1], 42);
    }
}

TEST_F(TestRobotLogger, start_stop_csv)
{
    // make sure the logger buffer is large enough
    uint32_t logger_buffer_limit = 2 * max_number_of_actions;
    Logger logger(data, logger_buffer_limit);

    backend->initialize();

    Action action;
    action.values[0] = 42;
    action.values[1] = 42;

    logger.start();

    robot_interfaces::TimeIndex t;
    for (uint32_t i = 0; i < max_number_of_actions; i++)
    {
        action.values[0] = i;
        t = frontend->append_desired_action(action);
        // wait for applied action to ensure all data of that time step is there
        frontend->get_applied_action(t);
    }

    // wait a moment to give the logger time to catch up
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    logger.stop_and_save(logfile, Logger::Format::CSV);

    check_csv_log(logfile, 0, max_number_of_actions, false);
}

TEST_F(TestRobotLogger, start_stop_csv_gzip)
{
    // make sure the logger buffer is large enough
    uint32_t logger_buffer_limit = 2 * max_number_of_actions;
    Logger logger(data, logger_buffer_limit);

    backend->initialize();

    Action action;
    action.values[0] = 42;
    action.values[1] = 42;

    logger.start();

    robot_interfaces::TimeIndex t;
    for (uint32_t i = 0; i < max_number_of_actions; i++)
    {
        action.values[0] = i;
        t = frontend->append_desired_action(action);
        // wait for applied action to ensure all data of that time step is there
        frontend->get_applied_action(t);
    }

    // wait a moment to give the logger time to catch up
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    logger.stop_and_save(logfile, Logger::Format::CSV_GZIP);

    check_csv_log(logfile, 0, max_number_of_actions, true);
}

TEST_F(TestRobotLogger, start_stop_continuous)
{
    // make sure the logger buffer is large enough
    uint32_t logger_buffer_limit = 2 * max_number_of_actions;
    // set a block-size that is smaller than the time series
    uint32_t block_size = 3;
    Logger logger(data, logger_buffer_limit, block_size);

    backend->initialize();

    Action action;
    action.values[0] = 42;
    action.values[1] = 42;

    logger.start_continous_writing(logfile);

    robot_interfaces::TimeIndex t;
    for (uint32_t i = 0; i < max_number_of_actions; i++)
    {
        action.values[0] = i;
        t = frontend->append_desired_action(action);
        // wait for applied action to ensure all data of that time step is there
        frontend->get_applied_action(t);
    }

    // wait a moment to give the logger time to catch up
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    logger.stop_continous_writing();

    check_csv_log(logfile, 0, max_number_of_actions, false);
}

