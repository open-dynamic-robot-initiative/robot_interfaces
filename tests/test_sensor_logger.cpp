/**
 * @file
 * @brief Basic functionality of sensor interface.
 * @copyright Copyright (c) 2019, Max Planck Gesellschaft.
 */
#include <gtest/gtest.h>
#include <cstdio>

#include <robot_interfaces/sensors/sensor_backend.hpp>
#include <robot_interfaces/sensors/sensor_data.hpp>
#include <robot_interfaces/sensors/sensor_frontend.hpp>
#include <robot_interfaces/sensors/sensor_log_reader.hpp>
#include <robot_interfaces/sensors/sensor_logger.hpp>

#include "dummy_sensor_driver.hpp"

using namespace robot_interfaces;

//! Test fixture to create and delete a temporary log file
class TestSensorLogger : public ::testing::Test
{
protected:
    std::string log_file;

    void SetUp() override
    {
        log_file = std::tmpnam(nullptr);
    }

    void TearDown() override
    {
        // clean up
        std::remove(log_file.c_str());
    }
};

// test if writing and reading the sensor log file is working correctly
TEST_F(TestSensorLogger, write_and_read_log)
{
    constexpr int NUM_OBSERVATIONS = 20;
    constexpr int BUFFER_LIMIT = 50;

    // write the log
    {
        auto data = std::make_shared<SingleProcessSensorData<int>>();
        auto driver =
            std::make_shared<robot_interfaces::testing::DummySensorDriver>();
        auto frontend = SensorFrontend<int>(data);
        auto logger = SensorLogger<int>(data, BUFFER_LIMIT);
        logger.start();

        // create backend last to ensure no message is missed
        auto backend = SensorBackend<int>(driver, data);

        for (int t = 0; t < NUM_OBSERVATIONS; t++)
        {
            int obs = frontend.get_observation(t);
            // ensure that the observations written to the log are actually what
            // we expect
            ASSERT_EQ(obs, t);
        }
        logger.stop_and_save(log_file);
    }

    // read the log
    {
        auto log = SensorLogReader<int>(log_file);
        ASSERT_GE(log.data.size(), NUM_OBSERVATIONS);
        for (int t = 0; t < NUM_OBSERVATIONS; t++)
        {
            ASSERT_EQ(log.data[t], t);
        }
    }
}

TEST_F(TestSensorLogger, buffer_limit)
{
    constexpr int NUM_OBSERVATIONS = 20;
    constexpr int BUFFER_LIMIT = 10;

    // write the log
    {
        auto data = std::make_shared<SingleProcessSensorData<int>>();
        auto driver =
            std::make_shared<robot_interfaces::testing::DummySensorDriver>();
        auto frontend = SensorFrontend<int>(data);

        // set buffer
        auto logger = SensorLogger<int>(data, BUFFER_LIMIT);
        logger.start();

        // create backend last to ensure no message is missed
        auto backend = SensorBackend<int>(driver, data);

        for (int t = 0; t < NUM_OBSERVATIONS; t++)
        {
            int obs = frontend.get_observation(t);
            // ensure that the observations written to the log are actually what
            // we expect
            ASSERT_EQ(obs, t);
        }
        logger.stop_and_save(log_file);
    }

    // read the log
    {
        auto log = SensorLogReader<int>(log_file);

        // make sure the log size equals the buffer limit
        ASSERT_EQ(log.data.size(), BUFFER_LIMIT);

        // also verify that the messages are the expected ones (i.e. the ones
        // from the start, before the limit is reached).
        for (int t = 0; t < log.data.size(); t++)
        {
            ASSERT_EQ(log.data[t], t);
        }
    }
}
