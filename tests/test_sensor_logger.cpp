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
    constexpr int num_observations = 20;
    // write the log
    {
        auto data = std::make_shared<SingleProcessSensorData<int>>();
        auto driver =
            std::make_shared<robot_interfaces::testing::DummySensorDriver>();
        auto frontend = SensorFrontend<int>(data);
        auto logger = SensorLogger<int>(data);
        logger.start();

        // create backend last to ensure no message is missed
        auto backend = SensorBackend<int>(driver, data);

        for (int t = 0; t < num_observations; t++)
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
        ASSERT_GE(log.data.size(), num_observations);
        for (int t = 0; t < num_observations; t++)
        {
            ASSERT_EQ(log.data[t], t);
        }
    }
}

