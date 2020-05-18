/**
 * @file
 * @brief Basic functionality of sensor interface.
 * @copyright Copyright (c) 2019, Max Planck Gesellschaft.
 */
#include <gtest/gtest.h>

#include <robot_interfaces/sensors/sensor_backend.hpp>
#include <robot_interfaces/sensors/sensor_data.hpp>
#include <robot_interfaces/sensors/sensor_frontend.hpp>

#include "dummy_sensor_driver.hpp"

using namespace robot_interfaces;

// test if getting observations through the frontend works as expected
TEST(TestSensorInterface, basic_pipeline)
{
    auto data = std::make_shared<SingleProcessSensorData<int>>();
    auto driver =
        std::make_shared<robot_interfaces::testing::DummySensorDriver>();
    auto frontend = SensorFrontend<int>(data);
    auto backend = SensorBackend<int>(driver, data);

    for (int t = 0; t < 20; t++)
    {
        int obs = frontend.get_observation(t);
        ASSERT_EQ(obs, t);
    }
}

