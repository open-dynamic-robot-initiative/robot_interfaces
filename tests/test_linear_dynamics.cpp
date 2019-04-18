
#include <eigen3/Eigen/Core>
#include <gtest/gtest.h>

#include <math.h>
#include <Eigen/Dense>



#include "robot_interfaces/finger.hpp"

using namespace robot_interfaces;

double epsilon = 1e-10;

bool approx_equal(double x, double y)
{
    return (std::fabs(x - y) < epsilon);
}

template<typename Vector>
bool contains(Vector v, double x)
{
    for(size_t i = 0; i < v.size(); i++)
    {
        if(approx_equal(v[i], x))
        {
            return true;
        }
    }
    return false;
}



TEST(linear_dynamics, evolution_with_zero_initial_position)
{
    double jerk = 1.0;
    LinearDynamics linear_dynamics(jerk, 0.0, 0.0, 0.0);

    for(size_t t = 0; t < 100; t++)
    {
        EXPECT_TRUE(approx_equal(linear_dynamics.get_acceleration(t),
                                 t * jerk));
        EXPECT_TRUE(approx_equal(linear_dynamics.get_velocity(t),
                                 0.5 * t * t * jerk));
        EXPECT_TRUE(approx_equal(linear_dynamics.get_position(t),
                                 0.5 / 3.0 * t * t * t * jerk));
    }
}

// should be linear in initial state and control
TEST(linear_dynamics, check_linearity)
{
    Eigen::Vector4d parameters_a = Eigen::Vector4d(3.4, -5.6, 2.3, 7.2);
    Eigen::Vector4d parameters_b = Eigen::Vector4d(-2.1, -3.0, 5.5, -10.2);

    LinearDynamics linear_dynamics_a(parameters_a);
    LinearDynamics linear_dynamics_b(parameters_b);
    LinearDynamics linear_dynamics_sum(parameters_a + parameters_b);

    for(size_t t = 0; t < 100; t++)
    {
        EXPECT_TRUE(approx_equal(linear_dynamics_a.get_acceleration(t) +
                                 linear_dynamics_b.get_acceleration(t),
                                 linear_dynamics_sum.get_acceleration(t)));
        EXPECT_TRUE(approx_equal(linear_dynamics_a.get_velocity(t) +
                                 linear_dynamics_b.get_velocity(t),
                                 linear_dynamics_sum.get_velocity(t)));
        EXPECT_TRUE(approx_equal(linear_dynamics_a.get_position(t) +
                                 linear_dynamics_b.get_position(t),
                                 linear_dynamics_sum.get_position(t)));
    }
}


TEST(linear_dynamics, test_find_t_given_velocity_basic)
{
    Eigen::Vector4d parameters_basic = Eigen::Vector4d(1.0, -2.0, 0.0, 0.0);
    LinearDynamics dynamics_basic(parameters_basic);

    LinearDynamics::Vector solutions =
            dynamics_basic.find_t_given_velocity(- 3.0 / 2.0);

    EXPECT_TRUE(solutions.size() == 2);
    EXPECT_TRUE(contains(solutions, 1));
    EXPECT_TRUE(contains(solutions, 3));
    EXPECT_FALSE(contains(solutions, 4));
}


TEST(linear_dynamics, test_find_t_given_velocity_consistency)
{
    Eigen::Vector4d parameters_a = Eigen::Vector4d(3.4, -5.6, 2.3, 7.2);
    Eigen::Vector4d parameters_b = Eigen::Vector4d(-2.1, -3.0, 5.5, -10.2);
    LinearDynamics linear_dynamics_a(parameters_a);
    LinearDynamics linear_dynamics_b(parameters_b);

    for(size_t t = 0; t < 100; t++)
    {
        auto solutions_a = linear_dynamics_a.find_t_given_velocity(
                    linear_dynamics_a.get_velocity(t));
        EXPECT_TRUE(contains(solutions_a, t));
        auto solutions_b = linear_dynamics_b.find_t_given_velocity(
                    linear_dynamics_b.get_velocity(t));
        EXPECT_TRUE(contains(solutions_b, t));
    }
}



TEST(linear_dynamics_with_acceleration_constraint, test_time_evolution)
{
    Eigen::Matrix<double, 5, 1> parameters;
    parameters << -2.0, 15.0, 5.5, -10.2, 17.0;
    double jerk_duration = (17.0 + 15.0) / 2.0;


    LinearDynamicsWithAccelerationConstraint constrained_dynamics(parameters);
    LinearDynamics dynamics(parameters.topRows(4));
    LinearDynamicsWithAccelerationConstraint
            constrained_dynamics_b(-2.0,
                                   dynamics.get_acceleration(jerk_duration),
                                   dynamics.get_velocity(jerk_duration),
                                   dynamics.get_position(jerk_duration), 17.0);

    EXPECT_TRUE(approx_equal(dynamics.get_acceleration(jerk_duration), -17));
    EXPECT_TRUE(approx_equal(-17, constrained_dynamics.get_acceleration(
                                 jerk_duration + 100)));
    EXPECT_TRUE(approx_equal(dynamics.get_position(jerk_duration),
                             constrained_dynamics_b.get_position(0)));


    // make sure that ther is no discontinuity at jerk_duration ----------------
    double large_epsilon = std::sqrt(epsilon) / 100.0;
    double numeric, exact;

    numeric = constrained_dynamics.get_position(jerk_duration)
            + large_epsilon * constrained_dynamics.get_velocity(jerk_duration);
    exact = constrained_dynamics.get_position(jerk_duration + large_epsilon);
    ASSERT_TRUE(approx_equal(numeric, exact));

    numeric = constrained_dynamics.get_position(jerk_duration)
            - large_epsilon * constrained_dynamics.get_velocity(jerk_duration);
    exact = constrained_dynamics.get_position(jerk_duration - large_epsilon);
    ASSERT_TRUE(approx_equal(numeric, exact));

    // make sure dynamics coincide
    for(size_t t = 0; t < 100; t++)
    {
        double acceleration_b = dynamics.get_acceleration(jerk_duration);
        double velocity_b =
                dynamics.get_velocity(jerk_duration) +
                dynamics.get_acceleration(jerk_duration) * t;
        double position_b =
                dynamics.get_position(jerk_duration) +
                dynamics.get_velocity(jerk_duration) * t +
                dynamics.get_acceleration(jerk_duration) * 0.5 * pow(t, 2);
        EXPECT_TRUE(approx_equal(acceleration_b,
                                 constrained_dynamics_b.get_acceleration(t)));
        EXPECT_TRUE(approx_equal(velocity_b,
                                 constrained_dynamics_b.get_velocity(t)));
        EXPECT_TRUE(approx_equal(position_b,
                                 constrained_dynamics_b.get_position(t)));


        if(t <= jerk_duration)
        {
            EXPECT_TRUE(approx_equal(dynamics.get_acceleration(t),
                                     constrained_dynamics.get_acceleration(t)));
            EXPECT_TRUE(approx_equal(dynamics.get_velocity(t),
                                     constrained_dynamics.get_velocity(t)));
            EXPECT_TRUE(approx_equal(dynamics.get_position(t),
                                     constrained_dynamics.get_position(t)));
        }
        else
        {
            EXPECT_TRUE(approx_equal(constrained_dynamics_b.get_acceleration(t - jerk_duration),
                                     constrained_dynamics.get_acceleration(t)));
            EXPECT_TRUE(approx_equal(constrained_dynamics_b.get_velocity(t - jerk_duration),
                                     constrained_dynamics.get_velocity(t)));
            EXPECT_TRUE(approx_equal(constrained_dynamics_b.get_position(t - jerk_duration),
                                     constrained_dynamics.get_position(t)));
        }
    }
}

TEST(linear_dynamics_with_acceleration_constraint, consistency_with_linear_dynamics)
{

    LinearDynamicsWithAccelerationConstraint
            constrained_dynamics(-1.5, -13.0, 5.5, 10.2, 13.0);
    LinearDynamics  dynamics(0.0, -13.0, 5.5, 10.2);

    // make sure dynamics coincide
    for(size_t t = 0; t < 100; t++)
    {

        EXPECT_TRUE(approx_equal(dynamics.get_acceleration(t),
                                 constrained_dynamics.get_acceleration(t)));
        EXPECT_TRUE(approx_equal(dynamics.get_velocity(t),
                                 constrained_dynamics.get_velocity(t)));
        EXPECT_TRUE(approx_equal(dynamics.get_position(t),
                                 constrained_dynamics.get_position(t)));

        LinearDynamics::Vector solutions =
                dynamics.find_t_given_velocity(t - 50.0);
        LinearDynamics::Vector constrained_solutions =
                constrained_dynamics.find_t_given_velocity(t - 50.0);

        EXPECT_TRUE(solutions.size() == constrained_solutions.size());

        for(size_t i = 0; i < solutions.size(); i++)
        {
            EXPECT_TRUE(contains(solutions, constrained_solutions[i]));
            EXPECT_TRUE(contains(constrained_solutions, solutions[i]));
            EXPECT_TRUE(approx_equal(solutions[i], - (t - 50.0 -5.5) / 13.0));
        }
    }
}

TEST(linear_dynamics_with_acceleration_constraint,
     test_find_t_given_velocity_basic_1)
{
    LinearDynamicsWithAccelerationConstraint
            dynamics_basic(1.0, -2.0, 0.0, 0.0, 100.0);

    LinearDynamics::Vector solutions =
            dynamics_basic.find_t_given_velocity(- 3.0 / 2.0);

    EXPECT_TRUE(solutions.size() == 2);
    EXPECT_TRUE(contains(solutions, 1));
    EXPECT_TRUE(contains(solutions, 3));
    EXPECT_FALSE(contains(solutions, 4));
}


TEST(linear_dynamics_with_acceleration_constraint,
     test_find_t_given_velocity_basic_2)
{
    LinearDynamicsWithAccelerationConstraint
            dynamics_basic(1.0, -5.0, 3.0, -10.0, 10.0);

    LinearDynamics::Vector solutions =
            dynamics_basic.find_t_given_velocity(-9);

    EXPECT_TRUE(solutions.size() == 2);
    EXPECT_TRUE(contains(solutions, 4));
    EXPECT_TRUE(contains(solutions, 6));
    EXPECT_FALSE(contains(solutions, 7));
}


TEST(linear_dynamics_with_acceleration_constraint,
     test_find_t_given_velocity_basic_3)
{
    LinearDynamicsWithAccelerationConstraint
            dynamics(1.0, -1.0, 3.0, -2.0, 2.0);

    EXPECT_TRUE(approx_equal(dynamics.get_velocity(10.0), 18.5));

    LinearDynamics::Vector solutions = dynamics.find_t_given_velocity(18.5);

    EXPECT_TRUE(solutions.size() == 1);
    EXPECT_TRUE(approx_equal(solutions[0], 10.0));
}


TEST(linear_dynamics_with_acceleration_constraint,
     test_find_t_given_velocity_consistency)
{
    LinearDynamicsWithAccelerationConstraint
            dynamics_a(3.4, -5.6, 2.3, 7.2, 20);
    LinearDynamicsWithAccelerationConstraint
            dynamics_b(-2.1, -3.0, 5.5, -10.2, 44.3);

    for(size_t t = 0; t < 100; t++)
    {
        auto solutions_a = dynamics_a.find_t_given_velocity(
                    dynamics_a.get_velocity(t));
        EXPECT_TRUE(contains(solutions_a, t));
        auto solutions_b = dynamics_b.find_t_given_velocity(
                    dynamics_b.get_velocity(t));
        EXPECT_TRUE(contains(solutions_b, t));
    }
}


