
#include <eigen3/Eigen/Core>
#include <gtest/gtest.h>

#include <math.h>
#include <Eigen/Dense>



#include "robot_interfaces/finger.hpp"

using namespace robot_interfaces;

double epsilon = 1e-10;




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
    for(size_t t = 0; t < 10000; t++)
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



        EXPECT_TRUE(contains(constrained_dynamics.find_t_given_velocity(
                                 constrained_dynamics.get_velocity(t)), t));


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


TEST(linear_dynamics_with_acceleration_constraint,
     will_exceed_jointly)
{
    LinearDynamicsWithAccelerationConstraint
            dynamics(-0.4, 2.6, 200.3, 7.2, 3);
    double max_t = dynamics.find_t_given_velocity(0).maxCoeff() * 2;

    std::vector<Eigen::Vector2d> trajectory;
    size_t n_iterations = 1000;
    for(size_t i = 0; i < n_iterations; i++)
    {
        double t = double(i) / n_iterations * max_t;

        Eigen::Vector2d point;
        point[0] = dynamics.get_velocity(t);
        point[1] = dynamics.get_position(t);
        trajectory.push_back(point);
    }

    for(auto& point : trajectory)
    {
        std::vector<Eigen::Vector2d> constraints;
        constraints.push_back(point + Eigen::Vector2d(0.001, 0.001));
        constraints.push_back(point - Eigen::Vector2d(epsilon, epsilon));

        constraints.push_back(point + Eigen::Vector2d(-epsilon, std::numeric_limits<double>::infinity()));
        constraints.push_back(point + Eigen::Vector2d(-epsilon, -std::numeric_limits<double>::infinity()));
        constraints.push_back(point + Eigen::Vector2d(0.001, -std::numeric_limits<double>::infinity()));

        constraints.push_back(point + Eigen::Vector2d(std::numeric_limits<double>::infinity(), -epsilon));
        constraints.push_back(point + Eigen::Vector2d(-std::numeric_limits<double>::infinity(), -epsilon));
        constraints.push_back(point + Eigen::Vector2d(-std::numeric_limits<double>::infinity(), 0.001));

        for(auto& constraint : constraints)
        {

            bool label = false;
            for(auto& point : trajectory)
            {
                if(point[0] > constraint[0] && point[1] > constraint[1])
                {
                    //                    std::cout << "point: " << point.transpose() << std::endl;
                    label = true;
                    break;
                }
            }

            double certificate_time;
            bool assigned_label =
                    dynamics.will_exceed_jointly(constraint[0], constraint[1],
                    certificate_time);

            if(label != assigned_label)
            {
                std::cout << "---------------------" << std::endl;
                std::cout << "constraint: " << constraint.transpose() << std::endl;

                std::cout << "label: " << label << "  assigned_label: " << assigned_label << std::endl;
                std::cout << "certificate time: " << certificate_time
                          << " certificate point: "
                          << dynamics.get_velocity(certificate_time) << ", "
                          << dynamics.get_position(certificate_time) << std::endl;
            }
            EXPECT_TRUE(label == assigned_label);
        }
    }
}



TEST(linear_dynamics_with_acceleration_constraint,
     will_exceed_jointly_2)
{
    LinearDynamicsWithAccelerationConstraint
            dynamics(-0.4, 20.6, -2.3, 7.2, 30);
    double max_t = dynamics.find_t_given_velocity(0).maxCoeff() * 2;

    std::vector<Eigen::Vector2d> trajectory;
    size_t n_iterations = 1000;
    for(size_t i = 0; i < n_iterations; i++)
    {
        double t = double(i) / n_iterations * max_t;

        Eigen::Vector2d point;
        point[0] = dynamics.get_velocity(t);
        point[1] = dynamics.get_position(t);
        trajectory.push_back(point);
    }

    for(auto& point : trajectory)
    {
        std::vector<Eigen::Vector2d> constraints;
        constraints.push_back(point + Eigen::Vector2d(0.001, 0.001));
        constraints.push_back(point - Eigen::Vector2d(epsilon, epsilon));

        constraints.push_back(point + Eigen::Vector2d(-epsilon, std::numeric_limits<double>::infinity()));
        constraints.push_back(point + Eigen::Vector2d(-epsilon, -std::numeric_limits<double>::infinity()));
        constraints.push_back(point + Eigen::Vector2d(0.001, -std::numeric_limits<double>::infinity()));

        constraints.push_back(point + Eigen::Vector2d(std::numeric_limits<double>::infinity(), -epsilon));
        constraints.push_back(point + Eigen::Vector2d(-std::numeric_limits<double>::infinity(), -epsilon));
        constraints.push_back(point + Eigen::Vector2d(-std::numeric_limits<double>::infinity(), 0.001));

        for(auto& constraint : constraints)
        {

            bool label = false;
            for(auto& point : trajectory)
            {
                if(point[0] > constraint[0] && point[1] > constraint[1])
                {
                    //                    std::cout << "point: " << point.transpose() << std::endl;
                    label = true;
                    break;
                }
            }

            double certificate_time;
            bool assigned_label =
                    dynamics.will_exceed_jointly(constraint[0], constraint[1],
                    certificate_time);

            if(label != assigned_label)
            {
                std::cout << "---------------------" << std::endl;
                std::cout << "constraint: " << constraint.transpose() << std::endl;

                std::cout << "label: " << label << "  assigned_label: " << assigned_label << std::endl;
                std::cout << "certificate time: " << certificate_time
                          << " certificate point: "
                          << dynamics.get_velocity(certificate_time) << ", "
                          << dynamics.get_position(certificate_time) << std::endl;
            }
            EXPECT_TRUE(label == assigned_label);
        }
    }
}


TEST(linear_dynamics_with_acceleration_constraint,
     will_deceed_jointly)
{
    LinearDynamicsWithAccelerationConstraint
            dynamics(0.4, 2.6, -200.3, 7.2, 3);
    double max_t = dynamics.find_t_given_velocity(0).maxCoeff() * 2;

    std::vector<Eigen::Vector2d> trajectory;
    size_t n_iterations = 1000;
    for(size_t i = 0; i < n_iterations; i++)
    {
        double t = double(i) / n_iterations * max_t;

        Eigen::Vector2d point;
        point[0] = dynamics.get_velocity(t);
        point[1] = dynamics.get_position(t);
        trajectory.push_back(point);
    }

    for(auto& point : trajectory)
    {
        std::vector<Eigen::Vector2d> constraints;
        constraints.push_back(point - Eigen::Vector2d(0.001, 0.001));
        constraints.push_back(point + Eigen::Vector2d(epsilon, epsilon));

        constraints.push_back(point + Eigen::Vector2d(epsilon, std::numeric_limits<double>::infinity()));
        constraints.push_back(point + Eigen::Vector2d(epsilon, -std::numeric_limits<double>::infinity()));
        constraints.push_back(point + Eigen::Vector2d(-0.001, std::numeric_limits<double>::infinity()));

        constraints.push_back(point + Eigen::Vector2d(std::numeric_limits<double>::infinity(), epsilon));
        constraints.push_back(point + Eigen::Vector2d(-std::numeric_limits<double>::infinity(), epsilon));
        constraints.push_back(point + Eigen::Vector2d(std::numeric_limits<double>::infinity(), -0.001));

        for(auto& constraint : constraints)
        {

            bool label = false;
            for(auto& point : trajectory)
            {
                if(point[0] < constraint[0] && point[1] < constraint[1])
                {
                    //                    std::cout << "point: " << point.transpose() << std::endl;
                    label = true;
                    break;
                }
            }

            double certificate_time;
            bool assigned_label =
                    dynamics.will_deceed_jointly(constraint[0], constraint[1],
                    certificate_time);

            if(label != assigned_label)
            {
                std::cout << "---------------------" << std::endl;
                std::cout << "constraint: " << constraint.transpose() << std::endl;

                std::cout << "label: " << label << "  assigned_label: " << assigned_label << std::endl;
                std::cout << "certificate time: " << certificate_time
                          << " certificate point: "
                          << dynamics.get_velocity(certificate_time) << ", "
                          << dynamics.get_position(certificate_time) << std::endl;
            }
            EXPECT_TRUE(label == assigned_label);
        }
    }
}

double sample_uniformely(const double& min, const double& max)
{
    return min + double(rand()) / RAND_MAX * (max - min);
}

TEST(find_max_admissible_initial_acceleration, generated_trajectories)
{
    srand(0);

    for(size_t unused = 0; unused < 100; unused++)
    {
        // initialize parameters randomly --------------------------------------


        double initial_velocity = sample_uniformely(-20.0, 20.0);
        double initial_position = sample_uniformely(-20.0, 20.0);
        NonnegDouble abs_jerk_limit = sample_uniformely(epsilon, 20.0);
        NonnegDouble abs_acceleration_limit = sample_uniformely(epsilon, 20.0);

        double initial_acceleration = sample_uniformely(-abs_acceleration_limit,
                                                        abs_acceleration_limit);



        // find some constraints which is just barely satisfied --------------------
        LinearDynamicsWithAccelerationConstraint dynamics(-abs_jerk_limit,
                                                          initial_acceleration,
                                                          initial_velocity,
                                                          initial_position,
                                                          abs_acceleration_limit);

        double max_t = 20.0;
        if(dynamics.find_t_given_velocity(0).size() > 0)
        {
            max_t = dynamics.find_t_given_velocity(0).maxCoeff() * 2;
        }

        size_t n_iterations = 10000;
        int T = rand() % n_iterations;
        Eigen::Vector2d constraint;
        constraint[0] = dynamics.get_velocity(T);
        constraint[1] = dynamics.get_position(T);

        Eigen::Vector2d position_constraint(-std::numeric_limits<double>::infinity(),
                                            -std::numeric_limits<double>::infinity());
        Eigen::Vector2d velocity_constraint(-std::numeric_limits<double>::infinity(),
                                            -std::numeric_limits<double>::infinity());

        for(size_t i = 0; i < n_iterations; i++)
        {
            double t = double(i) / n_iterations * max_t;
            Eigen::Vector2d point;
            point[0] = dynamics.get_velocity(t);
            point[1] = dynamics.get_position(t);
            if(point[0] > constraint[0] && point[1] > constraint[1])
                constraint = point;
            velocity_constraint[0] = std::max(velocity_constraint[0], point[0]);
            position_constraint[1] = std::max(position_constraint[1], point[1]);
        }

        // test that we get the same initial acceleration --------------------------
        for(auto& c : {constraint, position_constraint, velocity_constraint})
        {


            double max_admissible_initial_acceleration =
                    find_max_admissible_initial_acceleration(
                        initial_velocity,
                        initial_position,
                        c[0],
                    c[1],
                    abs_jerk_limit,
                    abs_acceleration_limit);

            if(std::fabs(initial_acceleration -
                         max_admissible_initial_acceleration) > 0.02)
            {
                if(max_admissible_initial_acceleration - 0.0001 >
                        -abs_acceleration_limit)
                {
                    LinearDynamicsWithAccelerationConstraint
                            below_limit_dynamics(-abs_jerk_limit,
                                                 max_admissible_initial_acceleration
                                                 - 0.0001,
                                                 initial_velocity,
                                                 initial_position,
                                                 abs_acceleration_limit);
                    ASSERT_TRUE(below_limit_dynamics.will_exceed_jointly(c[0], c[1])
                            == false);
                }


                if(max_admissible_initial_acceleration + 0.0001 <
                        abs_acceleration_limit)
                {
                    LinearDynamicsWithAccelerationConstraint
                            above_limit_dynamics(-abs_jerk_limit,
                                                 max_admissible_initial_acceleration
                                                 + 0.0001,
                                                 initial_velocity,
                                                 initial_position,
                                                 abs_acceleration_limit);
                    ASSERT_TRUE(above_limit_dynamics.will_exceed_jointly(c[0], c[1])
                            == true);
                }


                //                std::cout << "------------------------------------  " << std::endl;


                //                dynamics.print_parameters();

                //                std::cout << "constraint " << c << std::endl;

                //                std::cout << " initial_acceleration " << initial_acceleration
                //                          << " max_admissible_initial_acceleration "
                //                          << max_admissible_initial_acceleration << std::endl;

                //                bool will_exceed_epsilon = dynamics.will_exceed_jointly(c[0], c[1]);

                //                bool will_exceed_minus_epsilon = dynamics.will_exceed_jointly(c[0] - epsilon, c[1] - epsilon);


                //                bool limit_dynamics_will_exceed = limit_dynamics.will_exceed_jointly(c[0] - epsilon, c[1] - epsilon);



                //                std::cout << " will_exceed_epsilon " << will_exceed_epsilon
                //                          << " will_exceed_minus_epsilon " << will_exceed_minus_epsilon
                //                          << " limit_dynamics_will_exceed " << limit_dynamics_will_exceed

                //                          << std::endl;

                ASSERT_TRUE(approx_equal(c[0], initial_velocity) ||
                        approx_equal(c[1], initial_position));
            }

            //            ASSERT_TRUE(std::fabs(initial_acceleration -
            //                                  max_admissible_initial_acceleration) <= 0.001);
        }
    }
}


TEST(find_max_admissible_initial_acceleration, random_points)
{
    srand(0);

    for(size_t unused = 0; unused < 10000; unused++)
    {
        // initialize parameters randomly --------------------------------------
        double initial_velocity = sample_uniformely(-20.0, 20.0);
        double initial_position = sample_uniformely(-20.0, 20.0);

        double max_velocity = sample_uniformely(-20.0, 20.0);
        double max_position = sample_uniformely(-20.0, 20.0);

        NonnegDouble abs_jerk_limit = sample_uniformely(epsilon, 20.0);
        NonnegDouble abs_acceleration_limit = sample_uniformely(epsilon, 20.0);


        double max_admissible_acceleration =
                find_max_admissible_initial_acceleration(
                    initial_velocity,
                    initial_position,
                    max_velocity,
                    max_position,
                    abs_jerk_limit,
                    abs_acceleration_limit);

            if(max_admissible_acceleration - 0.0001 >
                    -abs_acceleration_limit)
            {
                LinearDynamicsWithAccelerationConstraint
                        below_limit_dynamics(-abs_jerk_limit,
                                             max_admissible_acceleration
                                             - 0.0001,
                                             initial_velocity,
                                             initial_position,
                                             abs_acceleration_limit);
                ASSERT_TRUE(below_limit_dynamics.will_exceed_jointly(max_velocity, max_position)
                        == false);
            }
            if(max_admissible_acceleration + 0.0001 <
                    abs_acceleration_limit)
            {
                LinearDynamicsWithAccelerationConstraint
                        above_limit_dynamics(-abs_jerk_limit,
                                             max_admissible_acceleration
                                             + 0.0001,
                                             initial_velocity,
                                             initial_position,
                                             abs_acceleration_limit);
                ASSERT_TRUE(above_limit_dynamics.will_exceed_jointly(max_velocity, max_position)
                        == true);
            }
        }
    }




