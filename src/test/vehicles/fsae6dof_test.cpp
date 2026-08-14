#include "gtest/gtest.h"
#include "src/core/vehicles/fsae6dof.h"
#include "src/main/c/fastestlapc.h"
#include <cmath>

using Front_left_tire_type  = fsae6dof<scalar>::Front_left_tire_type;
using Front_right_tire_type = fsae6dof<scalar>::Front_right_tire_type;
using Rear_left_tire_type   = fsae6dof<scalar>::Rear_left_tire_type;
using Rear_right_tire_type  = fsae6dof<scalar>::Rear_right_tire_type;

using Front_axle_t = fsae6dof<scalar>::Front_axle_t;
using Rear_axle_t  = fsae6dof<scalar>::Rear_axle_t;
using Chassis_t    = fsae6dof<scalar>::Chassis_t;
using Road_t       = fsae6dof<scalar>::Road_cartesian_t;

static_assert(Front_axle_t::input_names::KAPPA_LEFT  == 0);
static_assert(Front_axle_t::input_names::KAPPA_RIGHT == 1);
static_assert(Rear_axle_t::input_names::KAPPA_LEFT   == 2);
static_assert(Rear_axle_t::input_names::KAPPA_RIGHT  == 3);
static_assert(Chassis_t::input_names::velocity_x_mps == 4);
static_assert(Chassis_t::input_names::velocity_y_mps == 5);
static_assert(Chassis_t::input_names::yaw_rate_radps == 6);
static_assert(Chassis_t::input_names::Z              == 7);
static_assert(Chassis_t::input_names::PHI            == 8);
static_assert(Chassis_t::input_names::MU             == 9);
static_assert(Chassis_t::input_names::DZDT           == 10);
static_assert(Chassis_t::input_names::DPHIDT         == 11);
static_assert(Chassis_t::input_names::DMUDT          == 12);
static_assert(Road_t::input_names::X                 == 13);
static_assert(Road_t::input_names::Y                 == 14);
static_assert(Road_t::input_names::PSI               == 15);
static_assert(fsae6dof<scalar>::cartesian::number_of_inputs == 16);

static_assert(Front_axle_t::control_names::STEERING == 0);
static_assert(Chassis_t::control_names::throttle   == 1);
static_assert(Chassis_t::control_names::brake_bias == 2);
static_assert(fsae6dof<scalar>::cartesian::number_of_controls == 3);

class fsae6dof_test : public testing::Test
{
 protected:
    Xml_document database = {"./database/vehicles/fsae/ubco-2026-ev.xml", true};
};

TEST_F(fsae6dof_test, indexes)
{
    EXPECT_EQ(Front_axle_t::input_names::KAPPA_LEFT,  0);
    EXPECT_EQ(Front_axle_t::input_names::KAPPA_RIGHT, 1);
    EXPECT_EQ(Rear_axle_t::input_names::KAPPA_LEFT,   2);
    EXPECT_EQ(Rear_axle_t::input_names::KAPPA_RIGHT,  3);
    EXPECT_EQ(Chassis_t::input_names::velocity_x_mps, 4);
    EXPECT_EQ(Chassis_t::input_names::Z,              7);
    EXPECT_EQ(Road_t::input_names::PSI,              15);
    EXPECT_EQ(fsae6dof<scalar>::cartesian::number_of_inputs, 16);
    EXPECT_EQ(Front_axle_t::control_names::STEERING, 0);
    EXPECT_EQ(Chassis_t::control_names::throttle,    1);
    EXPECT_EQ(Chassis_t::control_names::brake_bias,  2);
    EXPECT_EQ(fsae6dof<scalar>::cartesian::number_of_controls, 3);
}

TEST_F(fsae6dof_test, vehicle_from_xml_variable_names)
{
    fsae6dof<double>::cartesian car(database);
    auto [s_name, q_names, u_names] = car.get_state_and_control_names();

    EXPECT_EQ(q_names[Front_axle_t::input_names::KAPPA_LEFT],  "front-axle.left-tire.kappa");
    EXPECT_EQ(q_names[Front_axle_t::input_names::KAPPA_RIGHT], "front-axle.right-tire.kappa");
    EXPECT_EQ(q_names[Rear_axle_t::input_names::KAPPA_LEFT],   "rear-axle.left-tire.kappa");
    EXPECT_EQ(q_names[Rear_axle_t::input_names::KAPPA_RIGHT],  "rear-axle.right-tire.kappa");
    EXPECT_EQ(q_names[Chassis_t::input_names::velocity_x_mps], "chassis.velocity.x");
    EXPECT_EQ(q_names[Chassis_t::input_names::Z],              "chassis.position.z");
    EXPECT_EQ(u_names[Front_axle_t::control_names::STEERING],  "front-axle.steering-angle");
    EXPECT_EQ(u_names[Chassis_t::control_names::throttle],     "chassis.throttle");
    EXPECT_EQ(u_names[Chassis_t::control_names::brake_bias],   "chassis.brake-bias");
}

TEST_F(fsae6dof_test, is_ready)
{
    fsae6dof<double>::cartesian car(database);
    EXPECT_TRUE(car.is_ready());
}

TEST_F(fsae6dof_test, ode_straight_running_is_finite)
{
    fsae6dof<double>::cartesian car(database);

    std::array<scalar, fsae6dof<scalar>::cartesian::number_of_inputs> q{};
    q[Chassis_t::input_names::velocity_x_mps] = 20.0;
    q[Chassis_t::input_names::Z] = 0.02;

    std::array<scalar, fsae6dof<scalar>::cartesian::number_of_controls> u{};
    u[Chassis_t::control_names::brake_bias] = 0.53;

    auto [states, dqdt] = car(q, u, 0.0);

    for (size_t i = 0; i < dqdt.size(); ++i)
        EXPECT_TRUE(std::isfinite(dqdt[i])) << "dqdt[" << i << "] is not finite";

    const auto Fz = car.get_chassis().get_force().z();
    EXPECT_TRUE(std::isfinite(Fz));

    const auto Fz_fl = car.get_chassis().get_front_axle().template get_tire<0>().get_force().z();
    const auto Fz_fr = car.get_chassis().get_front_axle().template get_tire<1>().get_force().z();
    const auto Fz_rl = car.get_chassis().get_rear_axle().template get_tire<0>().get_force().z();
    const auto Fz_rr = car.get_chassis().get_rear_axle().template get_tire<1>().get_force().z();

    EXPECT_TRUE(std::isfinite(Fz_fl));
    EXPECT_TRUE(std::isfinite(Fz_fr));
    EXPECT_TRUE(std::isfinite(Fz_rl));
    EXPECT_TRUE(std::isfinite(Fz_rr));
    EXPECT_LT(Fz_fl, 0.0);
    EXPECT_LT(Fz_fr, 0.0);
    EXPECT_LT(Fz_rl, 0.0);
    EXPECT_LT(Fz_rr, 0.0);
}

TEST_F(fsae6dof_test, ev_envelope_torque_limited_at_low_speed)
{
    fsae6dof<double>::cartesian car(database);

    std::array<scalar, fsae6dof<scalar>::cartesian::number_of_inputs> q{};
    q[Chassis_t::input_names::velocity_x_mps] = 5.0;
    q[Chassis_t::input_names::Z] = 0.02;

    std::array<scalar, fsae6dof<scalar>::cartesian::number_of_controls> u{};
    u[Chassis_t::control_names::throttle] = 1.0;
    u[Chassis_t::control_names::brake_bias] = 0.53;

    (void)car(q, u, 0.0);

    const auto power = car.get_chassis().get_rear_axle().get_engine().get_power();
    EXPECT_GT(power, 0.0);
    EXPECT_LT(power, 80.0e3);
}

TEST_F(fsae6dof_test, ev_envelope_power_capped)
{
    fsae6dof<double>::cartesian car(database);

    std::array<scalar, fsae6dof<scalar>::cartesian::number_of_inputs> q{};
    q[Chassis_t::input_names::velocity_x_mps] = 40.0;
    q[Chassis_t::input_names::Z] = 0.02;

    std::array<scalar, fsae6dof<scalar>::cartesian::number_of_controls> u{};
    u[Chassis_t::control_names::throttle] = 1.0;
    u[Chassis_t::control_names::brake_bias] = 0.53;

    (void)car(q, u, 0.0);

    const auto power = car.get_chassis().get_rear_axle().get_engine().get_power();
    EXPECT_GT(power, 50.0e3);
    EXPECT_LT(power, 80.0e3 * 1.01);
}

TEST_F(fsae6dof_test, battery_energy_integral_is_motor_power)
{
    fsae6dof<double>::cartesian car(database);

    std::array<scalar, fsae6dof<scalar>::cartesian::number_of_inputs> q{};
    q[Chassis_t::input_names::velocity_x_mps] = 20.0;
    q[Chassis_t::input_names::Z] = 0.02;

    std::array<scalar, fsae6dof<scalar>::cartesian::number_of_controls> u{};
    u[Chassis_t::control_names::throttle] = 1.0;
    u[Chassis_t::control_names::brake_bias] = 0.53;

    (void)car(q, u, 0.0);

    const auto integrals = car.compute_integral_quantities();
    EXPECT_EQ(integrals.size(), 5u);
    EXPECT_NEAR(integrals[0], car.get_chassis().get_rear_axle().get_engine().get_power()*1.0e-6, 1.0e-12);
}

TEST_F(fsae6dof_test, create_vehicle_from_xml_c_api)
{
#ifdef TEST_LIBFASTESTLAPC
    create_vehicle_from_xml("ubco", "./database/vehicles/fsae/ubco-2026-ev.xml");

    int n_inputs = 0, n_control = 0, n_outputs = 0;
    vehicle_type_get_sizes(&n_inputs, &n_control, &n_outputs, "fsae-6dof");
    EXPECT_EQ(n_inputs, 16);
    EXPECT_EQ(n_control, 3);
    EXPECT_GT(n_outputs, 0);

    char type[32] = {};
    variable_type(type, 32, "ubco");
    EXPECT_STREQ(type, "fsae-6dof");

    delete_variable("ubco");
#else
    GTEST_SKIP();
#endif
}
