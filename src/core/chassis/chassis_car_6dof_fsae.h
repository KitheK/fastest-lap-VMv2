#ifndef CHASSIS_CAR_6DOF_FSAE_H
#define CHASSIS_CAR_6DOF_FSAE_H

#include "chassis_car_6dof.h"
#include <algorithm>

//! 6DOF chassis for an FSAE car: kart heave/roll/pitch plus throttle, brake bias, and aero pressure center.
template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
class Chassis_car_6dof_fsae : public Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>
{
 public:
    using base_type = Chassis_car_6dof<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>;
    using Front_axle_type = FrontAxle_t;
    using Rear_axle_type  = RearAxle_t;

    struct input_names : public base_type::input_names {};
    struct state_names : public base_type::state_names {};

    struct control_names : public base_type::control_names
    {
        enum
        {
            throttle = base_type::control_names::end,
            brake_bias,
            end
        };
    };

    Chassis_car_6dof_fsae() : base_type() {}

    Chassis_car_6dof_fsae(const FrontAxle_t& front_axle, const RearAxle_t& rear_axle, Xml_document& database, const std::string& path="")
    : base_type(front_axle, rear_axle, database, path)
    {
        read_parameters(database, path, get_parameters(), __used_parameters);
        _brake_bias = _brake_bias_0;
    }

    Chassis_car_6dof_fsae(Xml_document& database)
    : Chassis_car_6dof_fsae(
           FrontAxle_t("front-axle",
                       typename FrontAxle_t::Tire_left_type("front-axle.left-tire", database, "vehicle/front-tire/"),
                       typename FrontAxle_t::Tire_right_type("front-axle.right-tire", database, "vehicle/front-tire/"),
                       database, "vehicle/front-axle/"),
           RearAxle_t("rear-axle",
                      typename RearAxle_t::Tire_left_type("rear-axle.left-tire", database, "vehicle/rear-tire/"),
                      typename RearAxle_t::Tire_right_type("rear-axle.right-tire", database, "vehicle/rear-tire/"),
                      database, "vehicle/rear-axle/"),
           database, "vehicle/chassis/")
    {}

    void update(const Vector3d<Timeseries_t>& ground_position_vector_m,
                const Euler_angles<scalar>& road_euler_angles_rad,
                const Timeseries_t& track_heading_angle_rad,
                const Euler_angles<Timeseries_t>& road_euler_angles_dot_radps,
                const Timeseries_t& track_heading_angle_dot_radps,
                const Timeseries_t& ground_velocity_z_body_mps);

    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_controls(const std::array<Timeseries_t,number_of_inputs>& inputs,
                                const std::array<Timeseries_t,number_of_controls>& controls);

    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_control_names(std::array<std::string, number_of_inputs>& inputs,
        std::array<std::string, number_of_controls>& controls) const;

    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_control_upper_lower_and_default_values(std::array<scalar, number_of_inputs>& inputs_def,
        std::array<scalar, number_of_inputs>& inputs_lb,
        std::array<scalar, number_of_inputs>& inputs_ub,
        std::array<scalar, number_of_controls>& control_def,
        std::array<scalar, number_of_controls>& control_lb,
        std::array<scalar, number_of_controls>& control_ub
    ) const;

    static std::string type() { return "chassis_car_6dof_fsae"; }

    const Timeseries_t& get_cl_scale() const { return _cl_scale; }
    const Timeseries_t& get_cd_scale() const { return _cd_scale; }
    const Timeseries_t& get_front_aero_distribution() const { return _front_aero_distribution; }

    std::unordered_map<std::string,Timeseries_t> get_outputs_map() const
    {
        auto map = base_type::get_outputs_map();
        map[base_type::get_name() + ".aerodynamics.cl_scale"] = _cl_scale;
        map[base_type::get_name() + ".aerodynamics.cd_scale"] = _cd_scale;
        map[base_type::get_name() + ".aerodynamics.front_distribution"] = _front_aero_distribution;
        map[base_type::get_name() + ".attitude.heave"] = this->_z;
        map[base_type::get_name() + ".attitude.roll"] = this->_phi;
        map[base_type::get_name() + ".attitude.pitch"] = this->_mu;
        return map;
    }

    bool is_ready() const { return base_type::is_ready() &&
        std::all_of(__used_parameters.begin(), __used_parameters.end(), [](const auto& v) -> auto { return v; }); }

 private:
    sVector3d _x_aero = {0.0, 0.0, 0.0};
    Timeseries_t _brake_bias_0 = 0.5;
    Timeseries_t _throttle = 0.0;
    Timeseries_t _brake_bias = 0.5;
    scalar _dCl_dz = 0.0;
    scalar _dCl_dmu = 0.0;
    scalar _dCd_dz = 0.0;
    scalar _dCd_dmu = 0.0;
    Timeseries_t _cl_scale = 1.0;
    Timeseries_t _cd_scale = 1.0;
    Timeseries_t _front_aero_distribution = 0.5;

    DECLARE_PARAMS(
        { "pressure_center", _x_aero },
        { "brake_bias", _brake_bias_0 },
        { "aero-maps/dCl_dz", _dCl_dz },
        { "aero-maps/dCl_dmu", _dCl_dmu },
        { "aero-maps/dCd_dz", _dCd_dz },
        { "aero-maps/dCd_dmu", _dCd_dmu },
    );
};

#include "chassis_car_6dof_fsae.hpp"

#endif
