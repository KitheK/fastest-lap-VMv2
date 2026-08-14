#ifndef CHASSIS_CAR_6DOF_FSAE_HPP
#define CHASSIS_CAR_6DOF_FSAE_HPP

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
inline void Chassis_car_6dof_fsae<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::update(
    const Vector3d<Timeseries_t>& ground_position_vector_m,
    const Euler_angles<scalar>& road_euler_angles_rad,
    const Timeseries_t& track_heading_angle_rad,
    const Euler_angles<Timeseries_t>& road_euler_angles_dot_radps,
    const Timeseries_t& track_heading_angle_dot_radps,
    const Timeseries_t& ground_velocity_z_body_mps)
{
    base_type::base_type::update(ground_position_vector_m, road_euler_angles_rad, track_heading_angle_rad,
                                 road_euler_angles_dot_radps, track_heading_angle_dot_radps, ground_velocity_z_body_mps);

    auto& front_axle = this->get_front_axle();
    auto& rear_axle  = this->get_rear_axle();
    auto& road_frame = this->get_road_frame();
    const auto& m = this->get_mass();

    this->get_chassis_frame().set_origin(this->get_com_position(), this->get_com_velocity(), Frame<Timeseries_t>::Frame_velocity_types::parent_frame);

    front_axle.update(this->get_front_axle_position(), this->get_front_axle_velocity(), this->_phi, this->_dphi,
                      _throttle, _brake_bias, road_frame);
    rear_axle.update(this->get_rear_axle_position(), this->get_rear_axle_velocity(), this->_phi, this->_dphi,
                     _throttle, 1.0 - _brake_bias, road_frame);

    const Matrix3x3<Timeseries_t> Q_front = front_axle.get_frame().get_rotation_matrix(road_frame);
    const Matrix3x3<Timeseries_t> Q_rear  = rear_axle.get_frame().get_rotation_matrix(road_frame);

    const Vector3d<Timeseries_t> x_front = std::get<0>(front_axle.get_frame().get_position_and_velocity_in_target(road_frame));
    const Vector3d<Timeseries_t> x_rear  = std::get<0>(rear_axle.get_frame().get_position_and_velocity_in_target(road_frame));

    const Vector3d<Timeseries_t> F_front = Q_front*front_axle.get_force();
    const Vector3d<Timeseries_t> F_rear  = Q_rear*rear_axle.get_force();
    const Vector3d<Timeseries_t> T_front = Q_front*front_axle.get_torque();
    const Vector3d<Timeseries_t> T_rear  = Q_rear*rear_axle.get_torque();

    this->_total_force_N = F_front + F_rear;
    this->_total_force_N[Z] += m*g0;

    this->_total_torque_Nm = T_front + cross(x_front, F_front) + T_rear + cross(x_rear, F_rear);

    const auto aerodynamic_forces = this->get_aerodynamic_force();
    const auto F_aero = aerodynamic_forces.lift + aerodynamic_forces.drag;
    this->_total_force_N += F_aero;
    this->_total_torque_Nm += cross(_x_aero + Vector3d<Timeseries_t>(0.0, 0.0, this->_z), F_aero);

    const Vector3d<Timeseries_t> dvdt = -this->Newton_lhs() + this->_total_force_N/m;

    this->_com_velocity_x_mps = this->get_u();
    this->_com_velocity_y_mps = this->get_v();
    this->_com_velocity_x_dot_mps2 = dvdt[X];
    this->_com_velocity_y_dot_mps2 = dvdt[Y];
    this->_d2z = dvdt[Z];

    const Vector3d<Timeseries_t> d2phi = linsolve(this->Euler_m(), -this->Euler_lhs() + this->_total_torque_Nm);
    this->_d2phi = d2phi[X];
    this->_d2mu  = d2phi[Y];
    this->_yaw_rate_dot_radps2 = d2phi[Z];
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Chassis_car_6dof_fsae<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state_and_control_names(
    std::array<std::string, number_of_inputs>& inputs, std::array<std::string, number_of_controls>& controls) const
{
    base_type::set_state_and_control_names(inputs, controls);
    controls[control_names::throttle] = "chassis.throttle";
    controls[control_names::brake_bias] = "chassis.brake-bias";
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Chassis_car_6dof_fsae<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state_and_controls(
    const std::array<Timeseries_t,number_of_inputs>& inputs, const std::array<Timeseries_t,number_of_controls>& controls)
{
    base_type::set_state_and_controls(inputs, controls);
    _throttle   = controls[control_names::throttle];
    _brake_bias = controls[control_names::brake_bias];
}

template<typename Timeseries_t, typename FrontAxle_t, typename RearAxle_t, size_t state_start, size_t control_start>
template<size_t number_of_inputs, size_t number_of_controls>
void Chassis_car_6dof_fsae<Timeseries_t,FrontAxle_t,RearAxle_t,state_start,control_start>::set_state_and_control_upper_lower_and_default_values(
    std::array<scalar, number_of_inputs>& inputs_def, std::array<scalar, number_of_inputs>& inputs_lb,
    std::array<scalar, number_of_inputs>& inputs_ub,
    std::array<scalar, number_of_controls>& controls_def, std::array<scalar, number_of_controls>& controls_lb,
    std::array<scalar, number_of_controls>& controls_ub) const
{
    base_type::set_state_and_control_upper_lower_and_default_values(
        inputs_def, inputs_lb, inputs_ub, controls_def, controls_lb, controls_ub);

    controls_def[control_names::throttle] = 0.0;
    controls_lb[control_names::throttle]  = -1.0;
    controls_ub[control_names::throttle]  =  1.0;

    controls_def[control_names::brake_bias] = Value(_brake_bias_0);
    controls_lb[control_names::brake_bias]  = 0.0;
    controls_ub[control_names::brake_bias]  = 1.0;

    // Wheel radius is larger than the kart default used by Chassis_car_6dof.
    inputs_ub[input_names::Z] = 0.21;
}

#endif
