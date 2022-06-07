/*! @package pid_controller
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include "motion_control/pid_controller.hpp"

#include<cmath>
#define EPSILON 1e-9
#define PI 3.1516

PIDController::PIDController() {}

float PIDController::ThrottlePID(float ref_vx, float cur_vx, double dt)
{
    // Don't use controller
    if (!m_throttle_ctrl) 
        return ref_vx;
    // reference 0
    if (std::fabs(ref_vx) < EPSILON)
    { 
        m_vx_int_error = 0.0;
        return 0.0f;
    }
    
    float error = ref_vx - cur_vx;
    // Error derivate
    float e_diff = (m_vx_prop_ek1 - error) / dt;

    // Integral control component
    float ui = m_vx_int_error + (m_ki_thr * error * dt);
    
    float controller_signal = m_kp_thr * error
                              + ui
                              + m_kd_thr * e_diff;

    // Anti-Windup: Back-calculation
    float ep = 0;
    // saturated signal
    if (m_max_linear_spd < std::fabs(cur_vx))
    {
        ep = cur_vx > 0 ? 
                m_max_linear_spd - cur_vx : 
                -m_max_linear_spd - cur_vx;
    }         
    m_vx_int_error += (m_ki_thr * error + ep / m_tt_thr) * dt;

    m_vx_prop_ek1 = error;
    m_prev_ref_vx = ref_vx;

    return controller_signal;
}


float PIDController::SteeringPID(float ref_wz, float cur_wz, double dt)
{
    // Don't use controller
    if (!m_steering_ctrl) 
        return ref_wz;
    // reference 0
    if(std::fabs(ref_wz) < EPSILON)
    {
        m_vx_int_error = 0.0;
        return 0.0f;
    }
    
    float error = ref_wz - cur_wz;
    //Error derivate
    float e_diff = (m_wz_prop_ek1 - error) / dt;

    // Integral control component
    float ui = m_wz_int_error + (m_ki_str * error * dt);
    
    // Feedback control
    float controller_signal = m_kp_str * m_wz_prop_ek1
                              + ui
                              + m_kd_str * e_diff;

    // Feed forward
    controller_signal += m_kff_str * ref_wz;

    // Anti-Windup: Back-calculation
    float ep = 0;
    // saturated signal
    if(m_max_angular_spd < std::fabs(cur_wz))
    {
        ep = cur_wz > 0 ? 
                m_max_angular_spd - cur_wz : 
                -m_max_angular_spd - cur_wz;
    }
    m_wz_int_error += (m_ki_str * error + ep / m_tt_str) *dt;
    
    m_wz_prop_ek1 = error;
    m_prev_ref_wz = ref_wz;

    return controller_signal;
}
