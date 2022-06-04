/*! @package pid_controller
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include "motion_control/pid_controller.hpp"

#include<cmath>
#define EPSILON 1e-9

PIDController::PIDController() {}

float PIDController::ThrottlePID(float ref_vx, float cur_vx, double dt)
{
    // Don't use controller
    if(m_throttle_ctrl) 
        return ref_vx;
    // reference 0
    if(std::abs(ref_vx) < EPSILON){ 
        m_vx_int_error = 0.0;
        return 0.0f;
    }
    
    float error = ref_vx - cur_vx;
    //Error derivate
    float e_diff = (m_vx_prop_ek1 - error) / dt;
    m_vx_int_error += m_vx_prop_ek1 * dt;
    
    float controller_signal = m_kp_thr * error
                              + m_ki_thr * m_vx_int_error
                              + m_kd_thr * e_diff;
    
    m_vx_prop_ek1 = error;
    m_prev_ref_vx = ref_vx;

    return controller_signal;
}


float PIDController::SteeringPID(float ref_wz, float cur_wz, double dt)
{
    // Don't use controller
    if(m_steering_ctrl) 
        return ref_wz;
    // reference 0
    if(std::abs(ref_wz) < EPSILON){
        m_vx_int_error = 0.0;
        return 0.0f;
    }
    
    float error = ref_wz - cur_wz;
    //Error derivate
    float e_diff = (m_wz_prop_ek1 - error) / dt;
    m_wz_int_error += m_wz_prop_ek1 * dt;
    
    //feedback control
    float controller_signal = m_kp_str * m_wz_prop_ek1
                              + m_ki_str * m_wz_int_error
                              + m_kd_str * e_diff;

    //feed forward
    controller_signal += m_kff_str * ref_wz;
    
    m_wz_prop_ek1 = error;
    m_prev_ref_wz = ref_wz;

    return controller_signal;
}
