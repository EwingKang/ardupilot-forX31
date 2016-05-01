// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

void Plane::ndi_ewing(float speed_scaler)
{
    static uint32_t _last_t;
    bool reset_i;
    uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
    if (_last_t == 0 || dt > 1000) {
		dt = 0;
        reset_i = true;
	}
	_last_t = tnow;
	float delta_time    = (float)dt * 0.001f;

    Vector3f omega_command;
    Vector3f aero_dot_dym;
    
    if ((aero_available)&&(vel_available)) {
        // slow loop
        float alpha_dot_des, beta_dot_des, mu_dot_des;
        float alpha_dot_dym, beta_dot_dym, mu_dot_dym;
        
        aero_desired_rate(alpha_dot_des, beta_dot_des, mu_dot_des, delta_time, reset_i);
        Vector3f aero_dot_des(alpha_dot_des, beta_dot_des, mu_dot_des);
             
        if(slow_dynamic_rate(alpha_dot_dym, beta_dot_dym, mu_dot_dym)) {
            // slow loop NDI
            aero_dot_dym = Vector3f(alpha_dot_dym, beta_dot_dym, mu_dot_dym);
            float tan_beta = tanf(-eular132.z);
            float sec_beta = 1/cosf(-eular132.z);
            float cos_alpha = cosf(eular132.y);
            float sin_alpha = sinf(eular132.y);
            Matrix3f g_s1(Vector3f( -tan_beta*cos_alpha, 1, -tan_beta*sin_alpha  ), 
                          Vector3f( sin_alpha          , 0, -cos_alpha           ), 
                          Vector3f( sec_beta*cos_alpha , 0, sec_beta*sin_alpha   ) );
            Matrix3f g_s1_inv;
            if(inverse3x3(&g_s1[0][0], &g_s1_inv[0][0])) {
                // (5.3.29)
                omega_command = g_s1_inv*( aero_dot_des - aero_dot_dym );
            } else {
                //fail to get inverse
                ew_fbwa_backup();
                Log_Write_EWNDI(-2, 0, aero_dot_dym, aero_dot_des);
                return;
            }
        }else {
            // fail to get dynamic because of excess AOA
            ew_fbwa_backup();
            Log_Write_EWNDI(-1, 0, Vector3f(0, 0, 0), aero_dot_des );
            return;
        }
        
        // fast loop (5.2.3)
        Vector3f omega_dot_des, omega_dot_dym;
        Vector3f actuator_command;
        omega_desired_rate(omega_command, omega_dot_des, delta_time, reset_i);
        if(fast_dynamic_rate(omega_dot_dym)) {
            Matrix3f g_fR, g_fRinv;
            if( get_fl_input_mat(g_fR,g_fRinv) ) {
                // (5.2.36)
                actuator_command = g_fRinv*(omega_dot_des - omega_dot_dym);
            } else {
                //fail to get inverse of input matrix
                ew_fbwa_backup();
                Log_Write_EWNDI(1, -2, aero_dot_dym, omega_command);
                return;
            }
        }else {
            // fail to get dynamic because of excess AOA
            ew_fbwa_backup();
            Log_Write_EWNDI(1, -1, aero_dot_dym, omega_command);
            return;
        }
    ndi_set_servo(actuator_command);
    }else {
        // if no aero & velocity available
       ew_fbwa_backup();
       Log_Write_EWNDI(0, 0, Vector3f(0, 0, 0), Vector3f(0, 0, 0));
    }
    Log_Write_EWNDI(1, 1, aero_dot_dym, omega_command);
    return;
}


void Plane::aero_desired_rate(float &a_dot_des, float &b_dot_des, float &m_dot_des, const float &delta_time, const bool &reset_i){
    float ahrs_aoa = ToDeg(eular132.y);
    float ahrs_mu = ToDeg(eular132.x);
    float ahrs_beta = -ToDeg(eular132.z);
    float aoa_err = (ew_AOA_cd + g.aoa_trim_cd_ew)/100 - ahrs_aoa;
    float mu_err = ew_MU_cd/100 - ahrs_mu;
    float beta_err = g.beta_trim_cd_ew/100 - ahrs_beta;
        
    static float alpha_integrator, beta_integrator, mu_integrator;
    static float _last_a_dot_des, _last_b_dot_des, _last_m_dot_des;
    
    if(reset_i){
         alpha_integrator = 0;
         beta_integrator = 0;
         mu_integrator = 0;
    }
    
    float aspeed = vel_NED.length();
    //only integrate if gain and time step are positive and airspeed above min value.
    if (delta_time > 0 && aspeed > g.ctrl_trans_lb_ew) {
        float alpha_intg_delta = aoa_err * g.ndi_ki_aoa_ew * delta_time;
        float beta_intg_delta = beta_err * g.ndi_ki_ss_ew * delta_time;
        float mu_intg_delta =  mu_err * g.ndi_ki_mu_ew * delta_time;
        
        // prevent the integrator from increasing if surface defln demand is above the upper limit
        // prevent the integrator from decreasing if surface defln demand  is below the lower limit
        if (_last_a_dot_des < -40) {
            alpha_intg_delta = MAX(alpha_intg_delta , 0);
        } else if (_last_a_dot_des > 40) {
            alpha_intg_delta = MIN(alpha_intg_delta , 0);
        }
        if (_last_b_dot_des < -30) {
            beta_intg_delta = MAX(beta_intg_delta , 0);
        } else if (_last_b_dot_des > 30) {
            beta_intg_delta = MIN(beta_intg_delta , 0);
        }
        if (_last_m_dot_des < -40) {
            mu_intg_delta = MAX(mu_intg_delta , 0);
        } else if (_last_m_dot_des > 40) {
            mu_intg_delta = MIN(mu_intg_delta , 0);
        }
        
        alpha_integrator += alpha_intg_delta;
        beta_integrator += beta_intg_delta;
        mu_integrator += mu_intg_delta;
    } else {
        alpha_integrator = 0;
        beta_integrator = 0;
        mu_integrator = 0;
    }
    a_dot_des = aoa_err * g.ndi_kp_aoa_ew + alpha_integrator;
    b_dot_des = beta_err * g.ndi_kp_ss_ew + alpha_integrator;
    m_dot_des = mu_err * g.ndi_kp_mu_ew + alpha_integrator;
    _last_a_dot_des = a_dot_des;
    _last_b_dot_des = b_dot_des;
    _last_m_dot_des = m_dot_des;
    Log_Write_EWAero(aspeed);
    return;
}


bool Plane::slow_dynamic_rate(float &alpha_dot_dym, float &beta_dot_dym, float &mu_dot_dym)
{
    float ahrs_aoa = ToDeg(eular132.y);
    //float ahrs_mu = ToDeg(eular132.x);
    float ahrs_beta = -ToDeg(eular132.z);
    
    if( ((ahrs_aoa > 85)&&(ahrs_aoa<-80)) || (fabsf(ahrs_beta)>80) ) {
        return false;
    }
    
    float Q = 0.5*1.23*vel_NED.length()*vel_NED.length();
    float thrust = channel_throttle->norm_input() * g.max_thrust_ew;
    float lift = Q * g.ndi_mw_S_ew * aero_coef(211, ahrs_aoa);
    float side_force = Q * g.ndi_mw_S_ew * aero_coef(231, ahrs_aoa) * (-eular132.z);
    
    //(5.3.8)
    Quaternion q_mu;
    q_mu.from_axis_angle_fast(Vector3f(1,0,0), eular132.x);
    Vector3f aero_mu_g(0,0,g.ndi_mass_ew*9.81);
    (velQuat*q_mu).earth_to_body(aero_mu_g);
    
    alpha_dot_dym = (-lift + aero_mu_g.z - thrust * sinf(eular132.y)) / (g.ndi_mass_ew * vel_NED.length() * cosf(eular132.z));
    
    //(5.3.9)
    beta_dot_dym = (side_force*cosf(-eular132.z) + aero_mu_g.y) / (g.ndi_mass_ew * vel_NED.length());
  
    //(5.3.10)
    float eq5310a = lift*( tanf(vel_gamma)*sinf(eular132.x) + tanf(-eular132.z) );
    float eq5310b = side_force * tanf(vel_gamma) * cosf(eular132.x) * cosf(-eular132.z)
                    - aero_mu_g.z * tanf(-eular132.z);
    float eq5310c = tanf(vel_gamma) * (sinf(eular132.x)*sinf(eular132.y) - cosf(eular132.x)*cosf(eular132.y)*sinf(-eular132.z))
                    + tanf(-eular132.z)*sinf(eular132.y);
    mu_dot_dym = (eq5310a + eq5310b + eq5310c * thrust) / (g.ndi_mass_ew * vel_NED.length());
    return true;
}

void Plane::omega_desired_rate(const Vector3f &omega_c, Vector3f &omega_dot_des, const float &delta_time, const bool &reset_i)
{
    // (5.2.3)
    Vector3f omega_err(omega_c - ahrs.get_gyro());          // return in radians/s
    Matrix3f omega_gain_kp(Vector3f(g.ndi_kp_p_ew, 0, 0),
                           Vector3f(0, g.ndi_kp_q_ew, 0),
                           Vector3f(0, 0, g.ndi_kp_r_ew) );
    Matrix3f omega_gain_ki(Vector3f(g.ndi_ki_p_ew, 0, 0),
                           Vector3f(0, g.ndi_ki_q_ew, 0),
                           Vector3f(0, 0, g.ndi_ki_r_ew) );
    static Vector3f omega_integrator;
    static Vector3f _last_omega_dot_des;
    
    if(reset_i){
         omega_integrator.zero();
    }
    
    float aspeed = vel_NED.length();
    //only integrate if gain and time step are positive and airspeed above min value.
    if (delta_time > 0 && aspeed > g.ctrl_trans_lb_ew) {
        
        Vector3f omega_intg_delta = (omega_gain_ki * omega_err) * delta_time;
                
        // prevent the integrator from increasing if surface defln demand is above the upper limit
        // prevent the integrator from decreasing if surface defln demand  is below the lower limit
        for(int i=0;i<3;i++) {
            if (_last_omega_dot_des[i] < -40) {
                  omega_intg_delta[i] = MAX(omega_intg_delta[i] , 0);
            } else if (_last_omega_dot_des[i] > 40) {
                omega_intg_delta[i] = MIN(omega_intg_delta[i] , 0);
            }
        }
        
        omega_integrator += omega_intg_delta;
    } else {
        omega_integrator.zero();
    }
    
    omega_dot_des = (omega_gain_kp * omega_err) + omega_integrator;
    
    _last_omega_dot_des = omega_dot_des;
    return;
}

bool Plane::fast_dynamic_rate(Vector3f &omega_dot_dym)
{
    float ahrs_aoa = ToDeg(eular132.y);
    //float ahrs_beta = -ToDeg(eular132.z);
    float vel = vel_NED.length();
    float Q = 0.5*1.23*vel*vel;
    float b_div_2v = g.ndi_mw_b_ew/2/vel;
    
    float p = ahrs.get_gyro().x;
    float q = ahrs.get_gyro().y;
    float r = ahrs.get_gyro().z;
    
    float Cl_beta = aero_coef(241, ahrs_aoa);   // Cl/beta
    float Cl_p = aero_coef(251, ahrs_aoa);      // Cl/p
    float Cl_r = aero_coef(261, ahrs_aoa);      // Cl/r
    float Cn_beta = aero_coef(242, ahrs_aoa);   // Cn/beta
    float Cn_p = aero_coef(252, ahrs_aoa);      // Cn/p
    float Cn_r = aero_coef(262, ahrs_aoa);      // Cn/r
    float Cm_0 = aero_coef(291, ahrs_aoa);      // Cm0(a)
    float Cm_q = aero_coef(292, ahrs_aoa);      // Cm/q
    //(5.2.13)~(5.2.15)
    float l = Q*g.ndi_mw_S_ew*g.ndi_mw_b_ew*( Cl_beta * (-eular132.z) + 
                                             (Cl_p*p + Cl_r*r)*b_div_2v );
    float m = Q*g.ndi_mw_S_ew*g.ndi_mw_c_ew*( Cm_0 + Cm_q*q*g.ndi_mw_c_ew/2/vel );
    float n = Q*g.ndi_mw_S_ew*g.ndi_mw_b_ew*( Cn_beta * (-eular132.z) + 
                                             (Cn_p*p + Cn_r*r)*b_div_2v );
    // M = [I]a + w%[I]w  ==> a = [I]^-1 * (M-w%[I]w)
    Vector3f moment(l,m,n);
    omega_dot_dym = inertiaMatInv * (moment - ahrs.get_gyro()%(inertiaMat*ahrs.get_gyro()));
    
    return true;
}

bool Plane::get_fl_input_mat(Matrix3f &g_fR, Matrix3f &g_fRinv)
{
    float ahrs_aoa = ToDeg(eular132.y);
    float Clda = aero_coef(271, ahrs_aoa);  // Cl/ail
    float Cnda = aero_coef(272, ahrs_aoa);  // Cn/ail
    float Cldr = aero_coef(281, ahrs_aoa);  // Cl/rud
    float Cndr = aero_coef(282, ahrs_aoa);  // Cn/rud
    float Cmdc = aero_coef(2101, ahrs_aoa); // Cm/can
    Matrix3f input_mat( Vector3f(Clda,    0, Cldr), 
                        Vector3f(   0, Cmdc,    0),
                        Vector3f(Cnda,    0, Cndr)  );
    g_fR = inertiaMatInv * input_mat;   // (5.2.16)
    return inverse3x3(&g_fR[0][0], &g_fRinv[0][0]);
}

void Plane::ndi_set_servo(const Vector3f &actuator_cmd)
{
    float ail_deg = constrain_float(actuator_cmd.x, -g.max_aileron_ang_ew, g.max_aileron_ang_ew);
    float can_deg = constrain_float( (actuator_cmd.y + aero_coef(2102, ToDeg(eular132.y))),
                                                                  g.min_canard_ang_ew, 
                                                                  g.max_canard_ang_ew );
    float rud_deg = constrain_float(actuator_cmd.x, -g.max_rudder_ang_ew, g.max_rudder_ang_ew);
    
    if(can_deg>0) {
        can_deg = can_deg * 45 / g.max_canard_ang_ew;
    }else{
        can_deg = can_deg * -45 / g.min_canard_ang_ew;
    }
    
    channel_roll->servo_out  = ail_deg * 4500 / g.max_aileron_ang_ew;
    channel_pitch->servo_out = can_deg * 100;
    channel_rudder->servo_out = rud_deg * 4500 / g.max_rudder_ang_ew;
    return;
}

/*
  This function assigned the coefficient of specific aerodynamic variable
  according to the data provided in "Nonlinear Dynamic Inversion Flight 
  Of Supermaneuverable Aircraft" by Snell, Sidney Antony, U. of Minnesota
  ,1991.
  The Index corresponds to the figure index in the thesis, for example, 
  Figure2-2-6b will have the index of 262.
  Function will return false if AOA is out of range.
*/
float Plane::aero_coef(const uint16_t &ind, const float &alpha)
{
    double x = fabs(alpha), y;
    switch (ind) {
    case 211:           // [ODD ][a] Cl
        y = - 1.73074E-13l*pow(x,6) - 4.55181E-09l*pow(x,5) + 1.05377E-06l*pow(x,4) - 7.53902E-05l*pow(x,3)
            + 0.000838565l*pow(x,2) + 0.068166435l*x        + 0.031321913l;
        if(alpha < 0) { y = -y; }
        break;
        
    case 212:           // [EVEN][a] Cd
        y =   2.16121E-09l*pow(x,5) - 4.79288E-07l*pow(x,4) + 2.91271E-05l*pow(x,3) - 0.000177299l*pow(x,2)
            + 0.009653169l*x        + 0.003815982l;
        break;
        
    case 221:           // [EVEN][a] Cl delta canard
        y =   3.85119E-14l*pow(x,6) - 1.36183E-11l*pow(x,5) + 1.83108E-09l*pow(x,4) - 1.12387E-07l*pow(x,3)
            + 2.87863E-06l*pow(x,2) - 2.98973E-05l*x        + 0.002310071l;
        break;
        
    case 222:           // [ODD ][a] canard lift offset
        y =   1.83146E-10l*pow(x,6) - 1.82458E-08l*pow(x,5) - 4.89738E-06l*pow(x,4) + 0.001116867l*pow(x,3)
            - 0.061576775l*pow(x,2) - 0.161987094l*x        - 0.015086919l;
        if(alpha < 0) { y = -y; }
        break;
        
    case 231:           // [EVEN][a] CY 
        y = - 1l;
        break;
        
    case 232:           // [EVEN][a] CY with rudder(deg)
        y =   1.931E-11l*pow(x,5) - 4.98703E-09l*pow(x,4) + 4.58837E-07l*pow(x,3) - 1.68066E-05l*pow(x,2)
            + 0.000129449l*x      + 0.003236518l;
        break;
        
    case 241:           // [EVEN][a] Cl with beta(rad)
        y =   4.91796E-13l*pow(x,6) + 1.81067E-09l*pow(x,5) - 4.51658E-07l*pow(x,4) + 3.75585E-05l*pow(x,3)
            - 0.001180732l*pow(x,2) + 0.010568349l*x - 0.083588788l;
        break;
        
    case 242:           // [EVEN][a] Cn with beta
        y = - 6.5435E-13l*pow(x,6)  + 1.27567E-09l*pow(x,5) - 2.69626E-07l*pow(x,4) + 1.97183E-05l*pow(x,3)
            - 0.000415504l*pow(x,2) - 0.009013965l*x        + 0.175202791l;
        break;
        
    case 251:           // [EVEN][a] Cl with p
        y =   2.46726E-13l*pow(x,6) + 2.89722E-09l*pow(x,5) - 6.593E-07l*pow(x,4) + 5.02484E-05l*pow(x,3)
            - 0.001403253l*pow(x,2) + 0.009368111l*x        - 0.078285489l;
        break;
        
    case 252:           // [ODD ][a] Cn with p
        y =   1.11711E-12l*pow(x,6) + 3.84628E-09l*pow(x,5) - 7.95876E-07l*pow(x,4) + 5.19084E-05l*pow(x,3)
            - 0.001140853l*pow(x,2) + 0.001564168l*x        - 0.000286817l;
        if(alpha < 0) { y = -y; }
        break;
        
    case 261:           // [ODD ][a] Cl with r
        y =   4.30185E-12l*pow(x,6) - 7.98271E-09l*pow(x,5) + 1.67113E-06l*pow(x,4) - 0.000125695l*pow(x,3)
            + 0.003573161l*pow(x,2) - 0.022525193l*x        - 0.0758901l;
        if(alpha < 0) { y = -y; }
        break;
        
    case 262:           // [EVEN][a] Cn with r
        y =   3.48365E-13l*pow(x,6) + 2.49322E-09l*pow(x,5) - 6.69396E-07l*pow(x,4) + 6.01596E-05l*pow(x,3) 
            - 0.001987539l*pow(x,2) + 0.017397535l*x        - 0.32966209l;
        break;
        
    case 271:           // [EVEN][a] Cl with ailron(deg)
        y = - 1.20465E-14l*pow(x,6) + 8.54994E-12l*pow(x,5) - 1.7579E-09l*pow(x,4) + 1.57143E-07l*pow(x,3) 
            - 6.16355E-06l*pow(x,2) + 5.11295E-05l*x        + 0.002346565l;
        break;
        
    case 272:           // [EVEN][a] Cn with ailron(deg)
        y = - 1.07828E-14l*pow(x,6) + 5.8109E-12l*pow(x,5) - 1.01864E-09l*pow(x,4) + 7.90021E-08l*pow(x,3)
            - 2.72807E-06l*pow(x,2) + 2.49532E-05l*x       + 0.000760346l;
        break;
        
    case 281:           // [EVEN][a] Cl with rudder(deg)
        y = - 5.31398E-15l*pow(x,6) + 3.3101E-12l*pow(x,5) - 6.15656E-10l*pow(x,4) + 4.94371E-08l*pow(x,3)
            - 1.74829E-06l*pow(x,2) + 1.61374E-05l*x       + 0.000507964l;
        break;
        
    case 282:           // [EVEN][a] Cn with rudder(deg)
        y =   6.616196E-14l*pow(x,6) - 2.822004E-11l*pow(x,5) + 4.568825E-09l*pow(x,4) - 3.463623E-07l*pow(x,3) 
            + 1.168576E-05l*pow(x,2) - 9.462342E-05l*x        - 1.910177E-03l;
        break;
        
    case 291:           // [ODD ][a] Cm0 (4.65% static instability)
        y =   4.6273E-12l*pow(x,6)  - 1.34607E-09l*pow(x,5) + 1.44272E-07l*pow(x,4) - 6.66764E-06l*pow(x,3)
            + 8.80921E-05l*pow(x,2) + 0.002907947l*x        +  0.001436932l;
        if(alpha < 0) { y = -y; }
        break;
        
    case 292:           // [EVEN][a] Cm with q
        y = -3;
        break;
        
    case 2101:          // [EVEN][a] Cm with canard(deg)
        y = - 9.9798E-15l*pow(x,6)  - 5.0072E-12l*pow(x,5) + 1.54809E-09l*pow(x,4) - 1.32161E-07l*pow(x,3)
            + 3.84916E-06l*pow(x,2) - 4.46671E-05l*x       + 0.005052863l;
        break;
        
    case 2102:          // [ODD ][a] canard pitching-moment offset(deg)
        y =   4.63561E-08l*pow(x,5) - 1.2966E-05l*pow(x,4) + 0.001549639l*pow(x,3) - 0.070398444l*pow(x,2) 
            - 0.137580555l*x        - 0.099241141l;
        if(alpha < 0) { y = -y; }
        break;
        
    default:
        y = -100;
        break;
    }
    return (float)y;
}

void Plane::ew_fbwa_backup()
{
    float speed_scaler = get_speed_scaler();
    // fail to activate NDI controller, fall back to FBWA calling method
    if (g.stick_mixing == STICK_MIXING_FBW && control_mode != STABILIZE) {
        stabilize_stick_mixing_fbw();
    }
    stabilize_roll(speed_scaler);       //EWING stabilize call
    stabilize_pitch(speed_scaler);
    if (g.stick_mixing == STICK_MIXING_DIRECT || control_mode == STABILIZE) {
        stabilize_stick_mixing_direct();
    }
    stabilize_yaw(speed_scaler);
}

void Plane::ndi_calculate_inertia()
{
    inertiaMat = Matrix3f(Vector3f( g.ndi_ixx_ew,            0, -g.ndi_ixz_ew), 
                          Vector3f(            0, g.ndi_iyy_ew,             0), 
                          Vector3f(-g.ndi_ixz_ew,            0,  g.ndi_izz_ew) );
    inverse3x3(&inertiaMat[0][0], &inertiaMatInv[0][0]);
}
