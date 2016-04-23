// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

void Plane::ndi_ewing(float speed_scaler)
{
    float Vspeed = 0;
    if ((aero_available)&&(vel_available)) {
        // avaliable: eular132        
        // do my alpha controller here~
        float ahrs_aoa_cd, aoa_err, ahrs_mu_cd, mu_err;
        float svo_aileron, svo_canard;
        ahrs_aoa_cd = ToDeg(eular132.y)*100;
        ahrs_mu_cd = ToDeg(eular132.x)*100;
        aoa_err = ew_AOA_cd + g.aoa_trim_cd_ew - ahrs_aoa_cd;
        mu_err = ew_MU_cd - ahrs_mu_cd;
        
        // rate controller takes in desired rate in deg/s, and output servo angle of +-4500
        svo_aileron = rollController.get_rate_out(mu_err * g.k_aoa_to_rate_ew * 0.01f,  speed_scaler);
        svo_canard = pitchController.get_rate_out(aoa_err * g.k_mu_to_rate_ew * 0.01f, speed_scaler);
        
        // if using unstable canard, the canard always point toward the wind
        // AOA compensation of canard
        float aoa_compensation = 0;
        if(g.enable_aoa_cmpnstr_ew) {
            if(ew_AOA_cd > 0) {
                aoa_compensation = ew_AOA_cd * (g.max_canard_aoa_ew*100.0f/4500);
            }else {
                aoa_compensation = -ew_AOA_cd * (g.min_canard_aoa_ew*100.0f/4500);
            }
        }
        
        // low speed compensation, in case the AOA estimator is not so accurate,
        // we need to do low speed controller mixing. Which is borrow from FBWA
        Vspeed = vel_NED.length();
        float ctrl_trans_intvl = (g.ctrl_trans_ub_ew - g.ctrl_trans_lb_ew);
        if( Vspeed < g.ctrl_trans_lb_ew ) {  
            svo_canard = pitchController.get_servo_out(ew_AOA_cd + g.pitch_trim_cd + channel_throttle->servo_out * g.kff_throttle_to_pitch - ahrs.pitch_sensor, 
                                                        speed_scaler,  false);
            svo_aileron = rollController.get_servo_out(ew_MU_cd - ahrs.roll_sensor, speed_scaler, false);
        } else if(Vspeed <= g.ctrl_trans_ub_ew) {
            int32_t demanded_pitch = ew_AOA_cd + g.pitch_trim_cd + channel_throttle->servo_out * g.kff_throttle_to_pitch;
            float pitch_command, roll_command;
            // normal FBWA command output
            pitch_command = pitchController.get_servo_out(demanded_pitch - ahrs.pitch_sensor, speed_scaler,  false);
            roll_command = rollController.get_servo_out(ew_MU_cd - ahrs.roll_sensor, speed_scaler, false);                
            
            // linear scaling with ew_aero controller;
            // when compensating, higher the AOA, lower the angle is.
            svo_canard = ( (svo_canard-aoa_compensation) * (Vspeed-g.ctrl_trans_lb_ew)/ctrl_trans_intvl ) + (pitch_command* (g.ctrl_trans_ub_ew-Vspeed)/ ctrl_trans_intvl);
            svo_aileron = (svo_aileron* (Vspeed-g.ctrl_trans_lb_ew)/ ctrl_trans_intvl) + (roll_command* (g.ctrl_trans_ub_ew-Vspeed)/ ctrl_trans_intvl);

        } else {       // speed > g.ctrl_trans_ub_ew
            svo_canard -= aoa_compensation;
        }   // if (speed range)
        
        channel_roll->servo_out = svo_aileron;
        channel_pitch->servo_out = constrain_float(svo_canard, -4500, 4500);
        stabilize_yaw(speed_scaler);
    }else {
        // fail to activate my controller, fall back to FBWA calling method
        if (g.stick_mixing == STICK_MIXING_FBW && control_mode != STABILIZE) {
            stabilize_stick_mixing_fbw();
        }
        stabilize_roll(speed_scaler);       //EWING stabilize call
        stabilize_pitch(speed_scaler);
        if (g.stick_mixing == STICK_MIXING_DIRECT || control_mode == STABILIZE) {
            stabilize_stick_mixing_direct();
        }
        stabilize_yaw(speed_scaler);
    }     // if((aero_available)&&(vel_available))
    Log_Write_EWAero(Vspeed);
    return;
}