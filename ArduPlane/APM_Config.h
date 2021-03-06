// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If
// you wish to change any of the setup parameters from their default
// values, place the appropriate #define statements here.

// If you used to define your CONFIG_APM_HARDWARE setting here, it is no
// longer valid! You should switch to using CONFIG_HAL_BOARD via the HAL_BOARD
// flag in your local config.mk instead.

//////////////////////////////////////////////////////////////////////////////
#define     ROLL_MAX_CD_EW      70*100
#define     MAX_AOA_IN_CD_EW    30*100    
#define     MIN_AOA_IN_CD_EW    -10*100
#define     AOA_CMPNSTR_EW      0
#define     K_AOA_TO_RATE_EW    2
#define     K_MU_TO_RATE_EW     1.5
#define     AOA_TRIM_CD_EW      500
#define     BETA_TRIM_CD_EW     0
#define     CTRL_TRANS_UB_EW    15
#define     CTRL_TRANS_LB_EW    3
#define     MAX_THRUST_EW       14.715
#define     MAX_AILE_ANG_EW     30
#define     MAX_CNRD_ANG_EW     38
#define     MIN_CNRD_ANG_EW     -88
#define     MAX_RUDD_ANG_EW     30
#define     MAX_TVC_Y_ANG_EW    15
#define     MAX_TVC_X_ANG_EW    15
    
//NDI, value is incorrect
#define     NDI_KP_AOA_EW       1
#define     NDI_KP_MU_EW        1
#define     NDI_KP_SS_EW        0.9
#define     NDI_KP_P_EW         5
#define     NDI_KP_Q_EW         5
#define     NDI_KP_R_EW         5
#define     NDI_KI_AOA_EW       0.05
#define     NDI_KI_MU_EW        0.05
#define     NDI_KI_SS_EW        0.02
#define     NDI_KI_P_EW         0.5
#define     NDI_KI_Q_EW         0.5
#define     NDI_KI_R_EW         0.5
#define     NDI_SL_SPDBUF_EW    4
#define     NDI_FL_SPDBUF_EW    3.5
#define     NDI_IXX_EW          0.02158             // By CATIA X31v2.3_material(2016.5.9)
#define     NDI_IYY_EW          0.15193
#define     NDI_IZZ_EW          0.15744
#define     NDI_IXZ_EW          0.00336
#define     NDI_MASS_EW         1.69
#define     NDI_MW_B_EW         1
#define     NDI_MW_C_EW         0.23
#define     NDI_MW_S_EW         0.3357
#define     NDI_AIR_DNSTY_EW    1.15
#define     NDI_SL_AOABFR_EW    2.5
#define     NDI_FL_AOABFR_EW    2
#define     NDI_AOABFR_UB_EW    50
#define     NDI_AOABFR_LB_EW    30
#define     NDI_THR_BDWTH_EW    7.5