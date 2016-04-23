// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If
// you wish to change any of the setup parameters from their default
// values, place the appropriate #define statements here.

// If you used to define your CONFIG_APM_HARDWARE setting here, it is no
// longer valid! You should switch to using CONFIG_HAL_BOARD via the HAL_BOARD
// flag in your local config.mk instead.

//////////////////////////////////////////////////////////////////////////////
#define     ROLL_MAX_CD_EW      60*100
#define     MAX_AOA_IN_CD_EW    70*100    
#define     MIN_AOA_IN_CD_EW    -20*100
#define     AOA_CMPNSTR_EW      0
#define     MAX_CNRD_AOA_EW     80
#define     MIN_CNRD_AOA_EW     -15
#define     K_AOA_TO_RATE_EW    2
#define     K_MU_TO_RATE_EW     1.5
#define     AOA_TRIM_CD_EW      800
#define     CTRL_TRANS_UB_EW    13
#define     CTRL_TRANS_LB_EW    5
#define     MAX_THRUST_EW       2*9.81
    
//NDI, value is incorrect
#define     NDI_KP_AOA_EW       0.8
#define     NDI_KP_MU_EW        0.8
#define     NDI_KP_SS_EW        0.8
#define     NDI_KP_P_EW         0.8
#define     NDI_KP_Q_EW         0.8
#define     NDI_KP_R_EW         0.8
#define     NDI_IXX_EW          0.01
#define     NDI_IYY_EW          0.01
#define     NDI_IZZ_EW          0.01
#define     NDI_IXZ_EW          0.01
#define     NDI_MASS_EW         2.2
#define     NDI_MW_B_EW         1
#define     NDI_MW_C_EW         0.13