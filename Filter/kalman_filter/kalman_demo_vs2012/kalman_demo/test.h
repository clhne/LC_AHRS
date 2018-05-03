/*  Model level defines for the test case of SfM with only
    camera translation.          */


#define SFM_STATE_SIZE         7

/*  These are the components of the state vector   */

#define STATE_Tx               1
#define STATE_Ty               2
#define STATE_Tz               3
#define STATE_Wx               4
#define STATE_Wy               5
#define STATE_Wz               6
#define STATE_B                7
#define STATE_FEATURE_START    8

#define ARBITRARY_SCALE        1

/*  These constants define initial estimates   */

#define ESTIMATE_BETA          80         /* 1 / 12mm  */
#define ESTIMATE_WORLD_Z       5.0        /* meters from focal point  */

    /*  System noise covariance values  */
#define COV_SYSTEM_Tx_Tx       0.01      /* 10 cm std dev ?  */
#define COV_SYSTEM_Ty_Ty       0.01
#define COV_SYSTEM_Tz_Tz       1.0e-3
#define COV_SYSTEM_Wx_Wx       1.0e-10
#define COV_SYSTEM_Wy_Wy       1.0e-10
#define COV_SYSTEM_Wz_Wz       1.0e-10
#define COV_SYSTEM_BETA        1.0e-3
#define COV_SYSTEM_Fn_Fn       1.0e-8  /*  features assumed STATIC ! */

    /*  Initial estimate error covariance  */
#define COV_ESTIMATE_Td_Td     1.0e-6  /*  to within a centimeter */
#define COV_ESTIMATE_Wd_Wd     1.0e-6  /*  to within a centimeter */
#define COV_ESTIMATE_BETA      1.0e1
#define COV_ESTIMATE_Fn_Fn     0.1     /* a meter ? */
#define COV_ESTIMATE_Fz_Fz     0.6     /*  more ambiguity  */

extern m_elem  global_rotation[];  /*  needed by kalman_camera  */
extern int     feature_size;       /*  needed by eval_camera    */
extern int     debug;
