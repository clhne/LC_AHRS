/*  kalman.c

    This file contains the code for a kalman filter, an
    extended kalman filter, and an iterated extended kalman filter.

    For ready extensibility, the apply_measurement() and apply_system()
    functions are located in a separate file: kalman_cam.c is an example.

    It uses the matmath functions provided in an accompanying file
    to perform matrix and quaternion manipulation.


    J. Watlington, 11/15/95

    Modified:
    11/30/95  wad  The extended kalman filter section seems to be
                   working now.
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kalman.h"

#define ITERATION_THRESHOLD      2.0
#define ITERATION_DIVERGENCE     20

/*  The following are the global variables of the Kalman filters,
    used to point to data structures used throughout.     */

static m_elem  *state_pre;         /* ptr to apriori state vectors, x(-)     */
static m_elem  *state_post;        /* ptr to aposteriori state vectors, x(+) */

static m_elem  *iter_state0;
static m_elem  *iter_state1;

static m_elem  **cov_pre;          /* ptr to apriori covariance matrix, P(-) */
static m_elem  **cov_post;         /* ptr to apriori covariance matrix, P(-) */
static m_elem  **sys_noise_cov;    /* system noise covariance matrix (GQGt)  */
static m_elem  **mea_noise_cov;    /* measurement noise variance vector (R)  */

static m_elem  **sys_transfer;     /* system transfer function (Phi)    */
static m_elem  **mea_transfer;     /* measurement transfer function (H) */

static m_elem  **kalman_gain;      /* The Kalman Gain matrix (K) */

int            global_step = 0;    /* the current step number (k) */
int            measurement_size;   /* number of elems in measurement */
int            state_size;         /* number of elements in state    */

/*  Temporary variables, declared statically to avoid lots of run-time
    memory allocation.      */

static m_elem  *z_estimate;        /* a measurement_size x 1 vector */
static m_elem  **temp_state_state; /* a state_size x state_size matrix */
static m_elem  **temp_meas_state;  /* a measurement_size x state_size matrix */
static m_elem  **temp_meas_meas;   /* a measurement_size squared matrix */
static m_elem  **temp_meas_2;      /* another one ! */

/*  Prototypes of internal functions  */

static void alloc_globals( int num_state,
			  int num_measurement );
static void update_system( m_elem *z, m_elem *x_minus,
			 m_elem **kalman_gain, m_elem *x_plus );
static void estimate_prob( m_elem **P_post, m_elem **Phi, m_elem **GQGt,
			  m_elem **P_pre );
static void update_prob( m_elem **P_pre, m_elem **R, m_elem **H,
			m_elem **P_post, m_elem **K );
static void take_inverse( m_elem **in, m_elem **out, int n );
static m_elem calc_state_change( m_elem *a, m_elem *b );


/******************************************************************

  Linear Kalman Filtering

  kalman_init()
  This function initializes the kalman filter.  Note that for a
  straight-forward (linear) Kalman filter, this is the only place that
  K and P are computed...      */

void kalman_init( m_elem **GQGt, m_elem **Phi, m_elem **H, m_elem **R,
		m_elem **P, m_elem *x, int num_state, int num_measurement )
{
  alloc_globals( num_state, num_measurement );

  /*  Init the global variables using the arguments.  */

  vec_copy( x, state_post, state_size );
  mat_copy( P, cov_post, state_size, state_size );

  sys_noise_cov = GQGt;
  mea_noise_cov = R;

  sys_transfer = Phi;
  mea_transfer = H;

  /*****************  Gain Loop  *****************/

  estimate_prob( cov_post, sys_transfer, sys_noise_cov, cov_pre );
  update_prob( cov_pre, mea_noise_cov, mea_transfer, cov_post, kalman_gain );
}


/*  kalman_step()
    This function takes a set of measurements, and performs a single
    recursion of the straight-forward kalman filter.
*/

void kalman_step( m_elem *z_in )
{
  /**************  Estimation Loop  ***************/

  apply_system( state_post, state_pre );
  update_system( z_in, state_pre, kalman_gain, state_post );

  global_step++;
}

/*  kalman_get_state
    This function returns a pointer to the current estimate (a posteriori)
    of the system state.         */

m_elem *kalman_get_state( void )
{
  return( state_post );
}

/******************************************************************

  Non-linear Kalman Filtering

  extended_kalman_init()
  This function initializes the extended kalman filter.
*/

void extended_kalman_init( m_elem **GQGt, m_elem **R, m_elem **P, m_elem *x,
			  int num_state, int num_measurement )
{
#ifdef PRINT_DEBUG
  printf( "ekf: Initializing filter\n" );
#endif

  alloc_globals( num_state, num_measurement );

  sys_transfer = matrix( 1, num_state, 1, num_state );
  mea_transfer = matrix( 1, num_measurement, 1, num_state );

  /*  Init the global variables using the arguments.  */

  vec_copy( x, state_post, state_size );
  vec_copy( x, state_pre, state_size );
  mat_copy( P, cov_post, state_size, state_size );
  mat_copy( P, cov_pre, state_size, state_size );

  sys_noise_cov = GQGt;
  mea_noise_cov = R;
}


/*  extended_kalman_step()
    This function takes a set of measurements, and performs a single
    recursion of the extended kalman filter.
*/

void extended_kalman_step( m_elem *z_in )
{
#ifdef PRINT_DEBUG
  printf( "ekf: step %d\n", global_step );
#endif
  /*****************  Gain Loop  *****************
    First, linearize locally, then do normal gain loop    */

  generate_system_transfer( state_pre, sys_transfer );
  generate_measurement_transfer( state_pre, mea_transfer );

  estimate_prob( cov_post, sys_transfer, sys_noise_cov, cov_pre );
  update_prob( cov_pre, mea_noise_cov, mea_transfer, cov_post, kalman_gain );

  /**************  Estimation Loop  ***************/

  apply_system( state_post, state_pre );
  update_system( z_in, state_pre, kalman_gain, state_post );

  global_step++;
}


/* iter_ext_kalman_init()
   This function initializes the iterated extended kalman filter
*/

void iter_ext_kalman_init( m_elem **GQGt, m_elem **R, m_elem **P, m_elem *x,
			  int num_state, int num_measurement )
{
#ifdef PRINT_DEBUG
  printf( "iekf: Initializing filter\n" );
#endif

  alloc_globals( num_state, num_measurement );

  iter_state0  = vector( 1, num_state );
  iter_state1  = vector( 1, num_state );
  sys_transfer = matrix( 1, num_state, 1, num_state );
  mea_transfer = matrix( 1, num_measurement, 1, num_state );

  /*  Init the global variables using the arguments.  */

  vec_copy( x, state_post, state_size );
  vec_copy( x, state_pre, state_size );
  mat_copy( P, cov_post, state_size, state_size );
  mat_copy( P, cov_pre, state_size, state_size );

  sys_noise_cov = GQGt;
  mea_noise_cov = R;
}

/*  iter_ext_kalman_step()
    This function takes a set of measurements, and iterates over a single
    recursion of the extended kalman filter.
*/

void iter_ext_kalman_step( m_elem *z_in )
{
  int     iteration = 1;
  m_elem  est_change;
  m_elem  *prev_state;
  m_elem  *new_state;
  m_elem  *temp;

  generate_system_transfer( state_pre, sys_transfer );
  estimate_prob( cov_post, sys_transfer, sys_noise_cov, cov_pre );
  apply_system( state_post, state_pre );

  /*  Now iterate, updating the probability and the system model
      until no change is noticed between iteration steps      */

  prev_state = iter_state0;
  new_state  = iter_state1;

  generate_measurement_transfer( state_pre, mea_transfer );
  update_prob( cov_pre, mea_noise_cov, mea_transfer,
	      cov_post, kalman_gain );
  update_system( z_in, state_pre, kalman_gain, prev_state );
  est_change = calc_state_change( state_pre, prev_state );

  while( (est_change < ITERATION_THRESHOLD) &&
	(iteration++ < ITERATION_DIVERGENCE) )
    {
#ifdef DEBUG_ITER      
      print_vector( "\titer state", prev_state, 10 );
#endif
      /*  Update the estimate  */

      generate_measurement_transfer( prev_state, mea_transfer );
      update_prob( cov_pre, mea_noise_cov, mea_transfer,
		  cov_post, kalman_gain );
      update_system( z_in, prev_state, kalman_gain, new_state );
      est_change = calc_state_change( prev_state, new_state );

      temp = prev_state;
      prev_state = new_state;
      new_state = temp;
    }

  vec_copy( prev_state, state_post, state_size );

#ifdef PRINT_DEBUG
  printf( "iekf: step %3d, %2d iters\n", global_step, iteration );
#endif
  global_step++;
}

/************************************************************

   Internal Functions, defined in order of appearance

   alloc_globals()
   This function allocates memory for the global variables used by this
   code module.     */

static void alloc_globals( int num_state, int num_measurement )
{
#ifdef PRINT_DEBUG
  printf( "ekf: allocating memory\n" );
#endif
  state_size = num_state;
  measurement_size = num_measurement;

  /*  Allocate some global variables.  */

  state_pre = vector( 1, state_size );
  state_post = vector( 1, state_size );
  cov_pre = matrix( 1, state_size, 1, state_size );
  cov_post = matrix( 1, state_size, 1, state_size );
  kalman_gain = matrix( 1, state_size, 1, measurement_size );

  /*  Alloc some temporary variables   */

  z_estimate = vector( 1, measurement_size );
  temp_state_state = matrix( 1, state_size, 1, state_size );
  temp_meas_state = matrix( 1, measurement_size, 1, state_size );
  temp_meas_meas = matrix( 1, measurement_size, 1, measurement_size );
  temp_meas_2 = matrix( 1, measurement_size, 1, measurement_size );
}
   
/* update_system()
   This function generates an updated version of the state estimate,
   based on what we know about the measurement system.       */

static void update_system( m_elem *z, m_elem *x_pre,
			 m_elem **K, m_elem *x_post )
{
#ifdef PRINT_DEBUG
  printf( "ekf: updating system\n" );
#endif

  apply_measurement( x_pre, z_estimate );
  vec_sub( z, z_estimate, z_estimate, measurement_size );
  mat_mult_vector( K, z_estimate, x_post, state_size, measurement_size );
  vec_add( x_post, x_pre, x_post, state_size );
}

/* estimate_prob()
   This function estimates the change in the variance of the state
   variables, given the system transfer function.   */

static void estimate_prob( m_elem **P_post, m_elem **Phi, m_elem **GQGt,
			  m_elem **P_pre )
{
#ifdef PRINT_DEBUG
  printf( "ekf: estimating prob\n" );
#endif

  mat_mult_transpose( P_post, Phi, temp_state_state,
		     state_size, state_size, state_size );
  mat_mult( Phi, temp_state_state, P_pre,
		     state_size, state_size, state_size );
  mat_add( P_pre, GQGt, P_pre, state_size, state_size );
}


/*  update_prob()
    This function updates the state variable variances.
    Inputs:
    P_pre - the apriori probability matrix ( state x state )
    R     - measurement noise covariance   ( meas x meas )
    H     - the measurement transfer matrix ( meas x state )
    Outputs:
    P_post - the aposteriori probability matrix (state x state )
    K     - the Kalman gain matrix ( state x meas )
*/

static void update_prob( m_elem **P_pre, m_elem **R, m_elem **H,
			m_elem **P_post, m_elem **K )
{
#ifdef PRINT_DEBUG
  printf( "ekf: updating prob\n" );
#endif
#ifdef DIV_DEBUG
  print_matrix( "P", P_pre, state_size, state_size );
#endif
  mat_mult( H, P_pre, temp_meas_state,
	   measurement_size, state_size, state_size );
  mat_mult_transpose( H, temp_meas_state, temp_meas_meas,
		     measurement_size, state_size, measurement_size );
  mat_add( temp_meas_meas, R, temp_meas_meas,
	  measurement_size, measurement_size );

  take_inverse( temp_meas_meas, temp_meas_2, measurement_size );

#ifdef DIV_DEBUG
  print_matrix( "1 / (HPH + R)", temp_meas_2,
	       measurement_size, measurement_size );
#endif
  mat_transpose_mult( temp_meas_state, temp_meas_2, K,
		     state_size, measurement_size, measurement_size );

/*
  print_matrix( "Kalman Gain", K, state_size, measurement_size );
*/
  mat_mult( K, temp_meas_state, temp_state_state,
	   state_size, measurement_size, state_size );
#ifdef PRINT_DEBUG
  printf( "ekf: updating prob 3\n" );
#endif
  mat_add( temp_state_state, P_pre, P_post, state_size, state_size );
}


static void take_inverse( m_elem **in, m_elem **out, int n )
{
#ifdef PRINT_DEBUG
  printf( "ekf: calculating inverse\n" );
#endif
  /*  Nothing fancy for now, just a Gauss-Jordan technique,
      with good pivoting (thanks to NR).     */

  gaussj( in, n, out, 0 );  /* out is SCRATCH  */
  mat_copy( in, out, n, n );
}

static m_elem calc_state_change( m_elem *a, m_elem *b )
{
  int     m;
  m_elem  acc = 0.0;

  for( m = 1; m <= state_size; m++ )
    {
      acc += fabs( a[m] - b[m] );
    }

  return( acc );
}