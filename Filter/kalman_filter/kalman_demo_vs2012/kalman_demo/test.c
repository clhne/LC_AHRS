/*  test.c

    This file contains the code wrapper for processing data using the
    Kalman filter routines provided in kalman.c and kalman_camera.c.

    Usage:  test <features_fname> <num_points> <num_frames>
                  [-o <output_fname>][-c <covariance_fname>]
		  [-l <improved_est_output_fname> ]
		  [-monte_carlo <num_trials>]
		  [-iekf|-ekf][-debug]

    J. Watlington, 11/27/95

    Modified:
    12/4/95  Added ekf/iekf and monte_carlo flags, cause I got this sucker
             to converge !  The measurement linearization was the culprit.
    12/9/95  Added initial state estimation, using an iterative Levenberg-
             Marquardt minimization technique applied alternatively to the
	     camera parameters and the feature point location.
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kalman.h"
#include "test.h"

#define HELLO "Camera Motion Estimator\n"

#define IMPROVE_DEBUG

/*  Constants for the Levenberg-Marquardt minimization to improve the
    initial estimate.    */

#define FEATURE_MOTION_THRESHOLD    1.0e-4
#define FINAL_CHI_THRESHOLD         1.0e-7
#define MAX_LEVENBERG_ITER          40
#define FIXED_FEATURE               ( STATE_FEATURE_START + 2 )
#define DEFAULT_SAMPLE_STD_DEV      1.0

/*  Default filename suffixes.  A . isn't used so MatLab can load results */

#define COVARIANCE_SUFFIX   "_cov"
#define OUTPUT_SUFFIX       "_out"

/*  Global variables reflecting command line options  */

int   debug = 0;
char  feature_fname[ FILENAME_MAX ];
char  covariance_fname[ FILENAME_MAX ];
char  output_fname[ FILENAME_MAX ];
char  estimate_fname[ FILENAME_MAX ] = "";
int   feature_size;
int   num_states;
int   num_frames;
int   iterate = 0;
int   num_trials = 1;

/*   Some components of the state are maintained globally, outside of
     the framework of a particular algorithm.    */

m_elem   global_rotation[ QUATERNION_SIZE ];

/*   Random Number Generator state variable   */

extern float gasdev( long *idum );  /* random # gen, in random.c */
long  rseed = -1;

char  dbgstr[ 64 ];

/*  Prototypes of local functions  */

void parse_arguments( int argc, char **argv );
void usage( char *str );
void alloc_buffers( int num_features, int num_states );
void free_buffers( void );

void load_features( char *name, int num_features, int num_frames, 
		   m_elem **features );
void load_noise_cov( char *name, int num_features, m_elem **R );
void add_noise( int num_steps, int num_features, m_elem **R,
	       m_elem **features, m_elem **noisy_features );
void save_track( char *name, int num_steps, int num_states,
		m_elem **trajectory );
void init_system_parms( int num_states, int num_features, m_elem **Q );
void init_estimate( int num_states, int feature_size,
		   m_elem *x, m_elem **P, m_elem **features );
void improve_estimate( int state_size, int feature_size, int num_frames,
		      m_elem *state, m_elem **P, m_elem **features );

extern void eval_camera( m_elem x, m_elem a[], m_elem *yfit, m_elem dyda[],
		 int state_size );

void save_estimate( char *name, int num_states,	m_elem *state );


void main( int argc, char **argv )
{
  int      trial, time, i; /* various iteration variables   */
  m_elem   *track;        /* ptr to state vector of kalman filter */
  m_elem   **features;    /* features being tracked        */
  m_elem   **clean_features;    /* features being tracked        */
  m_elem   ***trajectory_set; /* set of extracted motion parameters   */
  m_elem   **trajectory;  /* mean of extracted motion parameters   */
  m_elem   *traj_fptr;    /* ptr to single row of ext. motion parameters   */

                    /*  n = feature_size   m = num_states  */
  m_elem   **Q;     /*  System noise covariance (mxm)      */
  m_elem   **R;     /*  Measurement noise covariance (nxn) */
  m_elem   **P;     /*  Estimate Covariance        (mxm)   */
  m_elem   *x;      /*  Starting state             (1xm)   */

  parse_arguments( argc, argv );   /*  Parse the command line arguments  */

  /*  Allocate memory for system, measurement, and state variables.
      Then either init them algorithmically or from data stored in a file.  */

  Q = matrix( 1, num_states, 1, num_states );
  R = matrix( 1, feature_size, 1, feature_size );
  P = matrix( 1, num_states, 1, num_states );
  x = vector( 1, num_states );
  features = matrix( 1, num_frames, 1, feature_size );
  clean_features = matrix( 1, num_frames, 1, feature_size );
  trajectory = matrix( 1, num_frames, 1, num_states );

  if( (trajectory_set = (m_elem ***)malloc((num_trials+1)*sizeof(m_elem**)))
     == NULL )
    {
      fprintf( stderr, "Unable to allocate tiny amount of memory !\n" );
      exit( -1 );
    }

  for( trial = 1; trial <= num_trials; trial++ )
    trajectory_set[ trial ] = matrix( 1, num_frames, 1, num_states );

  load_features( feature_fname, feature_size, num_frames, clean_features );
  load_noise_cov( covariance_fname, feature_size, R );

  init_system_parms( num_states, feature_size, Q );

  for( trial = 1; trial <= num_trials; trial++ )
    {
      if( num_trials > 1 )
	printf( "Trial #%d\n", trial );

      /*  For each test run, add shaped noise to the feature input,
	  and initialize the kalman filter package.   */

      add_noise( num_frames, feature_size, R, clean_features,
		features );

      init_estimate( num_states, feature_size, x, P, features );
      improve_estimate( num_states, feature_size, num_frames, x, P, features );

      if( iterate )
	iter_ext_kalman_init( Q, R, P, x, num_states, feature_size );
      else
	extended_kalman_init( Q, R, P, x, num_states, feature_size );
    
      /*  For each frame in the test run, perform one estimation and
	  copy the results into the trajectory history   */

      for( time = 1; time <= num_frames; time++ )
	{
	  if( iterate )
	    iter_ext_kalman_step( &features[ time ][0] );
	  else
	    extended_kalman_step( &features[ time ][0] );

	  track = kalman_get_state();
	  if( debug )
	    {
	      sprintf( dbgstr, "State @ t = %d", time );
	      print_vector( dbgstr, track, SFM_STATE_SIZE + 6 );
	      print_quaternion( "global rot.", global_rotation );
	    }
	  traj_fptr = trajectory_set[ trial ][ time ];
	  for( i = 1; i <= num_states; i++ )
	    traj_fptr[ i ] = track[ i ];
	}
    }

  /*  Calculate the Mean and Variance over the entire set of trials, and
      report that !     */

  for( time = 1; time <= num_frames; time++ )
    for( i = 1; i < num_states; i++ )
      trajectory[ time ][ i ] = 0.0;

  for( trial = 1; trial <= num_trials; trial++ )
    for( time = 1; time <= num_frames; time++ )
      for( i = 1; i < num_states; i++ )
	trajectory[ time ][ i ] += trajectory_set[ trial ][ time ][ i ];

  if( num_trials > 1 )
    for( time = 1; time <= num_frames; time++ )
      for( i = 1; i < num_states; i++ )
	trajectory[ time ][ i ] = trajectory[ time ][ i ] / (m_elem)num_trials;

  /*  Save the set of estimated parameters into a file  */
  
  save_track( output_fname, num_frames, num_states, trajectory );

  free_matrix( Q, 1, num_states, 1, num_states );
  free_matrix( R, 1, feature_size, 1, feature_size );
  free_matrix( P, 1, num_states, 1, num_states );
  free_vector( x, 1, num_states );
  free_matrix( features, 1, num_frames, 1, feature_size );
  free_matrix( trajectory, 1, num_frames, 1, num_states );
  for( trial = 1; trial <= num_trials; trial++ )
    free_matrix( trajectory_set[ trial ], 1, num_frames, 1, num_states );
  free( trajectory_set );
}

void load_features( char *name, int num_features, int num_steps,
		   m_elem **features )
{
  FILE    *fptr;
  int     frame;
  int     sample;
  double  datax;
  double  datay;

  if( debug )
    printf( "Loading features from %s\n", name );

  if( (fptr = fopen( name, "r" )) == NULL )
    {
      printf( "load_features: Unable to open %s for reading !\n",
	     name );
      exit( -1 );
    }

  for( frame = 1; frame <= num_steps; frame++ )
    for( sample = 1; sample <= num_features; sample += 2 )
      {
	if( fscanf( fptr, " %lf %lf", &datax, &datay) != 2 )
	  {
	    printf("load_features: Error reading %s ! (%d frames %d points read )\n",
		   name, frame, sample/2 );
	    fclose( fptr );
	    exit( -1 );
	  }
	features[ frame ][ sample ] = (m_elem)datax;
	features[ frame ][ sample+1 ] = (m_elem)datay;
      }

  fclose( fptr );
}


/*  load_noise_cov
    The name sez it all.      */

void load_noise_cov( char *name, int num_features, m_elem **R )
{
  FILE    *fptr;
  int     row;
  int     sample;
  double  data;

  if( debug )
    printf( "Loading measurement covariances from %s\n", name );

  if( (fptr = fopen( name, "r" )) == NULL )
    {
      printf( "load_noise_cov: Unable to open %s for reading !\n",
	     name );
      exit( -1 );
    }

  for( row = 1; row <= num_features; row++ )
    for( sample = 1; sample <= num_features; sample += 1 )
      {
	if( fscanf( fptr, " %lf", &data ) != 1 )
	  {
	    printf("load_noise_cov: Error reading %s ! (%d:%d points read )\n",
		   name, row, sample );
	    fclose( fptr );
	    exit( -1 );
	  }
	R[ row ][ sample ] = (m_elem)data;
      }

  fclose( fptr );
}


/*  init_system_parms
    This function initializes some system functions, such as
    the transfer and noise functions, and the measurement
    transfer function.              */

void init_system_parms( int num_states, int num_features, m_elem **Q )
{
  int    x, y;

  /*  Initialize the system noise covariance matrix   */

  for( y = 1; y <= num_states; y++ )
    for( x = 1; x <= num_states; x++ )
      Q[ y ][ x ] = 0;

  Q[ STATE_Tx ][ STATE_Tx ] = COV_SYSTEM_Tx_Tx;
  Q[ STATE_Ty ][ STATE_Ty ] = COV_SYSTEM_Ty_Ty;
  Q[ STATE_Tz ][ STATE_Tz ] = COV_SYSTEM_Tz_Tz;

  Q[ STATE_Wx ][ STATE_Wx ] = COV_SYSTEM_Wx_Wx;
  Q[ STATE_Wy ][ STATE_Wy ] = COV_SYSTEM_Wy_Wy;
  Q[ STATE_Wz ][ STATE_Wz ] = COV_SYSTEM_Wz_Wz;

  Q[ STATE_B ][ STATE_B ]   = COV_SYSTEM_BETA;

  for( x = STATE_FEATURE_START; x <= num_states; x++ )
    Q[ x ][ x ] = COV_SYSTEM_Fn_Fn;

/*  Q[ STATE_FEATURE_START ][ STATE_FEATURE_START ] = 0.0;  */
/*  Q[ STATE_FEATURE_START + 2 ][ STATE_FEATURE_START + 2 ] = 0.0;  */
}

/*  improve_estimate
    This function uses the initial estimate as the starting point for
    a Levenberg-Marquardt gradient descent algorithm, which is
    alternatively iterated over the camera parameters and the feature
    point location (structure).
*/

void improve_estimate( int state_size, int feature_size, int num_frames,
		      m_elem *state, m_elem **P, m_elem **features )
{
  int         right_index = 1;
  int         i, f;
  m_elem      *left;
  m_elem      *right;
  m_elem      threshold;
  m_elem      diff = 0.0;
  m_elem      max_diff = 0.0;
  int         max_diff_right = num_frames;

  m_elem      lambda;
  m_elem      chisq, old_chisq, delta_chisq;
  m_elem      **alpha;
  m_elem      **cov;
  m_elem      *x_dumb, *y, *sig;
  int         *state_mask;
  int         watchdog;

  y = vector( 1, feature_size * 2 );
  x_dumb = vector( 1, feature_size * 2 );
  sig = vector( 1, feature_size * 2 );
  alpha = matrix( 1, state_size, 1, state_size );
  cov = matrix( 1, state_size, 1, state_size );
  state_mask = ivector( 1, state_size );

  for( i = 1; i <= feature_size*2; i++ )
    sig[ i ] = DEFAULT_SAMPLE_STD_DEV;

  /*  The first task is to select which frames to use for the feature
      detection.  We don't want to automatically use the first two,
      since there may not be enough difference between the camera views
      to provide a good estimate of the feature point location in three
      space.    */

  left = features[ 1 ];
  threshold = feature_size * FEATURE_MOTION_THRESHOLD;

  while( (diff < threshold) && (++right_index <= num_frames) )
    {
      /*  calculate the difference between the two sets of measurements. */

      right = features[ right_index ];
      diff = 0.0;
      for( f = 1; f <= feature_size; f++ )
	{
	  lambda = left[ f ] - right[ f ];
	  diff += lambda * lambda;
	}

      if( diff > max_diff )
	{
	  max_diff = diff;
	  max_diff_right = right_index;
	}
    }

  if( right_index > num_frames )
    {
      printf( "improve_est: This sequence is really boring (%lg %lg max)\n",
	      diff, max_diff );  /*  but we will give it a try anyway !  */
      right = features[ max_diff_right ];
      right_index = max_diff_right;
    }

  if( debug )
    printf( "improve_est: using frames 1 and %d\n", right_index );

  /*  Build the arrays representing the data being fitted.  This consists
      of measurements from two different frames (left and right).  The
      left frame data is not affected by the motion estimation, and is
      only used for optimizing the structure estimate.   */

  f = 1;
  for( i = 1; i <= feature_size; i++ )
    y[ f++ ] = right[ i ];

  for( i = 1; i <= feature_size; i++ )
    y[ f++ ] = left[ i ];

  mat_copy( P, cov, state_size, state_size );

  /*  Generate a mask, which will indicate the state components to
      be minimized over for the motion estimation and the structure est. */

  for( i = STATE_Tx; i <= state_size; i++ )
    state_mask[ i ] = 1;

  state_mask[ STATE_B ] = 0;
  state_mask[ FIXED_FEATURE ] = 0;

  /*  The Levenberg-Marquardt minimization routine provided by NR
      is designed for scalar functions, but easily extends by
      providing an index as the x input, and allowing the function
      application (in this case eval_camera() ) global access to the
      inputs.  Since our inputs are also components of the state vector,
      we have it really easy.    */

  for( i = 1; i <= (feature_size*2); i ++ )
    x_dumb[ i ] = (m_elem)i;

  lambda = -1.0;         /*  this signals to mrqmin to init everything. */
  chisq = old_chisq = (m_elem)( 1000.0 );
  delta_chisq = -FINAL_CHI_THRESHOLD * 2.0;
  watchdog = 0;

  while( ((delta_chisq > 0.0) || (delta_chisq < -FINAL_CHI_THRESHOLD))
	&& (watchdog++ < MAX_LEVENBERG_ITER) )
    {
      mrqmin( x_dumb, y, sig, feature_size*2, state, state_mask,
	     state_size, cov, alpha, &chisq, eval_camera, &lambda );
	  
      delta_chisq = chisq - old_chisq;
      old_chisq = chisq;
	  
      if( debug )
	{
	  sprintf( dbgstr, "Optimizing state estimate:  (pass %d chisq %lg)",
		  watchdog, chisq );
	  print_vector( dbgstr, state, state_size );
	}
    }
      
  lambda = 0;
  mrqmin( x_dumb, y, sig, feature_size*2, state, state_mask,
	 state_size, cov, alpha, &chisq, eval_camera, &lambda );

  if( watchdog >= MAX_LEVENBERG_ITER )
    printf( "improve_est: Maximum number of iterations reached\n" );

  if( estimate_fname[0] != '\0' )
    save_estimate( estimate_fname, state_size, state );

  /*  We really only care about the estimate of structure, since the
      motion estimate was probably (hopefully) for a frame far into the
      sequence.  Here we zero out the motion estimate.    */

  for( i = STATE_Tx; i <= STATE_Wz; i++ )
    state[ i ] = 0.0;

  if( debug )
    print_vector( "Final (Initial ?) state estimate:", state, state_size );

  free_ivector( state_mask, 1, state_size );
  free_vector( y, 1, feature_size * 2 );
  free_vector( x_dumb, 1, feature_size * 2 );
  free_matrix( alpha, 1, state_size, 1, state_size );
  free_matrix( cov, 1, state_size, 1, state_size );
}


/*  init_estimate
    This function initializes the estimate of the state, which includes
    both a mean (x) and a covariance (P).    */

void init_estimate( int num_states, int feature_size,
		   m_elem *x, m_elem **P, m_elem **features )
{
  int  i, j;

  if( debug )
    printf( "Initializing the estimate\n" );

  /*  Init the covariance matrix to zeroes   */

  for( i = 1; i <= num_states; i++ )
    for( j = 1; j <= num_states; j++ )
      {
	P[ i ][ j ] = 0;
      }

  /*  Some of the initial estimates are arbitrarily set to zero,
      such as the initial camera translation and rotation    */

  for( i = STATE_Tx; i <= STATE_Tz; i++ )
    {
      x[ i ] = 0;
      P[ i ][ i ] = COV_ESTIMATE_Td_Td;
    }

  for( i = STATE_Wx; i <= STATE_Wz; i++ )
    {
      x[ i ] = 0;
      P[ i ][ i ] = COV_ESTIMATE_Wd_Wd;
    }

  /*  Init the global rotation quaternion  */

  global_rotation[0] = 1.0;    /*  q0  */
  global_rotation[1] = 0.0;    /*  q1  */
  global_rotation[2] = 0.0;    /*  q2  */
  global_rotation[3] = 0.0;    /*  q3  */

  /*  The focal length is estimated to be a common value   */

  x[ STATE_B ] = ESTIMATE_BETA;
  P[ STATE_B ][ STATE_B ] = COV_ESTIMATE_BETA;

  /*  Assume a single depth for all points, and calculate starting
      value of X and Y for them.   */

#define Z_FACTOR  ((ESTIMATE_WORLD_Z * ESTIMATE_BETA + 1)/ARBITRARY_SCALE)

  for( j = 1, i = STATE_FEATURE_START; j <= feature_size; j += 2, i += 3 )
    {
      x[ i ] = features[ 1 ][ j ] * Z_FACTOR;           /*  X estimate  */
      x[ i + 1 ] = features[ 1 ][ j + 1 ] * Z_FACTOR;   /*  Y estimate  */
      x[ i + 2 ] = ESTIMATE_WORLD_Z;

      P[ i ][ i  ] = COV_ESTIMATE_Fn_Fn;
      P[ i + 1 ][ i + 1 ] = COV_ESTIMATE_Fn_Fn;
      P[ i + 2 ][ i + 2 ] = COV_ESTIMATE_Fz_Fz;
    } 

  P[ FIXED_FEATURE ][ FIXED_FEATURE ] = 0.0;

  if( debug )
    print_vector( "Initial state estimate", x, num_states );
}

/*  add_noise
    This function adds gaussian noise with zero mean and specified
    variance to the feature inputs.         */

void add_noise( int num_steps, int num_features, m_elem **R,
	       m_elem **features, m_elem **noisy_features )
{
  int      i, t;
  m_elem   temp;

  if( debug )
    printf( "Adding noise to input data\n" );

   /*  For now, assume that we will only be specifying a diagonal
       covariance (ie. a variance vector).     */

  for( t = 1; t <= num_steps; t++ )
    for( i = 1; i <= num_features; i++ )
      {
	temp = (m_elem)gasdev( &rseed );
	noisy_features[t][i] = features[t][i]
	  + (temp * R[ i ][ i ]);
      }
}


void save_estimate( char *name, int num_states,	m_elem *state )
{
  FILE     *fptr;
  int      frame, state_var;

  if( (fptr = fopen( name, "w" )) == NULL )
    {
      printf( "save_track: Unable to open %s for writing !\n", name );
      exit( -1 );
    }

  for( state_var = 1; state_var <= num_states; state_var++ )
    fprintf( fptr, "%lf ", (double)state[ state_var ] );

  fprintf( fptr, "\n" );
  fclose( fptr );
}

void save_track( char *name, int num_steps, int num_states,
		m_elem **trajectory )
{
  FILE     *fptr;
  int      frame, state_var;

  if( (fptr = fopen( name, "w" )) == NULL )
    {
      printf( "save_track: Unable to open %s for writing !\n", name );
      exit( -1 );
    }

  for( frame = 1; frame <= num_steps; frame++ )
    {
      for( state_var = 1; state_var <= num_states; state_var++ )
	fprintf( fptr, "%lf ", (double)trajectory[ frame ][ state_var ] );
      fprintf( fptr, "\n" );
    }

  fclose( fptr );
}


void parse_arguments( int argc, char **argv )
{
  int   i;

  printf( HELLO );

  if( argc < 4 )
    usage( "Not enough arguments" );
  else
    {
      /* Parse the three required arguments, then calculate some
	 default values for file names, in case the user decides not
	 to provide them.  */

      sscanf( argv[1], "%s", feature_fname );
      sscanf( argv[2], "%d", &feature_size );
      sscanf( argv[3], "%d", &num_frames );
      sprintf( output_fname, "%s%s", feature_fname, OUTPUT_SUFFIX );
      sprintf( covariance_fname, "%s%s", feature_fname, COVARIANCE_SUFFIX );

      /*  If more than the required number of arguments was provided,
	  take a look and see if any of them make sense.   */

      i = 4;
      while( i < argc )
	{
	  if( argv[i][0] = '-' )
	    {
	      switch( argv[i][1] ) {
	      case 'o':      /* -o <output_fname>  */
		if( ++i >= argc )
		  usage( "-out <fname> missing argument" );
		sscanf( argv[i], "%s", output_fname );
		break;
	      case 'c':      /* -c <covariance_fname>  */
		if( ++i >= argc )
		  usage( "-cov <fname> missing argument" );
		sscanf( argv[i], "%s", covariance_fname );
		break;
	      case 'l':      /* -l <improved_est_fname> */
		if( ++i >= argc )
		  usage( "-l <improved_est_fname> missing argument" );
		sscanf( argv[i], "%s", estimate_fname );
		break;
	      case 'm':      /* -monte_carlo <num_trials>  */
		if( ++i >= argc )
		  usage( "-monte_carlo <num_trials> missing argument" );
		sscanf( argv[i], "%d", &num_trials );
		break;
	      case 'd':      /* -debug   */
		debug = 1 - debug;
		break;
	      case 'i':        /*  -iekf  */
		iterate = 1;
		break;
	      case 'e':        /*  -ekf   */
		iterate = 0;
		break;
	      default:
		usage( "unknown argument" );
	      }
	    }
	  else
	    {
	      usage( "syntax error" );
	    }
	  i++;
	}
    }

  /*  Each feature provides two measurements, and three values in the
      state vector.    */

  num_states = feature_size * 3 + SFM_STATE_SIZE;
  feature_size = feature_size * 2;

  if( iterate == 1 )
    {
      printf( "Implementing an Iterated Extended Kalman Filter\n" );
    }
  else
    {
      printf( "Implementing an Extended Kalman Filter\n" );
    }
}

void usage( char *str )
{
  printf( "test: %s\n", str );
  printf( "usage: test <feature_fname> <num_features> <num_frames>\n" );
  printf( "\t[ -o <output_fname> ][ -c <covariance_fname> ]\n" );
  printf( "\t[ -l <improved_est_output_fname> ]\n" );
  printf( "\t[ -monte_carlo <num_trials> ][ -iekf | -ekf ][ -debug ]\n" );
  exit( -1 );
}