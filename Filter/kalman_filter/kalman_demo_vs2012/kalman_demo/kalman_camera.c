/*  kalman_camera.c
    
    This file contains code segments for a Kalman Filter that characterize
    a particular system and measurement model.

    In this case, the system being characterized is a imaging model
    that supports camera translation and rotation.
    
    J. Watlington, 11/28/95

    Modified:
    12/9/95  wad   Added support for camera rotation.
*/

#include <stdio.h>
#include "kalman.h"
#include "test.h"

/****************  System Model Manifestations   **************
  
  apply_system()
  This function is called to predict the next state given a
  current state.  It is an evaluation of the system's transfer
  function.  If used with a Linear Kalman filter, this should be a
  linear function, otherwise it may be non-linear.     */


void apply_system( m_elem *old_state, m_elem *new_state )
{
  int     i;

#ifdef PRINT_DEBUG
  printf( "ekf: applying system\n" );
#endif
  /*  Handle motion model - not moving for now  */

  new_state[ STATE_Tx ] = old_state[ STATE_Tx ];
  new_state[ STATE_Ty ] = old_state[ STATE_Ty ];
  new_state[ STATE_Tz ] = old_state[ STATE_Tz ];

  new_state[ STATE_Wx ] = old_state[ STATE_Wx ];
  new_state[ STATE_Wy ] = old_state[ STATE_Wy ];
  new_state[ STATE_Wz ] = old_state[ STATE_Wz ];

  /*  Update the global estimate of rotation by multiplying it
      with an "incremental quaternion" which is a linearization of
      the euler angles used in the state estimation process.          */

  quaternion_update( global_rotation, new_state[ STATE_Wx ],
		    new_state[ STATE_Wy ], new_state[ STATE_Wz ] );

  /*  Handle lens parameters - not changing    */

  new_state[ STATE_B ] = old_state[ STATE_B ];

  /*  Assume that the features aren't moving.   */
  
  for( i = STATE_FEATURE_START; i <= state_size; i++ )
    new_state[ i ] = old_state[ i ];
}

/*  generate_system_transfer
    This function generates the pseudo-transfer
    function, representing the first terms of a taylor expansion
    of the actual non-linear system transfer function.     */

void generate_system_transfer( m_elem *state, m_elem **phi )
{
  int  row, col;
  
#ifdef PRINT_DEBUG
  printf( "ekf: linearizing system transfer\n" );
#endif

  for( row = 1; row <= state_size; row++ )
    for( col = 1; col <= state_size; col++ )
      phi[ row ][ col ] = 0.0;

  for( col = 1; col <= state_size; col++ )
    phi[ col ][ col ] = 1.0;
}


#define X        1
#define Y        2
#define Z        3
#define VECSIZE  4

/****************  Measurement Model Manifestations   **************
  
  apply_measurement()
  This function is called to predict the next measurement given a
  predicted state.  It is an evaluation of the measurement's transfer
  function.                                      */

void apply_measurement( m_elem *new_state, m_elem *est_measurement )
{
  int       m, n;
  int       in_index, out_index;
  m_elem    depth;
  m_elem    Beta;
  m_elem    world_pos[ VECSIZE ];
  m_elem    camera_pos[ VECSIZE ];
  m_elem    image_pos[ VECSIZE ];
  
#ifdef PRINT_DEBUG
  printf( "ekf: estimating measurement\n" );
#endif

  /*  This is where we model the imaging system.  First, we update
      some local variables (declared static global for allocation
      reasons) which depend on state variables, like the camera
      translation and rotation matrices     */

  for( n = X; n <= Z; n++ )
    camera_pos[ n ] = new_state[ n + STATE_Tx - 1 ];

  Beta = new_state[ STATE_B ];

  /*  Now perform the image transformation for each feature   */
  
  for( m = 0, in_index = STATE_FEATURE_START, out_index = 1;
      m < measurement_size; m += 2 )
    {
      image_pos[ X ] = new_state[ in_index++ ] + camera_pos[ X ];
      image_pos[ Y ] = new_state[ in_index++ ] + camera_pos[ Y ];
      image_pos[ Z ] = (new_state[ in_index++ ] * Beta) + camera_pos[ Z ];
      depth = ARBITRARY_SCALE / ( 1 + image_pos[Z]);

      est_measurement[ out_index++ ] = image_pos[X] *  depth;
      est_measurement[ out_index++ ] = image_pos[Y] *  depth;
    }
}

/*  generate_measurement_transfer
    If non-linear, this function generates the pseudo-transfer
    function, representing the first terms of a taylor expansion
    of the actual non-linear measurement transfer function.     */

void generate_measurement_transfer( m_elem *state, m_elem **H )
{
  int       n;
  int       in_index, x_index, y_index;
  m_elem    D, D2;
  m_elem    XcD2, YcD2;
  m_elem    XcBD2, YcBD2;
  m_elem    Beta, temp;
  m_elem    x_part, y_part, z_part;
  m_elem    camera_pos[ VECSIZE ];
  m_elem    image_pos[ VECSIZE ];
  static m_elem  **R = NULL;

#ifdef PRINT_DEBUG
  printf( "ekf: linearizing measurement transfer\n" );
#endif
  /*  First, we initialize some intermediate variables.   */
  
  for( n = X; n <= Z; n++ )
    camera_pos[ n ] = state[ n + STATE_Tx - 1 ];

  /* The angular velocity information is converted to a global
     rotation, which is the one used by all equations, in the
     apply_system() step.  This is because this function may be called
     multiple time in an iterative ekf...  */

  if( R == NULL )
    R = matrix( 1, 3, 1, 3 );

  quaternion_to_rotation( global_rotation, R );

  Beta = state[ STATE_B ];  /*  the inverse of the focal length  */

  /*  Second, clear the transfer matrix to zeroes  */

  for( in_index = 1; in_index <= measurement_size; in_index++ )
    for( n = 1; n <= state_size; n++ )
      H[ in_index ][ n ] = 0.0;

  /*  Now calculate the rows of the transfer function, two
      at a time    */

  for( in_index = STATE_FEATURE_START, x_index = 1, y_index = 2;
      in_index <= state_size; x_index += 2, y_index += 2, in_index += 3 )
    {
      /*  Calculate transformed version of the coordinates  */

      image_pos[X] = camera_pos[ X ] +
	(state[ in_index + X - 1 ] * R[X][X]) +
	(state[ in_index + Y - 1 ] * R[X][Y]) +
	(state[ in_index + Z - 1 ] * R[X][Z]);
      image_pos[Y] = camera_pos[ Y ] +
	(state[ in_index + X - 1 ] * R[Y][X]) +
	(state[ in_index + Y - 1 ] * R[Y][Y]) +
	(state[ in_index + Z - 1 ] * R[Y][Z]);
      image_pos[Z] = (state[ in_index + X - 1 ] * R[Z][X]) +
	(state[ in_index + Y - 1 ] * R[Z][Y]) + 
	  (state[ in_index + Z - 1 ] * R[Z][Z]);
      
      /*  And some useful intermediate values  */

      D = ARBITRARY_SCALE / ( 1 + camera_pos[Z] + (Beta * image_pos[Z]));
      D2 = (D * D);
      XcD2 = image_pos[X] * D2;
      YcD2 = image_pos[Y] * D2;
      XcBD2 = Beta * XcD2;
      YcBD2 = Beta * YcD2;

      H[ x_index ][ STATE_Tx ] = D;          /*  The  dh() / dTx terms  */
      H[ y_index ][ STATE_Tx ] = 0;

      H[ x_index ][ STATE_Ty ] = 0;          /*  The  dh() / dTy terms  */
      H[ y_index ][ STATE_Ty ] = D;

      H[ x_index ][ STATE_Tz ] = -XcD2;      /* dh() / dTz terms  */
      H[ y_index ][ STATE_Tz ] = -YcD2;

      /*  Calculate the rotational derivatives for the x measurement  */

      temp = - 1 / (2*global_rotation[0]);
      x_part = 	state[ STATE_Wx ] * temp;
      y_part = 	state[ STATE_Wy ] * temp;
      z_part = 	state[ STATE_Wz ] * temp;

      temp = (D*((global_rotation[0]*state[in_index+X-1])
		 - (global_rotation[3]*state[in_index+Y-1])
		 + (global_rotation[2]*state[in_index+Z-1])))
	- (XcBD2*((-global_rotation[2]*state[in_index+X-1])
		  + (global_rotation[1]*state[in_index+Y-1])
		  + (global_rotation[0]*state[in_index+Z-1])));

      H[ x_index ][ STATE_Wx ] = (x_part * temp) +
	(D*((global_rotation[1]*state[in_index+X-1])
	    + (global_rotation[2]*state[in_index+Y-1])
	    + (global_rotation[3]*state[in_index+Z-1])))
	  - (XcBD2*((global_rotation[3]*state[in_index+X-1])
		    +(global_rotation[0]*state[in_index+Y-1])
		    -(global_rotation[1]*state[in_index+Z-1])));
      
      H[ x_index ][ STATE_Wy ] = (y_part * temp) +
	(D*((-global_rotation[2]*state[in_index+X-1])
	    + (global_rotation[1]*state[in_index+Y-1])
	    + (global_rotation[0]*state[in_index+Z-1])))
	  - (XcBD2*(-(global_rotation[0]*state[in_index+X-1])
		    +(global_rotation[3]*state[in_index+Y-1])
		    -(global_rotation[2]*state[in_index+Z-1])));

      H[ x_index ][ STATE_Wz ] = (z_part * temp) +
	(D*((-global_rotation[3]*state[in_index+X-1])
	    - (global_rotation[0]*state[in_index+Y-1])
	    + (global_rotation[1]*state[in_index+Z-1])))
	  - (XcBD2*((global_rotation[1]*state[in_index+X-1])
		    +(global_rotation[2]*state[in_index+Y-1])
		    +(global_rotation[3]*state[in_index+Z-1])));

      /*  Calculate the rotational derivatives for the y measurement  */

      temp = (D*((global_rotation[3]*state[in_index+X-1])
		 + (global_rotation[0]*state[in_index+Y-1])
		 - (global_rotation[1]*state[in_index+Z-1])))
	- (YcBD2*((-global_rotation[2]*state[in_index+X-1])
		  + (global_rotation[1]*state[in_index+Y-1])
		  + (global_rotation[0]*state[in_index+Z-1])));

      H[ y_index ][ STATE_Wx ] = (x_part * temp) +
	(D*((global_rotation[2]*state[in_index+X-1])
	    - (global_rotation[1]*state[in_index+Y-1])
	    - (global_rotation[0]*state[in_index+Z-1])))
	  - (YcBD2*((global_rotation[3]*state[in_index+X-1])
		    +(global_rotation[0]*state[in_index+Y-1])
		    -(global_rotation[1]*state[in_index+Z-1])));
      
      H[ y_index ][ STATE_Wy ] = (y_part * temp) +
	(D*((global_rotation[1]*state[in_index+X-1])
	    + (global_rotation[2]*state[in_index+Y-1])
	    + (global_rotation[3]*state[in_index+Z-1])))
	  - (YcBD2*(-(global_rotation[0]*state[in_index+X-1])
		    +(global_rotation[3]*state[in_index+Y-1])
		    -(global_rotation[2]*state[in_index+Z-1])));

      H[ y_index ][ STATE_Wz ] = (z_part * temp) +
	(D*((global_rotation[0]*state[in_index+X-1])
	    - (global_rotation[3]*state[in_index+Y-1])
	    + (global_rotation[2]*state[in_index+Z-1])))
	  - (YcBD2*((global_rotation[1]*state[in_index+X-1])
		    +(global_rotation[2]*state[in_index+Y-1])
		    +(global_rotation[3]*state[in_index+Z-1])));

      /*  Now calculate the partial of Beta      */

      if( Beta > 0.1 )
	temp = ( image_pos[Z] + (camera_pos[Z]/Beta) ) * -D2;
      else
	temp = image_pos[Z] * -D2;

      H[ x_index ][ STATE_B ] = image_pos[X] * temp;   /*  dh() / dB terms  */
      H[ y_index ][ STATE_B ] = image_pos[Y] * temp;

      /*  Now calculate the d( h_u( x ))/d( feature location ) and
	  d( h_v( x ))/d( feature location ) elements    */

      for( n = STATE_FEATURE_START; n <= state_size; n++ );
      {
	H[ x_index ][ n ] = 0.0;
	H[ y_index ][ n ] = 0.0;
      }

       /*  The derivative relative to the X world location */
      H[ x_index ][ in_index ] = (D*R[X][X]) - (XcD2*Beta*R[Z][X]);
      H[ y_index ][ in_index ] = (D*R[Y][X]) - (YcD2*Beta*R[Z][X]);

       /*  The derivative relative to the Y world location */
      H[ x_index ][ in_index + 1 ] = (D*R[X][Y]) - (XcD2*Beta*R[Z][Y]);
      H[ y_index ][ in_index + 1 ] = (D*R[Y][Y]) - (YcD2*Beta*R[Z][Y]);

       /*  The derivative relative to the Z world location */
      H[ x_index ][ in_index + 2 ] = (D*R[X][Z]) - (XcD2*Beta*R[Z][Z]);
      H[ y_index ][ in_index + 2 ] = (D*R[Y][Z]) - (YcD2*Beta*R[Z][Z]);
    }
}

#undef X
#undef Y
#undef Z
#undef VECSIZE

