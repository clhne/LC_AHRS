/*  eval_camera.c
    This file contains the functions used for Levenberg-Marquardt minimization
    and parameter estimation of an imaging model.
    It is intended for use with mrqmin.c, the Levenberg-Marquardt code from
    Numerical Recipes in C.

    J. Watlington, 12/9/95

    Modified:
*/

#include <stdio.h>
#include <math.h>
#include "matmath.h"
#include "test.h"

#define X        1
#define Y        2
#define Z        3
#define VECSIZE  4

static m_elem  camera_origin[ SFM_STATE_SIZE + 1 ] = {
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};

/*  eval_camera
    This function evaluates the fitting function yfit, and the
    derivatives dyda[1..state_size] with respect to the fitting
    parameters (state) a[] at point x.
    In particular, it assumes that the x value input is merely an
    index, into an array of actual x inputs (which are vectors).
*/

void eval_camera( m_elem x, m_elem a[], m_elem *yfit, m_elem dyda[],
		 int state_size )
{
  int       n, f, i;
  int       x_index, odd;
  m_elem    D, D2;
  m_elem    *camera;
  m_elem    XcD2, YcD2;
  m_elem    XcBD2, YcBD2;
  m_elem    fx, fy, fz;
  m_elem    Beta, temp, temp2;
  m_elem    x_part, y_part, z_part;
  m_elem    image_pos[ VECSIZE ];
  m_elem    rotation_q[ QUATERNION_SIZE ];
  static m_elem  **R = NULL;

  /*  The x_index is the feature point being processed.
      The odd flag indicates whether the x measurement or the y measurement
      is being calculated.     */

  n = (int)x - 1;
  odd = n & 1;
  if( n >= feature_size )
    {
      x_index = (((n - feature_size) >> 1) * 3) + STATE_FEATURE_START;
      camera = camera_origin;
    }
  else
    {
      x_index = ((n >> 1) * 3) + STATE_FEATURE_START;
      camera = a;
    }

  /* The angular velocity information is converted to a global
     rotation, which is the one used by all equations.    */

  if( R == NULL )
    R = matrix( 1, 3, 1, 3 );

  rotation_q[0] = sqrt( 1.0 -(0.25*((camera[ STATE_Wx ]*camera[ STATE_Wx ]) +
				   (camera[ STATE_Wy ]*camera[ STATE_Wy ]) +
				   (camera[ STATE_Wz ]*camera[ STATE_Wz ]))));
  rotation_q[1] = camera[ STATE_Wx ] * 0.5;
  rotation_q[2] = camera[ STATE_Wy ] * 0.5;
  rotation_q[3] = camera[ STATE_Wz ] * 0.5;

  if( rotation_q[0] < MIN_QUATERNION_MAGNITUDE )
    {
      rotation_q[0] = 1.0;
      rotation_q[1] = 0.0; rotation_q[2] = 0.0; rotation_q[3] = 0.0;
    }

  quaternion_to_rotation( rotation_q, R );

  Beta = a[ STATE_B ];  /*  the inverse of the focal length  */

  fx = a[ x_index + X - 1 ];
  fy = a[ x_index + Y - 1 ];
  fz = a[ x_index + Z - 1 ];

  /*  Calculate transformed version of the coordinates, depending on the
      state estimate.      */

  image_pos[X] = camera[ STATE_Tx ]
    + (fx * R[X][X]) + (fy * R[X][Y]) + (fz * R[X][Z]);
  image_pos[Y] = camera[ STATE_Ty ]
    + (fx * R[Y][X]) + (fy * R[Y][Y]) + (fz * R[Y][Z]);
  image_pos[Z] = (fx * R[Z][X]) + (fy * R[Z][Y]) + (fz * R[Z][Z]);

  /*  And some useful intermediate values  */

  D = ARBITRARY_SCALE / ( 1.0 + camera[ STATE_Tz ] + (Beta * image_pos[Z]));
  D2 = (D * D); 

  for( n = STATE_FEATURE_START; n <= state_size; n++ )
    dyda[ n ] = 0.0;

  /*  The function eval and derivative calculation vary depending on
      whether the measurement being considered is x or y.  */

  if( !odd )
    {
      XcD2 = image_pos[X] * D2;
      XcBD2 = XcD2 * Beta;

      *yfit = image_pos[X] / (1 + camera[ STATE_Tz ] + (Beta*image_pos[Z]));

      dyda[ STATE_Tx ] = D;          /*  The  dh() / dTx terms  */
      dyda[ STATE_Ty ] = 0;          /*  The  dh() / dTy terms  */
      dyda[ STATE_Tz ] = -XcD2;      /* dh() / dTz terms  */

      /*  Calculate the rotational derivatives for the x measurement  */

      temp = - 1 / (2*rotation_q[0]);
      x_part = 	camera[ STATE_Wx ] * temp;
      y_part = 	camera[ STATE_Wy ] * temp;
      z_part = 	camera[ STATE_Wz ] * temp;

      temp = (D*((rotation_q[0]*fx)
		 - (rotation_q[3]*fy)
		 + (rotation_q[2]*fz)))
	- (XcBD2*((-rotation_q[2]*fx)
		  + (rotation_q[1]*fy)
		  + (rotation_q[0]*fz)));

      dyda[ STATE_Wx ] = (x_part * temp) +
	(D*((rotation_q[1]*fx)
	    + (rotation_q[2]*fy)
	    + (rotation_q[3]*fz)))
	  - (XcBD2*((rotation_q[3]*fx)
		    +(rotation_q[0]*fy)
		    -(rotation_q[1]*fz)));
      
      dyda[ STATE_Wy ] = (y_part * temp) +
	(D*((-rotation_q[2]*fx)
	    + (rotation_q[1]*fy)
	    + (rotation_q[0]*fz)))
	  - (XcBD2*(-(rotation_q[0]*fx)
		    +(rotation_q[3]*fy)
		    -(rotation_q[2]*fz)));

      dyda[ STATE_Wz ] = (z_part * temp) +
	(D*((-rotation_q[3]*fx)
	    - (rotation_q[0]*fy)
	    + (rotation_q[1]*fz)))
	  - (XcBD2*((rotation_q[1]*fx)
		    +(rotation_q[2]*fy)
		    +(rotation_q[3]*fz)));

      if( Beta > 1.0 )
	dyda[ STATE_B ] = -XcD2 * (image_pos[Z] + (camera[ STATE_Tz ]/Beta));
      else
	dyda[ STATE_B ] = -XcD2 * image_pos[Z];

       /*  The derivative relative to the X world location */
      dyda[ x_index ] = (D*R[X][X]) - (XcBD2*R[Z][X]);

       /*  The derivative relative to the Y world location */
      dyda[ x_index + 1 ] = (D*R[X][Y]) - (XcBD2*R[Z][Y]);

       /*  The derivative relative to the Z world location */
      dyda[ x_index + 2 ] = (D*R[X][Z]) - (XcBD2*R[Z][Z]);
    }
  else
    {
      YcD2 = image_pos[Y] * D2;
      YcBD2 = YcD2 * Beta;

      *yfit = image_pos[Y] / (1.0 + camera[ STATE_Tz ] + (Beta*image_pos[Z]));

      dyda[ STATE_Tx ] = 0;
      dyda[ STATE_Ty ] = D;
      dyda[ STATE_Tz ] = -YcD2;

      /*  Calculate the rotational derivatives for the y measurement  */

      temp = (D*((rotation_q[3]*fx)
		 + (rotation_q[0]*fy)
		 - (rotation_q[1]*fz)))
	- (YcBD2*((-rotation_q[2]*fx)
		  + (rotation_q[1]*fy)
		  + (rotation_q[0]*fz)));

      temp2 = (D*((rotation_q[2]*fx)
		  - (rotation_q[1]*fy)
		  - (rotation_q[0]*fz)));
      temp2 = temp2 - (YcBD2*((rotation_q[3]*fx)
			      +(rotation_q[0]*fy)
			      -(rotation_q[1]*fz)));
      dyda[ STATE_Wx ] = (x_part * temp) + temp2;

      
      temp2 = (D*((rotation_q[1]*fx)
		  + (rotation_q[2]*fy)
		  + (rotation_q[3]*fz)));
      temp2 = temp2 - (YcBD2*(-(rotation_q[0]*fx)
			      +(rotation_q[3]*fy)
			      -(rotation_q[2]*fz)));
      dyda[ STATE_Wy ] = (y_part * temp) + temp2;


      temp2 = (D*((rotation_q[0]*fx)
		  - (rotation_q[3]*fy)
		  + (rotation_q[2]*fz)));
      temp2 = temp2 - (YcBD2*((rotation_q[1]*fx)
			      +(rotation_q[2]*fy)
			      +(rotation_q[3]*fz)));
      dyda[ STATE_Wz ] = (z_part * temp) + temp2;

      if( Beta > 1.0 )
	dyda[ STATE_B ] = -YcD2 * (image_pos[Z] + (camera[ STATE_Tz ]/Beta));
      else
	dyda[ STATE_B ] = -YcD2 * image_pos[Z];

       /*  The derivative relative to the X world location */
      dyda[ x_index+X-1 ] = (D*R[Y][X]) - (YcBD2*R[Z][X]);

       /*  The derivative relative to the Y world location */
      dyda[ x_index+Y-1 ] = (D*R[Y][Y]) - (YcBD2*R[Z][Y]);

       /*  The derivative relative to the Z world location */
      dyda[ x_index+Z-1 ] = (D*R[Y][Z]) - (YcBD2*R[Z][Z]);
    }
  
}

#undef X
#undef Y
#undef Z
#undef VECSIZE