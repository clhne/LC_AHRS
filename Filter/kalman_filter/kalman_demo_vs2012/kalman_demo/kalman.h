/*  kalman.h
    This is the include file for the kalman filtering functions defined
    in kalman.c.

    J. Watlington, 11/16/95
*/

#undef PRINT_DEBUG

#include "matmath.h"    /*  m_elem decl. and matrix utilities */

/***********    Linear Kalman Filtering   *************/

extern void kalman_init( m_elem **GQGt, m_elem **Phi, m_elem **H, m_elem **R,
		m_elem **P, m_elem *x, int state_size, int measure_size );

extern void kalman_step( m_elem *z_in );

extern m_elem *kalman_get_state( void );  /* works for extended kalman too */


/***********   Extended Kalman Filtering   **************/

extern void extended_kalman_init( m_elem **GQGt, m_elem **R, m_elem **P,
			 m_elem *x, int state_size, int measure_size );

extern void extended_kalman_step( m_elem *z_in );


/***********   Iterated Extended Kalman Filtering   **************/

extern void iter_ext_kalman_init( m_elem **GQGt, m_elem **R, m_elem **P,
			 m_elem *x, int state_size, int measure_size );

extern void iter_ext_kalman_step( m_elem *z_in );


/*********    Application Specific Functions  ***********/

extern void apply_system( m_elem *old_state, m_elem *new_state );
extern void generate_measurement_transfer( m_elem *state, m_elem **H );

extern void apply_measurement( m_elem *new_state, m_elem *est_measurement );
extern void generate_system_transfer( m_elem *state, m_elem **phi );

/*  and a select set of global variables to communicate with them.  */

extern int     state_size;
extern int     measurement_size;
extern int     global_step;  /* the current step number (k) */