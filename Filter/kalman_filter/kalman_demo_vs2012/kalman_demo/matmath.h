#ifndef __MATMATH_H__
#define __MATMATH_H__
/*   matmath.h

     This is the declaration file for the matmath functions.
*/

/*  This is the datatype used for the math and non-type specific ops.  */

typedef double m_elem;

/*************     Vector math routines    ****************/

/*  vector B = vector A, of size num_elements   */
extern void vec_copy( m_elem *src, m_elem *dst, int num_elements );

/*  vector C = vector A + vector B , both of size n   */
extern void vec_add( m_elem *A, m_elem *B, m_elem *C, int n );

/*  vector C = vector A - vector B , both of size n   */
extern void vec_sub( m_elem *A, m_elem *B, m_elem *C, int n );

extern void print_vector( char *str, m_elem *x, int n );

/*************   Some matrix math routines  **************/

/*  matrix B = matrix A, of size num_rows x num_cols   */
extern void mat_copy( m_elem **A, m_elem **B, int num_rows,
			int num_cols );

/*  matrix C = matrix A + matrix B , both of size m x n   */
extern void mat_add( m_elem **A, m_elem **B, m_elem **C, int m, int n );

/*  matrix C = matrix A - matrix B , all of size m x n  */
extern void mat_sub( m_elem **A, m_elem **B, m_elem **C, int m, int n );

/*  matrix C = matrix A x matrix B , A(a_rows x a_cols), B(a_cols x b_cols) */
extern void mat_mult( m_elem **A, m_elem **B, m_elem **C,
		     int a_rows, int a_cols, int b_cols );

/*  matrix C = matrix A x vector B , A(a_rows x a_cols), B(a_cols x 1) */
extern void mat_mult_vector( m_elem **A, m_elem *B, m_elem *C,
			    int a_rows, int a_cols );

/*  C = matrix A x trans( matrix B ), A(a_rows x a_cols), B(b_cols x a_cols) */
extern void mat_mult_transpose( m_elem **A, m_elem **B, m_elem **C,
			   int a_rows, int a_cols, int b_cols );

/*  C = trans( matrix A ) x matrix B, A(a_cols x a_rows), B(a_cols x b_cols) */
extern void mat_transpose_mult( m_elem **A, m_elem **B, m_elem **C,
			   int a_rows, int a_cols, int b_cols );


extern void print_matrix( char *str, m_elem **A, int m, int n );


extern void gaussj( m_elem **A, int n, m_elem **B, int m );

extern void mrqmin( m_elem x[], m_elem y[], m_elem sig[], int ndata,
		   m_elem a[], int ia[], int ma, m_elem **covar,
		   m_elem **alpha, m_elem *chisq,
		   void (*funcs)(m_elem, m_elem [], m_elem *, m_elem [], int),
		   m_elem *alamda);

/*  quaternion routines  */

#define QUATERNION_SIZE    4
#define MIN_QUATERNION_MAGNITUDE   0.000001


extern m_elem *quaternion( void );
extern void quaternion_update( m_elem *quat, m_elem wx,
		       m_elem wy, m_elem wz );
extern void quaternion_to_rotation( m_elem *quat, m_elem **rot );
extern void print_quaternion( char *str, m_elem *quat );

/*   Vector allocation routines   */

extern m_elem *vector(long nl, long nh);
extern int *ivector(long nl, long nh);
extern unsigned char *cvector(long nl, long nh);
extern unsigned long *lvector(long nl, long nh);
extern float *fvector(long nl, long nh);
extern double *dvector(long nl, long nh);

/*   Matrix allocation routines    */

extern m_elem **matrix(long nrl, long nrh, long ncl, long nch);
extern float **fmatrix(long nrl, long nrh, long ncl, long nch);
extern double **dmatrix(long nrl, long nrh, long ncl, long nch);
extern int **imatrix(long nrl, long nrh, long ncl, long nch);

extern m_elem **submatrix(m_elem **a,
			 long oldrl, long oldrh, long oldcl, long oldch,
			 long newrl, long newcl);
extern m_elem **convert_matrix( m_elem *a, long nrl, long nrh,
			      long ncl, long nch);

/*   Deallocation routines   */

extern void free_vector(m_elem *v, long nl, long nh);
extern void free_fvector(float *v, long nl, long nh);
extern void free_ivector(int *v, long nl, long nh);
extern void free_cvector(unsigned char *v, long nl, long nh);
extern void free_lvector(unsigned long *v, long nl, long nh);
extern void free_dvector(double *v, long nl, long nh);

extern void free_matrix(m_elem **m, long nrl, long nrh, long ncl, long nch);
extern void free_fmatrix(float **m, long nrl, long nrh, long ncl, long nch);
extern void free_dmatrix(double **m, long nrl, long nrh, long ncl, long nch);
extern void free_imatrix(int **m, long nrl, long nrh, long ncl, long nch);
extern void free_submatrix(float **b, long nrl, long nrh, long ncl, long nch);
extern void free_convert_matrix(float **b, long nrl, long nrh, long ncl, long nch);
extern void nrerror(char error_text[]);   /*  internal error handler */

#endif