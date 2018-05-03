/*  matmath.c

    This is a roughly self-contained code module for matrix
    math, which started with the Numerical Recipes in C code
    and matrix representations.

    J. Watlington, 11/15/95

    Modified:
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "matmath.h"

#define NR_END 1
#define FREE_ARG char*

/**********************************************************

     Vector Math Functions

*/

void vec_copy( m_elem *src, m_elem *dst, int num_elements )
{
  int  i;

  for( i = 1; i <= num_elements; i++ )
    dst[ i ] = src[ i ];
}

void vec_add( m_elem *a, m_elem *b, m_elem *c, int n )
{
  int i;
  m_elem  *a_ptr, *b_ptr, *c_ptr;

  a_ptr = a + 1;
  b_ptr = b + 1;
  c_ptr = c + 1;

  for( i = 1; i <= n; i++)
    *c_ptr++ = *a_ptr++ + *b_ptr++;
}

/* vec_sub()
   This function computes C = A - B, for vectors of size n  */

void vec_sub( m_elem *a, m_elem *b, m_elem *c, int n )
{
  int i;
  m_elem  *a_ptr, *b_ptr, *c_ptr;

  a_ptr = a + 1;
  b_ptr = b + 1;
  c_ptr = c + 1;

  for( i = 1; i <= n; i++)
    *c_ptr++ = *a_ptr++ - *b_ptr++;
}


/**********************************************************

  Matrix math functions

*/

void mat_add( m_elem **a, m_elem **b, m_elem **c, int m, int n )
{
  int i,j;
  m_elem  *a_ptr, *b_ptr, *c_ptr;

  for( j = 1; j <= m; j++)
    {
      a_ptr = &a[j][1];
      b_ptr = &b[j][1];
      c_ptr = &c[j][1];

      for( i = 1; i <= n; i++)
	*c_ptr++ = *a_ptr++ + *b_ptr++;
    }
}


/* mat_sub()
   This function computes C = A - B, for matrices of size m x n  */

void mat_sub( m_elem **a, m_elem **b, m_elem **c, int m, int n )
{
  int i,j;
  m_elem  *a_ptr, *b_ptr, *c_ptr;

  for( j = 1; j <= m; j++)
    {
      a_ptr = &a[j][1];
      b_ptr = &b[j][1];
      c_ptr = &c[j][1];

      for( i = 1; i <= n; i++)
	*c_ptr++ = *a_ptr++ - *b_ptr++;
    }
}

/*  mat_mult
    This function performs a matrix multiplication.
*/
void mat_mult( m_elem **a, m_elem **b, m_elem **c,
	      int a_rows, int a_cols, int b_cols )
{
  int i, j, k;
  m_elem  *a_ptr;
  m_elem  temp;

  for( i = 1; i <= a_rows; i++)
    {
      a_ptr = &a[i][0];
      for( j = 1; j <= b_cols; j++ )
	{
	  temp = 0.0;
	  for( k = 1; k <= a_cols; k++ )
	    temp = temp + (a_ptr[k] * b[k][j]); 
	  c[i][j] = temp;
	}
    }
}


/*  mat_mult_vector
    This function performs a matrix x vector multiplication.
*/
void mat_mult_vector( m_elem **a, m_elem *b, m_elem *c,
	      int a_rows, int a_cols )
{
  int i, j, k;
  m_elem  *a_ptr, *b_ptr;
  m_elem  temp;

  for( i = 1; i <= a_rows; i++)
    {
      a_ptr = &a[i][0];
      b_ptr = &b[1];
      temp = 0.0;

      for( k = 1; k <= a_cols; k++ )
	temp += a_ptr[ k ] * *b_ptr++;

      c[i] = temp;
    }
}


/*  mat_mult_transpose
    This function performs a matrix multiplication of A x transpose B.
*/
void mat_mult_transpose( m_elem **a, m_elem **b, m_elem **c,
	      int a_rows, int a_cols, int b_cols )
{
  int i, j, k;
  m_elem  *a_ptr, *b_ptr;
  m_elem  temp;

  for( i = 1; i <= a_rows; i++)
    {
      a_ptr = &a[i][0];
      for( j = 1; j <= b_cols; j++ )
	{
	  b_ptr = &b[j][1];

	  temp = (m_elem)0;

	  for( k = 1; k <= a_cols; k++ )
	    temp += a_ptr[ k ] * *b_ptr++;

	  c[i][j] = temp;
	}
    }
}

/*  mat_transpose_mult
    This function performs a matrix multiplication of transpose A x B.
    a_rows refers to the transposed A, is a_cols in actual A storage
    a_cols is same, is a_rows in actual A storage
*/
void mat_transpose_mult( m_elem **A, m_elem **B, m_elem **C,
	      int a_rows, int a_cols, int b_cols )
{
  int i, j, k;
  m_elem  temp;

  for( i = 1; i <= a_rows; i++)
    {
      for( j = 1; j <= b_cols; j++ )
	{
	  temp = 0.0;

	  for( k = 1; k <= a_cols; k++ )
	    temp += A[ k ][ i ] * B[ k ][ j ];

	  C[ i ][ j ] = temp;
	}
    }
}

void mat_copy( m_elem **src, m_elem **dst, int num_rows, int num_cols )
{
  int  i, j;

  for( i = 1; i <= num_rows; i++ )
    for( j = 1; j <= num_cols; j++ )
      dst[ i ][ j ] = src[ i ][ j ];
}


/* gaussj()
   This function performs gauss-jordan elimination to solve a set
   of linear equations, at the same time generating the inverse of
   the A matrix.

   (C) Copr. 1986-92 Numerical Recipes Software `2$m.1.9-153.
*/

#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}

void gaussj(m_elem **a, int n, m_elem **b, int m)
{
  int *indxc,*indxr,*ipiv;
  int i,icol,irow,j,k,l,ll;
  m_elem big,dum,pivinv,temp;

  indxc=ivector(1,n);
  indxr=ivector(1,n);
  ipiv=ivector(1,n);
  for (j=1;j<=n;j++) ipiv[j]=0;
  for (i=1;i<=n;i++) {
    big=0.0;
    for (j=1;j<=n;j++)
      if (ipiv[j] != 1)
	for (k=1;k<=n;k++) {
	  if (ipiv[k] == 0) {
	    if (fabs(a[j][k]) >= big) {
	      big=fabs(a[j][k]);
	      irow=j;
	      icol=k;
	    }
	  } else if (ipiv[k] > 1) nrerror("gaussj: Singular Matrix-1");
	}
    ++(ipiv[icol]);
    if (irow != icol) {
      for (l=1;l<=n;l++) SWAP(a[irow][l],a[icol][l])
	for (l=1;l<=m;l++) SWAP(b[irow][l],b[icol][l])
    }
    indxr[i]=irow;
    indxc[i]=icol;
    if (a[icol][icol] == 0.0) nrerror("gaussj: Singular Matrix-2");
    pivinv=1.0/a[icol][icol];
    a[icol][icol]=1.0;
    for (l=1;l<=n;l++) a[icol][l] *= pivinv;
    for (l=1;l<=m;l++) b[icol][l] *= pivinv;
    for (ll=1;ll<=n;ll++)
      if (ll != icol) {
	dum=a[ll][icol];
	a[ll][icol]=0.0;
	for (l=1;l<=n;l++) a[ll][l] -= a[icol][l]*dum;
	for (l=1;l<=m;l++) b[ll][l] -= b[icol][l]*dum;
      }
  }
  for (l=n;l>=1;l--) {
    if (indxr[l] != indxc[l])
      for (k=1;k<=n;k++)
	SWAP(a[k][indxr[l]],a[k][indxc[l]]);
  }

  free_ivector(ipiv,1,n);
  free_ivector(indxr,1,n);
  free_ivector(indxc,1,n);
}
#undef SWAP


/**********************************************************

  Quaternion math

  A quaternion is a four vector used to represent rotation in
  three-space.  The representation being used here is from
  Ali Azarbayejani.

  It is stored in components 0 through 3 of the vector.
*/

m_elem *quaternion( void )
{
  m_elem  *v;

  v = vector( 0, 3 );
  v[0] = (m_elem)1.0;  v[1] = (m_elem)0.0;
  v[2] = (m_elem)0.0;  v[3] = (m_elem)0.0;
}

void quaternion_update( m_elem *quat, m_elem wx,
		       m_elem wy, m_elem wz )
{
  int           i;
  m_elem  temp;
  m_elem  delta[QUATERNION_SIZE], new[QUATERNION_SIZE];

  /*  First, define a quaternion from the angular velocities  */

  temp = wx*wx + wy*wy + wz*wz;
  delta[0] = sqrt( 1 - (temp * 0.25) );
  temp = 1.0 / 2.0;
  delta[1] = wx * temp;
  delta[2] = wy * temp;
  delta[3] = wz * temp;

  /*  Now multiply it with the input quaternion, to update it. */

  new[0] = quat[0]*delta[0] - quat[1]*delta[1] - quat[2]*delta[2]
    - quat[3]*delta[3];
  new[1] = quat[1]*delta[0] + quat[0]*delta[1] - quat[3]*delta[2]
    + quat[2]*delta[3];
  new[2] = quat[2]*delta[0] + quat[3]*delta[1] + quat[0]*delta[2]
    - quat[1]*delta[3];
  new[3] = quat[3]*delta[0] - quat[2]*delta[1] + quat[1]*delta[2]
    + quat[0]*delta[3];

  /*  Finally, normalize the quaternion and copy it back  */

  temp = new[0]*new[0] + new[1]*new[1] + new[2]*new[2] + new[3]*new[3];
  if( temp < MIN_QUATERNION_MAGNITUDE )
    {
      quat[0] = 1.0;
      quat[1] = 0.0; quat[2] = 0.0; quat[3] = 0.0;
      return;
    }

  temp = 1.0 / (m_elem)sqrt( (double)temp );
  if( temp != 1.0 )
    {
      for( i = 0; i < QUATERNION_SIZE; i++ )
	quat[i] = new[i] * temp;
    }
  else
    for( i = 0; i < QUATERNION_SIZE; i++ )
      quat[i] = new[i];
}

/*  quaternion_to_rotation
    This function takes pointers to a quaternion input, and a rotation
    matrix for the results.  It calculates the 3D rotation matrix that
    is equivalent to the quaternion.
*/
void quaternion_to_rotation( m_elem *quat, m_elem **rot )
{
  m_elem  q01 = quat[0] * quat[1];
  m_elem  q02 = quat[0] * quat[2];
  m_elem  q03 = quat[0] * quat[3];
  m_elem  q11 = quat[1] * quat[1];
  m_elem  q12 = quat[1] * quat[2];
  m_elem  q13 = quat[1] * quat[3];
  m_elem  q22 = quat[2] * quat[2];
  m_elem  q23 = quat[2] * quat[3];
  m_elem  q33 = quat[3] * quat[3];

  rot[1][1] = 1.0 - 2.0 * (q22 + q33);
  rot[1][2] =     - 2.0 * (q03 - q12);
  rot[1][3] =       2.0 * (q02 + q13);
  rot[2][1] =       2.0 * (q03 + q12);
  rot[2][2] = 1.0 - 2.0 * (q22 + q33);
  rot[2][3] =     - 2.0 * (q01 - q23);
  rot[3][1] =     - 2.0 * (q02 - q13);
  rot[3][2] =       2.0 * (q01 + q23);
  rot[3][3] = 1.0 - 2.0 * (q11 + q22);
}

/**********************************************************

  Vector and Matrix allocation and deallocation routines

*/

void nrerror(char error_text[])
/* Numerical Recipes standard error handler */
{
	fprintf(stderr,"Numerical Recipes run-time error...\n");
	fprintf(stderr,"%s\n",error_text);
	fprintf(stderr,"...now exiting to system...\n");
	exit(1);
}

m_elem *vector(long nl, long nh)
/* allocate a vector with subscript range v[nl..nh] */
{
	m_elem *v;

	v=(m_elem *)malloc((size_t) ((nh-nl+1+NR_END) *
					   sizeof(m_elem)));
	if (!v) nrerror("allocation failure in vector()");
	return v-nl+NR_END;
}

float *fvector(long nl, long nh)
/* allocate a vector with subscript range v[nl..nh] */
{
	float *v;

	v=(float *)malloc((size_t) ((nh-nl+1+NR_END) *
					   sizeof(float)));
	if (!v) nrerror("allocation failure in vector()");
	return v-nl+NR_END;
}

int *ivector(long nl, long nh)
/* allocate an int vector with subscript range v[nl..nh] */
{
	int *v;

	v=(int *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(int)));
	if (!v) nrerror("allocation failure in ivector()");
	return v-nl+NR_END;
}

unsigned char *cvector(long nl, long nh)
/* allocate an unsigned char vector with subscript range v[nl..nh] */
{
	unsigned char *v;

	v=(unsigned char *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(unsigned char)));
	if (!v) nrerror("allocation failure in cvector()");
	return v-nl+NR_END;
}

unsigned long *lvector(long nl, long nh)
/* allocate an unsigned long vector with subscript range v[nl..nh] */
{
	unsigned long *v;

	v=(unsigned long *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(long)));
	if (!v) nrerror("allocation failure in lvector()");
	return v-nl+NR_END;
}

double *dvector(long nl, long nh)
/* allocate a double vector with subscript range v[nl..nh] */
{
	double *v;

	v=(double *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(double)));
	if (!v) nrerror("allocation failure in dvector()");
	return v-nl+NR_END;
}

m_elem **matrix(long nrl, long nrh, long ncl, long nch)
/* allocate a m_elem matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	m_elem **m;

	/* allocate pointers to rows */
	m=(m_elem **)malloc((size_t)((nrow+NR_END)
					    *sizeof(m_elem *)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;

	/* allocate rows and set pointers to them */
	m[nrl]=(m_elem *)malloc((size_t)((nrow*ncol+NR_END) 
					       *sizeof(m_elem)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}


float **fmatrix(long nrl, long nrh, long ncl, long nch)
/* allocate a float matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	float **m;

	/* allocate pointers to rows */
	m=(float **) malloc((size_t)((nrow+NR_END)*sizeof(float*)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;

	/* allocate rows and set pointers to them */
	m[nrl]=(float *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(float)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

double **dmatrix(long nrl, long nrh, long ncl, long nch)
/* allocate a double matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	double **m;

	/* allocate pointers to rows */
	m=(double **) malloc((size_t)((nrow+NR_END)*sizeof(double*)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;

	/* allocate rows and set pointers to them */
	m[nrl]=(double *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(double)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

int **imatrix(long nrl, long nrh, long ncl, long nch)
/* allocate a int matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	int **m;

	/* allocate pointers to rows */
	m=(int **) malloc((size_t)((nrow+NR_END)*sizeof(int*)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;


	/* allocate rows and set pointers to them */
	m[nrl]=(int *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(int)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

m_elem **submatrix(m_elem **a,
			 long oldrl, long oldrh, long oldcl, long oldch,
	long newrl, long newcl)
/* point a submatrix [newrl..][newcl..] to a[oldrl..oldrh][oldcl..oldch] */
{
	long i,j,nrow=oldrh-oldrl+1,ncol=oldcl-newcl;
	m_elem **m;

	/* allocate array of pointers to rows */
	m=(m_elem **) malloc((size_t) ((nrow+NR_END)
					     *sizeof(m_elem*)));
	if (!m) nrerror("allocation failure in submatrix()");
	m += NR_END;
	m -= newrl;

	/* set pointers to rows */
	for(i=oldrl,j=newrl;i<=oldrh;i++,j++) m[j]=a[i]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

m_elem **convert_matrix(m_elem *a,
			      long nrl, long nrh, long ncl, long nch)
/* allocate a m_elem matrix m[nrl..nrh][ncl..nch] that points
to the matrix declared in the standard C manner as a[nrow][ncol], where
nrow=nrh-nrl+1 and ncol=nch-ncl+1. The routine should be called with the
address &a[0][0] as the first argument. */
{
	long i,j,nrow=nrh-nrl+1,ncol=nch-ncl+1;
	m_elem **m;

	/* allocate pointers to rows */
	m=(m_elem **)malloc((size_t)((nrow+NR_END)
					   *sizeof(m_elem*)));
	if (!m) nrerror("allocation failure in convert_matrix()");
	m += NR_END;
	m -= nrl;

	/* set pointers to rows */
	m[nrl]=a-ncl;
	for(i=1,j=nrl+1;i<nrow;i++,j++) m[j]=m[j-1]+ncol;
	/* return pointer to array of pointers to rows */
	return m;
}

void free_vector(m_elem *v, long nl, long nh)
/* free a vector allocated with vector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

void free_fvector(float *v, long nl, long nh)
/* free a float vector allocated with fvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

void free_ivector(int *v, long nl, long nh)
/* free an int vector allocated with ivector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

void free_cvector(unsigned char *v, long nl, long nh)
/* free an unsigned char vector allocated with cvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

void free_lvector(unsigned long *v, long nl, long nh)
/* free an unsigned long vector allocated with lvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

void free_dvector(double *v, long nl, long nh)
/* free a double vector allocated with dvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

void free_matrix(m_elem  **m, long nrl, long nrh, long ncl, long nch)
/* free a matrix allocated by matrix() */
{
	free((FREE_ARG) (m[nrl]+ncl-NR_END));
	free((FREE_ARG) (m+nrl-NR_END));
}

void free_fmatrix(float **m, long nrl, long nrh, long ncl, long nch)
/* free a float matrix allocated by fmatrix() */
{
	free((FREE_ARG) (m[nrl]+ncl-NR_END));
	free((FREE_ARG) (m+nrl-NR_END));
}

void free_dmatrix(double **m, long nrl, long nrh, long ncl, long nch)
/* free a double matrix allocated by dmatrix() */
{
	free((FREE_ARG) (m[nrl]+ncl-NR_END));
	free((FREE_ARG) (m+nrl-NR_END));
}

void free_imatrix(int **m, long nrl, long nrh, long ncl, long nch)
/* free an int matrix allocated by imatrix() */
{
	free((FREE_ARG) (m[nrl]+ncl-NR_END));
	free((FREE_ARG) (m+nrl-NR_END));
}

void free_submatrix(float **b, long nrl, long nrh, long ncl, long nch)
/* free a submatrix allocated by submatrix() */
{
	free((FREE_ARG) (b+nrl-NR_END));
}

void free_convert_matrix(float **b, long nrl, long nrh, long ncl, long nch)
/* free a matrix allocated by convert_matrix() */
{
	free((FREE_ARG) (b+nrl-NR_END));
}

/***************************************************

  Matrix and Vector Printing routines

*/


void print_vector( char *str, m_elem *x, int n )
{
  int     i;

  printf( "%s:\n", str );
  for( i = 1; i <= n; i++ )
    {
      if( (x[ i ] > 1) && (x[i] < 999) )
	printf( " %3.1lf", x[ i ] );
      else
	printf( " %2.2lg", x[ i ] );
    }
  printf( "\n" );
}

void print_quaternion( char *str, m_elem *x )
{
  int     i;

  printf( "%s:\n", str );
  for( i = 0; i < QUATERNION_SIZE; i++ )
    {
      if( (x[ i ] > 1) && (x[i] < 999) )
	printf( " %3.1lf", x[ i ] );
      else
	printf( " %2.2lg", x[ i ] );
    }
  printf( "\n" );
}

void print_matrix( char *str, m_elem **A, int m, int n )
{
  int     i, j;

  printf( "%s:  (%d x %d)\n", str, m, n );
  for( i = 1; i <= m; i++ )
    {
      printf( ">" );
      for( j = 1; j <= n; j++ )
	{
	  printf( " %2.2lg", A[ i ][ j ] );
	}
      printf( "\n" );
    }
}