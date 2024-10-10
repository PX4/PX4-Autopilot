#ifndef CONSTANTS_H
# define CONSTANTS_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus


/************************************
* Printing Constants to set Layout *
************************************/
# define HEADER_LINE_LEN 65


/******************
* Solver Status  *
******************/
# define TINY_SOLVED (1)
# define TINY_MAX_ITER_REACHED (-2)
# define TINY_MAX_ITER_RICCATI_REACHED (-3)
# define TINY_MAX_ITER_LS_REACHED (-4)
# define TINY_NON_CVX (-7) 
# define TINY_UNSOLVED (-10)            /* Unsolved. Only setup function has been called */


/******************
* Solver Errors  *
******************/
// Inherit from slap_ErrorCode and expand new errors for tinympc
enum tiny_ErrorCode {
  TINY_SLAP_ERROR = 0,
  TINY_MATRIX_NOT_PSD,
  TINY_NO_ERROR,
  TINY_NOT_SUPPORTED,
};


/**********************************
* Solver Parameters and Settings *
**********************************/

// Regularization
# define REG_MIN (1e-6)
# define REG_MAX (1e2)
# define REG_MUL (1.6)
# define EN_REG_UPDATE (0)

// Penalty (rho)
# define RHO_INIT (1e0)
# define RHO_MAX (1e6)
# define RHO_MUL (10.0)

// Line-search 
# define ALPHA_MUL (0.5)
# define ALPHA (1)

// Max iteration
# define MAX_ITER (100)
# define MAX_ITER_RICCATI (10)
# define MAX_ITER_LS (10)

// Tolerance
# define TOL_ABS_PRIM (1e-2)
# define TOL_ABS_DUAL (1e-2)

# define EN_CSTR_STATES (0)
# define EN_CSTR_INPUTS (1)
# define EN_CSTR_GOAL (0)

# define VERBOSE (1)
# define ADAPTIVE_HORIZON (0)
# define CHECK_RICCATI (0)
# define CHECK_TERMINATION (2)
# define WARM_START (1)
# define TIME_LIMIT (0.0)


# ifndef TINY_NULL_MAT
#  define TINY_NULL_MAT  \
  ((Matrix){      \
      0,          \
      0,          \
      0,          \
      0,          \
      TINY_NULL,       \
      slap_DENSE, \
  })
# endif /* ifndef TINY_NULL_MAT */

# ifndef TINY_NULL
#  define TINY_NULL 0
# endif /* ifndef TINY_NULL */

# ifndef TINY_NAN
#  define TINY_NAN ((float)0x7fc00000UL)  // not a number
# endif /* ifndef TINY_NAN */

# ifndef TINY_INFTY
#  define TINY_INFTY ((float)1e30)        // infinity
# endif /* ifndef TINY_INFTY */

/* Printing */
# define PRINT_INTERVAL 1

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef CONSTANTS_H
