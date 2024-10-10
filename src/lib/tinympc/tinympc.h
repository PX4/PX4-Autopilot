#ifndef TINYMPC_H
# define TINYMPC_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

// NOTE: this is an odd fix to get gcov to run correctly on GitHub Actions:
// https://www.osadl.org/fileadmin/dam/interface/docbook/howtos/coverage.pdf
// void __gcov_flush(void);

#include "constants.h"
#include "types.h"
#include "utils.h"
#include "model.h"
#include "auxil.h"
#include "cost_lqr.h"
#include "lqr.h"
#include "constraint_linear.h"
#include "admm.h"

// #include "lqr_lti.h"
// #include "lqr_ltv.h"

// #include "mpc_lti.h"
// #include "mpc_ltv.h"


// #if (MODEL == LTI_MODEL)
//   #include "dynamics_lti.h"
//   #if (CONSTRAINT == CONIC_CONSTRAINT)
//   // #include "conic_mpc_lti.h"
//     // #include "conic_constraint.h"
//   #endif
//   #if (CONSTRAINT == LINEAR_CONSTRAINT)
//     #include "mpc_lti.h"
//     #include "constraint.h"
//   #endif
//   #if (CONSTRAINT == NO_CONSTRAINT)
//     #include "lqr_lti.h"
//   #endif
// #endif

// #if (MODEL == LTV_MODEL)
//   #include "dynamics_ltv.h"
//   #if (CONSTRAINT == CONIC_CONSTRAINT)
//   // #include "conic_mpc_ltv.h"
//     // #include "conic_constraint.h"
//   #endif
//   #if (CONSTRAINT == LINEAR_CONSTRAINT)
//     #include "mpc_ltv.h"
//     #include "constraint.h"
//   #endif
//   #if (CONSTRAINT == NO_CONSTRAINT)
//     #include "lqr_ltv.h"
//   #endif
// #endif


# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef TINYMPC_H