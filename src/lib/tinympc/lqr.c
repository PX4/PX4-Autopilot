#include "lqr.h"

enum tiny_ErrorCode tiny_ForwardPass(tiny_AdmmWorkspace* work) {
  tiny_RollOutClosedLoop(work);
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_BackwardPassGrad(tiny_AdmmWorkspace* work) {
  tiny_Model* model = work->data->model;
  int N = model[0].nhorizon;

  if (model[0].ltv && model[0].affine) {
    return TINY_NOT_SUPPORTED;
  }
  if (model[0].ltv && !model[0].affine) {
    return TINY_NOT_SUPPORTED;
  }
  if (!model[0].ltv && model[0].affine) {
    return TINY_NOT_SUPPORTED;
  }
  // LTI model
  if (!model[0].ltv && !model[0].affine) {
    for (int k = N - 2; k >= 0; --k) {
      /* Compute  Qu = B'*p[k+1] + r[k] */
      slap_MatMulAtB(work->Qu, model[0].B[0], work->soln->p[k+1]);
      MatAdd(work->Qu, work->Qu, work->data->r_tilde[k], 1);

      /* Compute d = Quu\Qu */
      slap_MatMulAB(work->soln->d[k], work->Quu_inv, work->Qu);

      /* Compute p[k] .= q[k] + AmBKt*p[k+1] - Kinf'*r[k] + coeff_d2p*d[k] */
      slap_MatMulAtB(work->soln->p[k], work->soln->Kinf, work->data->r_tilde[k]);
      MatMulAdd(work->soln->p[k], work->coeff_d2p, work->soln->d[k], 1, -1);
      MatMulAdd(work->soln->p[k], work->AmBKt, work->soln->p[k+1], 1, 1);
      MatAdd(work->soln->p[k],work->soln->p[k], work->data->q[k], 1);      
    }
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SolveLqr(tiny_AdmmWorkspace* work) {
  MatCpy(work->soln->X[0], work->data->x0);
  tiny_BackwardPassGrad(work);  
  tiny_ForwardPass(work);  
  return TINY_NO_ERROR;
}