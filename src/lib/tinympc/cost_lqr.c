#include "cost_lqr.h"

enum tiny_ErrorCode tiny_AddStageCost(tiny_AdmmWorkspace* work, const int k) {
  int n = work->data->model[0].nstates;
  int m = work->data->model[0].ninputs;
  float dx_data[n];
  Matrix dx = slap_MatrixFromArray(n, 1, dx_data);
  MatAdd(dx, work->soln->X[k], work->data->Xref[k], -1);
  work->info->obj_val += (float)0.5 * slap_QuadraticForm(dx, work->data->Q, dx);
  Matrix du = slap_MatrixFromArray(m, 1, dx_data);
  MatAdd(du, work->soln->U[k], work->data->Uref[k], -1);
  work->info->obj_val += (float)0.5 * slap_QuadraticForm(du, work->data->R, du);
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_AddTerminalCost(tiny_AdmmWorkspace* work) {
  int n = work->data->model[0].nstates;
  int N = work->data->model[0].nhorizon;
  float dx_data[n];
  Matrix dx = slap_MatrixFromArray(n, 1, dx_data);
  MatAdd(dx, work->soln->X[N - 1], work->data->Xref[N - 1], -1);
  work->info->obj_val += (float)0.5 * slap_QuadraticForm(dx, work->soln->Pinf, dx);
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_UpdateLinearCost(tiny_AdmmWorkspace* work) {
  int N = work->data->model[0].nhorizon;
  for (int k = 0; k < N - 1; ++k) {
    /* Compute q[k] = -Q*Xref[k] */
    MatMulAdd(work->data->q[k], work->data->Q, work->data->Xref[k], -1, 0);

    /* Compute r[k] .= -R*Uref[k] */
    MatMulAdd(work->data->r[k], work->data->R, work->data->Uref[k], -1, 0);
  }
  /* Compute q[N-1] = -Pinf*Xref[N-1] */
  MatMulAdd(work->soln->p[N-1], work->soln->Pinf, work->data->Xref[N-1], -1, 0);
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_UpdateConstrainedLinearCost(tiny_AdmmWorkspace* work) {
  int N = work->data->model[0].nhorizon;
  for (int k = 0; k < N - 1; ++k) {
    /* Compute r_tilde[k] = r[k] - ρ*(z[k]-y[k]) */
    MatAdd(work->data->r_tilde[k], work->ZU_new[k], work->soln->YU[k], -1);
    MatAdd(work->data->r_tilde[k], work->data->r[k], work->data->r_tilde[k], -work->rho);
  }
  return TINY_NO_ERROR;
}
