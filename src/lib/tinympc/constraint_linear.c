#include "constraint_linear.h"

enum tiny_ErrorCode tiny_SetInputBound(tiny_AdmmWorkspace* work, float* Ac_data, float* lc_data, float* uc_data) {
  int n = work->data->model->ninputs;
  work->stgs->en_cstr_inputs = EN_CSTR_INPUTS;
  work->data->Acu = slap_MatrixFromArray(n, n, Ac_data);
  slap_SetIdentity(work->data->Acu, 1);
  work->data->lcu = slap_MatrixFromArray(n, 1, lc_data);
  work->data->ucu = slap_MatrixFromArray(n, 1, uc_data);
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetStateBound(tiny_AdmmWorkspace* work, float* Ac_data, float* lc_data, float* uc_data) {
  int n = work->data->model[0].nstates;
  work->stgs->en_cstr_states = EN_CSTR_STATES;
  work->data->Acx = slap_MatrixFromArray(n, n, Ac_data);
  slap_SetIdentity(work->data->Acx, 1);
  work->data->lcx = slap_MatrixFromArray(n, 1, lc_data);
  work->data->ucx = slap_MatrixFromArray(n, 1, uc_data);
  return TINY_NO_ERROR;
}

// enum tiny_ErrorCode tiny_ProjectInput(tiny_AdmmWorkspace* work) {
//   int n = work->data->model[0].ninputs;
//   int N = work->data->model[0].ninputs;

//   for (int k = 0; k < N - 1; ++k) {
//     for (int i = 0; i < n; ++i) {

//       work->ZU_new[k].data[i] = T_MIN(T_MAX(z[i],
//                                 work->data->lcu[i]),  // Between lower
//                                 work->data->ucu[i]);  // and upper bounds
//     } 
//   }
//   return TINY_NO_ERROR;
// }

int IsConstrained(tiny_AdmmWorkspace* work) {
  if (!work->stgs->en_cstr_goal && 
      !work->stgs->en_cstr_inputs && 
      !work->stgs->en_cstr_states) {
    return 0; // unconstrained
  }
  return 1;    
}