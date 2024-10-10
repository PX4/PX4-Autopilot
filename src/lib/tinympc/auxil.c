#include "auxil.h"

enum tiny_ErrorCode tiny_InitSettings(tiny_AdmmSettings* stgs) {
  SLAP_ASSERT(stgs != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
  "tiny_InitSettings: settings must not be TINY_NULL");
  stgs->reg_min       = (float)REG_MIN;
  stgs->reg_max       = (float)REG_MAX;
  stgs->reg_mul       = (float)REG_MUL;
  stgs->en_reg_update = EN_REG_UPDATE;

  stgs->rho_init  = (float)RHO_INIT;
  stgs->rho_max   = (float)RHO_MAX;
  stgs->rho_mul   = (float)RHO_MUL;

  stgs->alpha_mul = (float)ALPHA_MUL;

  stgs->max_iter          = MAX_ITER;
  stgs->max_iter_riccati  = MAX_ITER_RICCATI;
  stgs->max_iter_ls       = MAX_ITER_LS;

  stgs->tol_abs_prim    = (float)TOL_ABS_PRIM;
  stgs->tol_abs_dual    = (float)TOL_ABS_DUAL;

  stgs->en_cstr_states  = EN_CSTR_STATES;
  stgs->en_cstr_inputs  = EN_CSTR_INPUTS;
  stgs->en_cstr_goal    = EN_CSTR_GOAL;

  stgs->verbose           = VERBOSE;
  stgs->adaptive_horizon  = ADAPTIVE_HORIZON;
  stgs->check_riccati     = CHECK_RICCATI;
  stgs->check_termination = CHECK_TERMINATION;
  stgs->warm_start        = WARM_START;
  stgs->time_limit        = TIME_LIMIT;

  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetUnconstrained(tiny_AdmmSettings* stgs) {
  stgs->en_cstr_states  = 0;
  stgs->en_cstr_inputs  = 0;
  stgs->en_cstr_goal    = 0;
  stgs->check_termination = 0;

  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_InitSolnTrajFromArray(tiny_AdmmWorkspace* work,
Matrix* X, Matrix* U,
float* X_data, float* U_data) {

  int N = work->data->model->nhorizon;
  int n = work->data->model[0].nstates;
  int m = work->data->model->ninputs;

  work->soln->X = X;  
  work->soln->U = U;

  for (int i = 0; i < N - 1; ++i) {
    U[i] = slap_MatrixFromArray(m, 1, &U_data[i * m]);
  }

  for (int i = 0; i < N; ++i) {
    X[i] = slap_MatrixFromArray(n, 1, &X_data[i * n]);
  }

  return TINY_NO_ERROR;  
}

enum tiny_ErrorCode tiny_InitSolnDualsFromArray(tiny_AdmmWorkspace* work,
Matrix* YX, Matrix* YU,
float* YX_data, float* YU_data, float* YG_data) {

  int N = work->data->model->nhorizon;
  int n = work->data->model[0].nstates;
  int m = work->data->model->ninputs;

  work->soln->YX = YX;  
  work->soln->YU = YU;
  if (YG_data) {
    work->soln->YG = slap_MatrixFromArray(n, 1, YG_data);
  }

  if (YU_data) {
    for (int i = 0; i < N - 1; ++i) {
      YU[i] = slap_MatrixFromArray(m, 1, &YU_data[i * m]);
    }
  }

  if (YX_data) {
    for (int i = 0; i < N; ++i) {
      YX[i] = slap_MatrixFromArray(n, 1, &YX_data[i * n]);
    }
  }

  return TINY_NO_ERROR;  
}

enum tiny_ErrorCode tiny_InitSolnGainsFromArray(tiny_AdmmWorkspace* work, 
Matrix* d, Matrix* p, float* d_data, float* p_data, 
float* Kinf_data, float* Pinf_data) {

  int N = work->data->model->nhorizon;
  int n = work->data->model[0].nstates;
  int m = work->data->model->ninputs;

  work->soln->Kinf = slap_MatrixFromArray(m, n, Kinf_data);
  work->soln->d = d;
  work->soln->Pinf = slap_MatrixFromArray(n, n, Pinf_data);
  work->soln->p = p;

  for (int i = 0; i < N - 1; ++i) {
    d[i] = slap_MatrixFromArray(m, 1, &d_data[i * m]);
  }
  for (int i = 0; i < N; ++i) {
    p[i] = slap_MatrixFromArray(n, 1, &p_data[i * n]);
  }

  return TINY_NO_ERROR;  
}

enum tiny_ErrorCode tiny_InitDataQuadCostFromArray(tiny_AdmmWorkspace* work, 
float* Q_data, 
float* R_data) {

  int n = work->data->model[0].nstates;
  int m = work->data->model->ninputs;
  work->data->Q = slap_MatrixFromArray(n, n, Q_data);
  work->data->R = slap_MatrixFromArray(m, m, R_data);
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_InitDataLinearCostFromArray(tiny_AdmmWorkspace* work, 
Matrix* q, Matrix* r, Matrix* r_tilde, float* q_data, float* r_data, float* r_tilde_data) {

  int N = work->data->model[0].nhorizon;
  int n = work->data->model[0].nstates;
  int m = work->data->model[0].ninputs;
  work->data->q = q;
  work->data->r = r;
  work->data->r_tilde = r_tilde;
  
  for (int i = 0; i < N - 1; ++i) {
    q[i] = slap_MatrixFromArray(n, 1, &q_data[i * n]);
    r[i] = slap_MatrixFromArray(m, 1, &r_data[i * m]);
    r_tilde[i] = slap_MatrixFromArray(m, 1, &r_tilde_data[i * m]);
  }

  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_InitWorkspace(tiny_AdmmWorkspace* work,
                                       tiny_AdmmInfo* info,
                                       tiny_Model* model,
                                       tiny_AdmmData* data,
                                       tiny_AdmmSolution* soln,
                                       tiny_AdmmSettings* stgs) {
  SLAP_ASSERT(work != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
  "tiny_InitWorkspace: work must not be TINY_NULL");
  SLAP_ASSERT(info != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
  "tiny_InitWorkspace: info must not be TINY_NULL");
  SLAP_ASSERT(model != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
  "tiny_InitWorkspace: model must not be TINY_NULL");
  SLAP_ASSERT(data != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
  "tiny_InitWorkspace: data must not be TINY_NULL");    
  SLAP_ASSERT(stgs != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
  "tiny_InitWorkspace: stgs must not be TINY_NULL");
  SLAP_ASSERT(soln != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
  "tiny_InitWorkspace: soln must not be TINY_NULL");  

  work->data = data;
  work->info = info;
  work->soln = soln;
  work->stgs = stgs;
  work->data->model = model;

  // tiny_InitSolution(work);
  // tiny_InitData(work);

  // int n = model->nstates;
  int m = model->ninputs;
  int N = model->nhorizon;

  work->reg = (float)REG_MIN;
  work->alpha = (float)ALPHA;
  work->rho = work->stgs->rho_init;

  work->data_size = m + 2*m*(N - 1);  // only input constraints
  work->first_run = 1;
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_InitWorkspaceTempData(tiny_AdmmWorkspace* work, 
Matrix* ZU, Matrix* ZU_new, Matrix* ZX, Matrix* ZX_new, float* temp_data) {
  int N = work->data->model[0].nhorizon;
  int n = work->data->model[0].nstates;
  int m = work->data->model[0].ninputs;

  float* ptr = temp_data;
  // work->Quu_inv = slap_MatrixFromArray(m, m, ptr); 
  // ptr += m*m;
  work->Qu = slap_MatrixFromArray(m, 1, ptr);   
  ptr += m;
  // work->AmBKt = slap_MatrixFromArray(n, n, ptr);
  // ptr += n*n; 
  // work->coeff_d2p = slap_MatrixFromArray(n, m, ptr); 
  // ptr += n*m; 

  work->ZU     = ZU;
  work->ZU_new = ZU_new;
  work->ZX     = ZX;
  work->ZX_new = ZX_new;

  if (ZU) {
    for (int i = 0; i < N - 1; ++i) {
      ZU[i] = slap_MatrixFromArray(m, 1, ptr); 
      ptr += m;
      ZU_new[i] = slap_MatrixFromArray(m, 1, ptr); 
      ptr += m;
    }
  }

  if (ZX) {
    for (int i = 0; i < N; ++i) {
      ZX[i] = slap_MatrixFromArray(n, 1, ptr); 
      ptr += n;
      ZX_new[i] = slap_MatrixFromArray(n, 1, ptr); 
      ptr += n;
    }
  }

  return TINY_NO_ERROR;
}

// enum tiny_ErrorCode tiny_EvalPrimalCache(tiny_AdmmWorkspace* work) {
//   tiny_Model* model = work->data->model;

//   int n = work->data->model[0].nstates;
//   int m = work->data->model->ninputs;

//   float PB_data[n*m];
//   T_INIT_ZEROS(PB_data);
//   Matrix PB = slap_MatrixFromArray(n, m, PB_data);

//   if (model[0].ltv) {

//   }
//   else {
//     // cache for (A - BK)'
//     // slap_Copy(work->AmBKt, model[0].A[0]);
//     // slap_MatMulAdd(work->AmBKt, model[0].B[0], work->soln->Kinf, -1, 1);
//     // work->AmBKt = slap_Transpose(work->AmBKt);
//     slap_MatMulAB(work->AmBKt, model[0].B[0], work->soln->Kinf);
//     slap_MatrixAddition(work->AmBKt, model[0].A[0], work->AmBKt, -1);

//     // cache for Kinf'*R - AmBKt*Pinf*B
//     slap_MatMulAtB(work->coeff_d2p, work->soln->Kinf, work->data->R);
//     slap_MatMulAB(PB, work->soln->Pinf, model[0].B[0]);
//     slap_MatMulAdd(work->coeff_d2p, work->AmBKt, PB, -1, 1);
//   }
//   return TINY_NO_ERROR;
// }

enum tiny_ErrorCode tiny_InitPrimalCache(tiny_AdmmWorkspace* work, 
float* Quu_inv_data, float* AmBKt_data, float* coeff_d2p_data) {

  int n = work->data->model[0].nstates;
  int m = work->data->model->ninputs;

  work->Quu_inv   = slap_MatrixFromArray(m, m, Quu_inv_data); 
  work->AmBKt     = slap_MatrixFromArray(n, n, AmBKt_data); 
  work->coeff_d2p = slap_MatrixFromArray(n, m, coeff_d2p_data); 

  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_ResetInfo(tiny_AdmmWorkspace* work) {
  work->info->iter = 0;
  work->info->iter_riccati = 0;
  work->info->status_val = TINY_UNSOLVED;

  work->info->obj_val = 0.0;
  work->info->pri_res = 0.0;
  work->info->dua_res = 0.0;
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetStateReference(tiny_AdmmWorkspace* work, Matrix* Xref, 
float* Xref_data) {
  int n = work->data->model[0].nstates;
  int N = work->data->model[0].nhorizon;
  work->data->Xref = Xref;
  for (int i = 0; i < N; ++i) {
    Xref[i] = slap_MatrixFromArray(n, 1, &Xref_data[i * n]);
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetInputReference(tiny_AdmmWorkspace* work, Matrix* Uref,
float* Uref_data) {
  int m = work->data->model[0].ninputs;
  int N = work->data->model[0].nhorizon;
  work->data->Uref = Uref;
  for (int i = 0; i < N - 1; ++i) {
    Uref[i] = slap_MatrixFromArray(m, 1, &Uref_data[i * m]);
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetReference(tiny_AdmmWorkspace* work, Matrix* Xref, 
Matrix* Uref, float* Xref_data, float* Uref_data) {

  tiny_SetStateReference(work, Xref, Xref_data);
  tiny_SetInputReference(work, Uref, Uref_data);
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetGoalReference(tiny_AdmmWorkspace* work, Matrix* Xref,
Matrix* Uref, float* xg_data, float* ug_data) {
  int n = work->data->model[0].nstates;
  int m = work->data->model[0].ninputs;
  int N = work->data->model[0].nhorizon;
  work->data->Xref = Xref;
  work->data->Uref = Uref;
  for (int i = 0; i < N; ++i) {
    Xref[i] = slap_MatrixFromArray(n, 1, xg_data);
  }
  for (int i = 0; i < N - 1; ++i) {
    Uref[i] = slap_MatrixFromArray(m, 1, ug_data);
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetInitialState(tiny_AdmmWorkspace* work, float* x0_data) {
  int n = work->data->model[0].nstates;
  work->data->x0 = slap_MatrixFromArray(n, 1, x0_data);
  return TINY_NO_ERROR;
}