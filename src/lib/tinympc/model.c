#include "model.h"

enum tiny_ErrorCode tiny_InitModel(tiny_Model* model, const int nstates,
                                   const int ninputs, const int nhorizon,
                                   const int ltv, const int affine, 
                                   const float dt) {
  SLAP_ASSERT(nstates > 0 && ninputs > 0 && nstates >= ninputs, SLAP_INVALID_DIMENSION, TINY_SLAP_ERROR, 
  "tiny_InitModel: nstates (%d) >= ninputs (%d) > 0", nstates, ninputs);  

  model->nstates  = nstates;
  model->ninputs  = ninputs;
  model->nhorizon = nhorizon;

  model->ltv      = ltv;
  model->affine   = affine;
  model->dt       = dt;

  model->A = TINY_NULL;
  model->B = TINY_NULL;
  model->f = TINY_NULL;
  model->get_jacobians = TINY_NULL;
  model->get_nonl_model = TINY_NULL; 

  if (model->ltv) {
    model->data_size = model->nstates*(model->nstates + model->ninputs)*(model->nhorizon - 1);
    if (affine) {
      model->data_size += model->nstates*(model->nhorizon - 1);
    }
  }
  else {
    model->data_size = model->nstates*(model->nstates + model->ninputs);
    if (model->affine) {
    model->data_size += model->nstates;
    }
  }
  return TINY_NO_ERROR;
}

// User provides array of slap matrices
enum tiny_ErrorCode tiny_InitModelDataMatrix(tiny_Model* model, 
    Matrix* A, Matrix* B, Matrix* f) {
  SLAP_ASSERT(A != TINY_NULL && B != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
  "InitModelData: A and B must not be TINY_NULL");
  model->A = A;
  model->B = B;
  if (model->affine) {
    SLAP_ASSERT(f != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
    "InitModelData: f must not be TINY_NULL");
    if (f) {  
      model->f = f;
    }
  }
  return TINY_NO_ERROR;  
}

// User provides matrix as column-major array
enum tiny_ErrorCode tiny_InitModelFromArray(tiny_Model* model, Matrix* A, 
    Matrix* B, Matrix* f, float* A_array, float* B_array, float* f_array) {
  SLAP_ASSERT(A != TINY_NULL && B != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
  "InitModelData: A and B must not be TINY_NULL");
  SLAP_ASSERT(A_array != TINY_NULL && A_array != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
  "InitModelData: A_array and B_array must not be TINY_NULL");  
  model->A = A;
  model->B = B;
  
  if (model->ltv) {
    float* A_ptr = A_array;
    float* B_ptr = B_array;
    for (int k = 0; k < model->nhorizon-1; ++k) {
      model->A[k] = slap_MatrixFromArray(model->nstates, model->nstates, A_ptr);
      A_ptr += model->nstates * model->nstates;
      model->B[k] = slap_MatrixFromArray(model->nstates, model->ninputs, B_ptr);
      B_ptr += model->nstates * model->ninputs; 
    }
    if (model->affine) {
    SLAP_ASSERT(f_array != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
    "InitModelData: f_array must not be TINY_NULL");  
    SLAP_ASSERT(f != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
    "InitModelData: f must not be TINY_NULL");
      if (f && f_array) {  
        model->f = f;
        float* f_ptr = f_array;
        for (int k = 0; k < model->nhorizon-1; ++k) {
          model->f[k] = slap_MatrixFromArray(model->nstates, 1, f_ptr);
          f_ptr += model->nstates;     
        }
      }
    }  
  }
  else {
    model->A[0] = slap_MatrixFromArray(model->nstates, model->nstates, A_array);
    model->B[0] = slap_MatrixFromArray(model->nstates, model->ninputs, B_array);
    if (model->affine) {
      SLAP_ASSERT(f_array != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
      "InitModelData: f_array must not be TINY_NULL");  
      SLAP_ASSERT(f != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
      "InitModelData: f must not be TINY_NULL");
      if (f && f_array) {
        model->f = f;
        model->f[0] = slap_MatrixFromArray(model->nstates, 1, f_array);
      }
    }
  }
  return TINY_NO_ERROR;  
}

enum tiny_ErrorCode tiny_SetModelJacFunc(
    tiny_Model* model, 
    void (*get_jacobians)(Matrix*, Matrix*, const Matrix, const Matrix)) {
  SLAP_ASSERT(get_jacobians != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
  "SetModelJacFunc: pointer to function must not be TINY_NULL");
  model->get_jacobians = get_jacobians;
  return TINY_NO_ERROR;      
}

enum tiny_ErrorCode tiny_SetModelNonlFunc(
    tiny_Model* model, 
    void (*get_nonl_model)(Matrix*, const Matrix, const Matrix)) {
  SLAP_ASSERT(get_nonl_model != TINY_NULL, SLAP_BAD_POINTER, TINY_SLAP_ERROR,
  "SetModelNonlinear: pointer to function must not be TINY_NULL");
  model->get_nonl_model = get_nonl_model;
  return TINY_NO_ERROR;    
}

enum tiny_ErrorCode tiny_EvalModel(Matrix* xn, const Matrix x, const Matrix u,
                                   tiny_Model* model, const int k) {
  if (model->affine) {
    return TINY_NOT_SUPPORTED;
  }
  else {
    slap_MatMulAB(*xn, model->A[k], x);
  }
  MatMulAdd(*xn, model->B[k], u, 1, 1);  // x[k+1] += B * u[k]
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_RollOutClosedLoop(tiny_AdmmWorkspace* work) {
  tiny_Model* model = work->data->model;
  int N = model[0].nhorizon;
  int adaptive_horizon = work->stgs->adaptive_horizon;
  
  if (model[0].ltv) { 
    return TINY_NOT_SUPPORTED;
  }
  else {
    for (int k = 0; k < N - 1; ++k) {
      // Control input: u = - d - K*x
      MatMulAdd2(work->soln->U[k],  work->soln->d[k], work->soln->Kinf, work->soln->X[k], -1, -1);
      // Next state: x = A*x + B*u + f
      if (adaptive_horizon && k > adaptive_horizon - 1) {
        tiny_EvalModel(&(work->soln->X[k + 1]), work->soln->X[k], work->soln->U[k], &model[1], 0);
      }
      else {
        tiny_EvalModel(&(work->soln->X[k + 1]), work->soln->X[k], work->soln->U[k], &model[0], 0);
      }
    }        
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_RollOutOpenLoop(tiny_AdmmWorkspace* work) {
  tiny_Model* model = work->data->model;
  int N = model[0].nhorizon;
  int adaptive_horizon = work->stgs->adaptive_horizon;
  
  if (model[0].ltv) {
    for (int k = 0; k < N - 1; ++k) {
      // Next state: x = A*x + B*u + f
      if (adaptive_horizon && k > adaptive_horizon - 1) {
        tiny_EvalModel(&(work->soln->X[k + 1]), work->soln->X[k], work->soln->U[k], &model[1], k);
      }
      else {
        tiny_EvalModel(&(work->soln->X[k + 1]), work->soln->X[k], work->soln->U[k], &model[0], k);
      }
    }    
  }
  else {
    for (int k = 0; k < N - 1; ++k) {
      // Next state: x = A*x + B*u + f
      if (adaptive_horizon && k > adaptive_horizon - 1) {
        tiny_EvalModel(&(work->soln->X[k + 1]), work->soln->X[k], work->soln->U[k], &model[1], 0);
      }
      else {
        tiny_EvalModel(&(work->soln->X[k + 1]), work->soln->X[k], work->soln->U[k], &model[0], 0);
      }
    }        
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_UpdateModelJac(tiny_AdmmWorkspace* work) {
  tiny_Model* model = work->data->model;
  int N = model[0].nhorizon;
  // LTV model
  if (model[0].ltv) {
    for (int i = 0; i < N - 1; ++i) {
      // get A and B
      model[0].get_jacobians(&(model[0].A[i]), 
                             &(model[0].B[i]), 
                             work->data->Xref[i], 
                             work->data->Uref[i]);
      if (model[0].affine) {
        // get f = x1 - Ax - Bu
        if (model[0].get_nonl_model != TINY_NULL) {
          model[0].get_nonl_model(&(model[0].f[i]),
                                  work->data->Xref[i], 
                                  work->data->Uref[i]);
          slap_MatMulAdd(model[0].f[i], 
                         model[0].A[i], 
                         work->data->Xref[i], -1, 1);
          slap_MatMulAdd(model[0].f[i], 
                         model[0].B[i], 
                         work->data->Uref[i], -1, 1);
        }
      }
    }
  }
  // LTI model
  else {
    // get A and B
    model[0].get_jacobians(&(model[0].A[0]), 
                            &(model[0].B[0]), 
                            work->data->Xref[N-1],
                            work->data->Uref[N-1]);
    if (model[0].affine) {                     
      // get f = x1 - Ax - Bu
      if (model[0].get_nonl_model != TINY_NULL) {
        model[0].get_nonl_model(model[0].f, 
                                work->data->Xref[N-1],
                                work->data->Uref[N-1]);
        slap_MatMulAdd(model[0].f[0], 
                       model[0].A[0], 
                       work->data->Xref[N-1], -1, 1);
        slap_MatMulAdd(model[0].f[0], 
                       model[0].B[0], 
                       work->data->Uref[N-1], -1, 1);
      }
    }
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_UpdateModelJacAbout(tiny_AdmmWorkspace* work,
                                             Matrix* X, Matrix* U) {
  tiny_Model* model = work->data->model;
  int N = model[0].nhorizon;
  // LTV model
  if (model[0].ltv) {
    for (int i = 0; i < N - 1; ++i) {
      // get A and B
      model[0].get_jacobians(&(model[0].A[i]), &(model[0].B[i]), X[i], U[i]);
      
      if (model[0].affine) {
        // get f = x1 - Ax - Bu
        if (model[0].get_nonl_model != TINY_NULL) {
          model[0].get_nonl_model(&(model[0].f[i]), X[i], U[i]);
          slap_MatMulAdd(model[0].f[i], model[0].A[i], X[i], -1, 1);
          slap_MatMulAdd(model[0].f[i], model[0].B[i], U[i], -1, 1);
        }
      }
    }
  }
  // LTI model
  else {
    // get A and B
    model[0].get_jacobians(&(model[0].A[0]), &(model[0].B[0]), X[N-1], U[N-1]);
    if (model[0].affine) {      
      // printf("A");               
      // get f = x1 - Ax - Bu
      if (model[0].get_nonl_model != TINY_NULL) {
        model[0].get_nonl_model(model[0].f, 
                                work->data->Xref[N-1],
                                work->data->Uref[N-1]);
        slap_MatMulAdd(model[0].f[0], model[0].A[0], X[N-1], -1, 1);
        slap_MatMulAdd(model[0].f[0], model[0].B[0], U[N-1], -1, 1);
      }
    }
  }
  return TINY_NO_ERROR;

}