#include "admm.h"

enum tiny_ErrorCode tiny_SolveAdmm(tiny_AdmmWorkspace* work) {

  tiny_ResetInfo(work);

  // Shortcut unconstrained problem
  if (!IsConstrained(work)) {
    tiny_SolveLqr(work);
    return TINY_NO_ERROR;
  }

  int N = work->data->model[0].nhorizon;
  int exitflag;
  int iter;
  int compute_cost_function; // Boolean: compute the cost function in the loop or not
  int can_check_termination; // Boolean: check termination or not
  int can_print;             // Boolean whether you can print

  // Initialize variables
  exitflag              = 0;
  can_check_termination = 0;
  can_print = work->stgs->verbose;
  compute_cost_function = work->stgs->verbose;

  // if (work->stgs->verbose) {
  // // Print Header for every column
  //   PrintHeader();
  // }

  // Main ADMM algorithm
  for (iter = 1; iter <= work->stgs->max_iter; iter++) {
    /* ADMM STEPS */

    // Update z_prev (preallocated, no malloc)
    for (int i = 0; i < N - 1; ++i) {
      SwapVectors(&(work->ZU_new[i].data), &(work->ZU[i].data));
    }

    /* Compute x^{k+1} */
    UpdatePrimal(work);

    /* Compute z^{k+1} and y^{k+1} */
    UpdateSlackDual(work);

    /* Update for next primal solve */
    tiny_UpdateConstrainedLinearCost(work);

    /* End of ADMM STEPS */
    // print()
    // Can we check for termination ?
    can_check_termination = work->stgs->check_termination &&
                            (iter % work->stgs->check_termination == 0);

    can_print = work->stgs->verbose &&
                ((iter % work->stgs->check_termination == 0) || (iter == 1));

    if (can_check_termination || can_print) { // Update status in either of
                                              // these cases
      // Update information
      UpdateInfo(work, iter, compute_cost_function);

      // if (can_print) {
      //   // Print summary
      //   PrintIteration(work);
      // }

      if (can_check_termination) {
        // Check algorithm termination
        exitflag = CheckTermination(work);
        if (exitflag) {
          // Terminate algorithm
          break;
        }
      }
    }
    
  } // End of ADMM `for` loop

  /* if max iterations reached, change status accordingly */
  if (work->info->status_val == TINY_UNSOLVED) {
    work->info->status_val = TINY_MAX_ITER_REACHED;
  }
  // if (work->stgs->verbose) PrintSummary(work->info);
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode UpdatePrimal(tiny_AdmmWorkspace* work) {
  tiny_SolveLqr(work);
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode UpdateSlackDual(tiny_AdmmWorkspace* work) {
  int n = work->data->model[0].ninputs;
  int N = work->data->model[0].nhorizon;

  for (int k = 0; k < N - 1; ++k) {
    MatAdd(work->soln->YU[k], work->soln->YU[k], work->soln->U[k], 1);
    for (int i = 0; i < n; ++i) { 
      work->ZU_new[k].data[i] = T_MIN(T_MAX(work->soln->YU[k].data[i],
                                work->data->lcu.data[i]),  // Between lower
                                work->data->ucu.data[i]);  // and upper bounds
    } 
    MatAdd(work->soln->YU[k], work->soln->YU[k], work->ZU_new[k], -1);
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode ComputePrimalResidual(tiny_AdmmWorkspace* work) {
  int n = work->data->model[0].ninputs;
  int N = work->data->model[0].nhorizon;
  work->info->pri_res = 0;
  for (int k = 0; k < N - 1; ++k) {    
    for (int i = 0; i < n; ++i) {      
      work->info->pri_res = T_MAX(work->info->pri_res, 
                    T_ABS(work->soln->U[k].data[i] - work->ZU_new[k].data[i]));
    } 
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode ComputeDualResidual(tiny_AdmmWorkspace* work) {
  int n = work->data->model[0].ninputs;
  int N = work->data->model[0].nhorizon;
  work->info->dua_res = 0;
  for (int k = 0; k < N - 1; ++k) {
    for (int i = 0; i < n; ++i) {      
      work->info->dua_res = T_MAX(work->info->dua_res, 
                    T_ABS(work->ZU_new[k].data[i] - work->ZU[k].data[i]));
    } 
  }
  work->info->dua_res = work->info->dua_res * work->rho;
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode ComputeObjectiveValue(tiny_AdmmWorkspace* work) {
  int N = work->data->model[0].nhorizon;
  work->info->obj_val = 0.0;
  
  for (int k = 0; k < N - 1; ++k) {
    tiny_AddStageCost(work, k);
  } 
  tiny_AddTerminalCost(work);       
  return TINY_NO_ERROR;
}

int CheckTermination(tiny_AdmmWorkspace* work) {
  // If residuals are too large, the problem is probably non convex
  if ((work->info->pri_res > TINY_INFTY) ||
      (work->info->dua_res > TINY_INFTY)){
    // Looks like residuals are diverging. Probably the problem is non convex!
    // Terminate and report it
    work->info->status_val = TINY_NON_CVX;
    work->info->obj_val = TINY_NAN;
    return 1;
  }

  if ((work->info->pri_res < work->stgs->tol_abs_prim) 
    && (work->info->dua_res < work->stgs->tol_abs_dual)) {
    work->info->status_val = TINY_SOLVED;
    return 1;
  }
  return 0;
}

enum tiny_ErrorCode UpdateInfo(tiny_AdmmWorkspace* work,
                                int                 iter,
                                int                 compute_objective) {

  work->info->iter = iter; // Update iteration number
  // Compute the objective if needed
  if (compute_objective) {
    ComputeObjectiveValue(work);
  }  
  ComputePrimalResidual(work);
  ComputeDualResidual(work);
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_WarmStartInput(tiny_AdmmWorkspace* work, float* U_data) {
  int N = work->data->model->nhorizon;
  int m = work->data->model->ninputs;
  for (int i = 0; i < N - 1; ++i) {
    slap_CopyFromArray(work->soln->U[i], &U_data[i * m]);
  }
  return TINY_NO_ERROR;
}
