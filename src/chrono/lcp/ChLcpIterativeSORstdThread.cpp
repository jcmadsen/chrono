//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//////////////////////////////////////////////////
//
//   ChLcpIterativeSORstdThread.cpp
//
//  An iterative VI solver based on projective
//  fixed point method, with overrelaxation
//  and immediate variable update as in SOR methods,
//  implemented on multicore processors.
//
//  Based on ChLcpIterativeSORstdThread, using
//  std::thread threading and mutex locks rather
//  than custom threading and locking implementation
//
//  Author: Justin Madsen, 2016
//
///////////////////////////////////////////////////

#include "ChLcpIterativeSORstdThread.h"
#include "ChLcpConstraintTwoTuplesFrictionT.h"
#include "ChLcpConstraintTwoTuplesRollingN.h"
#include "ChLcpConstraintTwoTuplesRollingT.h"
#include <stdio.h>
#include <thread>
#include <algorithm>

namespace chrono {

  // Register into the object factory, to enable run-time
  // dynamic creation and persistence
  ChClassRegister<ChLcpIterativeSORstdThread> a_registration_ChLcpIterativeSORstdThread;


  // Each thread will own an instance of the following data:

  Thread_data::Thread_data(ChLcpIterativeSORstdThread* solver, std::shared_ptr<std::mutex> q_mutex,
    size_t constr_from, size_t constr_to, size_t var_from, size_t var_to,
    std::vector<ChLcpConstraint*>* constraints, std::vector<ChLcpVariables*>* variables)
    : m_solver(solver), m_mutex(q_mutex), m_constr_from(constr_from), m_constr_to(constr_to),
    m_var_from(var_from), m_var_to(var_to), m_constraints(constraints), m_variables(variables)
  {}

  /// setupt aux data in constraints
  void Thread_data::Prepare()
  {
    //    Update auxiliary data in all constraints before starting,
    //    that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
    //
    for (size_t ic = m_constr_from; ic < m_constr_to; ++ic)
      (*m_constraints)[ic]->Update_auxiliary();

    //    Average all g_i for the triplet of contact constraints n,u,v.
    //
    int j_friction_comp = 0;
    double gi_values[3];
    for (size_t ic = m_constr_from; ic < m_constr_to; ++ic) {
      if ((*m_constraints)[ic]->GetMode() == CONSTRAINT_FRIC) {
        gi_values[j_friction_comp] = (*m_constraints)[ic]->Get_g_i();
        j_friction_comp++;
        if (j_friction_comp == 3) {
          double average_g_i = (gi_values[0] + gi_values[1] + gi_values[2]) / 3.0;
          (*m_constraints)[ic - 2]->Set_g_i(average_g_i);
          (*m_constraints)[ic - 1]->Set_g_i(average_g_i);
          (*m_constraints)[ic - 0]->Set_g_i(average_g_i);
          j_friction_comp = 0;
        }
      }
    }

  }

  /// for all items with variables, the initial guess for
  ///     still unconstrained system:
  void Thread_data::AddForces()
  {
    for (size_t iv = m_var_from; iv < m_var_to; ++iv)
      if ((*m_variables)[iv]->IsActive())
        (*m_variables)[iv]->Compute_invMb_v((*m_variables)[iv]->Get_qb(),
        (*m_variables)[iv]->Get_fb());  // q = [M]'*fb
  }

  /// iterate on lagrange multipliers until tolerance or max iterations is reached
  void Thread_data::LoopConstraints()
  {
    //     For all items with variables, add the effect of initial (guessed)
    //     lagrangian reactions of contraints, if a warm start is desired.
    //     Otherwise, if no warm start, simply resets initial lagrangians to zero.
    //
    if (m_solver->GetWarmStart()) {
      for (size_t ic = m_constr_from; ic < m_constr_to; ++ic)
        if ((*m_constraints)[ic]->IsActive()) {
          //	m_mutex->lock();   // this avoids double writing on shared q vector
          (*m_constraints)[ic]->Increment_q((*m_constraints)[ic]->Get_l_i());
          //	m_mutex->unlock(); // end critical section
        }
    }
    else {
      for (size_t ic = m_constr_from; ic < m_constr_to; ++ic)
        (*m_constraints)[ic]->Set_l_i(0.);
    }

    //    Perform the solver iteration loops on thread's constraints

    double maxviolation = 0.;
    double maxdeltalambda = 0.;
    int i_friction_comp = 0;
    double old_lambda_friction[3];

    for (size_t iter = 0; iter < m_solver->GetMaxIterations(); iter++) {
      maxviolation = 0;
      maxdeltalambda = 0;
      i_friction_comp = 0;

      for (size_t ic = m_constr_from; ic < m_constr_to; ic++) {
        // skip computations if constraint not active.
        if ((*m_constraints)[ic]->IsActive()) {
          // compute residual  c_i = [Cq_i]*q + b_i + cfm_i*l_i
          double mresidual = (*m_constraints)[ic]->Compute_Cq_q() + (*m_constraints)[ic]->Get_b_i() +
            (*m_constraints)[ic]->Get_cfm_i() * (*m_constraints)[ic]->Get_l_i();

          // true constraint violation may be different from 'mresidual' (ex:clamped if unilateral)
          double candidate_violation = fabs((*m_constraints)[ic]->Violation(mresidual));

          // compute:  delta_lambda = -(omega/g_i) * ([Cq_i]*q + b_i + cfm_i*l_i )
          double deltal = (m_solver->GetOmega() / (*m_constraints)[ic]->Get_g_i()) * (-mresidual);

          if ((*m_constraints)[ic]->GetMode() == CONSTRAINT_FRIC) {
            candidate_violation = 0;

            // update:   lambda += delta_lambda;
            old_lambda_friction[i_friction_comp] = (*m_constraints)[ic]->Get_l_i();
            (*m_constraints)[ic]->Set_l_i(old_lambda_friction[i_friction_comp] + deltal);
            i_friction_comp++;

            if (i_friction_comp == 1)
              candidate_violation = fabs(ChMin(0.0, mresidual));

            if (i_friction_comp == 3) {
              (*m_constraints)[ic - 2]->Project();  // the N normal component will take care of N,U,V
              double new_lambda_0 = (*m_constraints)[ic - 2]->Get_l_i();
              double new_lambda_1 = (*m_constraints)[ic - 1]->Get_l_i();
              double new_lambda_2 = (*m_constraints)[ic - 0]->Get_l_i();
              // Apply the smoothing: lambda= sharpness*lambda_new_projected +
              // (1-sharpness)*lambda_old
              if (m_solver->GetSharpnessLambda() != 1.0) {
                double shlambda = m_solver->GetSharpnessLambda();
                new_lambda_0 = shlambda * new_lambda_0 + (1.0 - shlambda) * old_lambda_friction[0];
                new_lambda_1 = shlambda * new_lambda_1 + (1.0 - shlambda) * old_lambda_friction[1];
                new_lambda_2 = shlambda * new_lambda_2 + (1.0 - shlambda) * old_lambda_friction[2];
                (*m_constraints)[ic - 2]->Set_l_i(new_lambda_0);
                (*m_constraints)[ic - 1]->Set_l_i(new_lambda_1);
                (*m_constraints)[ic - 0]->Set_l_i(new_lambda_2);
              }
              double true_delta_0 = new_lambda_0 - old_lambda_friction[0];
              double true_delta_1 = new_lambda_1 - old_lambda_friction[1];
              double true_delta_2 = new_lambda_2 - old_lambda_friction[2];
              //	m_mutex->lock();   // this avoids double writing on shared q vector
              (*m_constraints)[ic - 2]->Increment_q(true_delta_0);
              (*m_constraints)[ic - 1]->Increment_q(true_delta_1);
              (*m_constraints)[ic - 0]->Increment_q(true_delta_2);
              //	m_mutex->unlock(); // end critical section

              if (m_solver->GetRecordViolation())
              {
                maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_0));
                maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_1));
                maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta_2));
              }
              i_friction_comp = 0;
            }
          }
          else {
            // update:   lambda += delta_lambda;
            double old_lambda = (*m_constraints)[ic]->Get_l_i();
            (*m_constraints)[ic]->Set_l_i(old_lambda + deltal);

            // If new lagrangian multiplier does not satisfy inequalities, project
            // it into an admissible orthant (or, in general, onto an admissible set)
            (*m_constraints)[ic]->Project();

            // After projection, the lambda may have changed a bit..
            double new_lambda = (*m_constraints)[ic]->Get_l_i();

            // Apply the smoothing: lambda= sharpness*lambda_new_projected + (1-sharpness)*lambda_old
            if (m_solver->GetSharpnessLambda() != 1.0) {
              double shlambda = m_solver->GetSharpnessLambda();
              new_lambda = shlambda * new_lambda + (1.0 - shlambda) * old_lambda;
              (*m_constraints)[ic]->Set_l_i(new_lambda);
            }

            double true_delta = new_lambda - old_lambda;

            // For all items with variables, add the effect of incremented
            // (and projected) lagrangian reactions:
            m_mutex->lock();  // this avoids double writing on shared q vector
            (*m_constraints)[ic]->Increment_q(true_delta);
            m_mutex->unlock();  // end critical section

            if (m_solver->GetRecordViolation())
              maxdeltalambda = ChMax(maxdeltalambda, fabs(true_delta));


            maxviolation = ChMax(maxviolation, fabs(candidate_violation));

          }  // end IsActive()

        }  // end loop on constraints

        // For recording into violaiton history, if debugging
        if (m_solver->GetRecordViolation())
          AtIterationEnd(maxviolation, maxdeltalambda);

        // Terminate the loop if violation in constraints has been succesfully limited.
        if (maxviolation < m_solver->GetTolerance())
          break;

      }  // end iteration loop
    }
  }


  // When the solver object is created, threads are also
  // created and initialized, in 'wait' mode.

  ChLcpIterativeSORstdThread::ChLcpIterativeSORstdThread(int nthreads,
    int mmax_iters,
    bool mwarm_start,
    double mtolerance,
    double momega)
    : ChLcpIterativeSolver(mmax_iters, mwarm_start, mtolerance, momega),
    m_num_threads(nthreads) { }

  ChLcpIterativeSORstdThread::~ChLcpIterativeSORstdThread() {}


  double ChLcpIterativeSORstdThread::Solve(
    ChLcpSystemDescriptor& sysd  ///< system description with constraints and variables
    ) {
    std::vector<ChLcpConstraint*>& m_constraints = sysd.GetConstraintsList();
    std::vector<ChLcpVariables*>& mvariables = sysd.GetVariablesList();

    double maxviolation = 0.;
    double maxdeltalambda = 0.;
    int i_friction_comp = 0;

    /////////////////////////////////////////////
    /// THE PARALLEL SOLVER, PERFORMED IN STAGES

    // writing to state variables may require a lock to prevent race conditions
    auto q_mutex = std::make_shared<std::mutex>();

    // --0--  preparation:
    //        subdivide the workload to the threads and prepare their 'Thread_data':


    std::vector<Thread_data> thread_data;

    size_t var_slice = 0;
    size_t constr_slice = 0;
    for (size_t nth = 0; nth < m_num_threads; ++nth) {
      size_t var_from = var_slice;
      size_t var_to = var_from + ((size_t)mvariables.size() - var_from) / (m_num_threads - nth);
      size_t constr_from = constr_slice;
      size_t constr_to = constr_from + ((size_t)m_constraints.size() - constr_from) / (m_num_threads - nth);
      if (constr_to < m_constraints.size())  // do not slice the three contact multipliers (or six in case of rolling)
      {
        if (dynamic_cast<ChLcpConstraintTwoTuplesFrictionTall*>(m_constraints[constr_to]))
          constr_to++;
        if (dynamic_cast<ChLcpConstraintTwoTuplesFrictionTall*>(m_constraints[constr_to]))
          constr_to++;
        if (constr_to < m_constraints.size()) {
          if (dynamic_cast<ChLcpConstraintTwoTuplesRollingNall*>(m_constraints[constr_to]))
            constr_to++;
          if (dynamic_cast<ChLcpConstraintTwoTuplesRollingTall*>(m_constraints[constr_to]))
            constr_to++;
          if (dynamic_cast<ChLcpConstraintTwoTuplesRollingTall*>(m_constraints[constr_to]))
            constr_to++;
        }
      }

      // create the threads to operate on subdivisions of Variables, LcpConstraints
      thread_data.push_back(Thread_data(this, q_mutex, constr_from, constr_to,
        var_from, var_to, &m_constraints, &mvariables));

      var_slice = var_to;
      constr_slice = constr_to;
    }

    // launch threads in stages, sync between
    std::vector<std::thread> thread_stage;

    // --1--  stage:
    //        precompute aux variables in constraints.
    for (auto thread_data_n : thread_data) {
      thread_stage.push_back(std::thread(&Thread_data::Prepare, &thread_data_n));
    }
    // threads rejoin main execution thread when finished
    std::for_each(thread_stage.begin(), thread_stage.end(), std::mem_fn(&std::thread::join));
    thread_stage.clear();

    // --2--  stage:
    //        add external forces and mass effects, on variables.
    for (auto thread_data_n : thread_data) {
      thread_stage.push_back(std::thread(&Thread_data::AddForces, &thread_data_n));
    }
    // threads rejoin main execution thread when finished
    std::for_each(thread_stage.begin(), thread_stage.end(), std::mem_fn(&std::thread::join));
    thread_stage.clear();

    // --3--  stage:
    //        loop on constraints.
    for (auto thread_data_n : thread_data) {
      thread_stage.push_back(std::thread(&Thread_data::AddForces, &thread_data_n));
    }
    // threads rejoin main execution thread when finished
    std::for_each(thread_stage.begin(), thread_stage.end(), std::mem_fn(&std::thread::join));
    thread_stage.clear();

    // if tracking solver history, find max of each violation between threads
    if (GetRecordViolation())
      AtStepEnd(thread_data);


    return 0;
  }

  void ChLcpIterativeSORstdThread::ChangeNumberOfThreads(size_t threads) {
    threads < 1 ? m_num_threads = 1 : m_num_threads = threads;
  }

  void ChLcpIterativeSORstdThread::AtStepEnd(const std::vector<Thread_data>& threads){
    size_t max_iters = 0;
    for (auto tid : threads){
      max_iters = ChMax(max_iters, tid.Get_max_dlambda().size());
    }



  }


}  // END_OF_NAMESPACE____
