//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPITERATIVESORSTDTHREAD_H
#define CHLCPITERATIVESORSTDTHREAD_H

//////////////////////////////////////////////////
//
//   ChLcpIterativeSORstdThread.h
//
//  An iterative VI solver based on projective
//  fixed point method, with overrelaxation
//  and immediate variable update as in SOR methods,
//  implemented on multicore processors.
//
//  Based on ChLcpIterativeSORmultithread, using
//  std::thread and std::mutex rather
//  than custom multi-threading and locking implementations
//
//  Author: Justin Madsen, 2016
//
///////////////////////////////////////////////////

#include "ChLcpIterativeSolver.h"
#include <mutex>

namespace chrono {

  // forward reference
  class Thread_data;

  class ChApi ChLcpIterativeSORstdThread : public ChLcpIterativeSolver {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChLcpIterativeSORstdThread, ChLcpIterativeSolver);

  protected:
    // DATA
    size_t m_num_threads;

  public:
    // CONSTRUCTORS
    ChLcpIterativeSORstdThread(int nthreads = 2,          ///< number of threads
      int mmax_iters = 50,       ///< max.number of iterations
      bool mwarm_start = false,  ///< uses warm start?
      double mtolerance = 0.0,   ///< tolerance for termination criterion
      double momega = 1.0        ///< overrelaxation criterion
      );

    virtual ~ChLcpIterativeSORstdThread();

    //
    // FUNCTIONS
    //

    /// Performs the solution of the LCP.
    /// \return  the maximum constraint violation after termination.

    virtual double Solve(ChLcpSystemDescriptor& sysd  ///< system description with constraints and variables
      );

    /// Changes the number of threads which run in parallel (should be > 1 )
    void ChangeNumberOfThreads(size_t mthreads = 2);

  private:
    void AtStepEnd(const std::vector<Thread_data>& threads);
  };


  class Thread_data{
  public:
    Thread_data(ChLcpIterativeSORstdThread* solver, std::shared_ptr<std::mutex> q_mutex,
      size_t constr_from, size_t constr_to, size_t var_from, size_t var_to,
      std::vector<ChLcpConstraint*>* constraints, std::vector<ChLcpVariables*>* variables);

    Thread_data() = delete;

    /// setupt aux data in constraints
    void Prepare();

    /// for all items with variables, the initial guess for still unconstrained system:
    void AddForces();

    /// iterate on lagrange multipliers until tolerance or max iterations is reached
    void LoopConstraints();

    const std::vector<double>& Get_max_violation() { return m_max_violation; }

    const std::vector<double>& Get_max_dlambda() { return m_max_delta_lambda; }

  protected:

    // only call when record_violation_history is enabled 
    void AtIterationEnd(double maxviolation, double deltalambda) {
      m_max_violation.push_back(maxviolation);
      m_max_delta_lambda.push_back(deltalambda);
    }

    std::vector<ChLcpConstraint*>* m_constraints;
    std::vector<ChLcpVariables*>* m_variables;

    std::vector<double> m_max_violation;
    std::vector<double> m_max_delta_lambda;

    ChLcpIterativeSORstdThread* m_solver;  // reference to solver
    std::shared_ptr<std::mutex> m_mutex;   // this will be used to avoid race condition when writing to shared memory.

    // the range of scanned multipliers (for loops on multipliers)
    size_t m_constr_from;
    size_t m_constr_to;

    // the range of scanned 'variables' objects (for loops on variables, aka rigid bodies)
    size_t m_var_from;
    size_t m_var_to;

  };



}  // END_OF_NAMESPACE____

#endif  // END of ChLcpIterativeSORstdThread.h
