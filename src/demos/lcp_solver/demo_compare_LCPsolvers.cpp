//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   Demo code about
//
//     - compare LCP solvers for simulation accuracy, speed 
//      and solver errors.
//     - based on demo_banchmark.cpp   
//
//	 Author: Justin Madsen, 2016
///////////////////////////////////////////////////

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/lcp/ChLcpIterativeSolver.h"
#include <algorithm>

// Use the namespaces of Chrono
using namespace chrono;

// defaults
int STATIC_lcp_iters = 20;
int STATIC_lcp_iters_stab = 20;
double SPHERE_RAD = 0.35;  // 0.9
double SPHERE_RHO = 1000.0;
int SPHERE_NUM = 1000;  // 400

// optional: pass in the sphere CM positions
std::vector<ChVector<> > create_some_falling_items(ChSystem& phySystem, std::vector<ChVector<> > sphere_pos = std::vector<ChVector<> > {} ) {
  
  std::vector<ChVector<> > outVect;
  for (int bi = 0; bi < SPHERE_NUM; bi++) {
    auto msphereBody = std::make_shared<ChBodyEasySphere>(SPHERE_RAD,    // radius size
      SPHERE_RHO,   // density
      true,   // collide enable?
      false);  // visualization?

    // create some random 
    if (sphere_pos.size() == 0){
      outVect.push_back(ChVector<>(1.9*(ChRandom() - 0.5), bi*(-0.9 / (double)SPHERE_NUM), 1.9*(ChRandom() - 0.5)) );
      msphereBody->SetPos(outVect.back());
    }
    else {
      msphereBody->SetPos(sphere_pos[bi]);
    }
    
    msphereBody->GetMaterialSurface()->SetFriction(0.2f);

    phySystem.Add(msphereBody);
  }

  // Create the five walls of the rectangular container, using
  // fixed rigid bodies of 'box' type:

  auto floorBody = std::make_shared<ChBodyEasyBox>(2.2, 0.1, 2.2, 1000, true, false);
  floorBody->SetPos(ChVector<>(0, -1, 0));
  floorBody->SetBodyFixed(true);

  phySystem.Add(floorBody);

  auto wallBody1 = std::make_shared<ChBodyEasyBox>(0.1, 2.2, 1.1, 1000, true, false);
  wallBody1->SetPos(ChVector<>(-1, 0, 0));
  wallBody1->SetBodyFixed(true);

  phySystem.Add(wallBody1);

  auto wallBody2 = std::make_shared<ChBodyEasyBox>(0.1, 2.2, 1.1, 1000, true, false);
  wallBody2->SetPos(ChVector<>(1, 0, 0));
  wallBody2->SetBodyFixed(true);

  phySystem.Add(wallBody2);

  auto wallBody3 = std::make_shared<ChBodyEasyBox>(1.1, 2.2, 0.1, 1000, true, false);
  wallBody3->SetPos(ChVector<>(0, 0, -1));
  wallBody3->SetBodyFixed(true);

  phySystem.Add(wallBody3);

  auto wallBody4 = std::make_shared<ChBodyEasyBox>(1.1, 2.2, 0.1, 1000, true, false);
  wallBody4->SetPos(ChVector<>(0, 0, 1));
  wallBody4->SetBodyFixed(true);

  phySystem.Add(wallBody4);

  if (sphere_pos.size() == 0)
    return outVect;
  else
    return sphere_pos;

}

// create bodies for a given, ChSystem, given solver type/settings.
// run a simulation, report on: final body positions, solver error, clock time
void RunTest(ChSystem& sys, ChTimer<>& timer, std::vector<ChVector<> >& body_pos_final,
  std::vector<double>& violation_per_step, std::vector<double>& dlambda_per_step) {

}

int main(int argc, char* argv[]) {
  // Create a ChronoENGINE physical system
  ChSystem phySystem;


  // phySystem.SetIntegrationType(ChSystem::INT_EULER_IMPLICIT_LINEARIZED);

  phySystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD);

  phySystem.SetIterLCPsharpnessLambda(0.9);
  phySystem.SetIterLCPomega(0.9);
  phySystem.SetMaxPenetrationRecoverySpeed(1.6);  // used by Anitescu stepper only
  phySystem.SetIterLCPmaxItersSpeed(STATIC_lcp_iters);
  phySystem.SetIterLCPmaxItersStab(STATIC_lcp_iters_stab);  // unuseful for Anitescu, only Tasora uses this
  phySystem.SetIterLCPwarmStarting(true);

  // use this to compute error in final position of bodies
  std::vector<ChVector<> > body_pos_final;
  body_pos_final.resize(SPHERE_NUM);

  int endframe = 500;
  int total_tests = 4;
  double step_size = 0.01;
  // raw tests, all the same, only for profiling
  if (true) {
    GetLog() << "Raw tests for profiling... \n\n";
    ChTimer<double> timer;

    ChLcpIterativeSolver* msolver = (ChLcpIterativeSolver*)phySystem.GetLcpSolverSpeed();
    msolver->SetRecordViolation(true);

    for (int ntest = 0; ntest < total_tests; ++ntest) {
      // create spheres and walls
      ChSetRandomSeed(123);
      auto bodies = create_some_falling_items(phySystem);

      int totframes = 0;
      timer.start();

      // ------- begin simulation loop -----
      while (true) {
        totframes++;

        // integrate
        phySystem.DoStepDynamics(step_size);

        if (totframes > endframe)
          break;

      }  // ------- end of simulation loop -----

      timer.stop();

      // remove the ChBody objects to reset and prepare for new scene at next for() loop.
      phySystem.RemoveAllBodies();

      // Now the system should already have no bodies, because of full removal from Irrlicht scene.
      // Then, the following clearing should be unuseful...(do it anyway..)
      phySystem.Clear();
    }
    GetLog() << "total time spent: " << timer() << "\n";
    GetLog() << "time spent per second of simulation: " << timer() / (step_size * endframe / total_tests) << "\n\n";
    system("pause");
    return 0;
  }  // end test
}
