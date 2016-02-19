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

// #include "chrono_irrlicht/ChApiIrr.h"


// Use the namespaces of Chrono
using namespace chrono;

// defaults
static double SPHERE_RAD = 0.05;  // 0.9
static double SPHERE_RHO = 3000.0;
static size_t SPHERE_NUM = 1000;  // 400
static double STEP_SIZE = 0.01;
static double TEST_END_TIME = 4.0;

class SolverSettings{
public:
  SolverSettings(ChSystem::eCh_lcpSolver sol, size_t itSpeed=50, size_t itStab=10,
    double maxReb = 1.0, double omega = 1.0, double lam = 1.0): mSolver(sol),
    mIterSpeed(itSpeed), mIterStab(itStab), mMaxReb(maxReb), mOmega(omega), mLambda(lam) 
  {
    mInt = ChSystem::INT_EULER_IMPLICIT_LINEARIZED;
  }

  void Set_Solver(std::shared_ptr<ChSystem> sys) const {
    // set solver settings
    sys->SetLcpSolverType(mSolver);
    sys->SetIterLCPmaxItersSpeed(mIterSpeed);
    sys->SetIterLCPmaxItersStab(mIterStab);
    sys->SetIterLCPomega(mOmega);
    sys->SetIterLCPsharpnessLambda(mLambda);
    sys->SetMaxPenetrationRecoverySpeed(mMaxReb);
    sys->SetTol(0);
    sys->SetTolForce(0);
    // enable recording violation, lambda history
    dynamic_cast<ChLcpIterativeSolver*>(sys->GetLcpSolverSpeed())->SetRecordViolation(true);
  }

  ChSystem::eCh_lcpSolver mSolver;
  size_t mIterSpeed;
  size_t mIterStab;
  size_t mMaxReb;
  size_t mOmega;
  size_t mLambda;
  ChSystem::eCh_integrationType mInt;
};

// optional: pass in the sphere CM positions
std::vector<std::shared_ptr<ChBody> > create_bodies(std::shared_ptr<ChSystem> phySystem,
  std::vector<ChVector<> >& sphere_pos = std::vector<ChVector<> > {} ) {
  
  std::vector<ChVector<> > bodyPos; // initial body position
  std::vector<std::shared_ptr<ChBody> > outBodyPtr; // pointers to ChBody
  // collision envelope
  collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.1f*SPHERE_RAD);
  
  bool vis = false;
  // initialize the bodies
  for (int bi = 0; bi < SPHERE_NUM; bi++) {
    auto msphereBody = std::make_shared<ChBodyEasySphere>(SPHERE_RAD,    // radius size
      SPHERE_RHO,   // density
      true,   // collide enable?
      vis);  // visualization?

    // create spheres randomly, in a box that is 10x sphere dia. by 10x sphere dia.
    if (sphere_pos.size() == 0){
      bodyPos.push_back(ChVector<>(10.0*SPHERE_RAD*(ChRandom() - 0.5),
        bi*(2.0*SPHERE_RAD/100)+SPHERE_RAD,
        10.0*SPHERE_RAD*(ChRandom() - 0.5)) );
      msphereBody->SetPos(bodyPos.back());
    }
    else {
      msphereBody->SetPos(sphere_pos[bi]);
    }
    
    msphereBody->GetMaterialSurface()->SetFriction(0.3f);
    msphereBody->GetMaterialSurface()->SetCohesion(0.0f * STEP_SIZE);

    phySystem->Add(msphereBody);
    outBodyPtr.push_back(msphereBody);
  }

  // Create the five walls of the rectangular container, using
  // fixed rigid bodies of 'box' type:
  auto floorBody = std::make_shared<ChBodyEasyBox>(22.0*SPHERE_RAD, 2.0*SPHERE_RAD, 22.0*SPHERE_RAD,
    1000, true, vis);
  floorBody->SetPos(ChVector<>(0, -SPHERE_RAD, 0));
  floorBody->SetBodyFixed(true);

  phySystem->Add(floorBody);

  double wall_h = ChMax(2.0* (SPHERE_NUM * 2.0 * SPHERE_RAD / 100.0), 13.0*SPHERE_RAD);

  // add a lid
  auto lid = std::make_shared<ChBodyEasyBox>(22.0*SPHERE_RAD, 2.0*SPHERE_RAD, 22.0*SPHERE_RAD,
    1000, true, vis);
  lid->SetPos(ChVector<>(0, wall_h, 0));
  lid->SetBodyFixed(true);

  phySystem->Add(lid);

  auto wallBody1 = std::make_shared<ChBodyEasyBox>(2.0*SPHERE_RAD, wall_h+2*SPHERE_RAD, 22.0*SPHERE_RAD,
    1000, true, vis);
  wallBody1->SetPos(ChVector<>(-11.0*SPHERE_RAD, wall_h/2.0, 0));
  wallBody1->SetBodyFixed(true);

  phySystem->Add(wallBody1);

  auto wallBody2 = std::make_shared<ChBodyEasyBox>(2.0*SPHERE_RAD, wall_h + 2 * SPHERE_RAD, 22.0*SPHERE_RAD,
    1000, true, vis);
  wallBody2->SetPos(ChVector<>(11.0*SPHERE_RAD, wall_h/2.0, 0));
  wallBody2->SetBodyFixed(true);

  phySystem->Add(wallBody2);

  auto wallBody3 = std::make_shared<ChBodyEasyBox>(22.0*SPHERE_RAD, wall_h + 2 * SPHERE_RAD, 2.0*SPHERE_RAD,
    1000, true, false);
  wallBody3->SetPos(ChVector<>(0, wall_h/2.0, -11.0*SPHERE_RAD));
  wallBody3->SetBodyFixed(true);

  phySystem->Add(wallBody3);

  auto wallBody4 = std::make_shared<ChBodyEasyBox>(22.0*SPHERE_RAD, wall_h + 2 * SPHERE_RAD, 2.0*SPHERE_RAD,
    1000, true, false);
  wallBody4->SetPos(ChVector<>(0, wall_h/2.0, 11.0*SPHERE_RAD));
  wallBody4->SetBodyFixed(true);

  phySystem->Add(wallBody4);

  
  // if initializing the sphere positions the first time thru, set the values
  if (sphere_pos.size() == 0){
    // for (auto pos : bodyPos)
    //   sphere_pos.push_back(pos);
    sphere_pos = bodyPos;
    GetLog() << " \n particle mass : " << outBodyPtr.front()->GetMass() << "\n";
  }

  return outBodyPtr;
}

// create bodies for a given, ChSystem, given solver type/settings.
// run a simulation, report on: final body positions, solver error, clock time
void RunTest(const SolverSettings& solver, ChTimer<>& timer, 
  std::vector<ChVector<> >& body_pos_init, std::vector<ChVector<> >& body_pos_final,
  std::vector<double>& violation_per_step, std::vector<double>& dlambda_per_step) 
{

  // create the system
  auto sys = std::make_shared<ChSystem>();

  // add the bodies, return the locations they are created for the next test
  auto body_Ptrs = create_bodies(sys, body_pos_init);

  // set the system's solver settings
  solver.Set_Solver(sys);
    
  // reset the violation history
  violation_per_step.clear();
  dlambda_per_step.clear();
  auto lcpSolver = dynamic_cast<ChLcpIterativeSolver*>(sys->GetLcpSolverSpeed());


  /*
  irrlicht::ChIrrApp application(sys.get(), L"Contacts with cohesion", irr::core::dimension2d<irr::u32>(800, 600), false);

  // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
  irrlicht::ChIrrWizard::add_typical_Logo(application.GetDevice());
  irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
  irrlicht::ChIrrWizard::add_typical_Lights(application.GetDevice());
  irrlicht::ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(0, 0.5, -2));

  application.AssetBindAll();
  application.AssetUpdateAll();
  */




  // start the timer
  timer.reset();
  timer.start();
  
  // run the simulation
  while (sys->GetChTime() < TEST_END_TIME)
  // application.SetTimestep(STEP_SIZE);
  // while (application.GetDevice()->run() )
  {
    sys->DoStepDynamics(STEP_SIZE);
    
    /*
    application.GetVideoDriver()->beginScene(true, true, irr::video::SColor(255, 140, 161, 192));
    application.DrawAll();
    application.DoStep();
    application.GetVideoDriver()->endScene();
    */

    if (! lcpSolver->GetDeltalambdaHistory().empty()) {
      // report the delta lambda per step
      dlambda_per_step.push_back(lcpSolver->GetDeltalambdaHistory().back());
      // report the solver violation each step
      violation_per_step.push_back(lcpSolver->GetDeltalambdaHistory().back());
    }
  }

  // stop the timer
  timer.stop();
  
  // report the final body position
  body_pos_final.clear();
  for (auto body_p : body_Ptrs)
    body_pos_final.push_back(body_p->GetPos());
}

int main(int argc, char* argv[]) {
  
  // set up the solvers to be tested
  SolverSettings sor(ChSystem::LCP_ITERATIVE_SOR_STDTHREAD);
  SolverSettings bb(ChSystem::LCP_ITERATIVE_BARZILAIBORWEIN, 50, 10, 1.0, 0.2, 1.0);
  SolverSettings apgd(ChSystem::LCP_ITERATIVE_APGD, 50, 10, 1.0, 0.2, 1.0);

  std::vector<SolverSettings> solvers;
  solvers.push_back(sor);
  solvers.push_back(bb);
  solvers.push_back(apgd);

  // initial body position to be used in each test
  std::vector<ChVector<> > body_pos_init;
  // use this to compute difference in final position of bodies
  std::vector<ChVector<> > body_pos_final;

  // compare violations
  std::vector<double> violation;
  std::vector<double> d_lambda;

  ChTimer<> timer;

  // run the solver tests
  for (auto solver : solvers)
  {
    RunTest(solver, timer, body_pos_init, body_pos_final, violation, d_lambda);
    
    if (1) {
      GetLog() << "\n violation: " << violation.back();
      GetLog() << "\n delta lambda: " << d_lambda.back();
    }

    // run some diagnostics
    if (1){
      GetLog() << "\n final body pos: ";
      for (auto pos : body_pos_final)
        GetLog() << pos;
    }
  }

  GetLog() << " \n all done \n";
  int arg;
  std::cin >> arg;
}
