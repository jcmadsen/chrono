// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Justin Madsen
// =============================================================================
//
// model a single track chain system, as part of a tracked vehicle. Uses JSON input files
//
// =============================================================================

#include <cstdio>

#include "subsys/trackSystem/TrackSystem.h"

namespace chrono {

// -----------------------------------------------------------------------------
// Static variables

// idler, right side
const ChVector<> TrackSystem::m_idlerPos(-2.1904, -0.1443, 0.2447); // relative to local csys
const ChQuaternion<> TrackSystem::m_idlerRot(QUNIT);
  
// drive gear, right side
const ChVector<> TrackSystem::m_gearPos(1.7741, -0.0099, 0.2447);  // relative to local csys
const ChQuaternion<> TrackSystem::m_gearRot(QUNIT);

// Support rollers
const int TrackSystem::m_numRollers = 0;
const double TrackSystem::m_roller_mass = 100.0;
const ChVector<> TrackSystem::m_roller_inertia(19.82, 19.82, 26.06);  // rotates about z-axis initially
const double TrackSystem::m_roller_radius = 0.2;
const double TrackSystem::m_roller_width = 0.2;
  
// suspension
const int TrackSystem::m_numSuspensions = 5;

TrackSystem::TrackSystem(const std::string& name, int track_idx)
  : m_track_idx(track_idx), m_name(name)
{
  // FILE* fp = fopen(filename.c_str(), "r");
  // char readBuffer[65536];
  // fclose(fp);

  Create(track_idx);

}

// Create: 1) load/set the subsystem data, resize vectors 2) BuildSubsystems()
// TODO: replace hard-coded junk with JSON input files for each subsystem
void TrackSystem::Create(int track_idx)
{

  /*

  // read idler info
  assert(d.HasMember("Idler"));
  m_idlerMass = d["Idler"]["Mass"].GetDouble();
  m_idlerPos = loadVector(d["Idler"]["Location"]);
  m_idlerInertia = loadVector(d["Idler"]["Inertia"]);
  m_idlerRadius = d["Spindle"]["Radius"].GetDouble();
  m_idlerWidth = d["Spindle"]["Width"].GetDouble();
  m_idler_K = d["Idler"]["SpringK"].GetDouble();
  m_idler_C = d["Idler"]["SpringC"].GetDouble();

  // Read Drive Gear data
  assert(d.HasMember("Drive Gear"));
  assert(d["Drive Gear"].IsObject());

  m_gearMass = d["Drive Gear"]["Mass"].GetDouble();
  m_gearPos = loadVector(d["Drive Gear"]["Location"]);
  m_gearInertia = loadVector(d["Drive Gear"]["Inertia"]);
  m_gearRadius = d["Drive Gear"]["Radius"].GetDouble();
  m_gearWidth = d["Drive Gear"]["Width"].GetDouble();

  // Read Support Roller info
  assert(d.HasMember("Support Roller"));
  assert(d["Support Roller"].IsObject());

  m_rollerMass = d["Support Roller"]["Mass"].GetDouble();
  m_rollerInertia = loadVector(d["Support Roller"]["Inertia"]);
  m_rollerRadius = d["Support Roller"]["Radius"].GetDouble();
  m_rollerWidth = d["Support Roller"]["Width"].GetDouble();
  
  assert(d["Support Roller"]["Location"].IsArray());
  m_NumRollers = d["Support Roller"]["Location"].Size();

  */

  // no support rollers, for this model
  m_supportRollers.resize(m_numRollers);
  m_supportRollers_rev.resize(m_numRollers);

  /*
  
  m_rollerLocs.resize(m_NumRollers);
  for(int i = 0; i < m_NumRollers; i++ )
  {
	m_rollerLocs[i] = loadVector(d["Support Roller"]["Location"][i]);
  }

  // Read Suspension data
  assert(d.HasMember("Suspension"));
  assert(d["Suspension"].IsObject());
  assert(d["Suspension"]["Location"].IsArray() );
  
  m_suspensionFilename = d["Suspension"]["Input File"].GetString();
  m_NumSuspensions = d["Suspension"]["Location"].Size();

 

  m_suspensions.resize(m_numSuspensions);
  m_suspensionLocs.resize(m_numSuspensions);
  for(int j = 0; j < m_numSuspensions; j++)
  {
    // m_suspensionLocs[j] = loadVector(d["Suspension"]["Locaiton"][j]);
  }
  */

  m_suspensions.resize(m_numSuspensions);
  m_suspensionLocs.resize(m_numSuspensions);
  // hard-code positions relative to trackSystem csys. Start w/ one nearest sprocket
  m_suspensionLocs[0] = ChVector<>(1.3336, 0, 0);
  m_suspensionLocs[1] = ChVector<>(0.6668, 0, 0);
  // trackSystem c-sys aligned with middle suspension subsystem arm/chassis revolute constraint position
  m_suspensionLocs[2] = ChVector<>(0,0,0); 
  m_suspensionLocs[3] = ChVector<>(-0.6682, 0, 0);
  m_suspensionLocs[4] = ChVector<>(-1.3368, 0, 0);

  /*

  // Read Track Chain data
  assert(d.HasMember("Track Chain"));
  assert(d["Track Chain"].IsObject()); 
  m_trackChainFilename = d["Track Chain"]["Input File"].GetString()
  
  */

  // create the various subsystems, from the hardcoded static variables in each subsystem class
  BuildSubsystems();

 
}

void TrackSystem::BuildSubsystems()
{
  // build one of each of the following subsystems. VisualizationType and CollisionType defaults are PRIMITIVES
  m_driveGear = ChSharedPtr<DriveGear>(new DriveGear("drive gear "+std::to_string(m_track_idx)) );
  m_idler = ChSharedPtr<IdlerSimple>(new IdlerSimple("idler "+std::to_string(m_track_idx)) );
  m_chain = ChSharedPtr<TrackChain>(new TrackChain("chain "+std::to_string(m_track_idx)) );
  
  // build suspension/road wheel subsystems
  for(int i = 0; i < m_numSuspensions; i++)
  {
    m_suspensions[i] = ChSharedPtr<TorsionArmSuspension>(new TorsionArmSuspension("suspension "+std::to_string(i) +", chain "+std::to_string(m_track_idx)) );
  }
  
  // build support rollers manually (if any)
  for(int j = 0; j < m_numRollers; j++)
  {
    m_supportRollers[j] = ChSharedPtr<ChBody>(new ChBody);
    m_supportRollers[j]->SetNameString("support roller"+std::to_string(j)+", chain "+std::to_string(m_track_idx) );
    m_supportRollers[j]->SetMass(m_roller_mass);
    m_supportRollers[j]->SetInertiaXX(m_roller_inertia);
  }

}

void TrackSystem::Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
			 const ChVector<>&  local_pos)
{
  m_local_pos = local_pos;
  m_gearPosRel = m_gearPos;
  m_idlerPosRel = m_idlerPos;
  // if we're on the left side of the vehicle, switch lateral z-axis on all relative positions
  if(m_local_pos.z < 0)
  {
    m_gearPosRel.z *= -1;
    m_idlerPosRel.z *= -1;
  }
  
  // Create list of the center location of the rolling elements and their clearance.
  // Clearance is a sphere shaped envelope at each center location, where it can
  //  be guaranteed that the track chain geometry will not penetrate the sphere.
  std::vector<ChVector<>> rolling_elem_locs; // w.r.t. chassis ref. frame
  std::vector<double> clearance;  // 1 per rolling elem

  // initialize 1 of each of the following subsystems.
  // will use the chassis ref frame to do the transforms, since the TrackSystem
  // local ref. frame has same rot (just difference in position)
  m_driveGear->Initialize(chassis, ChCoordsys<>(m_local_pos + Get_gearPosRel(), QUNIT) );

  // drive sprocket is First added to the lists passed into TrackChain Init()
  rolling_elem_locs.push_back(m_local_pos + Get_gearPosRel() );
  clearance.push_back(m_driveGear->GetRadius() );

  // initialize the torsion arm suspension subsystems
  for(int s_idx = 0; s_idx < m_suspensionLocs.size(); s_idx++)
  {
    m_suspensions[s_idx]->Initialize(chassis,
      ChCoordsys<>(m_local_pos + m_suspensionLocs[s_idx], QUNIT) );

    // add to the lists passed into the track chain, find location of each wheel center w.r.t. chassis coords.
    rolling_elem_locs.push_back(m_local_pos + m_suspensionLocs[s_idx] + m_suspensions[s_idx]->GetWheelPosRel() );
    clearance.push_back(m_suspensions[s_idx]->GetWheelRadius() );
  }

  // initialize the support rollers.
  // NOTE: None for the M113
  for(int r_idx = 0; r_idx < m_rollerLocs.size(); r_idx++)
  {
    initialize_roller(m_supportRollers[r_idx], chassis,
      m_local_pos + m_rollerLocs[r_idx], QUNIT, r_idx);

    // add to the points passed into the track chain
    rolling_elem_locs.push_back( m_local_pos + m_rollerLocs[r_idx] );
    clearance.push_back(m_roller_radius);
  }
  
  // last control point: the idler body
  m_idler->Initialize(chassis,
    ChCoordsys<>(m_local_pos + Get_idlerPosRel(), QUNIT) );

  // add to the lists passed into the track chain Init()
  rolling_elem_locs.push_back(m_local_pos + Get_idlerPosRel() );
  clearance.push_back(m_idler->GetRadius() );

  // After all rolling elements have been initialized, now able to setup the TrackChain.
  // Assumed that start_pos is between idler and gear control points, e.g., on the top 
  //   of the track chain.
  ChVector<> start_pos = (rolling_elem_locs[0] + rolling_elem_locs.back())/2.0;
  start_pos.y += (clearance[0] + clearance.back() )/2.0;

  // Assumption: start_pos should lie close to where the actual track chain would 
  //             pass between the idler and driveGears.
  // MUST be on the top part of the chain so the chain wrap rotation direction can be assumed.
  // rolling_elem_locs, start_pos w.r.t. chassis c-sys
  m_chain->Initialize(chassis, rolling_elem_locs, clearance, start_pos );

}


// initialize a roller at the specified location and orientation, w.r.t. TrackSystem c-sys
void TrackSystem::initialize_roller(ChSharedPtr<ChBody> body, ChSharedPtr<ChBodyAuxRef>  chassis,
                                    const ChVector<>& loc, const ChQuaternion<>& rot, int idx)
{
  // express loc and rot in the global c-sys
  ChFrame<> frame_to_abs(loc, rot);
  

  body->SetPos(loc);
  body->SetRot(rot);

  // transform point to absolute frame and initialize

  // add the revolute joint at the location and w/ orientation specified


  // Add a visual asset

  // Add a collision shape
  body->SetCollide(true);
  body->GetCollisionModel()->ClearModel();
  // set collision family, gear is a rolling element like the wheels
  body->GetCollisionModel()->SetFamily((int)CollisionFam::WHEELS);
  // don't collide with other rolling elements or ground
  body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::GROUND);

  body->GetCollisionModel()->BuildModel();
}

} // end namespace chrono
