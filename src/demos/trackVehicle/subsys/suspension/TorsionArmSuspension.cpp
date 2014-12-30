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
// model a single track chain system, as part of a tracked vehicle.
// Static variable values are based on a M113 model in the report by Shabana
//
// =============================================================================

#include <cstdio>

#include "subsys/suspension/TorsionArmSuspension.h"

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"


namespace chrono {

// static variables
const double TorsionArmSuspension::m_armMass = 75.26; // [kg]
const ChVector<> TorsionArmSuspension::m_armInertia(0.77, 0.37, 0.77);  // [kg-m2]
const double TorsionArmSuspension::m_armRadius = 0.2; // [m]

const double TorsionArmSuspension::m_wheelMass = 561.1; // [kg]
const ChVector<> TorsionArmSuspension::m_wheelInertia(19.82, 19.82, 26.06); // [kg-m2]
const double TorsionArmSuspension::m_wheelWidth = 0.4;  // [m]
const double TorsionArmSuspension::m_wheelRadius = 0.7; // [m]
const ChVector<> TorsionArmSuspension::m_wheel_Pos(-0.116, -0.1136, 0); // loc of wheel CM in the local c-sys

const double TorsionArmSuspension::m_springK = 10000;	// torsional spring constant [N-m/rad]
const double TorsionArmSuspension::m_springC = 100;	// torsional damping constant [N-m-s/rad]
const double TorsionArmSuspension::m_TorquePreload = 10.0;  // torque preload [N-m]
const double TorsionArmSuspension::m_shaft_inertia = 0.5;  // [kg-m2]

TorsionArmSuspension::TorsionArmSuspension(const std::string& name,
                                           VisualizationType vis,
                                           CollisionType collide)
: m_vis(vis), m_collide(collide)
{
  // FILE* fp = fopen(filename.c_str(), "r");
  // char readBuffer[65536];
  // fclose(fp);

  Create(name);
}

// 1) load data from file, 2) create bodies using data
void TorsionArmSuspension::Create(const std::string& name)
{
/*
  // load data for the arm
  m_armMass = d["Arm"]["Mass"].GetDouble();
  m_armInertia = loadVector(d["Arm"]["Inertia"]);
  m_armRadius = d["Arm"]["Radius"].GetDouble();
  
  // load data for the wheel
  m_wheelMass = d["Wheel"]["Mass"].GetDouble();
  m_wheelInertia = loadVector(d["Wheel"]["Inertia"]);
  m_wheelRadius = d["Wheel"]["Radius"].GetDouble();
  m_wheelWidth = d["Wheel"]["Width"].GetDouble();
  m_wheelRelLoc = loadVector(d["Wheel"]["Location"]);
  
  // load data for the torsion bar
  m_springK = d["Torsion Bar"]["Stiffness"].GetDouble();
  m_springC = d["Torsion Bar"]["Damping"].GetDouble();

  */

  // create the suspension arm body
  m_arm = ChSharedPtr<ChBody>(new ChBody);
  m_arm->SetNameString(name + "_arm");
  m_arm->SetMass(m_armMass);
  m_arm->SetInertiaXX(m_armInertia);  // link distance along y-axis
  // create the roadwheel body
  m_wheel = ChSharedPtr<ChBody>(new ChBody);
  m_wheel->SetNameString(name + "_roadWheel");
  m_wheel->SetMass(m_wheelMass);
  m_wheel->SetInertiaXX(m_wheelInertia);  // wheel width along z-axis


  // create the constraints
  m_armChassis_rev = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_armChassis_rev->SetName("_arm-chassis_revolute");
  m_armWheel_rev = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_armWheel_rev->SetName("_arm-wheel-revolute");

  // create the torsional spring damper assembly
  // [Chassis] <m_shaft_chassis_connection>--- m_shaft_chassis ==|| m_shock ||== m_shaft_arm --- <m_shaft_arm_connection> [Arm]

  // two shafts
  m_shaft_chassis = ChSharedPtr<ChShaft>(new ChShaft);
  m_shaft_chassis->SetName("_chassis-shaft");
  m_shaft_chassis->SetInertia(m_shaft_inertia);
  m_shaft_arm = ChSharedPtr<ChShaft>(new ChShaft);
  m_shaft_arm->SetName("_arm-shaft");
  m_shaft_arm->SetInertia(m_shaft_inertia);

  // two shaftbody connectors
  m_shaft_chassis_connection = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);
  m_shaft_chassis_connection->SetName("_chassis-shaftConnect");
  m_shaft_arm_connection = ChSharedPtr<ChShaftsBody>(new ChShaftsBody); 
  m_shaft_arm_connection->SetName("_arm-shaftConnect");

  // and the 1-dof spring between the two shafts
  m_shock = ChSharedPtr<ChShaftsTorsionSpring>(new ChShaftsTorsionSpring); ///< torsional spring
  m_shock->SetName("_torsionalSpring");
  m_shock->SetTorsionalStiffness(m_springK);
  m_shock->SetTorsionalDamping(m_springC);

  // add visualization assets to the roadwheel and arm
  AddVisualization();
}

void TorsionArmSuspension::Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                                      const ChCoordsys<>& local_Csys)
{
  // add collision geometry
  AddCollisionGeometry();

  // Express the revolute joint location in the absolute coordinate system.
  ChFrame<> rev_loc_to_abs(local_Csys);
  rev_loc_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  // wheel is just offset from local csys, in local coordinates
  ChFrame<> wheel_pos_loc(local_Csys);
  wheel_pos_loc.SetPos(local_Csys.pos + m_wheel_Pos);
  ChFrame<> wheel_to_abs(wheel_pos_loc);
  wheel_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  // arm is between these two points, same rotation as the wheel
  m_arm->SetPos( (rev_loc_to_abs.GetPos() + wheel_to_abs.GetPos() )/2.0 );
  // y-axis should point along length of arm, according to inertia tensor
  ChVector<> v = (rev_loc_to_abs.GetPos()-wheel_to_abs.GetPos()).GetNormalized();
  // use the z-axis from the wheel frame
  ChVector<> w = (wheel_to_abs.GetRot().GetYaxis()).GetNormalized();
  ChVector<> u = Vcross(v, w);
  u.Normalize();
  ChMatrix33<> rot;
  rot.Set_A_axis(u,v,w);
  // should give the correct orientation to the arm
  m_arm->SetRot(rot);
  // pos, rot of arm set, add it to the system
  chassis->GetSystem()->Add(m_arm);

  // set the wheel in the correct position.
  m_wheel->SetPos(wheel_to_abs.GetPos());
  // inertia and visual assets have wheel width along z-axis locally. No need to rotate the wheel.
  m_wheel->SetRot(wheel_to_abs.GetRot());
  chassis->GetSystem()->Add(m_wheel);

  // init and add the revolute joints
  // arm-chassis
  m_armChassis_rev->Initialize(m_arm, chassis, ChCoordsys<>(rev_loc_to_abs.GetPos(), rev_loc_to_abs.GetRot()) );
  chassis->GetSystem()->AddLink(m_armChassis_rev);
  // wheel-arm
  m_armWheel_rev->Initialize(m_wheel, m_arm, ChCoordsys<>(wheel_to_abs.GetPos(), wheel_to_abs.GetRot()) );
  chassis->GetSystem()->AddLink(m_armWheel_rev);

  // initialize and add the torsional spring, shaft and shaftbody elements
  chassis->GetSystem()->Add(m_shaft_chassis);
  chassis->GetSystem()->Add(m_shaft_arm);

  m_shaft_chassis_connection->Initialize(m_shaft_chassis, chassis, VECT_Z);
  chassis->GetSystem()->Add(m_shaft_chassis_connection);
  m_shaft_arm_connection->Initialize(m_shaft_arm, m_arm, VECT_Z);
  chassis->GetSystem()->Add(m_shaft_arm_connection);

  m_shock->Initialize(m_shaft_arm, m_shaft_chassis);
  chassis->GetSystem()->Add(m_shock);
}

/// add a cylinder to model the torsion bar arm and the wheel
void TorsionArmSuspension::AddVisualization()
{
  // add visualization assets
  switch (m_vis) {
  case VisualizationType::PRIMITIVES:
  {
    // define the wheel cylinder shape with the two end points of the cylinder
    ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
    ChVector<> p1(0, 0, m_wheelWidth/2.0);
    ChVector<> p2(0, 0, -m_wheelWidth/2.0);
    cyl->GetCylinderGeometry().p1 = p1;
    cyl->GetCylinderGeometry().p2 = p2;
    cyl->GetCylinderGeometry().rad = m_wheelRadius;
    m_wheel->AddAsset(cyl);

    // define the arm cylinder shape, link is along the y-axis
    double armLength = m_wheel_Pos.Length();
    ChVector<> p1_arm(0, armLength/2.0, 0);
    ChVector<> p2_arm(0, -armLength/2.0, 0);
    ChSharedPtr<ChCylinderShape> arm_cyl(new ChCylinderShape);
    arm_cyl->GetCylinderGeometry().p1 = p1_arm;
    arm_cyl->GetCylinderGeometry().p2 = p2_arm;
    arm_cyl->GetCylinderGeometry().rad = m_armRadius;
    m_arm->AddAsset(arm_cyl);
    break;
  }
   case VisualizationType::MESH:
  {
    /*
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(getMeshFile(), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(getMeshName());
    m_wheel->AddAsset(trimesh_shape);

    ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.5f, 0.1f, 0.4f));
    m_wheel->AddAsset(mcolor);
    */

    break;
  }
  } // end switch
}

/// only the road wheels are used for collision
void TorsionArmSuspension::AddCollisionGeometry()
{
  // add collision geometrey, if enabled. Warn if not
  m_wheel->SetCollide(true);
  m_wheel->GetCollisionModel()->ClearModel();

  switch (m_collide) {
  case CollisionType::NONE:
  {
    m_wheel->SetCollide(false);
    GetLog() << " !!! Road Wheel " << m_wheel->GetName() << " collision deactivated !!! \n\n";
  }
  case CollisionType::PRIMITIVES:
  {
    // use a simple cylinder. Default is along the y-axis, here we have z-axis relative to chassis.
    m_wheel->GetCollisionModel()->AddCylinder(m_wheelRadius, m_wheelRadius, m_wheelWidth,
    ChVector<>(),Q_from_AngAxis(CH_C_PI_2,VECT_X));

    break;
  }
  case CollisionType::MESH:
  {
    // use a triangle mesh
    /*

		geometry::ChTriangleMeshSoup temp_trianglemesh; 
		
    // TODO: fill the triangleMesh here with some track shoe geometry

		m_wheel->GetCollisionModel()->SetSafeMargin(0.004);	// inward safe margin
		m_wheel->GetCollisionModel()->SetEnvelope(0.010);		// distance of the outward "collision envelope"
		m_wheel->GetCollisionModel()->ClearModel();

    // is there an offset??
    double shoelength = 0.2;
    ChVector<> mesh_displacement(shoelength*0.5,0,0);  // since mesh origin is not in body center of mass
    m_wheel->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false, mesh_displacement);
    */

    break;
  }
  case CollisionType::CONVEXHULL:
  {
    /*
    // use convex hulls, loaded from file
    ChStreamInAsciiFile chull_file(GetChronoDataFile("drive_gear.chulls").c_str());
    // transform the collision geometry as needed
    double mangle = 45.0; // guess
    ChQuaternion<>rot;
    rot.Q_from_AngAxis(mangle*(CH_C_PI/180.),VECT_X);
    ChMatrix33<> rot_offset(rot);
    ChVector<> disp_offset(0,0,0);  // no displacement offset
    m_wheel->GetCollisionModel()->AddConvexHullsFromFile(chull_file, disp_offset, rot_offset);
    */

    break;
  }
  } // end switch

  m_wheel->GetCollisionModel()->BuildModel();
}

} // end namespace chrono