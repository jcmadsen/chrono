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
// The drive gear propels the tracked vehicle
//
// =============================================================================

#include "DriveGear.h"

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
// collision mesh
#include "geometry/ChCTriangleMeshSoup.h"

#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"

namespace chrono {

// static variables
const std::string DriveGear::m_meshName = "gear_mesh";
const std::string DriveGear::m_meshFile = utils::GetModelDataFile("gearMesh.obj");

const double DriveGear::m_mass = 436.7;
const ChVector<> DriveGear::m_inertia(13.87, 12.22, 12.22);
const double DriveGear::m_radius = 0.3;
const double DriveGear::m_width = 0.25;
const double DriveGear::m_shaft_inertia = 0.4;


DriveGear::DriveGear(const std::string& name, 
                     VisualizationType vis, 
                     CollisionType collide)
  : m_vis(vis), m_collide(collide)
{
  // create the body, set the basic info
  m_gear = ChSharedPtr<ChBody>(new ChBody);
  m_gear->SetNameString(name+"_body");
  m_gear->SetMass(m_mass);
  m_gear->SetInertiaXX(m_inertia);

  // create the revolute joint
  m_revolute = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revolute->SetNameString(name+"_revolute");

  // create the shaft components
  m_axle = ChSharedPtr<ChShaft>(new ChShaft);
  m_axle->SetNameString(name + "_axle");
  m_axle->SetInertia(m_shaft_inertia );
  // create the shaft to gear connection
  m_axle_to_gear = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);
  m_axle_to_gear->SetNameString(name + "_axle_to_gear");

  AddVisualization();
 
}

void DriveGear::Initialize(ChSharedPtr<ChBodyAuxRef> chassis, 
                           const ChCoordsys<>& local_Csys)
{
  // add any collision geometry
  AddCollisionGeometry();

  // get the local frame in the absolute ref. frame
  ChFrame<> gear_to_abs(local_Csys);
  gear_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  // transform the drive gear body, add to system
  m_gear->SetPos(gear_to_abs.GetPos());
  m_gear->SetRot(gear_to_abs.GetRot());
  chassis->GetSystem()->Add(m_gear);

  // initialize the revolute joint, add to system


  // TODO: (check) may need to rotate the rotation axis
  m_revolute->Initialize(chassis, m_gear, ChCoordsys<>(gear_to_abs.GetPos(), gear_to_abs.GetRot()) );
  chassis->GetSystem()->AddLink(m_revolute);

  // initialize the axle shaft and connection to the drive gear body, add both to the system
  chassis->GetSystem()->Add(m_axle);
  m_axle_to_gear->Initialize(m_axle, m_gear, VECT_Z);
  chassis->GetSystem()->Add(m_axle_to_gear);

}

void DriveGear::AddVisualization()
{
   // Attach visualization asset
  switch (m_vis) {
  case VisualizationType::PRIMITIVES:
  {
    ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
    cyl->GetCylinderGeometry().rad = m_radius;
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, m_width / 2, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, -m_width / 2, 0);
    m_gear->AddAsset(cyl);

    ChSharedPtr<ChTexture> tex(new ChTexture);
    tex->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    m_gear->AddAsset(tex);

    break;
  }
  case VisualizationType::MESH:
  {
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(getMeshFile(), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(getMeshName());
    m_gear->AddAsset(trimesh_shape);

    ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.3f, 0.3f, 0.3f));
    m_gear->AddAsset(mcolor);

    break;
  }
  }
}

void DriveGear::AddCollisionGeometry()
{
  // add collision geometrey, if enabled. Warn if disabled
  m_gear->SetCollide(true);
  m_gear->GetCollisionModel()->ClearModel();

  switch (m_collide) {
  case CollisionType::NONE:
  {
      m_gear->SetCollide(false);
      GetLog() << " !!! DriveGear " << m_gear->GetName() << " collision deactivated !!! \n\n";
  }
  case CollisionType::PRIMITIVES:
  {
    // use a simple cylinder
    m_gear->GetCollisionModel()->AddCylinder(m_radius, m_radius, m_width,
      ChVector<>(),Q_from_AngAxis(CH_C_PI_2,VECT_X));

    break;
  }
  case CollisionType::MESH:
  {
    // use a triangle mesh
   
		geometry::ChTriangleMeshSoup temp_trianglemesh; 
		
    // TODO: fill the triangleMesh here with some track shoe geometry

		m_gear->GetCollisionModel()->SetSafeMargin(0.004);	// inward safe margin
		m_gear->GetCollisionModel()->SetEnvelope(0.010);		// distance of the outward "collision envelope"
		m_gear->GetCollisionModel()->ClearModel();

    // is there an offset??
    double shoelength = 0.2;
    ChVector<> mesh_displacement(shoelength*0.5,0,0);  // since mesh origin is not in body center of mass
    m_gear->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false, mesh_displacement);

    break;
  }
  case CollisionType::CONVEXHULL:
  {
    // use convex hulls, loaded from file
    ChStreamInAsciiFile chull_file(GetChronoDataFile("drive_gear.chulls").c_str());
    // transform the collision geometry as needed
    double mangle = 45.0; // guess
    ChQuaternion<>rot;
    rot.Q_from_AngAxis(mangle*(CH_C_PI/180.),VECT_X);
    ChMatrix33<> rot_offset(rot);
    ChVector<> disp_offset(0,0,0);  // no displacement offset
    m_gear->GetCollisionModel()->AddConvexHullsFromFile(chull_file, disp_offset, rot_offset);
    break;
  }
  } // end switch

  // set collision family
  m_gear->GetCollisionModel()->SetFamily( (int)CollisionFam::GEARS );
  // don't collide with other rolling elements or ground
  m_gear->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)CollisionFam::HULL );
  m_gear->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)CollisionFam::GEARS );
  m_gear->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)CollisionFam::WHEELS );
  m_gear->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)CollisionFam::GROUND );

  m_gear->GetCollisionModel()->BuildModel();

}

} // end namespace chrono
