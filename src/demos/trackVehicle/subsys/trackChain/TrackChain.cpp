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
// Generates a track chain aropund the track system
//
// =============================================================================

#include <cstdio>

#include "TrackChain.h"

#include "physics/ChBody.h"
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
const std::string TrackChain::m_collisionFile = utils::GetModelDataFile("track_data/M113/shoe_collision.obj");
const std::string TrackChain::m_meshName = "M113 shoe"; 
const std::string TrackChain::m_meshFile = utils::GetModelDataFile("track_data/M113/shoe_view.obj");

const double TrackChain::m_mass = 18.02;
const ChVector<> TrackChain::m_inertia(0.04, 0.22, 0.25);
const double TrackChain::m_shoe_width = 0.4;
const double TrackChain::m_shoe_height = 0.2;
const double TrackChain::m_pin_dist = 0.3;		// linear distance between a shoe's two pin joint center
const double TrackChain::m_pin_radius = 0.05;

TrackChain::TrackChain(const std::string& name, 
                       VisualizationType vis, 
                       CollisionType collide)
                       : m_vis(vis), m_collide(collide), m_numShoes(0)
{
  // clear vector holding list of body handles
  m_shoes.clear();
  // add first track shoe body
  m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody));
  m_shoes[0]->SetNameString("shoe 1, "+name);
  m_shoes[0]->SetMass(m_mass);
  m_shoes[0]->SetInertiaXX(m_inertia);
  m_numShoes++;

  // Attach visualization to the base track shoe
  AddVisualization(0);
}

// figure out the control points, 2 per rolling element to wrap the chain around.
void TrackChain::Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                            const std::vector<ChVector<>>& rolling_element_loc,
                            const std::vector<double>& clearance,
                            const ChVector<>& start_loc)
{
  // get the following in abs coords: 1) start_loc, 2) rolling_elem, 3) control_points
  ChFrame<> start_to_abs(start_loc);
  start_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  // find control points, which lie on the envelope of each rolling element, in abs coords
  std::vector<ChFrame<>> control_to_abs;
  std::vector<ChFrame<>> rolling_to_abs;

  size_t num_elem = rolling_element_loc.size();  // number of control points
  // define the envelope to wrap the chain around using 1) body locs and clearances, 2) control_points.
  // each rolling element has 2 control points, a start and end point for a given envenlope line segment.
  ChVector<> start_point;  // current start_loc for this segment
  ChVector<> end_point; // end point of the current segment
  ChVector<> rad_dir;   // center to segment start/end point on rolling elments
  ChVector<> r_21;      // vector between two pulley centerpoints
  ChVector<> norm_dir;  // norm = r_12 cross r_32
  // iterate over the line segments, first segment is between start_loc and rolling_elem 0
  // last segment  is between rolling_elem[last] and rolling_elem[last-1]
  for(size_t i = 0; i < num_elem; i++)
  { 
    // convert the center of the rolling body to abs coords
    rolling_to_abs.push_back(ChFrame<>(rolling_element_loc[i]));
    rolling_to_abs[i].ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // start and end points of line segment are found.
    // start and end point are known for the first and last segement, respectively.
    if(i == num_elem - 1)
    {
      // only different from intermediate segments in that the norm_dir is found differently
      r_21 = rolling_element_loc[i] - rolling_element_loc[i-1];
      // first rolling element being 3 in  norm = r_12 cross r_32
      norm_dir = Vcross( rolling_element_loc[i-1] - rolling_element_loc[i], rolling_element_loc[0] - rolling_element_loc[i]);
      norm_dir.Normalize();
      // this is the case for alpha = 0, e.g. the two rolling elements have the same radius
      rad_dir = Vcross(norm_dir, r_21);
      rad_dir.Normalize();
      // check to see if the two rolling elements have the same radius
      // when not the same size, find the angle of pulley wrap, alpha != 0
      if( abs(clearance[i] - clearance[i-1]) > 1.0e-3 )
      {
        // pulley wrap angle, a = (r2-r1)/center_len
        double alpha = asin( (clearance[i] - clearance[i-1]) / r_21.Length() );
        ChQuaternion<> alpha_rot(Q_from_AngAxis(alpha, norm_dir) );
        ChFrame<> rot_frame(ChVector<>(),alpha_rot);
        rad_dir =  rad_dir >> rot_frame;
      }

      // with a radial direction vector, start/end points are easy
      start_point = rolling_element_loc[i-1] + rad_dir * clearance[i-1];
      end_point = rolling_element_loc[i] + rad_dir * clearance[i];
    } else if(i == 0)
    {
      // first point, know start_point off the bat. r_21 found differently
      start_point = start_loc;
      // this is where the assumption that start_loc is on the top of the chain matters
      ChVector<> top_center = rolling_element_loc[i];
      top_center.y =+ clearance[i];
      r_21 = top_center - start_loc;

      // last rolling element being 1 in norm = r_12 cross r_32
      norm_dir = Vcross( rolling_element_loc[num_elem-1] - rolling_element_loc[i], rolling_element_loc[i+1] - rolling_element_loc[i]);
      norm_dir.Normalize();

      // this is the case for alpha = 0, e.g. the two rolling elements have the same radius
      rad_dir = Vcross(norm_dir, r_21);
      rad_dir.Normalize();
      // when not the same size, find the angle of pulley wrap, alpha != 0
      if( abs(clearance[i] - clearance[i-1]) > 1.0e-3 )
      {
        // pulley wrap angle, a = (r2-r1)/center_len
        double alpha = asin( (clearance[i] - clearance[i-1]) / r_21.Length() );
        ChQuaternion<> alpha_rot(Q_from_AngAxis(alpha, norm_dir) );
        ChFrame<> rot_frame(ChVector<>(),alpha_rot);
        rad_dir =  rad_dir >> rot_frame;
      }
      


      end_point = rolling_element_loc[i] + rad_dir * clearance[i];
    } else
    {
      // intermediate points, find start and end from roller elem locations
      // center distance vector
      r_21 = rolling_element_loc[i] - rolling_element_loc[i-1];

      // find the radial direction from the center to each control point
      // same for both rolling element bodies.
      norm_dir = Vcross( rolling_element_loc[i-1] - rolling_element_loc[i], rolling_element_loc[i+1] - rolling_element_loc[i]);
      norm_dir.Normalize();
      // this is the case for alpha = 0, e.g. the two rolling elements have the same radius
      rad_dir = Vcross(norm_dir, r_21);
      rad_dir.Normalize();
      // when not the same size, find the angle of pulley wrap, alpha != 0
      if( abs(clearance[i] - clearance[i-1]) > 1.0e-3 )
      {
        // pulley wrap angle, a = (r2-r1)/center_len
        double alpha = asin( (clearance[i] - clearance[i-1]) / r_21.Length() );
        ChQuaternion<> alpha_rot(Q_from_AngAxis(alpha, norm_dir) );
        ChFrame<> rot_frame(ChVector<>(),alpha_rot);
        rad_dir =  rad_dir >> rot_frame;
      }

      // with a radial direction vector, start/end points are easy
      start_point = rolling_element_loc[i-1] + rad_dir * clearance[i-1];
      end_point = rolling_element_loc[i] + rad_dir * clearance[i];
    }

    if(i == 0)
    {
      // first segment, only use the end_point
      control_to_abs.push_back(ChFrame<>(end_point));
      control_to_abs[0].ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs() );
    } else if(i == num_elem - 1)
    {
      // last segment, only use the start_point
      control_to_abs.push_back(ChFrame<>(start_point));
      (*control_to_abs.end()).ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs() );
    }
    else
    {
      // intermediate segments, use both start and end points
      control_to_abs.push_back(ChFrame<>(start_point));
      (*control_to_abs.end()).ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs() );
      control_to_abs.push_back(ChFrame<>(end_point));
      (*control_to_abs.end()).ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs() );
    }

  }






  // hard part: "wrap" the track chain around the trackSystem, e.g., drive-gear,
  // idler, road-wheels. First and last shoes are allowed to be in any orientation,
  // as long as the final pin joint connects correctly.
  CreateChain(control_to_abs, rolling_to_abs, clearance, start_to_abs.GetPos() );
}

void TrackChain::AddVisualization(size_t track_idx)
{
  assert(track_idx < m_numShoes);
  // Attach visualization asset
  switch (m_vis) {
  case VisualizationType::PRIMITIVES:
  {
    // primitive box, also used for collision.
    // shoes will be added to the same collision family so self-collision can be toggled
    ChSharedPtr<ChBoxShape> box(new ChBoxShape);
    box->GetBoxGeometry().SetLengths( 0.5 * ChVector<>( m_pin_dist-2*m_pin_radius, m_shoe_height, m_shoe_width) );
    m_shoes[track_idx]->AddAsset(box);

    ChSharedPtr<ChTexture> tex(new ChTexture);
    tex->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    m_shoes[track_idx]->AddAsset(tex);

    break;
  }
  case VisualizationType::MESH:
  {
    // mesh for visualization only.
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(getMeshFile(), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(getMeshName());
    m_shoes[track_idx]->AddAsset(trimesh_shape);

    ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.3f, 0.3f, 0.3f));
    m_shoes[track_idx]->AddAsset(mcolor);

    break;
  }
  }
}


void TrackChain::AddCollisionGeometry(size_t track_idx)
{
  assert(track_idx < m_numShoes);
   // add collision geometrey to the chassis, if enabled
  m_shoes[track_idx]->SetCollide(true);
  m_shoes[track_idx]->GetCollisionModel()->ClearModel();

  switch (m_collide) {
  case CollisionType::PRIMITIVES:
  {
    // use a simple box
    m_shoes[track_idx]->GetCollisionModel()->AddBox(m_pin_dist-2*m_pin_radius, m_shoe_height, m_shoe_width);

    break;
  }
  case CollisionType::MESH:
  {
    // use a triangle mesh
   
		geometry::ChTriangleMeshSoup temp_trianglemesh; 
		
    // TODO: fill the triangleMesh here with some track shoe geometry

		m_shoes[track_idx]->GetCollisionModel()->SetSafeMargin(0.004);	// inward safe margin
		m_shoes[track_idx]->GetCollisionModel()->SetEnvelope(0.010);		// distance of the outward "collision envelope"
		m_shoes[track_idx]->GetCollisionModel()->ClearModel();

    // is there an offset??
    double shoelength = 0.2;
    ChVector<> mesh_displacement(shoelength*0.5,0,0);  // since mesh origin is not in body center of mass
    m_shoes[track_idx]->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false, mesh_displacement);

    break;
  }
  case CollisionType::CONVEXHULL:
  {
    // use convex hulls, loaded from file
    ChStreamInAsciiFile chull_file(GetChronoDataFile("track_data/M113/shoe_collision.chulls").c_str());
    // transform the collision geometry as needed
    double mangle = 45.0; // guess
    ChQuaternion<>rot;
    rot.Q_from_AngAxis(mangle*(CH_C_PI/180.),VECT_X);
    ChMatrix33<> rot_offset(rot);
    ChVector<> disp_offset(0,0,0);  // no displacement offset
    m_shoes[track_idx]->GetCollisionModel()->AddConvexHullsFromFile(chull_file, disp_offset, rot_offset);
    break;
  }
  } // end switch

  // set collision family
  m_shoes[track_idx]->GetCollisionModel()->SetFamily( (int)CollisionFam::SHOES);
  // don't collide with other shoes, but with everything else
  m_shoes[track_idx]->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)CollisionFam::SHOES );

  m_shoes[track_idx]->GetCollisionModel()->BuildModel();

}

// two control points per rolling body.

void TrackChain::CreateChain(const std::vector<ChFrame<>>& control_points_abs,
                             const std::vector<ChFrame<>>& rolling_element_abs,
                             const std::vector<double>& clearance,
                             const ChVector<>& start_pos_abs)
{
  // add collision geometry to the first track shoe
  AddCollisionGeometry(0);
  m_numShoes = 1;
}


ChSharedPtr<ChBody> TrackChain::GetShoeBody(size_t track_idx)
{
  assert( track_idx < m_numShoes);
  return (track_idx > m_numShoes-1) ? m_shoes[track_idx] : m_shoes[0] ;
}

} // end namespace chrono
