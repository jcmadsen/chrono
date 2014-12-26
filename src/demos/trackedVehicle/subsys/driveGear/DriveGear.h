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

#ifndef DRIVEGEAR_H
#define DRIVEGEAR_H

#include "subsys/ChApiSubsys.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"
#include "ModelDefs.h"

namespace chrono {


/// Drive gear class, a single rigid body. Attached to the chassis via revolute joint.
/// Torque applied by the driveline.
class CH_SUBSYS_API DriveGear : public ChShared
{
public:

  DriveGear(const std::string& name,
    VisualizationType vis = VisualizationType::PRIMITIVES,
    CollisionType collide = CollisionType::PRIMITIVES);

  ~DriveGear() {}

  void Initialize(ChSharedPtr<ChBodyAuxRef> chassis, const ChVector<>& pos, const ChQuaternion<>& rot);

  // accessors
  ChSharedPtr<ChBody> GetBody() { return m_gear; }
  
  double GetRadius() { return m_radius; }

private:
  // private functions
  const std::string& getMeshName() const { return m_meshName; }
  const std::string& getMeshFile() const { return m_meshFile; }

  void AddVisualization();
  void AddCollisionGeometry();
  
  // private variables
  ChSharedPtr<ChBody> m_gear;
  VisualizationType m_vis;    // visual asset geometry type
  CollisionType m_collide;    // collision geometry type

  // static variables
  static const ChVector<> m_inertia;
  static const double m_mass;
  static const double m_radius;
  static const double m_width;

  static const std::string m_meshName;
  static const std::string m_meshFile;
  
};


} // end namespace chrono


#endif
