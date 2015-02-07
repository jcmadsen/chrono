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
#include "physics/ChShaft.h"
#include "physics/ChShaftsBody.h"
#include "ModelDefs.h"

// collision callback function
#include "subsys/collision/TrackCollisionCallback.h"

namespace chrono {


/// Drive gear class, a single rigid body. Attached to the chassis via revolute joint.
/// Torque applied by the driveline.
class CH_SUBSYS_API DriveGear : public ChShared
{
public:

  DriveGear(const std::string& name,
    VisualizationType vis = VisualizationType::PRIMITIVES,
    CollisionType collide = CollisionType::PRIMITIVES);

  /// override static values for mass, inertia
  DriveGear(const std::string& name,
    double gear_mass,
    const ChVector<>& gear_Ixx,
    VisualizationType vis = VisualizationType::PRIMITIVES,
    CollisionType collide = CollisionType::PRIMITIVES);

  ~DriveGear() {}

  /// init the gear with the initial pos. and rot., w.r.t. the chassis c-sys
  void Initialize(ChSharedPtr<ChBody> chassis,
    const ChFrame<>& chassis_REF,
    const ChCoordsys<>& local_Csys);

  // accessors
  ChSharedPtr<ChBody> GetBody() const { return m_gear; }

  ChSharedPtr<ChShaft> GetAxle() const { return m_axle; }

  double GetRadius() { return m_radius; }

private:
  // private functions
  const std::string& getMeshName() const { return m_meshName; }
  const std::string& getMeshFile() const { return m_meshFile; }

  void AddVisualization();
  void AddCollisionGeometry(double mu = 0.0,
                            double mu_sliding = 0.0,
                            double mu_roll = 0,
                            double mu_spin = 0);
  
  // private variables
  ChSharedPtr<ChBody> m_gear;
  ChSharedPtr<ChShaft> m_axle;                  ///< handle to axle shaft
  ChSharedPtr<ChShaftsBody>  m_axle_to_gear;    ///< handle to gear-shaft connector
  ChSharedPtr<ChLinkLockRevolute>  m_revolute;  ///< handle to revolute joint

  VisualizationType m_vis;    // visual asset geometry type
  CollisionType m_collide;    // collision geometry type

  const std::string m_meshName;
  const std::string m_meshFile;

  // data container for callback collision function
  const ChSharedPtr<GearPinGeometry> m_geom;

  // static variables
  static const ChVector<> m_inertia;
  static const double m_mass;
  static const double m_radius;
  static const double m_width;
  static const double m_widthGap; // inner distance between cydliners
  static const double m_shaft_inertia;
  
};



} // end namespace chrono


#endif
