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
// A simple suspension/road wheel system, that uses a torsional spring and rigid arm
//
// =============================================================================

#ifndef TORSION_ARM_SUSPENSION_H
#define TORSION_ARM_SUSPENSION_H

#include "subsys/ChApiSubsys.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"
#include "physics/ChShaftsBody.h"
#include "physics/ChShaftsTorsionSpring.h"

#include "ModelDefs.h"

namespace chrono {

/// Consists of two bodies, the torsion arm link and the road-wheel.
/// Arm constrained to chassis via revolute joint, as is the arm to wheel.
/// Rotational spring damper between arm and chassis
class CH_SUBSYS_API TorsionArmSuspension : public ChShared
{
public:

  TorsionArmSuspension(const std::string& name,
    VisualizationType vis = VisualizationType::PRIMITIVES,
    CollisionType collide = CollisionType::PRIMITIVES);

  ~TorsionArmSuspension() {}

  /// init the suspension with the initial pos. and rot., w.r.t. the chassis c-sys
  /// specifies the attachment point of the arm to the hull bodies
  void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                  const ChCoordsys<>& local_Csys);
  
  
  double getSpringCoefficient() const { return m_springK; }
  double getDampingCoefficient() const { return m_springC; }

  ChSharedPtr<ChBody> GetArmBody() { return m_arm; }
  ChSharedPtr<ChBody> GetWheelBody() { return m_wheel; }

  double GetWheelRadius() { return m_wheelRadius; }

private:

  // private functions
  void Create(const std::string& name);
  void AddVisualization();
  void AddCollisionGeometry();
  
  // private variables
  ChSharedPtr<ChBody> m_arm;  ///< arm body
  ChSharedPtr<ChLinkLockRevolute> m_armChassis_rev; ///< arm-chassis revolute joint

  // shock absorber, a torsional spring connected to two 1-D shafts
  ChSharedPtr<ChShaft> m_shaft_chassis; ///< rigidly connected to chassis, torsional spring
  ChSharedPtr<ChShaftsBody> m_shaft_chassis_connection; ///< connects shaft to chassis
  ChSharedPtr<ChShaft> m_shaft_arm; ///< rigidly attached to link arm, torsional spring
  ChSharedPtr<ChShaftsBody> m_shaft_arm_connection; ///< connects shaft to arm;
  ChSharedPtr<ChShaftsTorsionSpring> m_shock; ///< torsional spring

  ChSharedPtr<ChBody> m_wheel;  ///< wheel body
  ChSharedPtr<ChLinkLockRevolute> m_armWheel_rev; ///< arm-wheel revolute joint

  ChFrame<> m_Loc; // location of subsystem, relative to trackSystem ref c-sys

  // visual and collision geometry types
  VisualizationType m_vis;
  CollisionType m_collide;

  // static variables
  static const double m_armMass;
  static const ChVector<> m_armInertia;
  static const double m_armRadius;

  static const double m_wheelMass;
  static const ChVector<> m_wheelInertia;
  static const double m_wheelWidth;
  static const double m_wheelRadius;
  static const ChVector<> m_wheel_Pos;

  static const double m_springK;	// torsional spring constant
  static const double m_springC;	// torsional damping constant
  static const double m_TorquePreload;	// preload torque, on the spring/damper DOF
  
  static const double m_shaft_inertia;
};


} // end namespace chrono


#endif
