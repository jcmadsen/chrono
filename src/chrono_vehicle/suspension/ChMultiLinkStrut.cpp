// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Justin Madsen, Daniel Melanz, Radu Serban
// =============================================================================

#include "assets/ChCylinderShape.h"
#include "assets/ChColorAsset.h"

#include "chrono_vehicle/suspension/ChMultiLinkStrut.h"


namespace chrono {


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChMultiLinkStrut::m_pointNames[] = {
    "SPINDLE  ",
    "UPRIGHT  ",
    "UA_F     ",
    "UA_B     ",
    "UA_U     ",
    "UA_CM    ",
    "LAT_C    ",
    "LAT_U    ",
    "LAT_CM   ",
    "TL_C     ",
    "TL_U     ",
    "TL_CM    ",
    "SHOCK_C  ",
    "SHOCK_L  ",
    "SPRING_C ",
    "SPRING_L ",
    "TIEROD_C ",
    "TIEROD_U "
};


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChMultiLinkStrut::ChMultiLinkStrut(const std::string& name)
: ChSuspension(name)
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLinkStrut::Initialize(ChSharedPtr<ChBodyAuxRef>  chassis,
                             const ChVector<>&          location,
                             ChSharedPtr<ChBody>        tierod_body_right,
							 ChSharedPtr<ChBody>		tierod_body_left)
{
  // Express the suspension reference frame in the absolute coordinate system.
  ChFrame<> suspension_to_abs(location);
  suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  // Transform all points to absolute frame and initialize left side.
  std::vector<ChVector<> > points_R(NUM_POINTS);
  std::vector<ChVector<> > points_L(NUM_POINTS);

  for (int i = 0; i < NUM_POINTS; i++) {
    ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
    points_L[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
    rel_pos.y = -rel_pos.y;
    points_R[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
  }

  // Initialize left and right sides.
  InitializeSide(LEFT, chassis, tierod_body_left, points_L);
  InitializeSide(RIGHT, chassis, tierod_body_right, points_R);
}

void ChMultiLinkStrut::InitializeSide(ChVehicleSide                   side,
                                 ChSharedPtr<ChBodyAuxRef>       chassis,
                                 ChSharedPtr<ChBody>             tierod_body,
                                 const std::vector<ChVector<> >& points)
{
  std::string suffix = (side == LEFT) ? "_L" : "_R";

  // Chassis orientation (expressed in absolute frame)
  // Recall that the suspension reference frame is aligned with the chassis.
  ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

  // Create and initialize spindle body (same orientation as the chassis)
  m_spindle[side] = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
  m_spindle[side]->SetPos(points[SPINDLE]);
  m_spindle[side]->SetRot(chassisRot);
  m_spindle[side]->SetMass(getSpindleMass());
  m_spindle[side]->SetInertiaXX(getSpindleInertia());
  AddVisualizationSpindle(m_spindle[side], getSpindleRadius(), getSpindleWidth());
  chassis->GetSystem()->AddBody(m_spindle[side]);

  // Create and initialize upright body (same orientation as the chassis)
  m_upright[side] = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_upright[side]->SetNameString(m_name + "_upright" + suffix);
  m_upright[side]->SetPos(points[UPRIGHT]);
  m_upright[side]->SetRot(chassisRot);
  m_upright[side]->SetMass(getUprightMass());
  m_upright[side]->SetInertiaXX(getUprightInertia());
  AddVisualizationUpright(m_upright[side], points[UA_U], points[LAT_U], points[TL_U], points[TIEROD_U], points[UPRIGHT], getUprightRadius());
  chassis->GetSystem()->AddBody(m_upright[side]);

  // Unit vectors for orientation matrices.
  ChVector<> u;
  ChVector<> v;
  ChVector<> w;
  ChMatrix33<> rot;

  // Create and initialize Upper Arm body.
  // Determine the rotation matrix of the upper arm based on the plane of the hard points
  // (z axis normal to the plane of the upper arm)
  w = Vcross(points[UA_B] - points[UA_U], points[UA_F] - points[UA_U]);
  w.Normalize();
  u = points[UA_F] - points[UA_B];
  u.Normalize();
  v = Vcross(w, u);
  rot.Set_A_axis(u, v, w);

  m_upperArm[side] = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_upperArm[side]->SetNameString(m_name + "_upperArm" + suffix);
  m_upperArm[side]->SetPos(points[UA_CM]);
  m_upperArm[side]->SetRot(rot);
  m_upperArm[side]->SetMass(getUpperArmMass());
  m_upperArm[side]->SetInertiaXX(getUpperArmInertia());
  AddVisualizationUpperArm(m_upperArm[side], points[UA_F], points[UA_B], points[UA_U], getUpperArmRadius());
  chassis->GetSystem()->AddBody(m_upperArm[side]);

  // Create and initialize lateral body.
  // Determine the rotation matrix of the lateral based on the plane of the hard points
  // (z-axis along the length of the track rod)
  v = Vcross(points[LAT_U] - points[TL_U], points[LAT_C] - points[TL_U]);
  v.Normalize();
  w = points[LAT_C] - points[LAT_U];
  w.Normalize();
  u = Vcross(v, w);
  rot.Set_A_axis(u, v, w);

  m_lateral[side] = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_lateral[side]->SetNameString(m_name + "_lateral" + suffix);
  m_lateral[side]->SetPos(points[LAT_CM]);
  m_lateral[side]->SetRot(rot);
  m_lateral[side]->SetMass(getLateralMass());
  m_lateral[side]->SetInertiaXX(getLateralInertia());
  AddVisualizationLateral(m_lateral[side], points[LAT_U], points[LAT_C], getLateralRadius());
  chassis->GetSystem()->AddBody(m_lateral[side]);

  // Create and initialize trailing link body.
  // Determine the rotation matrix of the trailing link based on the plane of the hard points
  // (z-axis along the length of the trailing link)
  v = Vcross(points[TL_U] - points[SPRING_L], points[TL_C] - points[SPRING_L]);
  v.Normalize();
  w = points[TL_C] - points[TL_U];
  w.Normalize();
  u = Vcross(v, w);
  rot.Set_A_axis(u, v, w);

  m_trailingLink[side] = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_trailingLink[side]->SetNameString(m_name + "_trailingLink" + suffix);
  m_trailingLink[side]->SetPos(points[TL_CM]);
  m_trailingLink[side]->SetRot(rot);
  m_trailingLink[side]->SetMass(getTrailingLinkMass());
  m_trailingLink[side]->SetInertiaXX(getTrailingLinkInertia());
  AddVisualizationTrailingLink(m_trailingLink[side], points[TL_C], points[SPRING_L], points[TL_U], getTrailingLinkRadius());
  chassis->GetSystem()->AddBody(m_trailingLink[side]);

  // lower shock strut body
  u = ChVector<>(1, 0, 0);
  w = points[SHOCK_C] - points[SHOCK_L];
  w.Normalize();
  v = w % u;
  v.Normalize();
  u = v % w;
  rot.Set_A_axis(u, v, w);

  m_lowerStrut[side] = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_lowerStrut[side]->SetNameString(m_name + "_lowerStrut" + suffix);
  m_lowerStrut[side]->SetPos(points[LS_CM]);
  m_lowerStrut[side]->SetRot(rot.Get_A_quaternion());
  m_lowerStrut[side]->SetMass(getLowerStrutMass());
  m_lowerStrut[side]->SetInertiaXX(getLowerStrutInertia());
  AddVisualizationStrut(points[SHOCK_L], (points[SHOCK_L] + points[SHOCK_C]) / 2.0, getLowerStrutRadius());
  chassis->GetSystem()->AddBody(m_lowerStrut[side]);

  // upper strut body
  m_upperStrut[side] = ChSharedBodyPtr(new ChBody(chassis->GetSystem()->GetContactMethod()));
  m_upperStrut[side]->SetNameString(m_name + "_upperStrut" + suffix);
  m_upperStrut[side]->SetPos(points[US_CM]);
  m_upperStrut[side]->SetRot(rot.Get_A_quaternion());
  m_upperStrut[side]->SetMass(getUpperStrutMass());
  m_upperStrut[side]->SetInertiaXX(getUpperStrutInertia());
  AddVisualizationStrut((points[SHOCK_L] + points[SHOCK_C] / 2.0), points[SHOCK_C], getUpperStrutRadius());
  chassis->GetSystem()->AddBody(m_upperStrut[side]);

  // Create and initialize the revolute joint between upright and spindle.
  ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));

  m_revolute[side] = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
  m_revolute[side]->Initialize(m_spindle[side], m_upright[side], rev_csys);
  chassis->GetSystem()->AddLink(m_revolute[side]);

  // Create and initialize the revolute joint between chassis and upper arm.
  // Determine the joint orientation matrix from the hardpoint locations by
  // constructing a rotation matrix with the z axis along the joint direction
  // and the y axis normal to the plane of the upper arm.
  v = Vcross(points[UA_B] - points[UA_U], points[UA_F] - points[UA_U]);
  v.Normalize();
  w = points[UA_F] - points[UA_B];
  w.Normalize();
  u = Vcross(v, w);
  rot.Set_A_axis(u, v, w);

  m_revoluteUA[side] = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
  m_revoluteUA[side]->SetNameString(m_name + "_revoluteUA" + suffix);
  m_revoluteUA[side]->Initialize(chassis, m_upperArm[side], ChCoordsys<>((points[UA_F] + points[UA_B]) / 2, rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_revoluteUA[side]);

  // Create and initialize the spherical joint between upright and upper arm.
  m_sphericalUA[side] = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalUA[side]->SetNameString(m_name + "_sphericalUA" + suffix);
  m_sphericalUA[side]->Initialize(m_upperArm[side], m_upright[side], ChCoordsys<>(points[UA_U], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalUA[side]);

  // Create and initialize the spherical joint between upright and track rod.
  m_sphericalLateralUpright[side] = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalLateralUpright[side]->SetNameString(m_name + "_sphericalLateralUpright" + suffix);
  m_sphericalLateralUpright[side]->Initialize(m_lateral[side], m_upright[side], ChCoordsys<>(points[LAT_U], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalLateralUpright[side]);

  // Create and initialize the universal joint between chassis and track rod.
  u = ChVector<>(1, 0, 0);
  w = points[LAT_U] - points[LAT_C];
  w.Normalize();
  v = w % u;
  v.Normalize();
  u = v % w;
  rot.Set_A_axis(u, v, w);

  m_hookeLateralChassis[side] = ChSharedPtr<ChLinkUniversal>(new ChLinkUniversal);
  m_hookeLateralChassis[side]->SetNameString(m_name + "_universalLateralChassis" + suffix);
  m_hookeLateralChassis[side]->Initialize(m_lateral[side], chassis, ChFrame<>(points[LAT_C], rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_hookeLateralChassis[side]);

  // Create and initialize the spherical joint between upright and trailing link.
  m_sphericalTLUpright[side] = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalTLUpright[side]->SetNameString(m_name + "_sphericalTLUpright" + suffix);
  m_sphericalTLUpright[side]->Initialize(m_trailingLink[side], m_upright[side], ChCoordsys<>(points[TL_U], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalTLUpright[side]);

  // Create and initialize the universal joint between chassis and trailing link.
  u = ChVector<>(1, 0, 0);
  w = points[TL_U] - points[TL_C];
  w.Normalize();
  v = w % u;
  v.Normalize();
  u = v % w;
  rot.Set_A_axis(u, v, w);

  m_hookeTLChassis[side] = ChSharedPtr<ChLinkUniversal>(new ChLinkUniversal);
  m_hookeTLChassis[side]->SetNameString(m_name + "_universalTLChassis" + suffix);
  m_hookeTLChassis[side]->Initialize(m_trailingLink[side], chassis, ChFrame<>(points[TL_C], rot.Get_A_quaternion()));
  chassis->GetSystem()->AddLink(m_hookeTLChassis[side]);

  // Create and initialize the spring
  m_spring[side] = ChSharedPtr<ChLinkSpringCB>(new ChLinkSpringCB);
  m_spring[side]->SetNameString(m_name + "_spring" + suffix);
  m_spring[side]->Initialize(chassis, m_trailingLink[side], false, points[SPRING_C], points[SPRING_L], false, getSpringRestLength());
  m_spring[side]->Set_SpringCallback(getSpringForceCallback());
  chassis->GetSystem()->AddLink(m_spring[side]);

  // create the shock between the two struts
  m_shock[side] = ChSharedPtr<ChLinkSpringCB>(new ChLinkSpringCB);
  m_shock[side]->SetNameString(m_name + "_shock" + suffix);
  m_shock[side]->Initialize(chassis, m_trailingLink[side], false, points[SHOCK_C], points[SHOCK_L]);
  m_shock[side]->Set_SpringCallback(getShockForceCallback());
  chassis->GetSystem()->AddLink(m_shock[side]);

  // constraint the lower shock strut to the trailing link
  m_sphericalLowerStrut[side] = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalLowerStrut[side]->SetNameString(m_name + "_sphStrutTL" + suffix);
  m_sphericalLowerStrut[side]->Initialize(m_lowerStrut[side], m_trailingLink[side], ChCoordsys<>(points[SHOCK_L], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalLowerStrut[side]);

  // constraint upper shock strut to the chassis
  m_sphericalUpperStrut[side] = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_sphericalUpperStrut[side]->SetNameString(m_name + "_sphStrutChassis" + suffix);
  m_sphericalUpperStrut[side]->Initialize(m_upperStrut[side], chassis, ChCoordsys<>(points[SHOCK_C], QUNIT));
  chassis->GetSystem()->AddLink(m_sphericalUpperStrut[side]);

  // prismatic joint between the two struts
  w = points[SHOCK_C] - points[SHOCK_L];
  w.Normalize();
  u = ChVector<>(1, 0, 0);
  v = w % u;
  v.Normalize();
  u = v % w;
  rot.Set_A_axis(u, v, w);

  m_prismaticStrut[side] = ChSharedPtr<ChLinkLockPrismatic>(new ChLinkLockPrismatic);
  m_prismaticStrut[side]->SetNameString(m_name + "_prismaticStrut" + suffix);
  m_prismaticStrut[side]->Initialize(m_upperStrut[side], m_lowerStrut[side], ChCoordsys<>((points[SHOCK_C] + points[SHOCK_L]) / 2.0, rot.Get_A_quaternion()));

  // tierod distance constraint
  m_distTierod[side] = ChSharedPtr<ChLinkDistance>(new ChLinkDistance);
  m_distTierod[side]->SetNameString(m_name + "_distTierod" + suffix);
  m_distTierod[side]->Initialize(m_upright[side], chassis, false, points[TIEROD_U], points[TIEROD_C]);
  chassis->GetSystem()->AddLink(m_distTierod[side]);

  // Create and initialize the axle shaft and its connection to the spindle. Note that the
  // spindle rotates about the Y axis.
  m_axle[side] = ChSharedPtr<ChShaft>(new ChShaft);
  m_axle[side]->SetNameString(m_name + "_axle" + suffix);
  m_axle[side]->SetInertia(getAxleInertia());
  chassis->GetSystem()->Add(m_axle[side]);

  m_axle_to_spindle[side] = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);
  m_axle_to_spindle[side]->SetNameString(m_name + "_axle_to_spindle" + suffix);
  m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, -1, 0));
  chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLinkStrut::LogHardpointLocations(const ChVector<>& ref,
                                        bool              inches)
{
  double unit = inches ? 1 / 0.0254 : 1.0;

  for (int i = 0; i < NUM_POINTS; i++) {
    ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

    GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x << "  " << pos.y << "  " << pos.z << "\n";
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLinkStrut::LogConstraintViolations(ChVehicleSide side)
{
  // Revolute joints
  {
    ChMatrix<>* C = m_revoluteUA[side]->GetC();
    GetLog() << "Upper arm revolute    ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }
  {
    ChMatrix<>* C = m_revolute[side]->GetC();
    GetLog() << "Spindle revolute      ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";
  }

  // Spherical joints
  {
    ChMatrix<>* C = m_sphericalUA[side]->GetC();
    GetLog() << "Upper arm spherical   ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "\n";
  }
  {
    ChMatrix<>* C = m_sphericalLateralUpright[side]->GetC();
    GetLog() << "Lateral-Upright spherical  ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "\n";
  }
  {
    ChMatrix<>* C = m_sphericalTLUpright[side]->GetC();
    GetLog() << "TL-Upright spherical  ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "\n";
  }

  // Universal joints
  {
    ChMatrix<>* C = m_hookeLateralChassis[side]->GetC();
    GetLog() << "Lateral-Chassis universal  ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "\n";
  }
  {
    ChMatrix<>* C = m_hookeTLChassis[side]->GetC();
    GetLog() << "TL-Chassis universal  ";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "\n";
  }

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLinkStrut::AddVisualizationUpperArm(ChSharedBodyPtr   arm,
                                           const ChVector<>  pt_F,
                                           const ChVector<>  pt_B,
                                           const ChVector<>  pt_U,
                                           double            radius)
{
  // Express hardpoint locations in body frame.
  ChVector<> p_F = arm->TransformPointParentToLocal(pt_F);
  ChVector<> p_B = arm->TransformPointParentToLocal(pt_B);
  ChVector<> p_U = arm->TransformPointParentToLocal(pt_U);

  ChSharedPtr<ChCylinderShape> cyl_F(new ChCylinderShape);
  cyl_F->GetCylinderGeometry().p1 = p_F;
  cyl_F->GetCylinderGeometry().p2 = p_U;
  cyl_F->GetCylinderGeometry().rad = radius;
  arm->AddAsset(cyl_F);

  ChSharedPtr<ChCylinderShape> cyl_B(new ChCylinderShape);
  cyl_B->GetCylinderGeometry().p1 = p_B;
  cyl_B->GetCylinderGeometry().p2 = p_U;
  cyl_B->GetCylinderGeometry().rad = radius;
  arm->AddAsset(cyl_B);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(ChColor(0.6f, 0.2f, 0.6f));
  arm->AddAsset(col);
}

void ChMultiLinkStrut::AddVisualizationUpright(ChSharedBodyPtr   upright,
                                          const ChVector<>  pt_UA,
                                          const ChVector<>  pt_TR,
                                          const ChVector<>  pt_TL,
                                          const ChVector<>  pt_T,
                                          const ChVector<>  pt_U,
                                          double            radius)
{
  static const double threshold2 = 1e-6;

  // Express hardpoint locations in body frame.
  ChVector<> p_UA = upright->TransformPointParentToLocal(pt_UA);
  ChVector<> p_TR = upright->TransformPointParentToLocal(pt_TR);
  ChVector<> p_TL = upright->TransformPointParentToLocal(pt_TL);
  ChVector<> p_T = upright->TransformPointParentToLocal(pt_T);
  ChVector<> p_U = upright->TransformPointParentToLocal(pt_U);

  if (p_UA.Length2() > threshold2) {
    ChSharedPtr<ChCylinderShape> cyl_UA(new ChCylinderShape);
    cyl_UA->GetCylinderGeometry().p1 = p_UA;
    cyl_UA->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_UA->GetCylinderGeometry().rad = radius;
    upright->AddAsset(cyl_UA);
  }

  if (p_TR.Length2() > threshold2) {
    ChSharedPtr<ChCylinderShape> cyl_TR(new ChCylinderShape);
    cyl_TR->GetCylinderGeometry().p1 = p_TR;
    cyl_TR->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_TR->GetCylinderGeometry().rad = radius;
    upright->AddAsset(cyl_TR);
  }

  if (p_TL.Length2() > threshold2) {
    ChSharedPtr<ChCylinderShape> cyl_TL(new ChCylinderShape);
    cyl_TL->GetCylinderGeometry().p1 = p_TL;
    cyl_TL->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_TL->GetCylinderGeometry().rad = radius;
    upright->AddAsset(cyl_TL);
  }

  if (p_T.Length2() > threshold2) {
    ChSharedPtr<ChCylinderShape> cyl_T(new ChCylinderShape);
    cyl_T->GetCylinderGeometry().p1 = p_T;
    cyl_T->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_T->GetCylinderGeometry().rad = radius;
    upright->AddAsset(cyl_T);
  }

  if (p_U.Length2() > threshold2) {
    ChSharedPtr<ChCylinderShape> cyl_U(new ChCylinderShape);
    cyl_U->GetCylinderGeometry().p1 = p_U;
    cyl_U->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_U->GetCylinderGeometry().rad = radius;
    upright->AddAsset(cyl_U);
  }

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
  upright->AddAsset(col);
}

void ChMultiLinkStrut::AddVisualizationLateral(ChSharedBodyPtr   rod,
                                          const ChVector<>  pt_C,
                                          const ChVector<>  pt_U,
                                          double            radius)
{
  // Express hardpoint locations in body frame.
  ChVector<> p_C = rod->TransformPointParentToLocal(pt_C);
  ChVector<> p_U = rod->TransformPointParentToLocal(pt_U);

  ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
  cyl->GetCylinderGeometry().p1 = p_C;
  cyl->GetCylinderGeometry().p2 = p_U;
  cyl->GetCylinderGeometry().rad = radius;
  rod->AddAsset(cyl);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(ChColor(0.2f, 0.6f, 0.2f));
  rod->AddAsset(col);
}

void ChMultiLinkStrut::AddVisualizationTrailingLink(ChSharedBodyPtr   link,
                                               const ChVector<>  pt_C,
                                               const ChVector<>  pt_S,
                                               const ChVector<>  pt_U,
                                               double            radius)
{
  // Express hardpoint locations in body frame.
  ChVector<> p_C = link->TransformPointParentToLocal(pt_C);
  ChVector<> p_S = link->TransformPointParentToLocal(pt_S);
  ChVector<> p_U = link->TransformPointParentToLocal(pt_U);

  ChSharedPtr<ChCylinderShape> cyl1(new ChCylinderShape);
  cyl1->GetCylinderGeometry().p1 = p_C;
  cyl1->GetCylinderGeometry().p2 = p_S;
  cyl1->GetCylinderGeometry().rad = radius;
  link->AddAsset(cyl1);

  ChSharedPtr<ChCylinderShape> cyl2(new ChCylinderShape);
  cyl2->GetCylinderGeometry().p1 = p_S;
  cyl2->GetCylinderGeometry().p2 = p_U;
  cyl2->GetCylinderGeometry().rad = radius;
  link->AddAsset(cyl2);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  col->SetColor(ChColor(0.2f, 0.6f, 0.6f));
  link->AddAsset(col);
}


void ChMultiLinkStrut::AddVisualizationSpindle(ChSharedBodyPtr spindle,
                                          double          radius,
                                          double          width)
{
  ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
  cyl->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
  cyl->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
  cyl->GetCylinderGeometry().rad = radius;
  spindle->AddAsset(cyl);
}

void ChMultiLinkStrut::AddVisualizationStrut(const ChVector<> pt_L,
    const ChVector<> pt_U,
    double radius){

    // TODO
}


} // end namespace chrono
