//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChContact.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChContactRolling.h"
#include "physics/ChSystem.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


using namespace collision;
using namespace geometry;


ChContactRolling::ChContactRolling ()
{ 
	Rx.SetRollingConstraintU(&Ru);
	Rx.SetRollingConstraintV(&Rv);
	Rx.SetNormalConstraint(&Nx);
}

ChContactRolling::ChContactRolling (collision::ChCollisionModel* mmodA,	///< model A
						collision::ChCollisionModel* mmodB,	///< model B
						const ChLcpVariablesBody* varA, ///< pass A vars
						const ChLcpVariablesBody* varB, ///< pass B vars
						const ChFrame<>* frameA,		  ///< pass A frame
						const ChFrame<>* frameB,		  ///< pass B frame
						const ChVector<>& vpA,		  ///< pass coll.point on A
						const ChVector<>& vpB,		  ///< pass coll.point on B
						const ChVector<>& vN, 		  ///< pass coll.normal, respect to A
						double mdistance,		  ///< pass the distance (negative for penetration)
						float* mreaction_cache,	  ///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
						ChMaterialCouple& mmaterial		///< pass the reference to the material with friction, stiffness, etc.
				)
{ 
	Rx.SetRollingConstraintU(&Ru);
	Rx.SetRollingConstraintV(&Rv);
	Rx.SetNormalConstraint(&Nx);

	Reset(	mmodA, mmodB,
			varA, ///< pass A vars
			varB, ///< pass B vars
			frameA,		  ///< pass A frame
			frameB,		  ///< pass B frame
			vpA,		  ///< pass coll.point on A
			vpB,		  ///< pass coll.point on B
			vN, 		  ///< pass coll.normal, respect to A
			mdistance,		  ///< pass the distance (negative for penetration)
			mreaction_cache,	  ///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
			mmaterial		///< pass the reference to the material with friction, stiffness, etc.
				);
}

ChContactRolling::~ChContactRolling ()
{

}

void ChContactRolling::Reset(	collision::ChCollisionModel* mmodA,	///< model A
						collision::ChCollisionModel* mmodB,	///< model B
					    const ChLcpVariablesBody* varA, ///< pass A vars
						const ChLcpVariablesBody* varB, ///< pass B vars
						const ChFrame<>* frameA,		  ///< pass A frame
						const ChFrame<>* frameB,		  ///< pass B frame
						const ChVector<>& vpA,		  ///< pass coll.point on A
						const ChVector<>& vpB,		  ///< pass coll.point on B
						const ChVector<>& vN, 		  ///< pass coll.normal, respect to A
						double mdistance,		  ///< pass the distance (negative for penetration)
						float* mreaction_cache,	  ///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
						ChMaterialCouple& mmaterial		///< pass the reference to the material with friction, stiffness, etc.
				)
{
	// Base method call:
	ChContact::Reset(	mmodA, mmodB,
			varA, ///< pass A vars
			varB, ///< pass B vars
			frameA,		  ///< pass A frame
			frameB,		  ///< pass B frame
			vpA,		  ///< pass coll.point on A
			vpB,		  ///< pass coll.point on B
			vN, 		  ///< pass coll.normal, respect to A
			mdistance,		  ///< pass the distance (negative for penetration)
			mreaction_cache,	  ///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
			mmaterial		///< pass the reference to the material with friction, stiffness, etc.
				);

	Rx.SetVariables(const_cast<ChLcpVariablesBody*>(varA),const_cast<ChLcpVariablesBody*>(varB));
	Ru.SetVariables(const_cast<ChLcpVariablesBody*>(varA),const_cast<ChLcpVariablesBody*>(varB));
	Rv.SetVariables(const_cast<ChLcpVariablesBody*>(varA),const_cast<ChLcpVariablesBody*>(varB));

	Rx.SetRollingFrictionCoefficient(mmaterial.rolling_friction);
	Rx.SetSpinningFrictionCoefficient(mmaterial.spinning_friction);

	ChMatrix33<> Jx1, Jx2, Jr1, Jr2;

	Jr1.MatrTMultiply(contact_plane, frameA->GetA());
	Jr2.MatrTMultiply(contact_plane, frameB->GetA());
	Jr1.MatrNeg();

	Rx.Get_Cq_a()->PasteClippedMatrix(&Jx1, 0,0, 1,3, 0,0);
	Ru.Get_Cq_a()->PasteClippedMatrix(&Jx1, 1,0, 1,3, 0,0);
	Rv.Get_Cq_a()->PasteClippedMatrix(&Jx1, 2,0, 1,3, 0,0);
	Rx.Get_Cq_a()->PasteClippedMatrix(&Jr1, 0,0, 1,3, 0,3);
	Ru.Get_Cq_a()->PasteClippedMatrix(&Jr1, 1,0, 1,3, 0,3);
	Rv.Get_Cq_a()->PasteClippedMatrix(&Jr1, 2,0, 1,3, 0,3);

	Rx.Get_Cq_b()->PasteClippedMatrix(&Jx2, 0,0, 1,3, 0,0);
	Ru.Get_Cq_b()->PasteClippedMatrix(&Jx2, 1,0, 1,3, 0,0);
	Rv.Get_Cq_b()->PasteClippedMatrix(&Jx2, 2,0, 1,3, 0,0);
	Rx.Get_Cq_b()->PasteClippedMatrix(&Jr2, 0,0, 1,3, 0,3);
	Ru.Get_Cq_b()->PasteClippedMatrix(&Jr2, 1,0, 1,3, 0,3);
	Rv.Get_Cq_b()->PasteClippedMatrix(&Jr2, 2,0, 1,3, 0,3);

	this->complianceRoll = mmaterial.complianceRoll;
	this->complianceSpin = mmaterial.complianceSpin;

	react_torque = VNULL;

}


void ChContactRolling::ContIntStateGatherReactions(const unsigned int off_L,	ChVectorDynamic<>& L)
{
	// base behaviour too
	ChContact::ContIntStateGatherReactions(off_L, L);

	L(off_L+3) = react_torque.x;
	L(off_L+4) = react_torque.y;
	L(off_L+5) = react_torque.z;
}

void ChContactRolling::ContIntStateScatterReactions(const unsigned int off_L,	const ChVectorDynamic<>& L)
{
	// base behaviour too
	ChContact::ContIntStateScatterReactions(off_L, L);

	react_torque.x = L(off_L+3);
	react_torque.y = L(off_L+4);
	react_torque.z = L(off_L+5);
}

void ChContactRolling::ContIntLoadResidual_CqL(
					const unsigned int off_L,	 ///< offset in L multipliers
					ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*Cq'*L 
					const ChVectorDynamic<>& L,  ///< the L vector 
					const double c				 ///< a scaling factor
					)
{
	// base behaviour too
	ChContact::ContIntLoadResidual_CqL(off_L, R, L, c);

	this->Rx.MultiplyTandAdd(R, L(off_L+3) *c);
	this->Ru.MultiplyTandAdd(R, L(off_L+4) *c);
	this->Rv.MultiplyTandAdd(R, L(off_L+5) *c);
}
void ChContactRolling::ContIntLoadConstraint_C(
					const unsigned int off_L,	 ///< offset in Qc residual
					ChVectorDynamic<>& Qc,		 ///< result: the Qc residual, Qc += c*C 
					const double c,				 ///< a scaling factor
					bool do_clamp,				 ///< apply clamping to c*C?
					double recovery_clamp		 ///< value for min/max clamping of c*C
					)
{
	// base behaviour too
	ChContact::ContIntLoadConstraint_C(off_L, Qc, c, do_clamp, recovery_clamp);

	//Qc(off_L+3) += c * Rx.Get_b_i();
	//Qc(off_L+4) += c * Ru.Get_b_i();
	//Qc(off_L+5) += c * Rv.Get_b_i();
}
void ChContactRolling::ContIntToLCP(
					const unsigned int off_L,			///< offset in L, Qc
					const ChVectorDynamic<>& L,
					const ChVectorDynamic<>& Qc
					)
{
	// base behaviour too
	ChContact::ContIntToLCP(off_L, L, Qc);

	Rx.Set_l_i(L(off_L+3));
	Ru.Set_l_i(L(off_L+4));
	Rv.Set_l_i(L(off_L+5));

	Rx.Set_b_i(Qc(off_L+3));
	Ru.Set_b_i(Qc(off_L+4));
	Rv.Set_b_i(Qc(off_L+5));
}
void ChContactRolling::ContIntFromLCP(
					const unsigned int off_L,			///< offset in L
					ChVectorDynamic<>& L
					)
{
	// base behaviour too
	ChContact::ContIntFromLCP(off_L, L);

	L(off_L+3) = Rx.Get_l_i();
	L(off_L+4) = Ru.Get_l_i();
	L(off_L+5) = Rv.Get_l_i();
}

 


void ChContactRolling::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	// base behaviour too
	ChContact::InjectConstraints(mdescriptor);

	mdescriptor.InsertConstraint(&Rx);
	mdescriptor.InsertConstraint(&Ru);
	mdescriptor.InsertConstraint(&Rv); 
}

void ChContactRolling::ConstraintsBiReset()
{
	// base behaviour too
	ChContact::ConstraintsBiReset();

	Rx.Set_b_i(0.);
	Ru.Set_b_i(0.);
	Rv.Set_b_i(0.);
}
 
void ChContactRolling::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	// base behaviour too
	ChContact::ConstraintsBiLoad_C(factor,recovery_clamp,do_clamp);
	
	// If rolling and spinning compliance, set the cfm terms

	double h = 1.0/factor;  // timestep is inverse of factor
			
	double alpha=this->dampingf; // [R]=alpha*[K]
	double inv_hpa = 1.0/(h+alpha); // 1/(h+a)
	double inv_hhpa = 1.0/(h*(h+alpha)); // 1/(h*(h+a)) 

	this->Ru.Set_cfm_i( (inv_hhpa)*this->complianceRoll  );
	this->Rv.Set_cfm_i( (inv_hhpa)*this->complianceRoll  );
	this->Rx.Set_cfm_i( (inv_hhpa)*this->complianceSpin  );

	// Assume no residual ever, do not load in C
}

void ChContactRolling::ConstraintsFetch_react(double factor)
{
	// base behaviour too
	ChContact::ConstraintsFetch_react(factor);

	// From constraints to react torque:
	react_torque.x = Rx.Get_l_i() * factor;  
	react_torque.y = Ru.Get_l_i() * factor;
	react_torque.z = Rv.Get_l_i() * factor;
}


void  ChContactRolling::ConstraintsLiLoadSuggestedSpeedSolution()
{
	// base behaviour too
	ChContact::ConstraintsLiLoadSuggestedSpeedSolution();

	// [Note: no persistent cache used for rolling multipliers - do nothing]
}

void  ChContactRolling::ConstraintsLiLoadSuggestedPositionSolution()
{
	// base behaviour too
	ChContact::ConstraintsLiLoadSuggestedPositionSolution();

	// [Note: no persistent cache used for rolling multipliers - do nothing]
}

void  ChContactRolling::ConstraintsLiFetchSuggestedSpeedSolution()
{
	// base behaviour too
	ChContact::ConstraintsLiFetchSuggestedSpeedSolution();

	// [Note: no persistent cache used for rolling multipliers - do nothing]
}

void  ChContactRolling::ConstraintsLiFetchSuggestedPositionSolution()
{
	// base behaviour too
	ChContact::ConstraintsLiFetchSuggestedPositionSolution();

	// [Note: no persistent cache used for rolling multipliers - do nothing]
}












} // END_OF_NAMESPACE____


