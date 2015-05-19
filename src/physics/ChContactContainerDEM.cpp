//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChContactContainerDEM.cpp
//
// ------------------------------------------------
//             www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChContactContainerDEM.h"
#include "physics/ChSystem.h"
#include "physics/ChIndexedNodes.h"
#include "physics/ChBody.h"

#include "collision/ChCModelBulletBody.h"

namespace chrono {

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChContactContainerDEM> a_registration_ChContactContainerDEM;

ChContactContainerDEM::ChContactContainerDEM() {
    contactlist.clear();
    n_added = 0;
}

ChContactContainerDEM::~ChContactContainerDEM() {
    std::list<ChContactDEM*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        delete (*itercontact);
        (*itercontact) = 0;
        ++itercontact;
    }

    contactlist.clear();
}

void ChContactContainerDEM::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class, basically doing nothing :)
    ChContactContainerBase::Update(mytime, update_assets);
}

//// STATE BOOKKEEPING FUNCTIONS

void ChContactContainerDEM::IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                              ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                              const double c           ///< a scaling factor
                                              ) {
    ChContactDEM* cntct;

    std::list<ChContactDEM*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        cntct = (ChContactDEM*)(*itercontact);
        if ((cntct->GetContactPenetration() > 0)) {
            (*itercontact)->DemIntLoadResidual_F(R, c);
        }
        ++itercontact;
    }
}

////////// LCP INTERFACES ////

void ChContactContainerDEM::ConstraintsFbLoadForces(double factor) {
    ChContactDEM* cntct;

    std::list<ChContactDEM*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        cntct = (ChContactDEM*)(*itercontact);
        if ((cntct->GetContactPenetration() > 0)) {
            (*itercontact)->ConstraintsFbLoadForces(factor);
        }
        ++itercontact;
    }
}

void ChContactContainerDEM::RemoveAllContacts() {
    std::list<ChContactDEM*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        delete (*itercontact);
        (*itercontact) = 0;
        ++itercontact;
    }
    contactlist.clear();

    lastcontact = contactlist.begin();
    n_added = 0;
}

void ChContactContainerDEM::BeginAddContact() {
    lastcontact = contactlist.begin();
    n_added = 0;
}

void ChContactContainerDEM::EndAddContact() {
    // remove contacts that are beyond last contact
    while (lastcontact != contactlist.end()) {
        delete (*lastcontact);
        lastcontact = contactlist.erase(lastcontact);
    }
}

void ChContactContainerDEM::AddContact(const collision::ChCollisionInfo& mcontact) {
    // Do nothing if the shapes are separated
    if (mcontact.distance >= 0)
        return;

    // Return now if not expected contact models or no associated bodies.
    ChModelBulletBody* mmboA = dynamic_cast<ChModelBulletBody*>(mcontact.modelA);
    ChModelBulletBody* mmboB = dynamic_cast<ChModelBulletBody*>(mcontact.modelB);
    if (!mmboA || !mmboB)
        return;

    if (!mmboA->GetBody() || !mmboB->GetBody())
        return;

    // Return now if both bodies are inactive.
    if (!mmboA->GetBody()->IsActive() && !mmboB->GetBody()->IsActive())
        return;

    // Reuse an existing contact or create a new one
    if (lastcontact != contactlist.end()) {
        // reuse old contacts
        (*lastcontact)->Reset(mmboA, mmboB, mcontact);
        lastcontact++;
    } else {
        // add new contact
        ChContactDEM* mc = new ChContactDEM(mmboA, mmboB, mcontact);
        contactlist.push_back(mc);
        lastcontact = contactlist.end();
    }

    n_added++;
}

void ChContactContainerDEM::ReportAllContacts(ChReportContactCallback* mcallback) {
    std::list<ChContactDEM*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        bool proceed = mcallback->ReportContactCallback(
            (*itercontact)->GetContactP1(), (*itercontact)->GetContactP2(), (*itercontact)->GetContactPlane(),
            (*itercontact)->GetContactPenetration(), 0.0, (*itercontact)->GetContactForceLocal(),
            VNULL,  // no react torques
            (*itercontact)->GetModel1(), (*itercontact)->GetModel2());

        if (!proceed)
            break;

        ++itercontact;
    }
}

}  // END_OF_NAMESPACE____
