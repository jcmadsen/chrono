//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//////////////////////////////////////////////////
//
//   ChCCollisionModel.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCCollisionModel.h"
#include "physics/ChBody.h"

namespace chrono {
namespace collision {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChCollisionModel> a_registration_ChCollisionModel;


static double default_model_envelope = 0.03;
static double default_safe_margin = 0.01;

ChCollisionModel::ChCollisionModel() {
    model_envelope = (float)default_model_envelope;  //  0.03f;
    model_safe_margin = (float)default_safe_margin;  // 0.01f;
    
    mcontactable = 0;
}


ChPhysicsItem* ChCollisionModel::GetPhysicsItem() 
{
    return mcontactable->GetPhysicsItem();
};


void ChCollisionModel::SetDefaultSuggestedEnvelope(double menv) {
    default_model_envelope = menv;
}

void ChCollisionModel::SetDefaultSuggestedMargin(double mmargin) {
    default_safe_margin = mmargin;
}

// static
double ChCollisionModel::GetDefaultSuggestedEnvelope() {
    return default_model_envelope;
}

// static
double ChCollisionModel::GetDefaultSuggestedMargin() {
    return default_safe_margin;
}

bool ChCollisionModel::AddConvexHullsFromFile(ChStreamInAscii& mstream,
                                              const ChVector<>& pos,
                                              const ChMatrix33<>& rot) {
    std::vector<ChVector<double> > ptlist;

    char bufdata[200];
    int linechar = 0;
    while (!mstream.End_of_stream()) {
        // read one line
        linechar = 0;
        while (!mstream.End_of_stream()) {
            try {
                mstream >> bufdata[linechar];
            } catch (ChException mex) {
            };
            if ((bufdata[linechar] == (char)13) || (bufdata[linechar] == (char)10)) {
                bufdata[linechar] = 0;
                break;
            }
            linechar++;
            if (linechar >= 200)
                throw(ChException("Too long line in parsing"));
        }
        bufdata[linechar + 1] = 0;

        bool parsedline = false;
        if (bufdata[0] != *"#") {
            parsedline = true;
        }
        if (strcmp(bufdata, "hull") == 0) {
            if (ptlist.size())
                this->AddConvexHull(ptlist, pos, rot);
            ptlist.clear();
            parsedline = true;
        }
        float vx, vy, vz;
        if (sscanf(bufdata, "%g %g %g", &vx, &vy, &vz) == 3) {
            ptlist.push_back(ChVector<>(vx, vy, vz));
            parsedline = true;
        }
    }
    if (ptlist.size())
        this->AddConvexHull(ptlist, pos, rot);
    ptlist.clear();
    return true;
}





}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____
