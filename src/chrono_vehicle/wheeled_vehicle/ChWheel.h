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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a vehicle wheel.
// A wheel subsystem does not own a body. Instead, when attached to a suspension
// subsystem, the wheel's mass properties are used to update those of the
// spindle body owned by the suspension.
// A concrete wheel subsystem can optionally carry its own visualization assets
// (which are associated with the suspension's spindle body).
//
// =============================================================================

#ifndef CH_WHEEL_H
#define CH_WHEEL_H

#include <vector>

#include "chrono/core/ChShared.h"
#include "chrono/physics/ChBody.h"

#include "chrono_vehicle/ChApiVehicle.h"

namespace chrono {
namespace vehicle {

///
/// Base class for a vehicle wheel subsystem.
/// A wheel subsystem does not own a body. Instead, when attached to a suspension
/// subsystem, the wheel's mass properties are used to update those of the
/// spindle body owned by the suspension.
/// A concrete wheel subsystem can optionally carry its own visualization assets
/// (which are associated with the suspension's spindle body).
///
class CH_VEHICLE_API ChWheel : public ChShared {
  public:
    ChWheel() {}
    virtual ~ChWheel() {}

    /// Get the wheel mass.
    virtual double GetMass() const = 0;

    /// Get the wheel moments of inertia.
    virtual ChVector<> GetInertia() const = 0;

    /// Get the wheel radius
    virtual double GetRadius() const { return 0; }

    /// Get the wheel width
    virtual double GetWidth() const { return 0; }

    /// Initialize this wheel subsystem.
    /// The wheel mass and inertia are used to increment those of the spindle.
    virtual void Initialize(ChSharedPtr<ChBody> spindle  ///< handle to the associated spindle body
                            );
};

/// Vector of handles to wheel subsystems.
typedef std::vector<ChSharedPtr<ChWheel> > ChWheelList;

}  // end namespace vehicle
}  // end namespace chrono

#endif
