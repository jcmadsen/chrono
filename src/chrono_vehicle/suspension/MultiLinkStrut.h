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
// Authors: Justin Madsen, Daniel Melanz, Radu Serban
// =============================================================================
//
// Multi-link suspension constructed with data from file.
//
// =============================================================================

#ifndef MULTILINKSTRUT_H
#define MULTILINKSTRUT_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/suspension/ChMultiLinkStrut.h"

#include "thirdparty/rapidjson/document.h"

namespace chrono {


class CH_VEHICLE_API MultiLink : public ChMultiLinkStrut
{
public:

  MultiLink(const std::string& filename);
  MultiLink(const rapidjson::Document& d);
  ~MultiLink();

  virtual double getSpindleMass() const { return m_spindleMass; }
  virtual double getUpperArmMass() const { return m_upperArmMass; }
  virtual double getLateralMass() const { return m_lateralMass; }
  virtual double getTrailingLinkMass() const { return m_trailingLinkMass; }
  virtual double getUprightMass() const { return m_uprightMass; }
  virtual double getLowerStrutMass() const { return m_lowerStrutMass; }
  virtual double getUpperStrutMass() const { return m_upperStrutMass; }

  virtual double getSpindleRadius() const { return m_spindleRadius; }
  virtual double getSpindleWidth() const { return m_spindleWidth; }
  virtual double getUpperArmRadius() const { return m_upperArmRadius; }
  virtual double getLateralRadius() const { return m_lateralRadius; }
  virtual double getTrailingLinkRadius() const { return m_trailingLinkRadius; }
  virtual double getUprightRadius() const { return m_uprightRadius; }
  virtual double getLowerStrutRadius() const { return m_lowerStrutRadius; }
  virtual double getUpperStrutRadius() const { return m_upperStrutRadius; }

  virtual const ChVector<>& getSpindleInertia() const { return m_spindleInertia; }
  virtual const ChVector<>& getUpperArmInertia() const { return m_upperArmInertia; }
  virtual const ChVector<>& getLateralInertia() const { return m_lateralInertia; }
  virtual const ChVector<>& getTrailingLinkInertia() const { return m_trailingLinkInertia; }
  virtual const ChVector<>& getUprightInertia() const { return m_uprightInertia; }
  virtual const ChVector<>& getUpperStrutInertia() const { return m_upperStrutInertia; }
  virtual const ChVector<>& getLowerStrutInertia() const { return m_lowerStrutInertia; }
  
  virtual double getAxleInertia() const { return m_axleInertia; }

  virtual double getSpringRestLength() const { return m_springRestLength; }
  virtual ChSpringForceCallback* getSpringForceCallback() const { return m_springForceCB; }
  virtual ChSpringForceCallback* getShockForceCallback()  const { return m_shockForceCB; }

private:

  virtual const ChVector<> getLocation(PointId which) { return m_points[which]; }
  virtual const ChVector<> getDirection(DirectionId which) { return m_directions[which]; }

  void Create(const rapidjson::Document& d);

  ChSpringForceCallback* m_springForceCB;
  ChSpringForceCallback* m_shockForceCB;

  ChVector<>  m_points[NUM_POINTS];
  ChVector<>  m_directions[NUM_DIRS];

  double      m_spindleMass;
  double      m_upperArmMass;
  double      m_lateralMass;
  double      m_trailingLinkMass;
  double      m_uprightMass;
  double m_lowerStrutMass;
  double m_upperStrutMass;  

  double      m_spindleRadius;
  double      m_spindleWidth;
  double      m_upperArmRadius;
  double      m_lateralRadius;
  double      m_trailingLinkRadius;
  double      m_uprightRadius;
  double m_lowerStrutRadius;
  double m_upperStrutRadius;

  ChVector<>  m_spindleInertia;
  ChVector<>  m_upperArmInertia;
  ChVector<>  m_lateralInertia;
  ChVector<>  m_trailingLinkInertia;
  ChVector<>  m_uprightInertia;
  ChVector<> m_lowerStrutInertia;
  ChVector<> m_upperStrutInertia;

  double      m_axleInertia;

  double      m_springRestLength;
};


} // end namespace chrono


#endif
