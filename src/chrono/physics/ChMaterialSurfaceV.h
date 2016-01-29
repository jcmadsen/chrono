// Author: Justin Madsen, (c) 2016
// All Rights Reserved
//

#ifndef CHMATERIALSURFACEV_H
#define CHMATERIALSURFACEV_H

//
//   ChMaterialSurfaceV.h, suport for viscosity
//	 	as a material surface property
//
//   

#include "physics/ChMaterialSurface.h"

namespace chrono {

/// Material data for a surface: friction, compliance, etc.
/// This data is used to define surface properties owned by
/// ChBody rigid bodies and similar things; it carries information
/// that is used to make contacts.

class ChApi ChMaterialSurfaceV : public ChMaterialSurface {

    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChMaterialSurfaceV, ChMaterialSurfaceBase);

  public:
    //
    // DATA
    //

      float viscosity;

    //
    // CONSTRUCTORS
    //

    ChMaterialSurfaceV() : viscosity(0) {}

    ~ChMaterialSurfaceV(){};

    // Copy constructor
    ChMaterialSurfaceV(const ChMaterialSurfaceV& other) {
        viscosity = other.viscosity;
    }

    virtual ContactMethod GetContactMethod() { return DVI; };

    //
    // FUNCTIONS
    //

    /// Viscosity coefficient
	///	Typically, tau = eta * velocity_tangent, to find shear stress tau
	/// However, viscosity in contact constraints will result in an impulse
	/// units: [N-s^2/m]
    float GetViscosity() { return viscosity; }
    void SetViscosity(float val) { viscosity = val; }



    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive){
        // version number
        marchive.VersionWrite(1);

        // serialize parent class
        ChMaterialSurfaceBase::ArchiveOUT(marchive);
		ChMaterialSurface::ArchiveOUT(marchive);
		
        // serialize all member data:
        marchive << CHNVP(viscosity);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive){
        // version number
        int version = marchive.VersionRead();

        // deserialize parent class
        ChMaterialSurfaceBase::ArchiveIN(marchive);
		ChMaterialSurface::ArchiveIN(marchive);

        // stream in all member data:
        marchive >> CHNVP(viscosity); 
    }

};

}  // END_OF_NAMESPACE____

#endif
