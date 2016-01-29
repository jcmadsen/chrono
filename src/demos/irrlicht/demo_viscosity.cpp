///////////////////////////////////////////////////
//
//   Particles that are recorded as colliding will have
//      a viscosity component, where the reaction force
//      in the lagrange multiplier in the u,v directions
//      is also a function of relative tangent velocity * viscosity.
///////////////////////////////////////////////////

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/assets/ChTexture.h"

#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/physics/ChMaterialSurfaceV.h"

#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;


int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system
    ChSystem physicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&physicalSystem, L"Collisions between objects", core::dimension2d<u32>(800, 600), false);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 14, -20));

    // Create two rigid bodies, one is fixed and the other falls under gravity

    double fixed_r = 0.1;   // 10 cm radius
    double free_r = fixed_r;    // 10 cm radius

    // first of all, collision envelope will be same size as the radius
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(fixed_r);

    // fixed sphere, at the origin
    ChSharedPtr<ChBodyEasySphere> fixed(new ChBodyEasySphere(fixed_r, 1000, true, true));
    fixed->SetBodyFixed(true);
    fixed->SetPos(ChVector<>());

    physicalSystem.AddBody(fixed);

    // free sphere, offset in the x direction so there is a 1 cm gap between their true collision surfaces 
    // offset in y direction so there is initially no overlap of the collision envelopes
    double x_gap = 0.01;
    ChSharedPtr<ChBodyEasySphere> free(new ChBodyEasySphere(free_r, 100, true, true));
    free->SetPos(ChVector<>(fixed_r + free_r + x_gap, fixed_r + free_r, 0));

    physicalSystem.AddBody(free);
    
    // ChBody materialSurface by default doesn't include viscosity. Change to a material model that does
    float viscosity = 0.1f;   // viscosity !
    ChSharedPtr<ChMaterialSurfaceV> fixedMat(new ChMaterialSurfaceV);
    fixedMat->SetViscosity(viscosity);
    ChSharedPtr<ChMaterialSurfaceV> freeMat(new ChMaterialSurfaceV(*fixedMat.get()));

    // change the material model to use viscosity
    fixed->SetMaterialSurface(fixedMat);
    free->SetMaterialSurface(freeMat);

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Modify some setting of the physical system for the simulation, if you want
    physicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
    physicalSystem.SetIterLCPmaxItersSpeed(20);
    physicalSystem.SetIterLCPmaxItersStab(5);

    // physicalSystem.SetUseSleeping(true);

    application.SetStepManage(true);
    application.SetTimestep(0.02);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    while (application.GetDevice()->run()) {
        application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        application.DoStep();

        application.GetVideoDriver()->endScene();
    }

    return 0;
}
