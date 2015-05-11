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
// Chain drive.
//
// =============================================================================

#include <cstdio>
#include <algorithm>

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "assets/ChAssetLevel.h"

#include "core/ChFileutils.h"
#include "physics/ChGlobal.h"
#include "physics/ChContactContainer.h"

#include "DriveChain.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"

// collision mesh
#include "geometry/ChCTriangleMeshSoup.h"
// custom collision detection classes
#include "subsys/collision/TrackCollisionCallback.h"
#include "subsys/collision/CollisionReporters.h"

// to help write vectors and quats to file by overloading stringstream

namespace chrono {

// -----------------------------------------------------------------------------
// Static variables
// idler, relative to gear/chassis
const ChVector<> DriveChain::m_idlerPos(-2.1904, -0.1443, 0.2447);  // relative to local csys
const ChQuaternion<> DriveChain::m_idlerRot(QUNIT);

/// constructor sets the basic integrator settings for this ChSystem, as well as the usual stuff
DriveChain::DriveChain(const std::string& name,
                       VisualizationType::Enum gearVis,
                       CollisionType::Enum gearCollide,
                       double pin_damping_coef,
                       double tensioner_preload,
                       size_t num_idlers,
                       size_t num_wheels,
                       double gear_mass,
                       const ChVector<>& gear_inertia)
    : ChTrackVehicle(name, gearVis, gearCollide, gear_mass, gear_inertia, 1),
      m_num_wheels(num_wheels),
      m_num_idlers(num_idlers),
      m_pin_damping(pin_damping_coef),
      m_tensioner_preload(tensioner_preload) {
    // setup the chassis body
    m_chassis->SetIdentifier(0);
    // basic body info. Not relevant since it's fixed.
    m_chassis->SetFrame_COG_to_REF(ChFrame<>());
    // chassis is fixed to ground
    m_chassis->SetBodyFixed(true);

    // set any non-initialized base-class variables here
    // doesn't matter for the chassis, since no visuals used
    m_meshName = "na";
    m_meshFile = "none";
    m_chassisBoxSize = ChVector<>(2.0, 0.6, 0.75);

    // --------------------------------------------------------------------------
    // BUILD THE SUBSYSTEMS
    // drive gear, inherits drivechain's visual and collision types, mass, inertia, etc.
    m_gear = ChSharedPtr<DriveGear>(new DriveGear("drive gear", m_vis, m_collide, 0, gear_mass, gear_inertia));

    // idlers
    m_idlers.clear();
    m_idlers.resize(m_num_idlers);
    double idler_mass = 100.0;           // 429.6
    ChVector<> idler_Ixx(gear_inertia);  // 12.55, 12.55, 14.7
    double tensioner_K = 2e5;
    double tensioner_C = 2.8e3;
    m_idlers[0] = ChSharedPtr<IdlerSimple>(new IdlerSimple("idler", VisualizationType::Mesh,
                                                           // VisualizationType::Primitives,
                                                           CollisionType::Primitives, 0, idler_mass, idler_Ixx,
                                                           tensioner_K, tensioner_C));

    // track chain system
    double shoe_mass = 18.03 / 6.0;                           // 18.03
    ChVector<> shoe_Ixx(0.22 / 6.0, 0.25 / 6.0, 0.04 / 6.0);  // 0.22, 0.25, 0.04
    m_chain = ChSharedPtr<TrackChain>(new TrackChain("chain", VisualizationType::CompoundPrimitives,
                                                     CollisionType::Primitives, 0, shoe_mass, shoe_Ixx));

    // create the powertrain, connect transmission shaft directly to gear shaft
    m_ptrains[0] = ChSharedPtr<TrackPowertrain>(new TrackPowertrain("powertrain "));

    // support wheels, if any
    m_wheels.clear();
    m_wheels.resize(m_num_wheels);
    double wheel_mass = 50.0;
    double wheel_r = 0.305;
    double wheel_w = 0.384;
    // assume constant density cylinder
    ChVector<> wheel_Ixx =
        wheel_mass * ChVector<>((3.0 * wheel_r * wheel_r + wheel_w * wheel_w) / 12.0,
                                (3.0 * wheel_r * wheel_r + wheel_w * wheel_w) / 12.0, wheel_r * wheel_r / 2.0);
    // asume the arm is 8x smaller than wheel weight, Ixx
    double arm_mass = wheel_mass / 8.0;
    ChVector<> arm_Ixx = wheel_Ixx / 8.0;

    // create the wheels
    for (int j = 0; j < m_num_wheels; j++) {
        GetLog() << " I just manually calculated inertia, uh-oh \n\n Ixx = " << wheel_Ixx << "\n";

        std::stringstream ss_w;
        ss_w << "bogie " << j;
        m_wheels[j] = ChSharedPtr<TorsionArmSuspension>(new TorsionArmSuspension(
            ss_w.str(), VisualizationType::Primitives, CollisionType::Primitives, 0, wheel_mass, wheel_Ixx, arm_mass,
            arm_Ixx,
            (2.5e4 / 5.0),                     // torsional spring stiffness
            (5.0e2 / 5.0),                     // torsional spring damping
            (3.5e3 / 5.0),                     // spring preload
            0.384,                             // bogie wheel width
            0.0912,                            // gap
            0.305,                             // wheel radius
            ChVector<>(-.2034, -0.2271, 0)));  // relative pos. of wheel center to arm-chassis rev. joint
    }

    if (m_num_idlers > 1) {
        // for now, just create 1 more idler
        m_idlers[1] = ChSharedPtr<IdlerSimple>(new IdlerSimple("idler2", VisualizationType::Mesh,
                                                               // VisualizationType::Primitives,
                                                               CollisionType::Primitives, 0, idler_mass, idler_Ixx,
                                                               tensioner_K, tensioner_C));
    }
}

DriveChain::~DriveChain() {
    if (m_ownsSystem)
        delete m_system;
}

// Set any collision geometry on the hull, then Initialize() all subsystems
void DriveChain::Initialize(const ChCoordsys<>& gear_Csys) {
    // initialize the drive gear, idler and track chain
    // m_idlerPosRel = m_idlerPos;
    m_idlerPosRel = ChVector<>(-2.4, -0.15, 0);
    m_chassis->SetFrame_REF_to_abs(ChFrame<>(gear_Csys.pos, gear_Csys.rot));

    // Create list of the center location of the rolling elements and their clearance.
    // Clearance is a sphere shaped envelope at each center location, where it can
    //  be guaranteed that the track chain geometry will not penetrate the sphere.
    std::vector<ChVector<> > rolling_elem_locs;       // w.r.t. chassis ref. frame
    std::vector<double> clearance;                    // 1 per rolling elem
    std::vector<ChVector<> > rolling_elem_spin_axis;  /// w.r.t. abs. frame

    // initialize 1 of each of the following subsystems.
    // will use the chassis ref frame to do the transforms, since the TrackSystem
    // local ref. frame has same rot (just difference in position)
    // NOTE: move Gear Init() to after TrackChain Init(), but still add Gear info to list first.

    // drive sprocket is First added to the lists passed into TrackChain Init()
    rolling_elem_locs.push_back(ChVector<>());
    clearance.push_back(m_gear->GetRadius());
    rolling_elem_spin_axis.push_back(m_gear->GetBody()->GetRot().GetZaxis());

    // initialize the support rollers.
    // Usually use 2, so spacing is based on that
    double spacing = m_idlerPosRel.Length();
    for (int r_idx = 0; r_idx < m_num_wheels; r_idx++) {
        ChVector<> armConnection_loc = m_chassis->GetPos();
        armConnection_loc.y = -0.7 - r_idx * 0.2;
        armConnection_loc.x = gear_Csys.pos.x - r_idx * spacing / 2.0;

        m_wheels[r_idx]->Initialize(m_chassis, m_chassis->GetFrame_REF_to_abs(),
                                    ChCoordsys<>(armConnection_loc, QUNIT));

        // add to the points passed into the track chain
        // in this case, use the center of the bogie wheel as the center point.
        rolling_elem_locs.push_back(armConnection_loc + m_wheels[r_idx]->GetWheelPosRel());
        clearance.push_back(m_wheels[r_idx]->GetWheelRadius());
        rolling_elem_spin_axis.push_back(m_wheels[r_idx]->GetWheelBody()->GetRot().GetZaxis());
    }

    // init the idler last
    m_idlers[0]->Initialize(m_chassis, m_chassis->GetFrame_REF_to_abs(),
                            ChCoordsys<>(m_idlerPosRel, Q_from_AngAxis(CH_C_PI, VECT_Z)), m_tensioner_preload);

    // add to the lists passed into the track chain Init()
    rolling_elem_locs.push_back(m_idlerPosRel);
    clearance.push_back(m_idlers[0]->GetRadius());
    rolling_elem_spin_axis.push_back(m_idlers[0]->GetBody()->GetRot().GetZaxis());

    // when there's only 2 rolling elements, the above locations will repeat
    // the first rolling element at the front and end of the vector.
    // So, just use the mid-point between the first two rolling elements.

    ChVector<> start_pos;
    if (clearance.size() < 3) {
        start_pos = (rolling_elem_locs[0] + rolling_elem_locs[1]) / 2.0;
        start_pos.y += (clearance[0] + clearance[1]) / 2.0;
    } else {
        start_pos = (rolling_elem_locs.front() + rolling_elem_locs.back()) / 2.0;
        start_pos.y += (clearance.front() + clearance.back()) / 2.0;
    }
    start_pos.x += 0.04;
    // NOTE: start_pos needs to somewhere on the top length of chain.
    // Rolling_elem_locs MUST be ordered from front-most to the rear,
    //  w.r.t. the chassis x-dir, so chain links created in a clockwise direction.
    m_chain->Initialize(m_chassis, m_chassis->GetFrame_REF_to_abs(), rolling_elem_locs, clearance,
                        rolling_elem_spin_axis, start_pos);

    // can set pin friction between adjoining shoes by activing damping on the DOF
    m_chain->Set_pin_friction(m_pin_damping);  // [N-s/m]

    // now, init the gear
    m_gear->Initialize(m_chassis, m_chassis->GetFrame_REF_to_abs(), ChCoordsys<>(), m_chain->GetShoeBody(),
                       dynamic_cast<ChTrackVehicle*>(this));

    // initialize the powertrain, drivelines
    m_ptrains[0]->Initialize(m_chassis, m_gear->GetAxle());

    // extra idlers?
    if (m_num_idlers > 1) {
        // init this idler, and position it so
        // it snaps into place
        double preload_2 = 60000;
        double idler2_xoff = -1.5;
        double idler2_yoff = -2.1;
        double rotation_ang = CH_C_PI_4;

        // get the absolute c-sys of the gear, and set position relative to that point.
        ChCoordsys<> idler2_csys(m_gear->GetBody()->GetPos(), m_gear->GetBody()->GetRot());
        idler2_csys.pos.x += idler2_xoff;
        idler2_csys.pos.y += idler2_yoff;
        // rotation should set the -x dir
        idler2_csys.rot = Q_from_AngAxis(rotation_ang, VECT_Z);

        // set the idler relative to the chassis/gear origin
        m_idlers[1]->Initialize(m_chassis, m_chassis->GetFrame_REF_to_abs(), idler2_csys, preload_2);
    }
}

void DriveChain::Update(double time, double throttle, double braking) {
    // update left and right powertrains, with the new left and right throttle/shaftspeed
    m_ptrains[0]->Update(time, throttle, GetDriveshaftSpeed(0));
}

void DriveChain::Advance(double step) {
    double t = 0;
    m_system->SetIterLCPmaxItersStab(100);
    m_system->SetIterLCPmaxItersSpeed(150);
    double settlePhaseA = 0.05;
    double settlePhaseB = 0.1;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        if (m_system->GetChTime() < settlePhaseA) {
            m_system->SetIterLCPmaxItersStab(80);
            m_system->SetIterLCPmaxItersSpeed(100);
        } else if (m_system->GetChTime() < settlePhaseB) {
            m_system->SetIterLCPmaxItersStab(100);
            m_system->SetIterLCPmaxItersSpeed(150);
        }
        m_system->DoStepDynamics(h);
        t += h;
    }
}

// Log constraint violations
// -----------------------------------------------------------------------------
void DriveChain::LogConstraintViolations() {
    GetLog().SetNumFormat("%16.4e");

    // Report constraint violations for the gear revolute joint
    GetLog() << "\n---- Gear constraint violations\n\n";
    m_gear->LogConstraintViolations();

    // report violations for idler constraints
    for (int id = 0; id < m_num_idlers; id++) {
        GetLog() << "\n---- idler # " << id << " constraint violations\n\n";
        m_idlers[id]->LogConstraintViolations();
    }

    // violations of the roller revolute joints
    for (int roller = 0; roller < m_num_wheels; roller++) {
        GetLog() << "\n---- Roller # " << roller << " constrain violations\n\n";
        m_wheels[roller]->LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}

// Write constraint violations of subsystems, in order, to the ostraem
// -----------------------------------------------------------------------------
void DriveChain::SaveConstraintViolations() {
    if (!m_log_file_exists) {
        std::cerr << "Must call Save_DebugLog() before trying to save the log data to file!!! \n\n\n";
    }
    double t = m_system->GetChTime();
    // call the subsystems in the same order as the headers are set up
    std::stringstream ss_g;
    ss_g << t;
    m_gear->SaveConstraintViolations(ss_g);
    ChStreamOutAsciiFile ofileGCV(m_filename_GCV.c_str(), std::ios::app);
    ofileGCV << ss_g.str().c_str();

    // report violations for idler constraints
    for (int id = 0; id < m_num_idlers; id++) {
        std::stringstream ss_id;
        ss_id << t;
        m_idlers[id]->SaveConstraintViolations(ss_id);
        ChStreamOutAsciiFile ofileICV(m_filename_ICV[id].c_str(), std::ios::app);
        ofileICV << ss_id.str().c_str();
    }

    // violations of the roller revolute joints
    for (int w_idx = 0; w_idx < m_num_wheels; w_idx++) {
        std::stringstream ss_r;
        ss_r << t;
        m_wheels[w_idx]->SaveConstraintViolations(ss_r);
        ChStreamOutAsciiFile ofileRCV(m_filename_RCV[w_idx].c_str(), std::ios::app);
        ofileRCV << ss_r.str().c_str();
    }

    // TODO: include the violations in the inter-shoe revolute constraints
}

// write output

// set what to save to file each time .DebugLog() is called during the simulation loop
// creates a new file (or overwrites old existing one), and sets the first row w/ headers
// for easy postprocessing with python pandas scripts
virtual void Setup_logger(int what_subsys,  /// which vehicle objects (e.g. subsystems) to save data for?
    int debug_type,   /// data types: _BODY, _CONSTRAINTS, _CONTACTS
    const std::string& out_filename,
    const std::string& data_dirname = "data_test")
{
    m_save_log_to_file = false;
    m_log_what_to_file = what_subsys;
    m_log_debug_type = debug_type;

    // create the directory for the data files
    if (ChFileutils::MakeDirectory(data_dirname.c_str()) < 0) {
        std::cout << "Error creating directory " << data_dirname << std::endl;
    }
    m_log_file_name = data_dirname + "/" + filename;

    // has the chain been created?
    if (m_chain) {
        // have the chain, the last subsystem created, been initialized?
        if (m_chain->Get_numShoes()) {
            m_save_log_to_file = true;
            GetLog() << " SAVING OUTPUT DATA TO FILE: \n " << filename.c_str() << "\n";
            create_fileHeaders(what);
            m_log_file_exists = true;

            // write the system heirarchy and ChSystem data also
            GetLog() << " SAVING model heirarchy and ChSystem details \n";
            ChStreamOutAsciiFile ofile_hier((m_log_file_name + "_Heirarchy.csv").c_str());
            m_system->ShowHierarchy(ofile_hier);
            ChStreamOutAsciiFile ofile_system((m_log_file_name + "_ChSystem.csv").c_str());
            m_system->StreamOUT(ofile_system);

        } else {
            GetLog() << " no shoes were initialized, not saving data ";
        }
    } else {
        GetLog() << " chain subsystem not created yet, not saving data";
    }

    // initialize the rig input values to zero
}

void DriveChain::Log_to_console(int console_what) {
    GetLog().SetNumFormat("%f");  //%10.4f");

    if (console_what & DBG_FIRSTSHOE) {
        // first shoe COG state data, and first pin force/torque
        GetLog() << "\n---- shoe 0 : " << m_chain->GetShoeBody(0)->GetName()
                 << "\n COG Pos [m] : " << m_chain->GetShoeBody(0)->GetPos()
                 << "\n COG Vel [m/s] : " << m_chain->GetShoeBody(0)->GetPos_dt()
                 << "\n COG Acc [m/s2] : " << m_chain->GetShoeBody(0)->GetPos_dtdt()
                 << "\n COG omega [rad/s] : " << m_chain->GetShoeBody(0)->GetRot_dt() << "\n";

        // shoe pin tension
        GetLog() << " pin 0 reaction force [N] : " << m_chain->GetPinReactForce(0) << "\n";
        GetLog() << "pin 0 reaction torque [N-m] : " << m_chain->GetPinReactTorque(0) << "\n";

        // shoe - gear contact details
        if (1) {
            //  specify some collision info with the gear
            std::vector<ChVector<> > sg_info;             // output data set
            std::vector<ChVector<> > Force_mag_info;      // contact forces, (Fn, Ft, 0),
            std::vector<ChVector<> > Ft_info;             // per step friction contact force statistics
            std::vector<ChVector<> > PosRel_contact;      // location of contact point, relative to gear c-sysss
            std::vector<ChVector<> > VRel_contact;        // velocity of contact point, relative to gear c-sys
            std::vector<ChVector<> > NormDirRel_contact;  // tracked contact normal dir., w.r.t. gear c-sys
            // sg_info = (Num_contacts, t_persist, t_persist_max)
            reportShoeGearContact(m_chain->GetShoeBody(0)->GetNameString(), sg_info, Force_mag_info, PosRel_contact,
                                  VRel_contact, NormDirRel_contact);

            GetLog() << "\n ---- Shoe - gear contact info, INDEX 0 :"
                     << "\n (# contacts, time_persist, t_persist_max) : " << sg_info[0]
                     << "\n force magnitude, (Fn, Ft, 0) : " << Force_mag_info[0]
                     << "\n contact point rel pos : " << PosRel_contact[0]
                     << "\n contact point rel vel : " << VRel_contact[0]
                     << "\n contact point rel norm. dir : " << NormDirRel_contact[0] << "\n";
        }
    }

    if (console_what & DBG_GEAR) {
        // gear state data, contact info
        GetLog() << "\n---- Gear : " << m_gear->GetBody()->GetName()
                 << "\n COG Pos [m] : " << m_gear->GetBody()->GetPos()
                 << "\n COG Vel [m/s] : " << m_gear->GetBody()->GetPos_dt()
                 << "\n COG omega [rad/s] : " << m_gear->GetBody()->GetRot_dt().Q_to_NasaAngles() << "\n";

        // find what's in contact with the gear by processing all collisions with a special callback function
        ChVector<> Fn_info = ChVector<>();
        ChVector<> Ft_info = ChVector<>();
        // info is: (max, avg., variance)
        int num_gear_contacts = reportGearContact(Fn_info, Ft_info);

        GetLog() << "\n     Gear Contact info"
                 << "\n # of contacts : " << num_gear_contacts << "\n normal (max, avg, variance) : " << Fn_info
                 << "\n tangent (max, avg, variance) : " << Ft_info << "\n";
    }

    if (console_what & DBG_IDLER) {
        GetLog() << "\n---- Idler : " << m_idlers[0]->GetBody()->GetName()
                 << "\n COG Pos [m] : " << m_idlers[0]->GetBody()->GetPos()
                 << "\n COG Vel [m/s] : " << m_idlers[0]->GetBody()->GetPos_dt()
                 << "\n COG omega [rad/s] : " << m_idlers[0]->GetBody()->GetRot_dt().Q_to_NasaAngles()
                 << "\n spring react F [N] : " << m_idlers[0]->GetSpringForce()
                 << "\n react from K [N] : " << m_idlers[0]->Get_SpringReact_Deform()
                 << "\n react from C [N] : " << m_idlers[0]->Get_SpringReact_Deform_dt();
    }

    if (console_what & DBG_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }

    if (console_what & DBG_PTRAIN) {
        GetLog() << "\n ---- powertrain \n throttle : " << m_ptrains[0]->GetThrottle()
                 << "\n motor speed [RPM] : " << m_ptrains[0]->GetMotorSpeed() * 60.0 / (CH_C_2PI)
                 << "\n motor torque [N-m] : " << m_ptrains[0]->GetMotorTorque()
                 << "\n output torque [N-m : " << m_ptrains[0]->GetOutputTorque() << "\n";
    }

    if (console_what & DBG_COLLISIONCALLBACK & (GetCollisionCallback() != NULL)) {
        GetLog() << "\n ---- collision callback info :"
                 << "\n Contacts this step: " << GetCollisionCallback()->GetNcontacts()
                 << "\n Broadphase passed this step: " << GetCollisionCallback()->GetNbroadPhasePassed()
                 << "\n Sum contacts, +z side: " << GetCollisionCallback()->Get_sum_Pz_contacts()
                 << "\n Sum contacts, -z side: " << GetCollisionCallback()->Get_sum_Nz_contacts() << "\n";
    }

    if (console_what & DBG_ALL_CONTACTS) {
        // use the reporter class with the console log
        _contact_reporter m_report(GetLog());
        // add it to the system, should be called next timestep
        m_system->GetContactContainer()->ReportAllContacts(&m_report);
    }

    GetLog().SetNumFormat("%g");
}

// save info to file. Must have already called Setup_log_to_file  once before entering time stepping loop
void DriveChain::Log_to_file() {
    // told to save the data?
    if (m_save_log_to_file) {
        if (!m_log_file_exists) {
            std::cerr << "Must call Save_DebugLog() before trying to save the log data to file!!! \n\n\n";
        }
        // open the file to append
        // open the data file for writing the header

        // write the simulation time
        double t = m_system->GetChTime();

        // python pandas expects csv w/ no whitespace
        if (m_log_what_to_file & DBG_FIRSTSHOE) {
            std::stringstream ss;
            // time,x,y,z,vx,vy,vz,ax,ay,az,wx,wy,wz,fx,fy,fz
            ss << t << "," << m_chain->GetShoeBody(0)->GetPos() << "," << m_chain->GetShoeBody(0)->GetPos_dt() << ","
               << m_chain->GetShoeBody(0)->GetPos_dtdt() << "," << m_chain->GetShoeBody(0)->GetWvel_loc() << ","
               << m_chain->GetPinReactForce(0) << "," << m_chain->GetPinReactTorque(0) << "\n";
            // open the file for appending, write the data.
            ChStreamOutAsciiFile ofile(m_filename_DBG_FIRSTSHOE.c_str(), std::ios::app);
            ofile << ss.str().c_str();

            // second file, to specify some collision info with the gear
            double num_contacts = 0;
            std::vector<ChVector<> > sg_info;         // output data set
            std::vector<ChVector<> > Force_mag_info;  // per step contact force magnitude, (Fn, Ft, 0)
            std::vector<ChVector<> > PosRel_contact;  // location of a contact point relative to the gear c-sys
            std::vector<ChVector<> > VRel_contact;    // follow the vel. of a contact point relative to the gear c-sys
            std::vector<ChVector<> > NormDirRel_contact;  // tracked contact normal dir., w.r.t. gear c-sys
            // sg_info = (Num_contacts, t_persist, t_persist_max)
            num_contacts = reportShoeGearContact(m_chain->GetShoeBody(0)->GetNameString(), sg_info, Force_mag_info,
                                                 PosRel_contact, VRel_contact, NormDirRel_contact);

            // suffix "P" is the z-positive side of the gear, "N" is the z-negative side
            // "time,Ncontacts,t_persistP,t_persist_maxP,FnMagP,FtMagP,xRelP,yRelP,zRelP,VxRelP,VyRelP,VzRelP,normDirRelxP,normDirRelyP,normDirRelzP
            //  ,t_persistN,t_persist_maxN,FnMagN,FtMagN,xRelN,yRelN,zRelN,VxRelN,VyRelN,VzRelN,normDirRelxN,normDirRelyN,normDirRelzN
            //  "
            std::stringstream ss_sg;
            ss_sg << t << "," << num_contacts << "," << sg_info[0] << "," << Force_mag_info[0].x << ","
                  << Force_mag_info[0].y << "," << PosRel_contact[0] << "," << VRel_contact[0] << ","
                  << NormDirRel_contact[0] << "," << sg_info[1] << "," << Force_mag_info[1].x << ","
                  << Force_mag_info[1].y << "," << PosRel_contact[1] << "," << VRel_contact[1] << ","
                  << NormDirRel_contact[1] << "\n";
            ChStreamOutAsciiFile ofile_shoeGear(m_filename_DBG_shoeGear.c_str(), std::ios::app);
            ofile_shoeGear << ss_sg.str().c_str();
        }
        if (m_log_what_to_file & DBG_GEAR) {
            std::stringstream ss_g;
            // time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz
            ss_g << t << "," << m_gear->GetBody()->GetPos() << "," << m_gear->GetBody()->GetPos_dt() << ","
                 << m_gear->GetBody()->GetWvel_loc() << "\n";
            ChStreamOutAsciiFile ofileDBG_GEAR(m_filename_DBG_GEAR.c_str(), std::ios::app);
            ofileDBG_GEAR << ss_g.str().c_str();

            // second file, for the specific contact info
            std::stringstream ss_gc;

            // find what's in contact with the gear by processing all collisions with a special callback function
            ChVector<> Fn_info = ChVector<>();
            ChVector<> Ft_info = ChVector<>();
            // info is: (max, avg., variance)
            int num_gear_contacts = reportGearContact(Fn_info, Ft_info);
            // time,Ncontacts,FnMax,FnAvg,FnVar,FtMax,FtAvg,FtVar
            ss_gc << t << "," << num_gear_contacts << "," << Fn_info << "," << std::sqrt(Fn_info.z) << "," << Ft_info
                  << "," << std::sqrt(Ft_info.z) << "\n";
            ChStreamOutAsciiFile ofileDBG_GEAR_CONTACT(m_filename_DBG_GEAR_CONTACT.c_str(), std::ios::app);
            ofileDBG_GEAR_CONTACT << ss_gc.str().c_str();
        }

        if (m_log_what_to_file & DBG_IDLER) {
            std::stringstream ss_id;
            // time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz,F_tensioner,F_k,F_c
            ss_id << t << "," << m_idlers[0]->GetBody()->GetPos() << "," << m_idlers[0]->GetBody()->GetPos_dt() << ","
                  << m_idlers[0]->GetBody()->GetWvel_loc() << "," << m_idlers[0]->GetSpringForce() << ","
                  << m_idlers[0]->Get_SpringReact_Deform() << "," << m_idlers[0]->Get_SpringReact_Deform_dt() << "\n";
            ChStreamOutAsciiFile ofileDBG_IDLER(m_filename_DBG_IDLER.c_str(), std::ios::app);
            ofileDBG_IDLER << ss_id.str().c_str();
        }

        if (m_log_what_to_file & DBG_CONSTRAINTS) {
            // Report constraint violations for all joints
            SaveConstraintViolations();
        }

        if (m_log_what_to_file & DBG_PTRAIN) {
            std::stringstream ss_pt;
            // motor speed, mot torque, out torque
            ss_pt << t << "," << m_ptrains[0]->GetThrottle() << ","
                  << m_ptrains[0]->GetMotorSpeed() * 60.0 / (CH_C_2PI)  // RPM
                  << "," << m_ptrains[0]->GetMotorTorque() << "," << m_ptrains[0]->GetOutputTorque() << "\n";
            ChStreamOutAsciiFile ofilePT(m_filename_DBG_PTRAIN.c_str(), std::ios::app);
            ofilePT << ss_pt.str().c_str();
        }

        if (m_log_what_to_file & DBG_COLLISIONCALLBACK & (GetCollisionCallback() != NULL)) {
            std::stringstream ss_cc;
            // report # of contacts detected this step between shoe pins # gear.
            // time,Ncontacts,Nbroadphase,NcPz,NcNz
            ss_cc << t << "," << GetCollisionCallback()->GetNcontacts() << ","
                  << GetCollisionCallback()->GetNbroadPhasePassed() << ","
                  << GetCollisionCallback()->Get_sum_Pz_contacts() << ","
                  << GetCollisionCallback()->Get_sum_Nz_contacts() << "\n";
            ChStreamOutAsciiFile ofileCCBACK(m_filename_DBG_COLLISIONCALLBACK.c_str(), std::ios::app);
            ofileCCBACK << ss_cc.str().c_str();
        }

        if (m_log_what_to_file & DBG_ALL_CONTACTS) {
            // use the reporter class with the console log
            std::stringstream ss_fn;  // filename
            ss_fn << m_filename_DBG_ALL_CONTACTS << m_cnt_Log_to_file << ".csv";

            // new file created for each step
            ChStreamOutAsciiFile ofileContacts(ss_fn.str().c_str());

            // write headers to the file first
            std::stringstream ss_header;
            ss_header << "bodyA,bodyB,pAx,pAy,pAz,pBx,pBy,pBz,Nx,Ny,Nz,dist,Fx,Fy,Fz\n";
            ofileContacts << ss_header.str().c_str();

            _contact_reporter m_report(ofileContacts);
            // add it to the system, should be called next timestep
            m_system->GetContactContainer()->ReportAllContacts(&m_report);
        }

        // increment step number
        m_cnt_Log_to_file++;
    }
}

void DriveChain::create_fileHeaders(int what) {
    // creating files, reset counter of # of times Log_to_file was called
    m_cnt_Log_to_file = 0;

    GetLog() << " ------ Output Data ------------ \n\n";

    if (what & DBG_FIRSTSHOE) {
        // Shoe 0 : S0, Pin0: P0
        m_filename_DBG_FIRSTSHOE = m_log_file_name + "_shoe0.csv";
        ChStreamOutAsciiFile ofileDBG_FIRSTSHOE(m_filename_DBG_FIRSTSHOE.c_str());
        std::stringstream ss;
        ss << "time,x,y,z,Vx,Vy,Vz,Ax,Ay,Az,Wx,Wy,Wz,Fx,Fy,Fz,Tx,Ty,Tz\n";
        ofileDBG_FIRSTSHOE << ss.str().c_str();
        GetLog() << " writing to file: " << m_filename_DBG_FIRSTSHOE << "\n         data: " << ss.str().c_str() << "\n";

        // report the contact with the gear in a second file
        m_filename_DBG_shoeGear = m_log_file_name + "_shoe0GearContact.csv";
        ChStreamOutAsciiFile ofileDBG_shoeGear(m_filename_DBG_shoeGear.c_str());
        std::stringstream ss_sg;
        // suffix "P" is the z-positive side of the gear, "N" is the z-negative side
        ss_sg << "time,Ncontacts,NcontactsP,t_persistP,t_persist_maxP,FnMagP,FtMagP,xRelP,yRelP,zRelP,VxRelP,VyRelP,"
                 "VzRelP,normDirRelxP,normDirRelyP,normDirRelzP"
              << ",NcontactsN,t_persistN,t_persist_maxN,FnMagN,FtMagN,xRelN,yRelN,zRelN,VxRelN,VyRelN,VzRelN,"
                 "normDirRelxN,normDirRelyN,normDirRelzN\n";
        ofileDBG_shoeGear << ss_sg.str().c_str();
    }

    if (what & DBG_GEAR) {
        m_filename_DBG_GEAR = m_log_file_name + "_gear.csv";
        ChStreamOutAsciiFile ofileDBG_GEAR(m_filename_DBG_GEAR.c_str());
        std::stringstream ss_g;
        ss_g << "time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz\n";
        ofileDBG_GEAR << ss_g.str().c_str();
        GetLog() << " writing to file: " << m_filename_DBG_GEAR << "\n          data: " << ss_g.str().c_str() << "\n";

        // report on some specific collision info in a separate file
        m_filename_DBG_GEAR_CONTACT = m_log_file_name + "_gearContact.csv";
        ChStreamOutAsciiFile ofileDBG_GEAR_CONTACT(m_filename_DBG_GEAR_CONTACT.c_str());
        std::stringstream ss_gc;
        ss_gc << "time,Ncontacts,FnMax,FnAvg,FnVar,FnSig,FtMax,FtAvg,FtVar,FtSig\n";
        ofileDBG_GEAR_CONTACT << ss_gc.str().c_str();
        GetLog() << " writing to file : " << m_filename_DBG_GEAR_CONTACT << "\n          data: " << ss_gc.str().c_str()
                 << "\n";
    }

    if (what & DBG_IDLER) {
        m_filename_DBG_IDLER = m_log_file_name + "_idler.csv";
        ChStreamOutAsciiFile ofileDBG_IDLER(m_filename_DBG_IDLER.c_str());
        std::stringstream ss_id;
        ss_id << "time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz,F_tensioner,F_k,F_c\n";
        ofileDBG_IDLER << ss_id.str().c_str();
        GetLog() << " writing to file: " << m_filename_DBG_IDLER << "\n          data:" << ss_id.str().c_str() << "\n";
    }

    // write the data for each subsystem's constraint violation
    if (what & DBG_CONSTRAINTS) {
        // in the same order as listed in the header
        m_filename_GCV = m_log_file_name + "_GearCV.csv";
        ChStreamOutAsciiFile ofileGCV(m_filename_GCV.c_str());
        std::stringstream ss_gCV;
        ss_gCV << m_gear->getFileHeader_ConstraintViolations(0);
        ofileGCV << ss_gCV.str().c_str();
        GetLog() << " writing to file: " << m_filename_GCV << "\n          data: " << ss_gCV.str().c_str() << "\n";

        for (int id = 0; id < m_num_idlers; id++) {
            std::stringstream ss_iCV;
            ss_iCV << m_log_file_name << "_idler" << id << "CV.csv";
            m_filename_ICV.push_back(ss_iCV.str().c_str());
            ChStreamOutAsciiFile ofileICV(m_filename_ICV.back().c_str());
            std::stringstream ss_header;
            ss_header << m_idlers[id]->getFileHeader_ConstraintViolations();
            ofileICV << ss_header.str().c_str();
            GetLog() << " writing to file: " << m_filename_ICV[id] << "\n          data: " << ss_header.str().c_str()
                     << "\n";
        }

        // violations of the roller revolute joints
        for (int roller = 0; roller < m_num_wheels; roller++) {
            std::stringstream ss_rCV;
            ss_rCV << m_log_file_name << "_roller" << roller << "CV.csv";
            m_filename_RCV.push_back(ss_rCV.str());
            ChStreamOutAsciiFile ofileRCV(m_filename_RCV.back().c_str());
            std::stringstream ss_header;
            ofileRCV << ss_rCV.str().c_str();
            GetLog() << " writing to file: " << m_filename_RCV[roller] << "\n         data: " << ss_header.str().c_str()
                     << "\n";
        }
    }

    // write powertrian headers
    if (what & DBG_PTRAIN) {
        m_filename_DBG_PTRAIN = m_log_file_name + "_ptrain.csv";
        ChStreamOutAsciiFile ofileDBG_PTRAIN(m_filename_DBG_PTRAIN.c_str());
        std::stringstream ss_pt;
        ss_pt << "time,throttle,motSpeed,motT,outT\n";
        ofileDBG_PTRAIN << ss_pt.str().c_str();
        GetLog() << " writing to file: " << m_filename_DBG_PTRAIN << "\n          data: " << ss_pt.str().c_str()
                 << "\n";
    }

    // write broadphase, narrow phase contact info
    if (what & DBG_COLLISIONCALLBACK & (GetCollisionCallback() != NULL)) {
        m_filename_DBG_COLLISIONCALLBACK = m_log_file_name + "_Ccallback.csv";
        ChStreamOutAsciiFile ofileDBG_COLLISIONCALLBACK(m_filename_DBG_COLLISIONCALLBACK.c_str());
        std::stringstream ss_cc;
        ss_cc << "time,Ncontacts,Nbroadphase,NcPz,NcNz\n";
        ofileDBG_COLLISIONCALLBACK << ss_cc.str().c_str();
        GetLog() << " writing to file: " << m_filename_DBG_COLLISIONCALLBACK << "\n     data: " << ss_cc.str().c_str()
                 << "\n";
    }

    // write all contact info to a new file each step
    if (what & DBG_ALL_CONTACTS) {
        m_filename_DBG_ALL_CONTACTS = m_log_file_name + "_allContacts";
        // write a file each step, so we'll write a header then.
        GetLog() << " writing contact info to file name: " << m_filename_DBG_ALL_CONTACTS << "\n\n";
    }
}

}  // end namespace chrono
