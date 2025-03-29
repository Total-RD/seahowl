#pragma once

#include "seahowl/core/simulation.h"
#include "seahowl/io/output_manager.h"
#include "seahowl/core/system.h"
#include "seahowl/aero/system_aero.h"
#include "seahowl/elasto/system_elasto.h"

#include <memory>
#include <string>

namespace seahowl {
namespace core {

struct OpFM_InputType {
    float* pxVel;
    int pxVel_Len;  // x position of velocity interface (seahowl) nodes [m]
    float* pyVel;
    int pyVel_Len;  // y position of velocity interface (seahowl) nodes [m]
    float* pzVel;
    int pzVel_Len;  // z position of velocity interface (seahowl) nodes [m]
    float* pxForce;
    int pxForce_Len;  // x position of actuator force nodes [m]
    float* pyForce;
    int pyForce_Len;  // y position of actuator force nodes [m]
    float* pzForce;
    int pzForce_Len;  // z position of actuator force nodes [m]
    float* xdotForce;
    int xdotForce_Len;  // x velocity of actuator force nodes [m]
    float* ydotForce;
    int ydotForce_Len;  // y velocity of actuator force nodes [m]
    float* zdotForce;
    int zdotForce_Len;  // z velocity of actuator force nodes [m]
    float* pOrientation;
    int pOrientation_Len;  // Direction cosine matrix to transform vectors from global frame of reference to actuator
                           // force node frame of reference [-]

    float* fx;
    int fx_Len;  // normalized x force at actuator force nodes [N/kg/m^3]
    float* fy;
    int fy_Len;  // normalized y force at actuator force nodes [N/kg/m^3]
    float* fz;
    int fz_Len;  // normalized z force at actuator force nodes [N/kg/m^3]
    float* momentx;
    int momentx_Len;  // normalized x moment at actuator force nodes [Nm/kg/m^3]
    float* momenty;
    int momenty_Len;  // normalized y moment at actuator force nodes [Nm/kg/m^3]
    float* momentz;
    int momentz_Len;  // normalized z moment at actuator force nodes [Nm/kg/m^3]
    float* forceNodesChord;
    int forceNodesChord_Len;  // chord distribution at the actuator force nodes [m]
};

struct OpFM_OutputType {
    float* u;
    int u_Len;  // x velocity at interface (seahowl) nodes [m]
    float* v;
    int v_Len;  // y velocity at interface (seahowl) nodes [m]
    float* w;
    int w_Len;  // z velocity at interface (seahowl) nodes [m]
    float* WriteOutput;
    int WriteOutput_Len;  // Data to be written to an output file: see WriteOutputHdr for names of each variable [see
                          // WriteOutputUnt]
};

struct SC_DX_InputType {
    float* toSCglob;
    int toSCglob_Len;
    float* toSC;
    int toSC_Len;
};

struct SC_DX_OutputType {
    float* fromSCglob;
    int fromSCglob_Len;
    float* fromSC;
    int fromSC_Len;
};

class AmrWindAdapter {
  public:
    std::unique_ptr<seahowl::core::Simulation> simulation;

    double dt = 0.025;
    double dt_output = 0.;
    double duration = 1000.0;
    bool is_initialized = false;

    // air density to normailze the loads sent to AMR-Wind
    double airDens = 1.225;

    // number of blade
    int numBlade = 3;

    // number of aero distretisation on seahowl
    int numBladeNode = 50;
    int numTowerNode = 10;

    double bladeLength = 120.0;
    double towerHeight = 150.0;
    double towerBaseHeight = 15.0;

    // number of mapping
    int nMappings;

    // number of velocity nodes (seahowl nodes: hub + blade + tower)
    int nNodesVel;

    // number of actuator force nodes (amrwind nodes)
    int nNodesForce;

    // location of actuator force nodes on blade
    float* forceBldRnodes;
    int forceBldRnodes_Len;

    // location of actuator force nodes on tower
    float* forceTwrHnodes;
    int forceTwrHnodes_Len;

    AmrWindAdapter();

    // initialize seahowl turbine from input file
    void initialize_from_file(const std::string& filepath);

    // initialize CFD data structure
    void init_OpFM(int* numActForcePtsBlade,
                   int* numActForcePtsTower,
                   seahowl::core::OpFM_InputType* to_cfd,
                   seahowl::core::OpFM_OutputType* from_cfd);

    // Interpolate the chord distribution to the force nodes
    void InterpolateForceNodesChord(seahowl::core::OpFM_InputType* to_cfd);

    // Create the actuator line force point mesh
    // to do list
    void CreateActForceMotionsMesh();

    // set the positions
    void SetOpFMPositions(seahowl::core::OpFM_InputType* to_cfd, seahowl::core::OpFM_OutputType* from_cfd);

    // set the forces
    void SetOpFMForces(seahowl::core::OpFM_InputType* to_cfd, seahowl::core::OpFM_OutputType* from_cfd);

    // step function to move forward to next time step
    void step(seahowl::core::OpFM_InputType* to_cfd, seahowl::core::OpFM_OutputType* from_cfd);

    // void send_to_cfd(seahowl::core::OpFM_InputType to_cfd);
    // void get_from_cfd(seahowl::core::OpFM_OutputType from_cfd);

  private:
    int nstep = 0;
    std::string main_filepath;

    // Initialize array required by AMR-Wind
    void AllocPAry(float*& array, int size, const std::string& name);

    // Create the blade and tower nodes
    void CreateActForceBladeTowerNodes();

    // Create actuator point motion mesh
    // void CreateActForceMotionsMesh();
};

}  // namespace core
}  // namespace seahowl
