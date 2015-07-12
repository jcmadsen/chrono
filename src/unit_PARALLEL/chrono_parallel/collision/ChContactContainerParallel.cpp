#include "chrono_parallel/collision/ChContactContainerParallel.h"

#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChParticlesClones.h"
#include "lcp/ChLcpConstraintTwoContactN.h"
#include "collision/ChCModelBulletBody.h"
#include "collision/ChCModelBulletParticle.h"

namespace chrono {

using namespace collision;
using namespace geometry;

ChContactContainerParallel::ChContactContainerParallel(ChParallelDataManager* dc) : data_manager(dc) {
  n_added = 0;
}

ChContactContainerParallel::~ChContactContainerParallel() {
  n_added = 0;
}

}  // END_OF_NAMESPACE____
