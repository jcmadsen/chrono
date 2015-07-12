#ifndef CHCONTACTCONTAINERPARALLEL_H
#define CHCONTACTCONTAINERPARALLEL_H

#include <list>

#include "physics/ChContactContainer.h"

#include "chrono_parallel/ChApiParallel.h"
#include "chrono_parallel/ChDataManager.h"

namespace chrono {
/// Class representing a container of many contacts,
/// implemented as a typical linked list of ChContactGPUsimple
/// objects.
/// This contact container must be used for the preliminar CUDA solver
/// that was developed by Ale & Dan, but in future will be
/// replaced by ChContactContainerGPU, and advanced container
/// that does not use linked lists of cpu objects but rather
/// keeps all contact data as GPU buffers on the GPU device.

class CH_PARALLEL_API ChContactContainerParallel : public ChContactContainer {
  CH_RTTI(ChContactContainerParallel, ChContactContainer);

 public:
  ChContactContainerParallel(ChParallelDataManager* dc);

  virtual ~ChContactContainerParallel();
  virtual int GetNcontacts() { return data_manager->num_rigid_contacts; }

  ChParallelDataManager* data_manager;
};
}

#endif
