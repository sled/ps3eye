/*
 * stereo_exporter.h
 *
 *  Created on: Mar 31, 2013
 *      Author: sled
 */

#ifndef STEREO_EXPORTER_H_
#define STEREO_EXPORTER_H_

#include <boost/thread.hpp>

#include <queue>
#include "stereo_snapshot.h"

namespace stereo_extractor {

// base class for exporter

class StereoExporter {
public:

  virtual void exportSnapshot(StereoSnapshotConstPtr&) = 0;

  // make sure to use the right destructor!
  virtual ~StereoExporter() {}
};


}

#endif /* STEREO_EXPORTER_H_ */
