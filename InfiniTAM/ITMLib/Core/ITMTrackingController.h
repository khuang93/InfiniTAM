// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdexcept>
#include <vector>
#include <src/ObjSLAM/ObjSLAMDataTypes.h>

#include "../Engines/Visualisation/Interface/ITMSurfelVisualisationEngine.h"
#include "../Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "../Trackers/Interface/ITMTracker.h"
#include "../Utils/ITMLibSettings.h"


namespace ITMLib {
/** \brief
*/
class ITMTrackingController {
 private:
  const ITMLibSettings *settings;
  ITMTracker *tracker;

 public:
  void Track(ITMTrackingState *trackingState, const ITMView *view) {
    tracker->TrackCamera(trackingState, view);
  }

  template<typename TSurfel>
  void Prepare(ITMTrackingState *trackingState, const ITMSurfelScene<TSurfel> *scene, const ITMView *view,
               const ITMSurfelVisualisationEngine<TSurfel> *visualisationEngine, ITMSurfelRenderState *renderState) {
    if (!tracker->requiresPointCloudRendering())
      return;

    //render for tracking
    bool requiresColourRendering = tracker->requiresColourRendering();
    bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;

    if (requiresColourRendering) {
      // TODO: This should be implemented at some point.
      throw std::runtime_error("The surfel engine doesn't yet support colour trackers");
    } else {
      const bool useRadii = true;
      visualisationEngine->FindSurface(scene,
                                       trackingState->pose_d,
                                       &view->calib.intrinsics_d,
                                       useRadii,
                                       USR_FAUTEDEMIEUX,
                                       renderState);
      trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

      if (requiresFullRendering) {
        visualisationEngine->CreateICPMaps(scene, renderState, trackingState);
        trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
        if (trackingState->age_pointCloud == -1) trackingState->age_pointCloud = -2;
        else trackingState->age_pointCloud = 0;
      } else {
        trackingState->age_pointCloud++;
      }
    }
  }

  template<typename TVoxel, typename TIndex>
  void Prepare(ITMTrackingState *trackingState, const ITMScene<TVoxel, TIndex> *scene, const ITMView *view,
               const ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine, ITMRenderState *renderState) {
    if (!tracker->requiresPointCloudRendering())
      return;

    //render for tracking
    bool requiresColourRendering = tracker->requiresColourRendering();
    bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;

    if (requiresColourRendering) {
      ORUtils::SE3Pose pose_rgb(view->calib.trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM());
      visualisationEngine->CreateExpectedDepths(scene, &pose_rgb, &(view->calib.intrinsics_rgb), renderState);
      visualisationEngine->CreatePointCloud(scene, view, trackingState, renderState, settings->skipPoints);
      trackingState->age_pointCloud = 0;
    } else {

      visualisationEngine->CreateExpectedDepths(scene, trackingState->pose_d, &(view->calib.intrinsics_d), renderState);

      if (requiresFullRendering) {
        visualisationEngine->CreateICPMaps(scene, view, trackingState, renderState);
        trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
        if (trackingState->age_pointCloud == -1) trackingState->age_pointCloud = -2;
        else trackingState->age_pointCloud = 0;
      } else {
        visualisationEngine->ForwardRender(scene, view, trackingState, renderState);
        trackingState->age_pointCloud++;
      }
    }
  }

  template<typename TVoxel, typename TIndex>
  void Prepare(ITMTrackingState *trackingState,
               ITMRenderState *renderState,
               std::vector<ObjSLAM::ObjectInstance_ptr<TVoxel, TIndex>> obj_inst_ptr_vector,
               const ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine){
    if (!tracker->requiresPointCloudRendering())
      return;
    //render for tracking
    bool requiresColourRendering = tracker->requiresColourRendering();
    bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;

    for (size_t i = 0; i < obj_inst_ptr_vector.size(); ++i) {
      sceneIsBackground = i == 0 ? true : false;
      ObjSLAM::ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr = obj_inst_ptr_vector.at(i);

      const auto *scene = obj_inst_ptr->getScene().get();
      const auto *view = obj_inst_ptr->getCurrentView().get();

      if (requiresFullRendering) {
        visualisationEngine->CreateICPMaps(scene, view, trackingState, renderState);
        trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
      } else {
        visualisationEngine->ForwardRender(scene, view, trackingState, renderState);
        trackingState->age_pointCloud++;
      }
    }
    if (requiresFullRendering) {
      if (trackingState->age_pointCloud == -1) trackingState->age_pointCloud = -2;
      else trackingState->age_pointCloud = 0;
    }
  }

  //TODO New Prepare : this only works when first scene is BG, check why
  template<typename TVoxel, typename TIndex>
  void Prepare(ITMTrackingState *trackingState,
               ITMRenderState *renderState,
               std::vector<ITMScene<TVoxel, TIndex> *> scene_vec,
               std::vector<ITMView *> view_vec,
               const ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine) {

//    std::cout << "requiresPointCloudRendering"<<tracker->requiresPointCloudRendering();
    if (!tracker->requiresPointCloudRendering())
      return;
    //render for tracking
    bool requiresColourRendering = tracker->requiresColourRendering();
    bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;
//    std::cout << "requiresColourRendering"<<requiresColourRendering<<"requiresFullRendering"<<requiresFullRendering;
    if (scene_vec.size() != view_vec.size()) {
      return;
    }

    //cannot really add OMP because shared tracking and renderstate
    for (size_t i = 0; i < scene_vec.size() && i < view_vec.size(); ++i) {
      sceneIsBackground = i == 0 ? true : false;
      auto *scene = scene_vec.at(i);
      auto *view = view_vec.at(i);
//      Prepare(trackingState,scene,view, visualisationEngine,renderState);
      if (requiresFullRendering) {
        visualisationEngine->CreateICPMaps(scene, view, trackingState, renderState);
        trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
      } else {
        visualisationEngine->ForwardRender(scene, view, trackingState, renderState);
        trackingState->age_pointCloud++;
      }
    }
    if (requiresFullRendering) {
      if (trackingState->age_pointCloud == -1) trackingState->age_pointCloud = -2;
      else trackingState->age_pointCloud = 0;
    }
  }


  ITMTrackingController(ITMTracker *tracker, const ITMLibSettings *settings) {
    this->tracker = tracker;
    this->settings = settings;
  }

  const Vector2i &GetTrackedImageSize(const Vector2i &imgSize_rgb, const Vector2i &imgSize_d) const {
    return tracker->requiresColourRendering() ? imgSize_rgb : imgSize_d;
  }

  // Suppress the default copy constructor and assignment operator
  ITMTrackingController(const ITMTrackingController &);
  ITMTrackingController &operator=(const ITMTrackingController &);
};
}
