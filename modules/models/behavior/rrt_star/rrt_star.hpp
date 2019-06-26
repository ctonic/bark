// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_RRT_STAR_RRT_STAR_HPP_
#define MODULES_MODELS_BEHAVIOR_RRT_STAR_RRT_STAR_HPP_

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace models {
namespace behavior {

using dynamic::Trajectory;
using world::objects::AgentId;
using world::ObservedWorld;

class BehaviorRRTStar : public BehaviorModel {
 public:
  explicit BehaviorRRTStar(commons::Params *params) :
    BehaviorModel(params) {}

  virtual ~BehaviorRRTStar() {}

  Trajectory Plan(float delta_time,
                 const ObservedWorld& observed_world);

  virtual BehaviorModel *Clone() const;

 private:
    geometry::Point2d sample_point();
};

inline BehaviorModel *BehaviorRRTStar::Clone() const {
  return new BehaviorRRTStar(*this);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_RRT_STAR_RRT_STAR_HPP_
