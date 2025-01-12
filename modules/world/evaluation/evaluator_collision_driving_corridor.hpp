// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_COLLISION_COLLISION_CHECK_DRIVING_CORRIDOR_HPP_
#define MODULES_WORLD_COLLISION_COLLISION_CHECK_DRIVING_CORRIDOR_HPP_

#include "modules/world/evaluation/base_evaluator.hpp"


namespace modules
{
namespace world
{
class World;

namespace evaluation
{

class EvaluatorCollisionDrivingCorridor : public BaseEvaluator
{
   public:
    EvaluatorCollisionDrivingCorridor() {}
    virtual ~EvaluatorCollisionDrivingCorridor() {}

    virtual EvaluationReturn Evaluate(const world::World& world) const;

};

} // namespace collision
} // namespace world
} // namespace modules

#endif // MODULES_WORLD_COLLISION_COLLISION_CHECK_DRIVING_CORRIDOR_HPP_
