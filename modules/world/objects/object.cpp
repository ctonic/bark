// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/world/objects/object.hpp"

namespace modules {
namespace world {
namespace objects {

AgentId Object::agent_count = 0;

Object::Object(const geometry::Polygon& shape,
               commons::Params* params,
               const geometry::Model3D& model_3d,
               const geometry::Point2d& point) : 
               // "Point2d" will be replaced by "State" later
  BaseType(params),
  shape_(shape),
  model_3d_(model_3d),
  location_(point),
  agent_id_(agent_count++) {}

Object* Object::Clone() const {
  Object* new_Object = new Object(*this);
  return new_Object;
}

} // objects
} // world
} // modules