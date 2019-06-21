// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_OPENDRIVE_PLAN_VIEW_HPP_
#define MODULES_WORLD_OPENDRIVE_PLAN_VIEW_HPP_

#include <Eigen/Core>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "modules/geometry/commons.hpp"
#include "modules/geometry/line.hpp"
#include "modules/world/opendrive/lane.hpp"

namespace modules {
namespace world {
namespace opendrive {

class PlanView {
 public:
  PlanView() : length_(0.0) {}
  ~PlanView() {}

  geometry::Line create_line(int id, LaneWidth lane_width, float s_inc = 0.5f);
  LanePtr create_lane(LanePosition lane_position, LaneWidths lane_widths, float s_inc = 2.0f);

  //! setter functions
  // OpenDRIVE specification: http://www.opendrive.org/docs/OpenDRIVEFormatSpecRev1.5M.pdf
  // good text explaining line, arc etc.: https://gupea.ub.gu.se/bitstream/2077/23047/1/gupea_2077_23047_1.pdf
  bool add_line(geometry::Point2d start_point, float heading, float length);

  bool add_spiral(geometry::Point2d start_point, float heading, float length, float curvStart, float curvEnd, float s_inc = 2.0f);
  
  bool add_arc(geometry::Point2d start_point, float heading, float length, float curvature, float s_inc = 2.0f);
  void calc_arc_position(const float s, float initial_heading, float curvature, float &dx, float &dy);

  // TODO: add support for the pRange parameter (generalization)
  // TODO: add function add_poly3
  bool add_paramPoly3(geometry::Point2d start_point, float heading, float length, float aU, float bU, float cU, float dU, float aV, float bV, float cV, float dV, float s_inc = 0.1f);


  //! getter functions
  geometry::Line get_reference_line() const { return reference_line_; }

  geometry::Point2d test(geometry::Point2d p) { return p; }

  float get_length() const { return length_; }
  float get_distance( const geometry::Point2d &p) const { return boost::geometry::distance(reference_line_.obj_, p); }

 private:
  geometry::Line reference_line_;  // sequential build up
  float length_;
};

using PlanViewPtr = std::shared_ptr<PlanView>;

}  // namespace opendrive
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_OPENDRIVE_PLAN_VIEW_HPP_
