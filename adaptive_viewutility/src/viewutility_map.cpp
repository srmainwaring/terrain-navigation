/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Adaptive view utility estimation node
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "adaptive_viewutility/viewutility_map.h"
#include "terrain_navigation/profiler.h"

#include "grid_map_cv/grid_map_cv.hpp"
#include "grid_map_pcl/GridMapPclConverter.hpp"

#include <pcl/io/obj_io.h>
#include <fstream>
#include <iostream>

ViewUtilityMap::ViewUtilityMap(grid_map::GridMap &grid_map) : grid_map_(grid_map) {
  // Initialize gridmap layers
  grid_map_.add("visibility");
  grid_map_.add("geometric_prior");
  grid_map_.add("normalized_prior");
  grid_map_.add("roi");
  grid_map_.add("elevation_normal_x");
  grid_map_.add("elevation_normal_y");
  grid_map_.add("elevation_normal_z");
  grid_map_.add("ground_sample_distance");
  grid_map_.add("incident_prior");
  grid_map_.add("triangulation_prior");
  grid_map_.add("min_eigen_value");

  grid_map_["visibility"].setConstant(0);
  grid_map_["geometric_prior"].setConstant(limit_cramerrao_bounds);
  grid_map_["normalized_prior"].setConstant(0);
  grid_map_["roi"].setConstant(1.0);
  grid_map_["elevation_normal_x"].setConstant(0);
  grid_map_["elevation_normal_y"].setConstant(0);
  grid_map_["elevation_normal_z"].setConstant(1);
  grid_map_["ground_sample_distance"].setConstant(0);
  grid_map_["incident_prior"].setConstant(0);
  grid_map_["triangulation_prior"].setConstant(0);
  grid_map_["min_eigen_value"].setConstant(0);
}

ViewUtilityMap::~ViewUtilityMap() {}

double ViewUtilityMap::getBestJointPrior(const std::vector<GeometricPrior> &prior_list) {
  double best_prior{0.0};

  // Iterate through the list of priors to get the maximum geometric_prior
  for (auto prior : prior_list) {
    if (prior.joint > best_prior) best_prior = prior.joint;
  }
  return best_prior;
}

GeometricPrior ViewUtilityMap::getBestGeometricPrior(const std::vector<GeometricPrior> &prior_list) {
  double best_prior{0.0};
  GeometricPrior best_prior_info;

  // Iterate through the list of priors to get the maximum geometric_prior
  for (auto &prior : prior_list) {
    if (prior.joint > best_prior) {
      best_prior = prior.joint;
      best_prior_info = prior;
    }
  }
  return best_prior_info;
}

double ViewUtilityMap::getBestGroundSampleDistance(const std::vector<GeometricPrior> &prior_list) {
  double best_sample_distance{std::numeric_limits<double>::infinity()};

  // Iterate through the list of priors to get the maximum geometric_prior
  for (auto prior : prior_list) {
    if ((prior.sample_distance < best_sample_distance) && (best_sample_distance > 0))
      best_sample_distance = prior.sample_distance;
  }
  return best_sample_distance;
}

double ViewUtilityMap::CalculateViewUtility(std::vector<ViewPoint> &viewpoint_set, bool update_utility_map) {
  double view_utility = 0.0;
  // Copy cell information to append list with candidate views
  /// TODO: making deep copies of this map is not a good idea, but works for now
  std::vector<CellInfo> candidate_cell_information = cell_information_;
  grid_map::GridMap candidate_grid_map = grid_map_;
  for (auto &viewpoint : viewpoint_set) {
    double utility = CalculateViewUtility(viewpoint, true, candidate_cell_information, candidate_grid_map);
    view_utility += utility;
  }
  return view_utility;
}

grid_map::Polygon ViewUtilityMap::getVisibilityPolygon(ViewPoint &viewpoint, grid_map::GridMap &grid_map) {
  grid_map::Polygon polygon;
  double max_distance = 1000.0;

  Eigen::Vector3d viewpoint_center_local = viewpoint.getCenterLocal();

  polygon.setFrameId(grid_map.getFrameId());
  for (auto ray : viewpoint.getCornerRayVectors()) {
    /// TODO: Do something about ray tracing steps
    ray.normalize();
    for (double t = 0.0; t < max_distance; t += 1.0) {
      Eigen::Vector3d ray_point = viewpoint_center_local + t * ray;
      Eigen::Vector2d ray_point_2d(ray_point(0), ray_point(1));
      double elevation{0.0};
      if (grid_map.isInside(ray_point_2d)) {
        elevation = grid_map.atPosition("elevation", ray_point_2d);
      }
      if (elevation > ray_point(2)) {
        Eigen::Vector3d cell_pos;
        grid_map::Index idx;
        if (grid_map_.isInside(ray_point_2d)) {
          grid_map.getIndex(ray_point_2d, idx);
          grid_map.getPosition3("elevation", idx, cell_pos);
          polygon.addVertex(grid_map::Position(cell_pos(0), cell_pos(1)));
        } else {
          polygon.addVertex(grid_map::Position(ray_point_2d(0), ray_point_2d(1)));
        }
        break;
      }
    }
  }

  return polygon;
}

double ViewUtilityMap::CalculateViewUtility(ViewPoint &viewpoint, bool update_utility_map,
                                            std::vector<CellInfo> &cell_information, grid_map::GridMap &grid_map) {
  double view_utility = 0.0;

  grid_map::Matrix &terrain_data = grid_map["elevation"];

  grid_map::Polygon polygon = getVisibilityPolygon(viewpoint, grid_map);
  if (polygon.nVertices() == 4) {
    Eigen::Vector3d viewpoint_center_local = viewpoint.getCenterLocal();
    int width = grid_map.getSize()(0);
    int view_index = viewpoint.getIndex();
    grid_map::Matrix &layer_visibility = grid_map["visibility"];
    grid_map::Matrix &layer_geometricprior = grid_map["geometric_prior"];
    grid_map::Matrix &layer_sample_distance = grid_map["ground_sample_distance"];
    grid_map::Matrix &layer_incident_prior = grid_map["incident_prior"];
    grid_map::Matrix &layer_triangulation_prior = grid_map["triangulation_prior"];
    grid_map::Matrix &layer_utility = grid_map["normalized_prior"];
    grid_map::Matrix &layer_min_eigen_value = grid_map["min_eigen_value"];
    for (grid_map::PolygonIterator iterator(grid_map, polygon); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      int idx = index(0) + width * index(1);  // Index number of the gridcell
      if (cell_information.size() <= idx) {
        /// TODO: This is a workaround since the polygon iterator is not checking for map bounds
        continue;
      }
      /// No need to run any calculations if is not part of ROI
      if (grid_map.at("roi", index) < 0.5) continue;

      // Get position of the gridcell
      /// TODO: Treat terrain height as a distribution
      Eigen::Vector3d cell_pos;
      grid_map.getPosition3("elevation", index, cell_pos);

      Eigen::Vector3d cell_normal{Eigen::Vector3d(grid_map.at("elevation_normal_x", index),
                                                  grid_map.at("elevation_normal_y", index),
                                                  grid_map.at("elevation_normal_z", index))};

      // Calculate and store view vectors so that we don't need to repeat the calculation
      Eigen::Vector3d view_vector = viewpoint_center_local - cell_pos;
      Eigen::Vector3d view_vector_u = view_vector.normalized();
      double view_distance = view_vector.norm();
      Eigen::Vector3d optical_center = viewpoint.getCenterRayVector();
      double cell_area = std::pow(grid_map_.getResolution(), 2);

      switch (utility_type_) {
        case ViewUtilityType::GEOMETRIC_PRIOR: {
          std::vector<GeometricPrior> geometric_prior = getGeometricPrior(
              settings_, view_vector_u, view_distance, cell_normal, optical_center, cell_information[idx]);

          double best_prior = getBestJointPrior(geometric_prior);
          GeometricPrior best_geometric_prior = getBestGeometricPrior(geometric_prior);
          double best_sample_distance = getBestGroundSampleDistance(geometric_prior);
          bool prior_updated = false;
          /// TODO: This results in over estimation of view utility since it does not account for view
          if (best_prior > layer_geometricprior(index(0), index(1))) {
            prior_updated = true;
            if (best_prior < max_prior_) {
              // TODO: Why are we dividing to 100.0? where does 100 come from?
              view_utility += best_prior - layer_geometricprior(index(0), index(1)) * cell_area / 100.0;
            }
          }

          // Register view into the view utility map
          ViewInfo viewinfo;
          viewinfo.view_vector = view_vector_u;
          viewinfo.view_distance = view_distance;
          viewinfo.view_index = view_index;
          cell_information[idx].view_info.push_back(viewinfo);

          if (update_utility_map) {
            // Update Geometric Prior
            if (prior_updated) {
              layer_geometricprior(index(0), index(1)) = best_prior;
              layer_sample_distance(index(0), index(1)) = best_sample_distance;
              layer_utility(index(0), index(1)) = std::min(best_prior, max_prior_) / max_prior_;
              layer_sample_distance(index(0), index(1)) = best_geometric_prior.resolution;
              layer_incident_prior(index(0), index(1)) = best_geometric_prior.incident;
              layer_triangulation_prior(index(0), index(1)) = best_geometric_prior.triangulation;
            }
          }
          break;
        }
        case ViewUtilityType::SPHERICAL_COVERAGE: {
          /// Generate discrete samples
          /// Roberts 2017 generate samples through http://blog.marmakoide.org/?p=1
          const int K = 256;
          std::vector<Eigen::Vector3d> hemisphere_samples(K);
          double golden_angle = M_PI * (3.0 - std::sqrt(5.0));
          for (int i = 0; i < hemisphere_samples.size(); i++) {
            double theta = golden_angle * i;
            double z = std::abs((1.0 - 1.0 / double(K)) * (1.0 - 2.0 * i / (double(K) - 1.0)));
            double radius = sqrt(1.0 - std::pow(z, 2));
            hemisphere_samples[i](0) = radius * std::cos(theta);
            hemisphere_samples[i](1) = radius * std::sin(theta);
            hemisphere_samples[i](2) = z;
          }

          double F_c{0.0};
          for (auto &sample : hemisphere_samples) {
            double w_h = sample.dot(cell_normal);
            double vj = 0.0;  // Coverage indicator function
            for (auto view : cell_information[idx].view_info) {
              if (hemisphereInside(view.view_vector, sample, view.view_distance)) {
                vj = 1.0;
                break;  // If the sample is in the covered region, no need to iterate
              }
            }
            vj = hemisphereInside(view_vector_u, sample, view_distance) ? 1.0 : vj;
            vj = (w_h < 0.0) ? 0.0 : vj;
            F_c += (2 * M_PI / static_cast<double>(K)) * w_h * vj;
          }
          if (F_c > layer_geometricprior(index(0), index(1))) {
            // This condition should be redundant, but numerically unstable
            /// TODO: Investigate why this is numerically unstable
            view_utility += F_c - layer_geometricprior(index(0), index(1));
          }

          ViewInfo viewinfo;
          viewinfo.view_vector = view_vector_u;
          viewinfo.view_distance = view_distance;
          viewinfo.view_index = view_index;
          cell_information[idx].view_info.push_back(viewinfo);

          if (update_utility_map) {
            layer_geometricprior(index(0), index(1)) = F_c;
            layer_utility(index(0), index(1)) = F_c;
          }

          break;
        }
        case ViewUtilityType::FISHER_INFORMATION: {
          double sigma_k{30.0 / 180.0 * M_PI};
          double incident_prior = getIncidentPrior(view_vector_u, cell_normal, sigma_k);

          Eigen::Matrix3d cell_fim = cell_information[idx].fisher_information;

          // Calculate fisher information of each bearing vector observation
          double reference_view_distance = 100;
          Eigen::Vector3d bearing_vector = -view_vector;
          double gsd_prior = getGroundSamplePrior(bearing_vector, optical_center, reference_view_distance);
          double pixel_res = 0.5 * M_PI / 1080.0;
          double sigma = std::sin(pixel_res);
          Eigen::Matrix3d fim = getFisherInformationMatrix(bearing_vector, view_distance, sigma);
          Eigen::Matrix3d accumulated_fim = cell_fim + (incident_prior * gsd_prior * fim);

          // Compute Cramer Rao Bounds
          /// TODO: Catch if fim is singular
          Eigen::Matrix3d covariance_matrix = accumulated_fim.inverse();
          double max_cramer_rao_bound = std::sqrt(covariance_matrix.diagonal().maxCoeff());
          double cell_bounds = cell_information[idx].max_cramerrao_bounds;
          if (!std::isfinite(max_cramer_rao_bound) || (max_cramer_rao_bound > limit_cramerrao_bounds))
            max_cramer_rao_bound = limit_cramerrao_bounds;

          double min_cramer_rao_bound = 0.1;
          double utility = (max_cramer_rao_bound < min_cramer_rao_bound) ? 0.0 : cell_bounds - max_cramer_rao_bound;
          view_utility += utility;
          // Register view into the view utility map
          ViewInfo viewinfo;
          viewinfo.view_vector = view_vector_u;
          viewinfo.view_distance = view_distance;
          viewinfo.view_index = view_index;
          cell_information[idx].view_info.push_back(viewinfo);
          cell_information[idx].fisher_information = accumulated_fim;
          cell_information[idx].max_cramerrao_bounds = max_cramer_rao_bound;
          if (update_utility_map) {
            layer_geometricprior(index(0), index(1)) = max_cramer_rao_bound;
            layer_sample_distance(index(0), index(1)) = gsd_prior;
            layer_incident_prior(index(0), index(1)) = incident_prior;
            layer_utility(index(0), index(1)) = max_cramer_rao_bound / min_cramer_rao_bound;
          }

          break;
        }
      }

      if (update_utility_map) {
        // Update Visibility count
        /// TODO: This increases the visibility count even though it is very far away
        layer_visibility(index(0), index(1)) += 1.0;  // Increase visibility count
      }
    }

    if (update_utility_map) viewpoint.setUtility(view_utility);
  }

  return view_utility;
}

void ViewUtilityMap::UpdateUtility(ViewPoint &viewpoint) {
  CalculateViewUtility(viewpoint, true, cell_information_, grid_map_);
}

double ViewUtilityMap::getGroundSamplePrior(const Eigen::Vector3d &bearing_vector,
                                            const Eigen::Vector3d &optical_center,
                                            const double reference_view_distance) {
  double projected_distance = (bearing_vector).dot(optical_center);
  double relative_gsd = std::pow(reference_view_distance / projected_distance, 2);
  double gsd_prior = std::min(relative_gsd, 1 / relative_gsd);
  return gsd_prior;
}

double ViewUtilityMap::getIncidentPrior(const Eigen::Vector3d &unit_view_vector, const Eigen::Vector3d &cell_normal,
                                        double sigma_k) {
  double incident_angle = std::acos(unit_view_vector.dot(cell_normal));
  double incident_prior = std::exp(-std::pow(incident_angle, 2) / (2 * std::pow(sigma_k, 2)));
  return incident_prior;
}

Eigen::Matrix3d ViewUtilityMap::getFisherInformationMatrix(const Eigen::Vector3d &bearing_vector,
                                                           const double &view_distance, const double sigma) {
  const double inverse_depth = 1 / view_distance;
  Eigen::Matrix3d jacobian = inverse_depth * Eigen::Matrix3d::Identity() -
                             std::pow(inverse_depth, 3) * bearing_vector * bearing_vector.transpose();
  Eigen::Matrix3d fim = std::pow(1 / sigma, 2) * jacobian * jacobian.transpose();
  return fim;
}

std::vector<GeometricPrior> ViewUtilityMap::getGeometricPrior(const GeometricPriorSettings &settings,
                                                              const Eigen::Vector3d &view_vector_query,
                                                              const double &view_distance,
                                                              const Eigen::Vector3d &cell_normal,
                                                              const Eigen::Vector3d &center_ray, CellInfo &cell_info) {
  std::vector<GeometricPrior> prior;
  /// Calculate incidence prior
  double min_angle = settings.min_triangulation_angle;

  double sigma_k = settings.sigma_k;
  double incident_prior = getIncidentPrior(view_vector_query, cell_normal, sigma_k);

  /// Calculate ground sample distance prior
  double reference_view_distance = settings.reference_view_distance;
  Eigen::Vector3d bearing_vector = -view_distance * view_vector_query;
  double gsd_prior = getGroundSamplePrior(bearing_vector, center_ray, reference_view_distance);

  // Calculate pairwise priors
  int i = 0;
  for (auto view : cell_info.view_info) {
    /// Calculate resolution prior
    /// TODO: Experiment with pairwise resolution prior and resolution vs GSD
    double relative_area = std::pow(view_distance / view.view_distance, 2);
    double resolution_prior = std::min(relative_area, 1 / relative_area);

    // Calculate triangulation prior
    double angle = std::abs(
        std::acos(view_vector_query.dot(view.view_vector)));  /// TODO: Make sure the vectors are all normalized
    double triangulation_prior =
        1.0 - (std::pow((std::min(angle, min_angle) - min_angle), 2) / (min_angle * min_angle));

    GeometricPrior geometric_prior;
    geometric_prior.incident = incident_prior;
    geometric_prior.resolution = resolution_prior;
    geometric_prior.triangulation = triangulation_prior;
    geometric_prior.sample_distance = gsd_prior;
    double joint_geometric_prior = incident_prior * resolution_prior * gsd_prior * triangulation_prior;
    geometric_prior.joint = joint_geometric_prior;

    prior.push_back(geometric_prior);
    i++;
  }
  return prior;
}

void ViewUtilityMap::initializeFromGridmap() {
  grid_map::Matrix &layer_elevation = grid_map_["elevation"];
  grid_map::Matrix &layer_normal_x = grid_map_["elevation_normal_x"];
  grid_map::Matrix &layer_normal_y = grid_map_["elevation_normal_y"];
  grid_map::Matrix &layer_normal_z = grid_map_["elevation_normal_z"];

  unsigned width = grid_map_.getSize()(0);
  unsigned height = grid_map_.getSize()(1);
  double resolution = grid_map_.getResolution();
  setCellInformation(width * height);
  // Compute normals from elevation map
  // Surface normal calculation from: https://www.flipcode.com/archives/Calculating_Vertex_Normals_for_Height_Maps.shtml
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;

    /// TODO: Verify normal by visualization
    int x = gridMapIndex(0);
    int y = height - 1 - gridMapIndex(1);

    float sx = layer_elevation(x < width - 1 ? x + 1 : x, y) - layer_elevation(x > 0 ? x - 1 : x, y);
    if (x == 0 || x == width - 1) sx *= 2;

    float sy = layer_elevation(x, y < height - 1 ? y + 1 : y) - layer_elevation(x, y > 0 ? y - 1 : y);
    if (y == 0 || y == height - 1) sy *= 2;

    Eigen::Vector3d normal(Eigen::Vector3d(sx, sy, 2 * resolution));
    normal.normalize();

    layer_normal_x(x, y) = normal(0);
    layer_normal_y(x, y) = normal(1);
    layer_normal_z(x, y) = normal(2);
  }
  grid_map_["visibility"].setConstant(0);
  grid_map_["geometric_prior"].setConstant(limit_cramerrao_bounds);
  grid_map_["roi"].setConstant(1.0);
  grid_map_["normalized_prior"].setConstant(0);
}

bool ViewUtilityMap::initializeFromGeotiff(GDALDataset *dataset) {
  std::cout << std::endl << "Loading GeoTIFF file for gridmap" << std::endl;

  double originX, originY, pixelSizeX, pixelSizeY;

  double geoTransform[6];
  if (dataset->GetGeoTransform(geoTransform) == CE_None) {
    originX = geoTransform[0];
    originY = geoTransform[3];
    pixelSizeX = geoTransform[1];
    pixelSizeY = geoTransform[5];
  } else {
    std::cout << "Failed read geotransform" << std::endl;
    return false;
  }

  const char *pszProjection = dataset->GetProjectionRef();
  std::cout << std::endl << "Wkt ProjectionRef: " << pszProjection << std::endl;
  /// TODO: Get proper projection references using gDal
  // Fluela
  double center_latitude = 46.95240555555556;
  double center_longitude = 7.439583333333333;
  double center_altitude = 1000.0;  /// TODO: Get center altitude as minimum altitude

  // Cadastre
  // double center_latitude = 46.553215;
  // double center_longitude = 6.682505;
  // double center_altitude = 800; /// TODO: Get center altitude as minimum altitude

  // this->setOrigin(center_latitude, center_longitude, center_altitude);

  // Get image metadata
  unsigned width = dataset->GetRasterXSize();
  unsigned height = dataset->GetRasterYSize();
  double resolution = pixelSizeX;
  std::cout << "Width: " << width << " Height: " << height << " Resolution: " << resolution << std::endl;

  // pixelSizeY is negative because the origin of the image is the north-east corner and positive
  // Y pixel coordinates go towards the south
  const double lengthX = resolution * width;
  const double lengthY = resolution * height;
  grid_map::Length length(lengthX, lengthY);
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  grid_map_.setGeometry(length, resolution, position);
  grid_map_.setFrameId("map");
  grid_map_.setTimestamp(ros::Time::now().toNSec());
  grid_map_.add("elevation");

  GDALRasterBand *elevationBand = dataset->GetRasterBand(1);

  std::vector<float> data(width * height, 0.0f);
  elevationBand->RasterIO(GF_Read, 0, 0, width, height, &data[0], width, height, GDT_Float32, 0, 0);

  grid_map::Matrix &layer_elevation = grid_map_["elevation"];
  grid_map::Matrix &layer_normal_x = grid_map_["elevation_normal_x"];
  grid_map::Matrix &layer_normal_y = grid_map_["elevation_normal_y"];
  grid_map::Matrix &layer_normal_z = grid_map_["elevation_normal_z"];

  setCellInformation(lengthX * lengthY);

  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;

    int x = gridMapIndex(0);
    int y = height - 1 - gridMapIndex(1);

    layer_elevation(x, y) = data[gridMapIndex(0) + width * gridMapIndex(1)] - center_altitude;
  }

  // Compute normals from elevation map
  // Surface normal calculation from: https://www.flipcode.com/archives/Calculating_Vertex_Normals_for_Height_Maps.shtml
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;

    /// TODO: Verify normal by visualization
    int x = gridMapIndex(0);
    int y = height - 1 - gridMapIndex(1);

    float sx = layer_elevation(x < width - 1 ? x + 1 : x, y) - layer_elevation(x > 0 ? x - 1 : x, y);
    if (x == 0 || x == width - 1) sx *= 2;

    float sy = layer_elevation(x, y < height - 1 ? y + 1 : y) - layer_elevation(x, y > 0 ? y - 1 : y);
    if (y == 0 || y == height - 1) sy *= 2;

    Eigen::Vector3d normal(Eigen::Vector3d(sx, sy, 2 * resolution));
    normal.normalize();

    layer_normal_x(x, y) = normal(0);
    layer_normal_y(x, y) = normal(1);
    layer_normal_z(x, y) = normal(2);
  }
  grid_map_["visibility"].setConstant(0);
  grid_map_["geometric_prior"].setConstant(limit_cramerrao_bounds);
  grid_map_["roi"].setConstant(1);
  grid_map_["normalized_prior"].setConstant(0);

  return true;
}

bool ViewUtilityMap::initializeFromMesh(const std::string &path, const double res) {
  pcl::PolygonMesh mesh;
  pcl::io::loadOBJFile(path, mesh);
  grid_map::GridMapPclConverter::initializeFromPolygonMesh(mesh, res, grid_map_);
  grid_map::GridMapPclConverter::addLayerFromPolygonMesh(mesh, "elevation", grid_map_);

  double width = grid_map_.getSize()(0);
  double height = grid_map_.getSize()(1);
  double resolution = grid_map_.getResolution();

  grid_map::Matrix &layer_elevation = grid_map_["elevation"];
  grid_map::Matrix &layer_normal_x = grid_map_["elevation_normal_x"];
  grid_map::Matrix &layer_normal_y = grid_map_["elevation_normal_y"];
  grid_map::Matrix &layer_normal_z = grid_map_["elevation_normal_z"];

  setCellInformation(width * height);

  // Compute normals from elevation map
  // Surface normal calculation from: https://www.flipcode.com/archives/Calculating_Vertex_Normals_for_Height_Maps.shtml

  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridMapIndex = *iterator;

    /// TODO: Verify normal by visualization
    int x = gridMapIndex(0);
    int y = grid_map_.getSize()(1) - 1 - gridMapIndex(1);

    float sx = layer_elevation(x < width - 1 ? x + 1 : x, y) - layer_elevation(x > 0 ? x - 1 : x, y);
    if (x == 0 || x == width - 1) sx *= 2;

    float sy = layer_elevation(x, y < height - 1 ? y + 1 : y) - layer_elevation(x, y > 0 ? y - 1 : y);
    if (y == 0 || y == height - 1) sy *= 2;

    Eigen::Vector3d normal(Eigen::Vector3d(sx, sy, 2 * resolution));
    normal.normalize();

    layer_normal_x(x, y) = normal(0);
    layer_normal_y(x, y) = normal(1);
    layer_normal_z(x, y) = normal(2);
  }
  grid_map_["visibility"].setConstant(0);
  grid_map_["geometric_prior"].setConstant(limit_cramerrao_bounds);
  grid_map_["roi"].setConstant(1);
  grid_map_["normalized_prior"].setConstant(0);
  grid_map_.setFrameId("map");
  return true;
}

bool ViewUtilityMap::initializeEmptyMap() {
  grid_map_.setGeometry(grid_map::Length(100.0, 100.0), 10.0, grid_map::Position(0.0, 0.0));
  // Initialize gridmap layers
  grid_map_["visibility"].setConstant(0);
  grid_map_["geometric_prior"].setConstant(limit_cramerrao_bounds);
  grid_map_["elevation"].setConstant(1);
  grid_map_["roi"].setConstant(0);
  grid_map_["elevation_normal_x"].setConstant(0);
  grid_map_["elevation_normal_y"].setConstant(0);
  grid_map_["elevation_normal_z"].setConstant(1);

  // Initialize cell information
  double width = grid_map_.getSize()(0);
  double height = grid_map_.getSize()(1);
  setCellInformation(width * height);
  return true;
}

void ViewUtilityMap::OutputMapData(const std::string path) {
  // Write data to files

  std::ofstream output_file;
  output_file.open(path);
  output_file << "geometric_prior,visibility,\n";

  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);

    output_file << grid_map_.at("geometric_prior", index) << ",";
    output_file << grid_map_.at("visibility", index) << ",";
    output_file << "\n";
  }

  output_file.close();
  return;
}

void ViewUtilityMap::TransformMap(const Eigen::Vector3d &translation) {
  Eigen::Translation3d map_translation(translation(0), translation(1), translation(2));
  Eigen::AngleAxisd map_rotation(0.0, Eigen::Vector3d::UnitZ());

  Eigen::Isometry3d transform = map_translation * map_rotation;  // Apply affine transformation.
  grid_map_ = grid_map_.getTransformedMap(transform, "elevation", grid_map_.getFrameId(), true);
}

void ViewUtilityMap::SetRegionOfInterest(const grid_map::Polygon &polygon) {
  for (grid_map::PolygonIterator iterator(grid_map_, polygon); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    grid_map_.at("roi", index) = 1.0;
  }
  return;
}

std::vector<double> ViewUtilityMap::calculateErrors(grid_map::GridMap &groundtruth_map,
                                                    const grid_map::GridMap &reference_map) {
  groundtruth_map.add("elevation_difference");
  groundtruth_map["elevation_difference"].setConstant(NAN);

  std::vector<double> error_vector;
  for (grid_map::GridMapIterator iterator(groundtruth_map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    Eigen::Vector3d cell_pos_3d;
    bool valid = groundtruth_map.getPosition3("elevation", index, cell_pos_3d);
    if (valid) {
      double cell_elevation = cell_pos_3d(2);
      const Eigen::Vector2d cell_pos = Eigen::Vector2d(cell_pos_3d(0), cell_pos_3d(1));
      /// TODO: Check if this overlaps with the reference map
      /// TODO: Define ROI
      if (reference_map.isInside(cell_pos)) {
        double ref_elevation = reference_map.atPosition("elevation", cell_pos);
        double error = std::abs(ref_elevation - cell_elevation);
        groundtruth_map.at("elevation_difference", index) = error;
        error_vector.push_back(error);
      } else {
        groundtruth_map.at("elevation_difference", index) = NAN;
        error_vector.push_back(NAN);
      }
    }
  }
  return error_vector;
}

double ViewUtilityMap::CalculatePrecision(const std::vector<double> &error_vector, const double threshold) {
  int tp_count{0};
  int total_count{0};

  for (auto error_point : error_vector) {
    total_count++;
    if (std::isfinite(error_point) && (std::abs(error_point) < std::abs(threshold))) {
      tp_count++;
    }
  }
  double precision = static_cast<double>(tp_count) / static_cast<double>(total_count);
  return precision;
}

void ViewUtilityMap::CompareMapLayer(grid_map::GridMap &reference_map) {
  // Error statistics in Groundtruth -> Reconstructed mapf
  // Calculate Recall
  std::vector<double> error_vector = calculateErrors(grid_map_, reference_map);
  // Compute error values
  double cumulative_error{0.0};
  int num_valid_points{0};
  for (auto error_point : error_vector) {
    if (std::isfinite(error_point)) {
      cumulative_error += error_point;
      num_valid_points++;
    }
  }
  double mean = cumulative_error / num_valid_points;
  double cumulative_squared_error{0.0};
  for (auto error_point : error_vector) {
    if (std::isfinite(error_point)) cumulative_squared_error += std::pow(error_point - mean, 2);
  }
  double stdev = std::sqrt(cumulative_squared_error / num_valid_points);
  std::cout << "  - Average Error: " << mean << std::endl;
  std::cout << "  - Average STDEV: " << stdev << std::endl;

  // Error statistics in Reconstructed map -> Groundtruth
  // Calculate Precision
  std::vector<double> error_precision = calculateErrors(reference_map, grid_map_);

  /// TODO: Write raw error values into file for ruther analysis
  std::cout << "Elevation Map Error Statistics" << std::endl;

  // Compute error statistics
  {
    double error_threshold = 1.0;
    double precision = CalculatePrecision(error_precision, error_threshold);
    double recall = CalculatePrecision(error_vector, error_threshold);
    double f_score = 2 * precision * recall / (precision + recall);

    std::cout << "  - Precision   (" << error_threshold << " [m]): " << precision << std::endl;
    std::cout << "  - Recall      (" << error_threshold << " [m]): " << recall << std::endl;
    std::cout << "  - F-score     (" << error_threshold << " [m]): " << f_score << std::endl;
  }
  {
    double reference_precision{0.5};
    double accuracy = 0.0;
    int tp_count{0};
    int total_count{0};
    double precision{0.0};
    while (precision < reference_precision) {
      accuracy += 0.1;
      for (auto error_point : error_precision) {
        total_count++;
        if (std::isfinite(error_point) && (std::abs(error_point) < std::abs(accuracy))) {
          tp_count++;
        }
      }
      precision = static_cast<double>(tp_count) / static_cast<double>(total_count);
    }
    std::cout << "  - Accuracy(Precision): " << accuracy << "(" << precision << "[m])" << std::endl;
  }
  /// TODO: Implement Precision / Recall / F-scores
}
