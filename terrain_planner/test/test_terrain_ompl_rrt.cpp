#include <iostream>
#include <memory>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>

#include <terrain_navigation/path_segment.h>
#include <terrain_navigation/path.h>
#include <terrain_navigation/terrain_map.h>

#include <terrain_planner/DubinsAirplane.hpp>
#include <terrain_planner/DubinsPath.hpp>
#include <terrain_planner/ompl_setup.h>
#include <terrain_planner/terrain_ompl_rrt.h>
#include <terrain_planner/terrain_ompl.h>

// Test the terrain planner without invoking ROS.
int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "test_terrain_ompl_rt.cpp" << std::endl;

    // double home_lat = 56.6987387;
    // double home_lon = -6.1082210;
    double gamma = 0.1;
    double loiter_radius = 40.0;
    double max_altitude = 120.0;
    double min_altitude = 50.0;

    // # create terrain map
    // grid_map = GridMapSRTM(home_lat, home_lon)
    // grid_map.setGridLength(800)
    // terrain_map = TerrainMap()
    // terrain_map.setGridMap(grid_map)
    std::shared_ptr<TerrainMap> terrain_map{std::make_shared<TerrainMap>()};

    // create planner
    double goal_radius = loiter_radius;
    double max_elevation = max_altitude;
    double min_elevation = min_altitude;
  
    std::shared_ptr<fw_planning::spaces::DubinsAirplaneStateSpace> da_space{
      std::make_shared<fw_planning::spaces::DubinsAirplaneStateSpace>(goal_radius, gamma)
    };
    std::shared_ptr<TerrainOmplRrt> planner{
      std::make_shared<TerrainOmplRrt>(ompl::base::StateSpacePtr(da_space))
    };
    planner->setMap(terrain_map);
    planner->setAltitudeLimits(max_elevation, min_elevation);
    //! @todo raises exception as map not initialised
    // planner->setBoundsFromMap(terrain_map->getGridMap());

    // set start and goal
    // Eigen::Vector3d start_pos{Eigen::Vector3d(0.0, 0.0, 60.0)};
    // Eigen::Vector3d goal_pos{Eigen::Vector3d(-200.0, 200.0, 60.0)};
  
    // adjust start and goal altitudes
    // start_pos[2] += grid_map.atPosition("elevation", start_pos)
    // goal_pos[2] += grid_map.atPosition("elevation", goal_pos)

    // set up problem from start and goal positions and start loiter radius
    //! @todo raises exception as map not initialised
    // planner->setupProblem(start_pos, goal_pos, loiter_radius);

    // initialise an empty solution path
    std::shared_ptr<ompl::OmplSetup> problem = planner->getProblemSetup();
    ompl::base::SpaceInformationPtr si = problem->getSpaceInformation();
    ompl::geometric::PathGeometric solution_path = ompl::geometric::PathGeometric(si);

    // add states
    {
      fw_planning::spaces::DubinsAirplaneStateSpace::StateType *da_state
        = da_space->allocState()->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>();
      da_state->setXYZYaw(-40.0000, -0.0000, 73.9186, 1.5708);
      solution_path.append(da_state);
    }

    {
      fw_planning::spaces::DubinsAirplaneStateSpace::StateType *da_state
        = da_space->allocState()->as<fw_planning::spaces::DubinsAirplaneStateSpace::StateType>();
      da_state->setXYZYaw(-187.6393, 161.9577, 104.6088, -2.8274);
      solution_path.append(da_state);
    }

    Path trajectory_segments = Path();
    planner->solutionPathToPath(solution_path, trajectory_segments);

    std::cout << "segment count: " << trajectory_segments.segments.size() << std::endl;
    for (size_t i=0; i<trajectory_segments.segments.size(); ++i)
    {
      std::vector<Eigen::Vector3d> position = trajectory_segments.segments[i].position();
      std::cout << "segment[: " << i << "]: count: " << position.size() <<  std::endl;
      for (size_t j=0; j<position.size(); ++j)
      {
        std::cout << position[j].transpose() <<  std::endl;
      }
      std::cout << "----------" << std::endl;
    }

    return 0;
}
