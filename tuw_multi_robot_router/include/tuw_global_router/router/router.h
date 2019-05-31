#ifndef TUW_DEFAULT_ROUTER_H
#define TUW_DEFAULT_ROUTER_H

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <tuw_global_router/srr_utils.h>
#include <tuw_global_router/segment_expander.h>
#include <tuw_global_router/mrr_utils.h>
#include "agent.h"
#include "environment.h"
#include <unordered_map>

namespace multi_robot_router
{
    class Router
    {
    public:
        virtual void resize(const uint32_t _nr_robots) = 0;

        virtual std::unordered_map<std::string, std::vector<Checkpoint>> computePlan(const std::vector<Agent>& agents, const Environment& environment) = 0;

        /**
                * @brief generates the plan from (Vertex[odom robotPose] to Vertex[_goals]
                * @param _starts the start positions of all robots
                * @param _goals the goal positions fo all robots
                * @param _radius a vector of the robots radius'
                * @param _map the grid_map used to find the start and goal segments of the path
                * @param _graph the full graph of the map used for planning the path
                * @param _robot_names robot names
                */
        virtual bool makePlan(const std::vector<Eigen::Vector3d> &_starts, const std::vector<Eigen::Vector3d> &_goals,
                      const std::vector<float> &_radius, const cv::Mat &_map, const float &_resolution,
                      const Eigen::Vector2d &_origin, const std::vector<Segment> &_graph,
                      const std::vector<std::string> &_robot_names) = 0;

        /**
                 * @brief sets the CollisionResolverType used
                 */
        virtual void setCollisionResolutionType(const SegmentExpander::CollisionResolverType _cr) = 0;

        /**
                 * @brief returns the found Routing Table
                 * @param _robot the robot to whom the routing table belongs to
                 */
        virtual const std::vector<Checkpoint> &getRoute(const uint32_t _robot) const = 0;

        /**
                 * @brief getter
                 * @returns the duration of the planning attempt
                 */
        virtual uint32_t getDuration_ms() const = 0;

        /**
                 * @brief getter
                 * @returns the Overall path length of the planning attempt
                 */
        virtual float getOverallPathLength() const = 0;

        /**
                 * @brief getter
                 * @returns the longest Path length of the planning attempt
                 */
        virtual float getLongestPathLength() const = 0;

        /**
                 * @brief getter
                 * @returns the priority reschedule attempts
                 */
        virtual uint32_t getPriorityScheduleAttemps() const = 0;

        /**
                 * @brief getter
                 * @returns the speed reschedule attempts
                 */
        virtual uint32_t getSpeedScheduleAttemps() const = 0;

        std::shared_ptr<float> potential_;

        enum class goalMode
        {
            use_segment_goal,
            use_voronoi_goal,
            use_map_goal
        };
        enum class graphType
        {
            voronoi,
            random
        };
        enum class routerType
        {
            singleThread,
            multiThreadSrr
        };
        graphType graphMode_ = graphType::voronoi;
        goalMode goalMode_ = goalMode::use_voronoi_goal;
        float routerTimeLimit_s_ = 10.0;
        bool segmentOptimizations_ = false;
        bool speedRescheduling_ = true;
        bool priorityRescheduling_ = true;
        bool collisionResolver_ = true;

        virtual void setPlannerType(routerType _type, uint32_t _nr_threads) = 0;

    };
}

#endif
