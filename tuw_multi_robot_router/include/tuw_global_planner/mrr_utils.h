#ifndef POINT_H
#define POINT_H

#include <eigen3/Eigen/Dense>
#include <tuw_global_planner/srr_utils.h>
#include <ros/ros.h>

namespace multi_robot_router
{
    //Checkpoint used in a Route 
    class Checkpoint
    {
        public:
            struct Precondition
            {
                int32_t robotId;
                int32_t stepCondition;
            };

            uint32_t segId;
            Eigen::Vector3d start;
            Eigen::Vector3d end;
            std::vector<Precondition> preconditions;

            /**
             * @brief constructor to assign Checkpoint from a Route Vertex used in route candidates
             */
            Checkpoint(const RouteVertex &_v)
            {
                segId = _v.getSegment().getSegmentId();

                if(_v.direction == RouteVertex::path_direction::start_to_end)
                {
                    Eigen::Vector2d s = _v.getSegment().getStart();
                    start[0] = s[0];
                    start[1] = s[1];
                    start[2] = atan2(end[1] - start[1], end[0] - start[0]);
                    
                    Eigen::Vector2d e = _v.getSegment().getEnd();
                    end[0] = e[0];
                    end[1] = e[1];
                    end[2] = atan2(end[1] - start[1], end[0] - start[0]);
                }
                else
                {
                    Eigen::Vector2d s = _v.getSegment().getEnd();
                    start[0] = s[0];
                    start[1] = s[1];
                    start[2] = atan2(start[1] - end[1], start[0] - end[0]);
                    
                    Eigen::Vector2d e = _v.getSegment().getStart();
                    end[0] = e[0];
                    end[1] = e[1];
                    end[2] = atan2(start[1] - end[1], start[0] - end[0]);
                }
            }
            Checkpoint(): segId(-1)
            { }

            /**
             * @brief adds or updates a precondition of the checkpoint
             */
            void updatePreconditions(const Precondition &n_pc)
            {
                bool updatedPc = false;

                for(Checkpoint::Precondition & pc : preconditions)
                {
                    if(pc.robotId == n_pc.robotId)
                    {
                        pc.stepCondition = std::max(pc.stepCondition, n_pc.stepCondition);
                        updatedPc = true;
                        break;
                    }
                }

                if(!updatedPc)
                {
                    preconditions.push_back(n_pc);
                }
            }
            
            
            /**
             * @brief adds or updates multiple preconditions of the checkpoint
             */
            void updatePreconditions(const std::vector<Precondition> &n_pcs)
            {
                for(const Checkpoint::Precondition &n_pc : n_pcs)
                {
                    bool updatedPc = false;

                    for(Checkpoint::Precondition & pc : preconditions)
                    {
                        if(pc.robotId == n_pc.robotId)
                        {
                            pc.stepCondition = std::max(pc.stepCondition, n_pc.stepCondition);
                            updatedPc = true;
                            break;
                        }
                    }

                    if(!updatedPc)
                    {
                        preconditions.push_back(n_pc);
                    }      
                }
            }
    };
}
#endif
