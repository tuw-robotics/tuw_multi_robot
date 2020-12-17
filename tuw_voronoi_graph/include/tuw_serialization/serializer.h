
#ifndef _TUW_VORONOI_SERIALIZER
#define _TUW_VORONOI_SERIALIZER

#include <nav_msgs/OccupancyGrid.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <tuw_voronoi_graph/segment.h>

#include <fstream>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

#define DEFAULT_MAP_NAME    "voronoi_map"
#include <eigen3/Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/functional/hash.hpp>

namespace tuw_graph
{

    class PointSerializer
    {
        public:
            PointSerializer() : PointSerializer(0, 0)
            {}
            PointSerializer(float _x, float _y)
            {
                x = _x;
                y = _y;
            }
            PointSerializer(Eigen::Vector2d p)
            {
                x = p[0];
                y = p[1];
            }
            float x;
            float y;
        private:
            friend class boost::serialization::access;
            template<class archive> void serialize(archive & ar, const unsigned int version)
            {
                using boost::serialization::make_nvp;
                ar & boost::serialization::make_nvp("x", x);
                ar & boost::serialization::make_nvp("y", y);
            }
    };

    class GraphInfo
    {
        public:
            GraphInfo() : Origin(0, 0)
            {};
            GraphInfo(Eigen::Vector2d _pt, float _resolution, int _nrSegments_) : Origin(_pt)
            {
                Resolution = _resolution;
                SegmentLength = _nrSegments_;
            }
            PointSerializer Origin;
            float Resolution;
            int SegmentLength;

        private:
            friend class boost::serialization::access;
            template<class archive> void serialize(archive & ar, const unsigned int version)
            {
                using boost::serialization::make_nvp;
                ar & boost::serialization::make_nvp("Origin", Origin);
                ar & boost::serialization::make_nvp("Resolution", Resolution);
                ar & boost::serialization::make_nvp("SegmentLength", SegmentLength);
            }
    };

    class TreeInfo
    {
        public:
            TreeInfo(std::vector<Segment> _segs) : TreeInfo(_segs.size())
            {
                for(uint32_t i = 0; i < _segs.size(); i++)
                {
                    int * pred = Predecessors.get();
                    int * succ = Successors.get();
                    int * point = Points.get();
                    pred[i] = _segs[i].getPredecessors().size();
                    succ[i] = _segs[i].getSuccessors().size();
                    point[i] = _segs[i].getPath().size();
                }
            }
            TreeInfo(int _length)
            {
                Predecessors.reset(new int[_length]);
                Successors.reset(new int[_length]);
                Points.reset(new int[_length]);
                Length = _length;
                //predLength = _length;
                
            }
            int Length;
            std::unique_ptr<int> Predecessors;
            std::unique_ptr<int> Successors;
            std::unique_ptr<int> Points;

        private:
            //int predLength;
            //int succLength;
            //int pointsLength;
            friend class boost::serialization::access;
            template<class archive> void serialize(archive & ar, const unsigned int version)
            {
                using boost::serialization::make_nvp;
                ar & boost::serialization::make_nvp("Length", Length);
                ar & boost::serialization::make_array<int>(Predecessors.get(), Length);
                ar & boost::serialization::make_array<int>(Successors.get(), Length);
                ar & boost::serialization::make_array<int>(Points.get(), Length);
            }
    };



    class SegmentSerializer
    {
        public:
            SegmentSerializer() : SegmentSerializer(0, 0, 0)
            {
            }

            SegmentSerializer(int _predLength, int _succLength, int _pointLength)
            {
                predecessorLength = _predLength;
                successorLength = _succLength;
                pointLength = _pointLength;
                predecessors.reset(new int[predecessorLength]);
                successors.reset(new int[successorLength]);
                points.reset(new PointSerializer[pointLength]);

            }

            SegmentSerializer(const Segment &_s) :
                SegmentSerializer(_s.getId(), _s.getPredecessors(), _s.getSuccessors(), _s.getMinPathSpace(), _s.getPath())
            {
            }

            SegmentSerializer(const uint32_t _id, std::vector<uint32_t> _predecessors, std::vector<uint32_t> _successors, float _minDistance, std::vector<Eigen::Vector2d> _points):
                SegmentSerializer(_predecessors.size(), _successors.size(), _points.size())
            {
                id = _id;
                minDistance = _minDistance;

                int *pred = predecessors.get();
                int *succ = successors.get();
                PointSerializer *pts = points.get();


                for(int i = 0; i < predecessorLength; i++)
                {
                    pred[i] = _predecessors[i];
                }

                for(int i = 0; i < successorLength; i++)
                {
                    succ[i] = _successors[i];
                }

                for(uint32_t i = 0; i < _points.size(); i++)
                {
                    PointSerializer p(_points[i]);
                    pts[i] = p;
                }
            }

            int id;
            int predecessorLength;
            int successorLength;
            std::unique_ptr<int> predecessors;
            std::unique_ptr<int> successors;
            float minDistance;
            int pointLength;
            std::unique_ptr<PointSerializer> points;

        private:
            friend class boost::serialization::access;
            template<class archive> void serialize(archive & ar, const unsigned int version)
            {
                using boost::serialization::make_nvp;
                ar & boost::serialization::make_nvp("id", id);
                ar & boost::serialization::make_nvp("predecessorLength", predecessorLength);
                ar & boost::serialization::make_nvp("successorLength", successorLength);
                ar & boost::serialization::make_nvp("minDistance", minDistance);
                ar & boost::serialization::make_nvp("pointLength", pointLength);
                ar & boost::serialization::make_array<int>(predecessors.get(), predecessorLength);
                ar & boost::serialization::make_array<int>(successors.get(), successorLength);
                ar & boost::serialization::make_array<PointSerializer>(points.get(), pointLength);
            }
    };


    class GraphSerializer
    {
        public:
            GraphSerializer(std::vector<SegmentSerializer> &_segments)
            {
                segments_ = &_segments[0];
                Length = _segments.size();
            }
            int Length;
            SegmentSerializer *segments_;
        private:
            friend class boost::serialization::access;
            template<class archive> void serialize(archive & ar, const unsigned int version)
            {
                using boost::serialization::make_nvp;
                ar & boost::serialization::make_nvp("pointLength", Length);
                ar & boost::serialization::make_array<SegmentSerializer>(segments_, Length);
            }
    };

    class Serializer
    {
        public:
            Serializer();
            /** 
             * @brief saves the graph to a specific path in xml format
             * @param _mapPath the save path of the graph
             * @param _segs the segments generated Segments for the graph
             * @param _origin the origin of the graph
             * @param _resolution the resolution of the graph 
             */
            void save(const std::string &_mapPath, const std::vector<Segment> &_segs, const Eigen::Vector2d &_origin, const float &_resolution);
            /** 
             * @brief saves the graph to a specific path in xml format
             * @param _mapPath the save path of the graph
             * @param _segs the segments generated Segments for the graph
             * @param _origin the origin of the graph
             * @param _resolution the resolution of the graph 
             * @param _map map on which the graph is based
             */
            void save(const std::string &_mapPath, const std::vector<Segment> &_segs, const Eigen::Vector2d &_origin, const float &_resolution, const cv::Mat &_map);
            /** 
             * @brief loads a graph from memory which is saved in plain text
             * @param _mapPath the save path of the graph
             * @param _segs the segments generated Segments for the graph
             * @param _origin the origin of the graph
             * @param _resolution the resolution of the graph 
             */
            bool load(const std::string &_mapPath, std::vector<Segment> &_segs, Eigen::Vector2d &_origin, float &_resolution);
            /** 
             * @brief loads a graph from memory which is saved in plain text
             * @param _mapPath the save path of the graph
             * @param _segs the segments generated Segments for the graph
             * @param _origin the origin of the graph
             * @param _resolution the resolution of the graph 
             * @param _map map on which the graph is based
             */
            bool load(const std::string &_mapPath, std::vector<Segment> &_segs, Eigen::Vector2d &_origin, float &_resolution, cv::Mat &_map);
            /**
             * @brief generate a hash from a _map
             * @param _map the map data used for the hash 
             * @param _parameters parameters to check for changes
             */
            size_t getHash(const std::vector<signed char> &_map, const std::vector<double> &_parameters) const;
    };

}

#endif
