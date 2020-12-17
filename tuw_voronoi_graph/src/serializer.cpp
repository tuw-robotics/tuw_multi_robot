#include <ros/ros.h>
#include <tuw_serialization/serializer.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <queue>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>


namespace tuw_graph
{
#define GRAPH_INFO_NAME     "graphInfo"
#define TREE_INFO_NAME      "treeInfo"
#define DATA_NAME           "graphData"
#define MAP_NAME            "map.png"

    Serializer::Serializer()
    {

    }

    std::size_t Serializer::getHash(const std::vector<signed char> &_map, const std::vector<double> &_parameters) const
    {
        std::size_t seed = 0;

        for(const double & val : _parameters)
        {
            boost::hash_combine(seed, val);
        }

        for(const signed char & val : _map)
        {
            boost::hash_combine(seed, val);
        }

        return seed;
    }
    
    
    bool Serializer::load(const std::string &_mapPath, std::vector<Segment> &_segs, Eigen::Vector2d &_origin, float &_resolution)
    {
        {
            boost::filesystem::path graf(_mapPath + GRAPH_INFO_NAME);
            boost::filesystem::path tree(_mapPath + TREE_INFO_NAME);
            boost::filesystem::path data(_mapPath + DATA_NAME);

            if(!boost::filesystem::exists(graf) | !boost::filesystem::exists(tree) | !boost::filesystem::exists(data) )
            {
                return false;
            }
        }


        GraphInfo g;

        std::ifstream ifs(_mapPath + GRAPH_INFO_NAME);
        assert(ifs.good());
        boost::archive::xml_iarchive xml(ifs);
        xml >> boost::serialization::make_nvp("GraphInfo", g);



        TreeInfo t(g.SegmentLength);

        std::ifstream ifti(_mapPath + TREE_INFO_NAME);
        assert(ifti.good());
        boost::archive::xml_iarchive xmlti(ifti);
        xmlti >> boost::serialization::make_nvp("TreeInfo", t);

        _origin[0] = g.Origin.x;
        _origin[1] = g.Origin.y;
        _resolution = g.Resolution;

        std::vector<SegmentSerializer> segs;

        int *pred = t.Predecessors.get();
        int *succ = t.Successors.get();
        int *pts = t.Points.get();

        for(int i = 0; i < t.Length; i++)
        {

            segs.emplace_back(pred[i], succ[i], pts[i]);
        }

        GraphSerializer graph(segs);
        std::ifstream ifsDist(_mapPath + DATA_NAME);
        boost::archive::xml_iarchive iaDist(ifsDist);
        iaDist >> boost::serialization::make_nvp("graph",graph);

        _segs.clear();

        //Generate Segment List
        for(int i = 0; i < graph.Length; i++)
        {
            std::vector<Eigen::Vector2d> pts;

            for(int j = 0; j < graph.segments_[i].pointLength; j++)
            {
                PointSerializer *ptPtr = graph.segments_[i].points.get();
                pts.emplace_back(ptPtr[j].x, ptPtr[j].y);
            }

            Segment s(pts, graph.segments_[i].minDistance);
            _segs.push_back(s);//std::make_shared<Segment>(pts, graph.segments_[i].minDistance));
        }


        //Add Dependancies
        for(int i = 0; i < graph.Length; i++)
        {
            std::vector<uint32_t> predecessors;
            std::vector<uint32_t> successors;

            for(int j = 0; j < graph.segments_[i].predecessorLength; j++)
            {
                int *predPtr = graph.segments_[i].predecessors.get();

                if(!_segs[i].containsPredecessor(predPtr[j]))
                    _segs[i].addPredecessor(predPtr[j]);
            }

            for(int j = 0; j < graph.segments_[i].successorLength; j++)
            {
                int *succPtr = graph.segments_[i].successors.get();
                
                if(!_segs[i].containsSuccessor(succPtr[j]))
                    _segs[i].addSuccessor(succPtr[j]);
            }


        }

            
        return true;
    }
    bool Serializer::load(const std::string &_mapPath, std::vector<Segment> &_segs, Eigen::Vector2d &_origin, float &_resolution, cv::Mat &_map){
        if(load(_mapPath, _segs, _origin, _resolution) && boost::filesystem::exists(boost::filesystem::path(_mapPath + MAP_NAME))){            
             _map = cv::imread(_mapPath + MAP_NAME, cv::IMREAD_GRAYSCALE);
            return true;            
        } else {
            return false;
        }
    }

    /**
     * @brief saves the map in the given location
     * @param mapInfo the name of the mapInfo file
     **/
    void Serializer::save(const std::string &_mapPath, const std::vector<Segment> &_segs, const Eigen::Vector2d &_origin, const float &_resolution)
    {
        if(!boost::filesystem::exists(_mapPath))
            boost::filesystem::create_directories(_mapPath);


        //Save map info (Length of segments)
        GraphInfo info(_origin, _resolution, _segs.size());
        std::ofstream ofs(_mapPath + GRAPH_INFO_NAME);
        assert(ofs.good());
        boost::archive::xml_oarchive oa(ofs);
        oa << boost::serialization::make_nvp("GraphInfo", info);
        ofs.close();

        //Save data strucutre info (Length pred, succ, points)
        TreeInfo tInfo(_segs);
        std::ofstream ofsTree(_mapPath + TREE_INFO_NAME);
        assert(ofsTree.good());
        boost::archive::xml_oarchive ot(ofsTree);
        ot << boost::serialization::make_nvp("TreeInfo", tInfo);
        ofsTree.close();

        //Save data
        std::vector<SegmentSerializer> segs;

        for(const auto & seg : _segs)
        {
            segs.emplace_back(seg);
        }

        GraphSerializer graph(segs);

        std::ofstream ofsGraph(_mapPath + DATA_NAME);
        boost::archive::xml_oarchive oaGraph(ofsGraph);
        oaGraph <<  boost::serialization::make_nvp("graph", graph);
        ofsGraph.close();
        
    }
    void Serializer::save(const std::string &_mapPath, const std::vector<Segment> &_segs, const Eigen::Vector2d &_origin, const float &_resolution, const cv::Mat &_map){
        save(_mapPath, _segs, _origin, _resolution);  
        if(!boost::filesystem::exists(_mapPath))
            boost::filesystem::create_directories(_mapPath);      
        if(_map.size != 0){
            cv::imwrite(_mapPath + MAP_NAME,  _map);
        }
    }

}
