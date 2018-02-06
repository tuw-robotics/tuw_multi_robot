#include <ros/ros.h>
#include <tuw_voronoi_map/serializer.h>
#include <memory>
#include <opencv/cv.hpp>
#include <queue>
#include <string>


#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

using namespace cv;

namespace voronoi_graph
{
#define GRAPH_INFO_NAME     "graphInfo"
#define TREE_INFO_NAME      "treeInfo"
#define DATA_NAME           "data"

    Serializer::Serializer()
    {

    }

    bool Serializer::load(const std::string &_mapPath, std::vector<std::shared_ptr<Segment>> &_segs, Eigen::Vector2d &_origin, float &_resolution)
    {

        boost::filesystem::path graf(_mapPath + GRAPH_INFO_NAME);
        boost::filesystem::path tree(_mapPath + TREE_INFO_NAME);
        boost::filesystem::path data(_mapPath + DATA_NAME);

        if(!boost::filesystem::exists(graf) | !boost::filesystem::exists(tree) | !boost::filesystem::exists(data))
        {
            return false;
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

        int *preds = t.Predecessors.get();

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
        boost::archive::binary_iarchive iaDist(ifsDist);
        iaDist >> graph;

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

            _segs.push_back(std::make_shared<Segment>(pts, graph.segments_[i].minDistance));
        }


        //Add Dependancies
        for(int i = 0; i < graph.Length; i++)
        {
            std::vector<std::shared_ptr<Segment>> predecessors;
            std::vector<std::shared_ptr<Segment>> successors;

            for(int j = 0; j < graph.segments_[i].predecessorLength; j++)
            {
                int *predPtr = graph.segments_[i].predecessors.get();

                if(!_segs[i]->ContainsPredecessor(_segs[predPtr[j]]))
                    _segs[i]->AddPredecessor(_segs[predPtr[j]]);
            }

            for(int j = 0; j < graph.segments_[i].successorLength; j++)
            {
                int *succPtr = graph.segments_[i].successors.get();

                
                int num = succPtr[j];
                std::shared_ptr<Segment> seg = _segs[succPtr[j]];
                
                if(!_segs[i]->ContainsSuccessor(_segs[succPtr[j]]))
                    _segs[i]->AddSuccessor(_segs[succPtr[j]]);
            }


        }

        return true;
    }

    /**
     * @brief saves the map in the given location
     * @param mapInfo the name of the mapInfo file
     **/
    void Serializer::save(const std::string &_mapPath, const std::vector<std::shared_ptr<Segment>> &_segs, const Eigen::Vector2d &_origin, const float &_resolution)
    {
        if(!boost::filesystem::exists(_mapPath))
            boost::filesystem::create_directory(_mapPath);


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
        boost::archive::binary_oarchive oaGraph(ofsGraph);
        oaGraph << graph;
        ofsGraph.close();
    }

}
