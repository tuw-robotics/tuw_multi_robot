#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tuw_voronoi_map/voronoi_path_generator.h>
#include <tuw_voronoi_graph/voronoi_graph_node.h>
#include <tuw_voronoi_graph/voronoi_graph_generator.h>
#include <memory>
#include <tuw_multi_robot_msgs/Graph.h>
#include <string>

void publishTf(std::string mapName);


int main(int argc, char **argv)
{

    ros::init(argc, argv, "voronoi_graph_node"); /// initializes the ros node with default name
    ros::NodeHandle n;

    tuw_graph::VoronoiGeneratorNode mapNode(n);

    return 0;
}

namespace tuw_graph
{
VoronoiGeneratorNode::VoronoiGeneratorNode(ros::NodeHandle &n) : voronoi_map::VoronoiPathGenerator(), VoronoiGraphGenerator(), Serializer(), n_(n), n_param_("~")
{


    double loop_rate;
    n_param_.param<double>("loop_rate", loop_rate, 0.1);

    n_param_.param<bool>("publish_voronoi_map_image", publishVoronoiMapImage_, false);
    
    n_param_.param<double>("map_inflation", inflation_, 0.1); /// [meters]

    n_param_.param<float>("segment_length", segment_length_, 1.0); /// [meters]


    crossingOptimization_ = 0.2;
    n_param_.param<float>("opt_crossings", crossingOptimization_, 0.2);

    n_param_.param<float>("opt_end_segments", endSegmentOptimization_, 0.2);
    //endSegmentOptimization_ = std::min<float>(endSegmentOptimization_, 0.7 * path_length_);

    n_param_.param<std::string>("graph_cache_path", graphCachePath_, "/tmp");

    if (graphCachePath_.back() != '/'){
        graphCachePath_ += "/";
    }
    n_param_.param<std::string>("custom_graph_path", customGraphPath_, "");

    if (customGraphPath_.back() != '/' && customGraphPath_.size() != 0){
        customGraphPath_ += "/";
    }

    subMap_ = n.subscribe("map", 1, &VoronoiGeneratorNode::globalMapCallback, this);
    if(publishVoronoiMapImage_){
        pubVoronoiMapImage_    = n.advertise<nav_msgs::OccupancyGrid>( "map_eroded", 1);
    }
    pubSegments_ = n.advertise<tuw_multi_robot_msgs::Graph>("segments", 1);


    ros::Rate r(loop_rate);

    ROS_INFO("Initialization done!");

    while (ros::ok())
    {
        ros::spinOnce();
            
        publishSegments();

        r.sleep();
    }

}

void VoronoiGeneratorNode::globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &_map)
{
    std::vector<signed char> map = _map->data;

    
    std::vector<double> parameters;
    parameters.push_back(_map->info.origin.position.x);
    parameters.push_back(_map->info.origin.position.y);
    parameters.push_back(_map->info.resolution);
    parameters.push_back(inflation_);
    parameters.push_back(segment_length_);
    parameters.push_back(endSegmentOptimization_);
    parameters.push_back(crossingOptimization_);

    size_t new_hash = getHash(map, parameters);
    

    if (customGraphPath_.size() == 0)
    {
        if (new_hash != current_map_hash_ )
        {
            if (!loadGraph(new_hash) )
            {
                ROS_INFO("Graph generator: Graph not found! Generating new one!");
                createGraph(_map, new_hash);
            }
            else
            {
                ROS_INFO("Graph generator: Graph loaded from memory");
            }

            current_map_hash_ = new_hash;
        }
    }
    else
    {
        ROS_INFO("loading custom graph from: %s", customGraphPath_.c_str());
        if (loadCustomGraph(customGraphPath_))
            ROS_INFO("graph loaded");
        else
            ROS_INFO("failed to load graph");
    }
    
    if (publishVoronoiMapImage_ && (map_.size > 0))
    {
        voronoiMapImage_.header = _map->header;
        voronoiMapImage_.info = _map->info;
        voronoiMapImage_.data.resize(_map->data.size());

        for (unsigned int i = 0; i < voronoiMapImage_.data.size(); i++)
        {
            voronoiMapImage_.data[i] = map_.data[i];
        }
        pubVoronoiMapImage_.publish(voronoiMapImage_);
    }
    
}

void VoronoiGeneratorNode::createGraph(const nav_msgs::OccupancyGrid::ConstPtr &_map, size_t _map_hash)
{
    std::vector<signed char> map = _map->data;

    segments_.clear();
    Segment::resetId();

    ROS_INFO("Graph generator: Computing distance field ...");
    origin_[0] = _map->info.origin.position.x;
    origin_[1] = _map->info.origin.position.y;
    resolution_ = _map->info.resolution;

    cv::Mat m(_map->info.height, _map->info.width, CV_8SC1, map.data());
    prepareMap(m, map_, inflation_ / _map->info.resolution);
    computeDistanceField(map_, distField_);

    ROS_INFO("Graph generator: Computing voronoi graph ...");
    computeVoronoiMap(distField_, voronoiMap_);

    ROS_INFO("Graph generator: Generating graph ...");
    potential.reset(new float[m.cols * m.rows]);
    float pixel_path_length = segment_length_ / resolution_;
    segments_ = calcSegments(m, distField_, voronoiMap_, potential.get(), pixel_path_length, crossingOptimization_ / resolution_, endSegmentOptimization_ / resolution_);

    //Check Directroy
    save(graphCachePath_ + std::to_string(_map_hash) + "/", segments_, origin_, resolution_, map_);
    ROS_INFO("Graph generator: Created new Graph %lu", _map_hash);
}

bool VoronoiGeneratorNode::loadGraph(std::size_t _hash)
{
    segments_.clear();
    Segment::resetId();
    return load(graphCachePath_ + std::to_string(_hash) + "/", segments_, origin_, resolution_, map_);
}

bool VoronoiGeneratorNode::loadCustomGraph(std::string _path)
{
    segments_.clear();
    Segment::resetId();
    return load(_path, segments_, origin_, resolution_);
}

void VoronoiGeneratorNode::publishSegments()
{
    tuw_multi_robot_msgs::Graph graph;
    graph.header.frame_id = "map";
    graph.header.seq = 0;
    graph.header.stamp = ros::Time::now();

    graph.resolution = resolution_;

    graph.origin.position.x = origin_[0]; //TODO test
    graph.origin.position.y = origin_[1]; //TODO test

    for (auto it = segments_.begin(); it != segments_.end(); ++it)
    {
        tuw_multi_robot_msgs::Vertex seg;

        seg.id = (*it).getId();
        seg.weight = (*it).getLength();
        seg.width = (*it).getMinPathSpace();
        seg.valid = true;
        std::vector<Eigen::Vector2d> path = (*it).getPath();

        for (uint32_t i = 0; i < path.size(); i++)
        {
            geometry_msgs::Point pos;
            pos.x = path[i][0];
            pos.y = path[i][1];
            pos.z = 0;

            seg.path.push_back(pos);
        }

        //ROS_INFO("distORIG: %i/%i", (*it)->GetPredecessors().size(), (*it)->GetSuccessors().size());
        std::vector<uint32_t> predecessors = (*it).getPredecessors();

        for (uint32_t i = 0; i < predecessors.size(); i++)
        {
            seg.predecessors.push_back(predecessors[i]);
        }

        std::vector<uint32_t> successors = (*it).getSuccessors();

        for (uint32_t i = 0; i < successors.size(); i++)
        {
            seg.successors.push_back(successors[i]);
        }

        graph.vertices.push_back(seg);
    }

    pubSegments_.publish(graph);
}

} // namespace tuw_graph
