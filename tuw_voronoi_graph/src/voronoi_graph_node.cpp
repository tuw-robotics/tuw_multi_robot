#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tuw_voronoi_map/voronoi_path_generator.h>
#include <tuw_voronoi_graph/voronoi_graph_node.h>
#include <tuw_voronoi_graph/voronoi_graph_generator.h>
#include <memory>
#include <tuw_multi_robot_msgs/Graph.h>
#include <string>

void publishTf(std::string mapName);

bool allowPublish = false;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "voronoi_graph_node"); /// initializes the ros node with default name
    ros::NodeHandle n;

    tuw_graph::VoronoiGeneratorNode mapNode(n);

    ros::Rate r(0.5);

    ROS_INFO("Initialization done!");

    while (ros::ok())
    {
        ros::spinOnce();

        if (allowPublish)
        {
            mapNode.publishMap();
            mapNode.publishSegments();
        }

        r.sleep();
    }

    return 0;
}

namespace tuw_graph
{
VoronoiGeneratorNode::VoronoiGeneratorNode(ros::NodeHandle &n) : voronoi_map::VoronoiPathGenerator(), VoronoiGraphGenerator(), Serializer(), n_(n), n_param_("~")
{

    n_param_.param<std::string>("map_topic", topicGlobalMap_, "/map");

    n_param_.param<int>("map_smoothing", smoothing_, 0);

    n_param_.param<std::string>("voronoi_topic", topicVoronoiMap_, "voronoi_map");

    n_param_.param<std::string>("segments_topic", topicSegments_, "/segments");

    n_param_.param<float>("segment_length", path_length_, 1.0);

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

    subMap_ = n.subscribe(topicGlobalMap_, 1, &VoronoiGeneratorNode::globalMapCallback, this);
    //pubVoronoiMap_    = n.advertise<nav_msgs::OccupancyGrid>(topicVoronoiMap_, 1);                        //DEBUG
    pubSegments_ = n.advertise<tuw_multi_robot_msgs::Graph>(topicSegments_, 1);

    //ros::Rate(1).sleep();
    ros::spinOnce();
}

void VoronoiGeneratorNode::globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &_map)
{
    std::vector<signed char> map = _map->data;

    Eigen::Vector2d origin;
    origin[0] = _map->info.origin.position.x;
    origin[1] = _map->info.origin.position.y;

    size_t new_hash = getHash(map, origin, _map->info.resolution);

    if (customGraphPath_.size() == 0)
    {
        if (new_hash != current_map_hash_)
        {
            if (!loadGraph(new_hash))
            {
                ROS_INFO("Graph generator: Graph not found! Generating new one!");
                createGraph(_map, new_hash);
            }
            else
            {
                ROS_INFO("Graph generator: Graph loaded from memory");
            }

            current_map_hash_ = new_hash;

            allowPublish = true;
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
    prepareMap(m, map_, smoothing_);
    computeDistanceField(map_, distField_);

    ROS_INFO("Graph generator: Computing voronoi graph ...");
    computeVoronoiMap(distField_, voronoiMap_);

    ROS_INFO("Graph generator: Generating graph ...");
    potential.reset(new float[m.cols * m.rows]);
    float pixel_path_length = path_length_ / resolution_;
    segments_ = calcSegments(m, distField_, voronoiMap_, potential.get(), pixel_path_length, crossingOptimization_ / resolution_, endSegmentOptimization_ / resolution_);

    //Check Directroy
    save(graphCachePath_ + std::to_string(_map_hash) + "/", segments_, origin_, resolution_);
    ROS_INFO("Graph generator: Created new Graph %lu", _map_hash);
}

bool VoronoiGeneratorNode::loadGraph(std::size_t _hash)
{
    segments_.clear();
    Segment::resetId();
    return load(graphCachePath_ + std::to_string(_hash) + "/", segments_, origin_, resolution_);
}

bool VoronoiGeneratorNode::loadCustomGraph(std::string _path)
{
    if (!allowPublish)
    {
        segments_.clear();
        Segment::resetId();
        allowPublish = true;
        return load(_path, segments_, origin_, resolution_);
    }
    return false;
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

void VoronoiGeneratorNode::publishMap() //DEBUG
{
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = "map";
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution_;

    int nx = map_.cols;
    int ny = map_.rows;

    grid.info.width = nx;
    grid.info.height = ny;

    //double wx, wy;
    //costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = origin_[0];
    grid.info.origin.position.y = origin_[1];
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
        grid.data[i] = (map_.data[i]);
    }

    //pubVoronoiMap_.publish(grid);
}

} // namespace tuw_graph
