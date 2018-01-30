#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tuw_voronoi_map/voronoi_path_generator.h>
#include <tuw_voronoi_graph/voronoi_graph_node.h>
#include <memory>
#include <boost/functional/hash.hpp>

void publishTf(std::string mapName);

bool allowPublish = false;

int main(int argc, char** argv)
{

    ros::init(argc, argv, "voronoi_graph_node");     /// initializes the ros node with default name
    ros::NodeHandle n;

    voronoi_graph::VoronoiGeneratorNode mapNode(n);

    ros::Rate r(0.5);

    ROS_INFO("Initialization done!");

    while(ros::ok())
    {
        ros::spinOnce();

        if(allowPublish)
            mapNode.publishMap();

        r.sleep();
    }

    return 0;
}


namespace voronoi_graph
{
    /**
     * Constructor
     **/

    VoronoiGeneratorNode::VoronoiGeneratorNode(ros::NodeHandle & n) :  voronoi_graph::VoronoiPathGenerator(),
        n_(n),
        n_param_("~")
    {

        topicGlobalMap_ = "/map";
        n.param("map_topic", topicGlobalMap_, topicGlobalMap_);

        topicVoronoiMap_ = "voronoi_map";
        n.param("voronoi_topic", topicVoronoiMap_, topicVoronoiMap_);

        frameGlobalMap_ = "map";
        n.param("map_frame", frameGlobalMap_, frameGlobalMap_);
        frameVoronoiMap_ = "voronoi_map";
        n.param("voronoi_frame", frameVoronoiMap_, frameVoronoiMap_);

        subMap_       = n.subscribe(topicGlobalMap_, 1, &VoronoiGeneratorNode::globalMapCallback, this);
        pubVoronoiMap_    = n.advertise<nav_msgs::OccupancyGrid>(topicVoronoiMap_   , 1);

        //ros::Rate(1).sleep();
        ros::spinOnce();
    }

    void VoronoiGeneratorNode::globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& _map)
    {
        std::vector<signed char> map = _map->data;

        Eigen::Vector2d origin;
        origin[0] = _map->info.origin.position.x;
        origin[1] = _map->info.origin.position.y;

        size_t new_hash = getHash(map, origin, _map->info.resolution);

        if(new_hash != current_map_hash_)
        {
            origin_[0] = _map->info.origin.position.x;
            origin_[1] = _map->info.origin.position.y;
            resolution_ = _map->info.resolution;

            cv::Mat m(_map->info.height, _map->info.width, CV_8SC1, map.data());
            computeDistanceField(m, distField_);
            computeVoronoiMap(distField_, voronoiMap_);
            
            allowPublish = true;
        }
    }

    void VoronoiGeneratorNode::publishMap() //DEBUG
    {
        ROS_INFO("VorPub: %i %i", voronoiMap_.rows, voronoiMap_.cols);
        nav_msgs::OccupancyGrid grid;
        // Publish Whole Grid
        grid.header.frame_id = "map";
        grid.header.stamp = ros::Time::now();
        grid.info.resolution = resolution_;

        int nx = voronoiMap_.cols;
        int ny = voronoiMap_.rows;
        
        grid.info.width = nx;
        grid.info.height = ny;

        double wx, wy;
        //costmap_->mapToWorld(0, 0, wx, wy);
        grid.info.origin.position.x = origin_[0];
        grid.info.origin.position.y =  origin_[1];
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;

        grid.data.resize(nx * ny);


        for(unsigned int i = 0; i < grid.data.size(); i++)
        {
            grid.data[i] = (voronoiMap_.data[i]);

        }

        pubVoronoiMap_.publish(grid);
    }

    std::size_t VoronoiGeneratorNode::getHash(const std::vector<signed char> &_map, Eigen::Vector2d _origin, float _resolution)
    {
        std::size_t seed = 0;

        boost::hash_combine(seed, _origin[0]);
        boost::hash_combine(seed, _origin[1]);
        boost::hash_combine(seed, _resolution);

        for(const signed char & val : _map)
        {
            boost::hash_combine(seed, val);
        }

        return seed;
    }
}
