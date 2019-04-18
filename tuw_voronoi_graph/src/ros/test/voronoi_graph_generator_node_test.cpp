#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <ros/ros.h>
#include "ros/voronoi_graph_generator_node.h"

class VoronoiGraphGeneratorNodeTest : public ::testing::Test
{
public:
    void SetUp() override
    {
        int argc = 0;
        char **args = {};
        ros::init(argc, args, "tester");
        ros::NodeHandle nh;
        ros::NodeHandle param("~");
        param.setParam("publish_voronoi_map_image", publish_voronoi_map_image);
        param.setParam("map_inflation", inflation);
        param.setParam("segment_length", segment_length);
        param.setParam("opt_crossings", opt_crossings);
        param.setParam("opt_end_segments", opt_end_segments);
        param.setParam("graph_cache_path", graph_cache_path);
        param.setParam("custom_graph_path", custom_graph_path);
        nh_ptr_ = std::make_shared<ros::NodeHandle>(nh);
        param_ptr_ = std::make_shared<ros::NodeHandle>(param);
    }

    void TearDown() override
    {
        ros::shutdown();
    }

protected:
    std::shared_ptr<ros::NodeHandle> param_ptr_;
    std::shared_ptr<ros::NodeHandle> nh_ptr_;

private:
    bool publish_voronoi_map_image = false;
    double inflation = 0.0;
    float segment_length = 0.0f;
    float opt_crossings = 0.0f;
    float opt_end_segments = 0.0f;
    std::string graph_cache_path = "";
    std::string custom_graph_path = "";
};

TEST_F(VoronoiGraphGeneratorNodeTest, initializes)
{
    ASSERT_NO_THROW(tuw_graph::VoronoiGraphGeneratorNode n(*nh_ptr_)) << "Graph generator did not initialize correctly.";
}

TEST_F(VoronoiGraphGeneratorNodeTest, sets_params_correctly)
{
    auto publish_voronoi_map_image = true;
    auto inflation = 1.0;
    auto segment_length = 2.0f;
    auto opt_crossings = 3.0f;
    auto opt_end_segments = 4.0f;
    auto graph_cache_path = "cache/";
    auto custom_graph_path = "custom/";
    param_ptr_->setParam("publish_voronoi_map_image", publish_voronoi_map_image);
    param_ptr_->setParam("map_inflation", inflation);
    param_ptr_->setParam("segment_length", segment_length);
    param_ptr_->setParam("opt_crossings", opt_crossings);
    param_ptr_->setParam("opt_end_segments", opt_end_segments);
    param_ptr_->setParam("graph_cache_path", graph_cache_path);
    param_ptr_->setParam("custom_graph_path", custom_graph_path);

    tuw_graph::VoronoiGraphGeneratorNode n(*nh_ptr_);

    EXPECT_EQ(n.publish_voronoi_map_image(), publish_voronoi_map_image) << "Did not set 'publish_voronoi_map_image' parameter correctly.";
    EXPECT_EQ(n.inflation(), inflation) << "Did not set 'inflation' parameter correctly.";
    EXPECT_EQ(n.segment_length(), segment_length) << "Did not set 'segment_length' parameter correctly.";
    EXPECT_EQ(n.crossing_optimization(), opt_crossings) << "Did not set 'opt_crossings' parameter correctly.";
    EXPECT_EQ(n.end_segment_optimization(), opt_end_segments) << "Did not set 'opt_end_segments' parameter correctly.";
    EXPECT_STREQ(n.graph_cache_path().c_str(), graph_cache_path) << "Did not set 'graph_cache_path' parameter correctly.";
    EXPECT_STREQ(n.custom_graph_path().c_str(), custom_graph_path) << "Did not set 'custom_graph_path' parameter correctly.";
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}