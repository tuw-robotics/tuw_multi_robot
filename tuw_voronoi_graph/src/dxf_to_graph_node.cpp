/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <tuw_voronoi_graph/dxf_to_graph_node.h>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "dxf_graph_node");   
    ros::NodeHandle n;
    
    tuw_graph::DxfToGraphNode dxf2graph (n);
    dxf2graph.writeGraph();
    
    return 0;
}


namespace tuw_graph
{
    DxfToGraphNode::DxfToGraphNode(ros::NodeHandle &n) :
        DxfToGraph(),
        n_(n),
        n_param_("~")
    {
        dxfPath_ = "/home/benjamin/temp/roblab.dxf";
        n_param_.param("dxf_path", dxfPath_, dxfPath_);

        segmentLength_ = 0.6;   //meter
        n_param_.param("segment_length", segmentLength_, segmentLength_);

        segmentWidth_ = 1.0;   //meter
        n_param_.param("segment_width", segmentWidth_, segmentWidth_);
        
        graphPath_ = "/home/benjamin/temp/roblab_graph";
        n_param_.param("dxf_path", graphPath_, graphPath_);
        if(graphPath_.back() != '/')
            graphPath_ += "/";
        
    }

    void DxfToGraphNode::writeGraph()
    {
        parseGraph(dxfPath_, segmentLength_, segmentWidth_);
        serializeGraph(graphPath_);
    }
}