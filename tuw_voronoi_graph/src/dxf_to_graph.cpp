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
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <tuw_voronoi_graph/dxf_to_graph.h>
#include <dxflib/dl_dxf.h>
#include <tuw_voronoi_graph/dxf_line_arc_parser.h>
#include <sstream>

#include <math.h>

namespace tuw_graph
{
    bool DxfToGraph::parseGraph(const std::string &_dxfPath, const float _segLength, const float _segWidth)
    {
        DxfLineArcParser creationInterface;
        DL_Dxf dxf;

        if(!dxf.in(_dxfPath, &creationInterface))
        {
            std::cerr << "\t" << _dxfPath << " could not be opened.\n";
            return false;
        }

        std::vector<Line> lineSegments;
        const std::vector<DL_LineData> lines = creationInterface.getLines();

        for(const DL_LineData & line : lines)
        {
            std::vector<Line> segs = splitLine(line, _segLength);
            lineSegments.insert(lineSegments.end(), segs.begin(), segs.end());
        }

        const std::vector<DL_CircleData> circles = creationInterface.getCircles();

        for(const DL_CircleData & circle : circles)
        {
            std::vector<Line> segs = splitCircle(circle, _segLength);
            lineSegments.insert(lineSegments.end(), segs.begin(), segs.end());
        }

        const std::vector<DL_ArcData> arcs = creationInterface.getArcs();

        for(const DL_ArcData & arc : arcs)
        {
            std::vector<Line> segs = splitArc(arc, _segLength);
            lineSegments.insert(lineSegments.end(), segs.begin(), segs.end());
        }

        
        std::vector<DL_ImageData> image = creationInterface.getImage();
        if(image.size() > 1)
        {
            std::cerr << "\tToo many images in the dxf file" << std::endl;
            return false;
        }
        
        if(!getGraphData(image[0], scale_, offset_))
        {
            std::cerr << "\tInconsistant scale in image" << std::endl;
            return false;
        }
        
        graphData_ = generateGraph(lineSegments, _segWidth, scale_, offset_);
        
        return true;
    }


    std::vector< Line > DxfToGraph::splitLine(const DL_LineData &_line, const float _segLength) const
    {
        std::vector<Line> segments;
        Eigen::Vector2d start(_line.x1, _line.y1);
        Eigen::Vector2d end(_line.x2, _line.y2);

        float length = (end - start).norm();
        uint32_t splits = length / _segLength;

        Eigen::Vector2d increment = (end - start) / splits;
        Eigen::Vector2d current = start;

        for(uint32_t i = 0; i < splits; i++)
        {
            Line l(current, current + increment);
            segments.push_back(l);
            current += increment;
        }
        
        if(segments.size() == 0)
        {
            Line l(start, end);
            segments.push_back(l);
        }

        return segments;
    }

    std::vector< Line > DxfToGraph::splitCircle(const DL_CircleData &_circle, const float _segLength) const
    {
        DL_ArcData _arc(_circle.cx, _circle.cy, _circle.cz, _circle.radius, 0, 360);
        return splitArc(_arc, _segLength);
    }


    std::vector< Line > DxfToGraph::splitArc(const DL_ArcData &_arc, const float _segLength) const
    {
        std::vector<Line> segments;

        //Take care DL_ uses Degrees
        float angle1 = _arc.angle1 / 180 * M_PI;
        float angle2 = _arc.angle2 / 180 * M_PI;;

        if(_arc.angle1 >= _arc.angle2)
            angle2 += 2 * M_PI;

        float arcLength = (angle2 - angle1) * _arc.radius;
        uint32_t splits = arcLength / _segLength;

        float angleIncrement_radians = (arcLength / (float)splits) / _arc.radius;
        float current_angle_radians = angle1;
        Eigen::Vector2d current_point(_arc.cx + _arc.radius * cos(current_angle_radians), _arc.cy + _arc.radius * sin(current_angle_radians));

        for(uint32_t i = 0; i < splits; i++)
        {
            current_angle_radians += angleIncrement_radians;
            Eigen::Vector2d nextPoint(_arc.cx + _arc.radius * cos(current_angle_radians), _arc.cy + _arc.radius * sin(current_angle_radians));
            Line l(current_point, nextPoint);
            segments.push_back(l);

            current_point = nextPoint;
        }

        return segments;
    }

    bool DxfToGraph::getGraphData(const DL_ImageData &_image, float &_scale, Eigen::Vector2d &_offset) const
    {
        //Rotation doesnt matter because the lines are drawn in the right direction...
        //Scales should be equal (u, v)
        //Offset ip

        _offset[0] = _image.ipx;
        _offset[1] = _image.ipy;

        float scale_u = sqrt((_image.ux * _image.ux) + (_image.uy * _image.uy));
        float scale_v = sqrt((_image.vx * _image.vx) + (_image.vy * _image.vy));

        if(scale_u != scale_v)
            return false;

        _scale = scale_u;

        return true;
    }


    std::vector< Segment > DxfToGraph::generateGraph(const std::vector< Line > &_lines, const float _segWidth, const float &_scale, const Eigen::Vector2d &_offset) const
    {
        std::vector<Segment> segments;
        Segment::resetId();
        
        //Create segments
        for(uint32_t i = 0; i < _lines.size(); i++)
        {
            std::vector<Eigen::Vector2d> points({ (_lines[i].start-_offset)/scale_, (_lines[i].end-_offset)/scale_ });
            Segment s(points , _segWidth/scale_);
            
            segments.push_back(s);
        }
        
        
        //Assign Neighbors
        for(uint32_t i = 0; i < _lines.size(); i++)
        {
            for(uint32_t j = 0; j < _lines.size(); j++)
            {
                if(i != j)
                {
                    if(((segments[i].getStart() - segments[j].getStart()).norm() < _scale) || 
                        ((segments[i].getStart() - segments[j].getEnd()).norm() < _scale))
                    {
                        segments[i].addPredecessor(j);                        
                    }
                    if(((segments[i].getEnd() - segments[j].getStart()).norm() < _scale) || 
                        ((segments[i].getEnd() - segments[j].getEnd()).norm() < _scale))
                    {
                        segments[i].addSuccessor(j);    
                    }
                }
            }
        }
        
        return segments;
    }





    void DxfToGraph::serializeGraph(const std::string &_graphPath) const
    {
        Serializer s;
        s.save(_graphPath, graphData_, offset_, scale_);
    }

}
