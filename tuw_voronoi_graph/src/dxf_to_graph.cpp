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

namespace tuw_graph
{    
    void DxfToGraph::parseGraph(const std::string &_dxfPath, const float _segLength, const float _segWidth)
    {
        DxfLineArcParser creationInterface;

        DL_Dxf dxf;

        if(!dxf.in("/home/benjamin/temp/drawing.dxf", &creationInterface))
        {
            std::cerr << "drawing.dxf could not be opened.\n";
            return;
        }
        
        
        const std::vector<DL_LineData> lines = creationInterface.getLines();
        for(const DL_LineData &line : lines)
        {
            std::cout << "line: " << line.x1 << "/" << line.y1 << "\t-\t" << line.x2 << "/" << line.y2 << std::endl;
            std::vector<Line> segs = splitLine(line, _segLength);
            for(const Line &s : segs)
            {
                std::cout << "  seg: " << s.start[0] << "/" << s.start[1] << "\t-\t" << s.end[0] << "/" << s.end[1] << std::endl;
            }
            
        }
        
        const std::vector<DL_CircleData> circles = creationInterface.getCircles();
        for(const DL_CircleData &circle : circles)
        {
            std::cout << "circle: " << circle.cx << "/" << circle.cy << " r: " << circle.radius << std::endl;
        }
        
        const std::vector<DL_ArcData> arcs = creationInterface.getArcs();
        for(const DL_ArcData &arc : arcs)
        {
            std::cout << "arc: " << arc.cx << "/" << arc.cy << " r: " << arc.radius << " a_1: " << arc.angle1 << " a_2: " << arc.angle2 << std::endl;
        }
    }
    

    std::vector< Line > DxfToGraph::splitLine(const DL_LineData &_line, const float _segLength)
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
        
        return segments;
    }
    
    std::vector< Segment > DxfToGraph::generateGraph(const std::vector< Eigen::Vector2d > &_lines, const float _segWidth)
    {
        std::vector<Segment> todo;
        return todo;
    }


    
    

    void DxfToGraph::serializeGraph(const std::string &_graphPath)
    {

    }

}
