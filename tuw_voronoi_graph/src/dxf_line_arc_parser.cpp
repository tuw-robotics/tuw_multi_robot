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

#include <tuw_voronoi_graph/dxf_line_arc_parser.h>
#include <iostream>

namespace tuw_graph
{    
    void DxfLineArcParser::addLine(const DL_LineData &_line)
    {
        //Arc: start / end
        lines_.push_back(_line);
    }

    void DxfLineArcParser::addArc(const DL_ArcData &_arc)
    {
        //Arc: center / radius / angle in pos rotational direction
        arcs_.push_back(_arc);
    }

    void DxfLineArcParser::addCircle(const DL_CircleData &_circle)
    {
        //Arc: center / radius
        circles_.push_back(_circle);
    }

    void DxfLineArcParser::reset()
    {
        lines_.clear();
        arcs_.clear();
        circles_.clear();
    }

    void DxfLineArcParser::addImage(const DL_ImageData &_image)
    {
        //std::cout << "image: h " << _image.height << " w " << _image.width << " ipx " << _image.ipx << " ipy " << _image.ipy << " ref " << _image.ref << " ux " << _image.ux << " uy " << _image.uy << " vx " << _image.vx << " vy " << _image.vy << std::endl;
        //std::cout << "scale: " << sqrt((_image.ux * _image.ux) + (_image.uy * _image.uy)) << std::endl;
        
        
        images_.push_back(_image);
        
    }

    const std::vector<DL_ImageData> &DxfLineArcParser::getImage()
    {
        return images_;
    }

    const std::vector< DL_ArcData > &DxfLineArcParser::getArcs()
    {
        return arcs_;
    }

    const std::vector< DL_CircleData > &DxfLineArcParser::getCircles()
    {
        return circles_;
    }

    const std::vector< DL_LineData > &DxfLineArcParser::getLines()
    {
        return lines_;
    }

}
