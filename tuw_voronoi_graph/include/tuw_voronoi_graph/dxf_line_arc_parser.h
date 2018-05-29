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

#ifndef DXF_CREATION_INTERFACE_H
#define DXF_CREATION_INTERFACE_H

#include <dxflib/dl_creationadapter.h>
#include <dxflib/dl_entities.h>
#include <vector>

namespace tuw_graph
{
    class DxfLineArcParser : public DL_CreationAdapter 
    {
    public:
        //overloaded functions for receiving the dxf entities
        virtual void addLine(const DL_LineData& _line);
        virtual void addArc(const DL_ArcData& _arc);
        virtual void addCircle(const DL_CircleData& _circle);
        virtual void addImage(const DL_ImageData& _image);
        /**
         * @brief resets the creation interface 
         */
        void reset();
        /**
         * @return a list of all found lines
         */
        const std::vector<DL_LineData> &getLines();
        /**
         * @return a list of all found arcs
         */
        const std::vector<DL_ArcData> &getArcs();
        /**
         * @return a list of all found circles
         */
        const std::vector<DL_CircleData> &getCircles();
        /**
         * @return a list of all found images
         */
        const std::vector<DL_ImageData> &getImage();
    private:
        std::vector<DL_LineData> lines_;
        std::vector<DL_ArcData> arcs_;
        std::vector<DL_CircleData> circles_;
        std::vector<DL_ImageData> images_;
        bool imageFound_ = false;
        bool error_ = false;
    };
}

#endif