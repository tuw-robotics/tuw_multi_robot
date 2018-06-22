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

#ifndef POTENTIAL_CALCULATOR_H
#define POTENTIAL_CALCULATOR_H

#include <tuw_global_router/srr_utils.h>

namespace multi_robot_router
{
class PotentialCalculator
{
  public:
    /**
             * @brief constructor
             */
    PotentialCalculator();
    /**
            * @brief constructor
            * @param _multiplier the multiplier used to multiply the potential with (used for reducing robots speeds)
            */
    PotentialCalculator(const float &_multiplier);
    /**
            * @brief sets the Potential multiplier
            * @param _multiplier the multiplier used to multiply the potential with (used for reducing robots speeds)
            */
    void SetMultiplier(const float &_multiplier);
    /**
             * @brief calculates the potential for a vertex
             * @param _vertex the vertext used for the calculation
             * @returns the calculated Potential 
             */
    float CalculatePotential(const Vertex &_vertex) const;
    /**
             * @brief calculates the potential for a segment
             * @param _segment the segment used for the calculation
             * @returns the calculated Potential 
             */
    float CalculatePotential(const Segment &_segment) const;

  private:
    float multiplier_;
};
} // namespace multi_robot_router
#endif
