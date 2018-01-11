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


#ifndef VELOCITY_CALCULATOR_H
#define VELOCITY_CALCULATOR_H

#include <tuw_global_planner/utils.h>

class VelocityCalculator
{
public:		VelocityCalculator(int nr_of_paths);
public:		void Init(int nr_of_paths);
public:		bool CalculateProfile(std::vector< std::vector< PathSegment > > _pathsWithPreconditions, std::vector< std::vector< float > >& _relativeVelocityProfile);
private:	bool MovePath(std::vector<PathSegment> &_pathWithPreconditions, int _pathNr);
private:	bool MovePathUntil(std::vector<PathSegment> &_pathWithPreconditions, int _pathNr, int _stepNr);

private:	std::vector<int> pathStep_;
private:	std::vector<std::vector<float>> actualPathSteps_;
private:	std::vector<bool> lockedPath_;
private:	std::vector<std::vector<PathSegment>> paths_;
};

#endif