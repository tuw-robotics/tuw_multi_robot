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

#ifndef PATHGENERATOR_H
#define PATHGENERATOR_H

#include <tuw_global_planner/segment.h>
#include <tuw_global_planner/utils.h>
#include <tuw_global_planner/path_coordinator.h>

template <class T>
class PathGenerator
{
    public:
        PathGenerator(std::shared_ptr<Path_Coordinator> _pathQuerry)
        {
            pathQuerry_  = _pathQuerry;
        }

        std::vector<std::vector<T>> generatePath(const std::vector<std::vector<std::shared_ptr<Segment>>>& _paths, const std::vector<Point>& _start, const std::vector<Point>& _end)
        {
            segmentNrs_.clear();
            std::vector<std::vector<T>> generatedPaths;

            for(int i = 0; i < _paths.size(); i++)
            {
                std::vector<T> generatedPath;
                segmentNrs_.emplace_back();

				preparePath(generatedPath, _start[i]);
				
                if(_paths[i].size() == 1)
                {
                    generatedPath.push_back(createElementStartEnd(_paths[i][0], _start[i], _end[i]));
                }
                else
                {
                    for(int j = 0; j < _paths[i].size(); j++)
                    {
                        if(_paths[i][j]->planning.Direction == Segment::none)
                        {
                            generatedPath.clear();
                            return generatedPaths;
                        }


                        T seg;

                        if(j == 0)
                        {
                            seg = createElementStart(_paths[i][j], _start[i]);
                        }
                        else if(j == _paths[i].size() - 1)
                        {
                            seg = createElementEnd(_paths[i][j], _end[i]);
                        }
                        else
                        {
                            seg = createElement(_paths[i][j]);
                        }

                        addPreconditions(seg, _paths[i][j], i, _paths);
                        //generatedPath.push_back(seg);


                        pushBackPath(generatedPaths, generatedPath, seg, _paths[i][j]);
                    }
                }

                generatedPaths.push_back(generatedPath);
            }

            return generatedPaths;
        }

    private:
        T createElement(const std::shared_ptr<Segment>& _element)
        {
            T type;
            return type;
        }
        T createElementStart(const std::shared_ptr<Segment>& _element, Point start)
        {
            return createElement(_element);
        }
        T createElementEnd(const std::shared_ptr<Segment>& _element, Point end)
        {
            return createElement(_element);
        }
        T createElementStartEnd(const std::shared_ptr<Segment>& _element, Point _start, Point _end)
        {
            return createElement(_element);
        }
        void addPreconditions(T& _segment, std::shared_ptr<Segment> _segToFind, int _pathNr, const std::vector<std::vector<std::shared_ptr<Segment>>>& _paths)
        {
        }
        void pushBackPath(const std::vector<std::vector<T>>& _actPath, std::vector<T>& _path, T seg, std::shared_ptr<Segment> originalSegment)
        {
            _path.push_back(seg);
        }
        void preparePath(std::vector<T>& _path, Point _start)
		{ }

    private:
        std::shared_ptr<Path_Coordinator> pathQuerry_;
        std::vector<std::vector<int>> segmentNrs_;

};


template<>
Point PathGenerator<Point>::createElement(const std::shared_ptr<Segment>& _element)
{
    Point pt;

    if(_element->planning.Direction == Segment::start_to_end)
    {
        pt[0] = _element->getStart()[0];
        pt[1] = _element->getStart()[1];
    }
    else
    {
        pt[0] = _element->getEnd()[0];
        pt[1] = _element->getEnd()[1];
    }

    return pt;
}

template<>
void PathGenerator<Point>::pushBackPath(const std::vector<std::vector<Point>>& _actPath, std::vector<Point>& _path, Point seg, std::shared_ptr<Segment> originalSegment)
{
    if(originalSegment->planning.Collision == -1)
    {
        segmentNrs_.back().push_back(originalSegment->getIndex());
        _path.push_back(seg);
    }
    else
    {
        int nrIterations = 0;
        int segId = pathQuerry_->findSegNr(originalSegment->planning.Collision, originalSegment->planning.Potential);   //Segment after the collisiton


        if(segId != -1)
        {
            for(int i = segmentNrs_[originalSegment->planning.Collision].size() - 1; i >= 0; i--)
            {
                if(segmentNrs_[originalSegment->planning.Collision][i] == segId)
                {
                    nrIterations = i;
                    break;
                }
            }
        }

        do
        {
            segmentNrs_.back().push_back(originalSegment->getIndex());
            _path.push_back(seg);
        }
        while(_path.size() <= nrIterations + 1);
    }
}

template<>
Point PathGenerator<Point>::createElementEnd(const std::shared_ptr<Segment>& _element, Point _end)
{
    Point pt(_end);
    return pt;
}
template<>
Point PathGenerator<Point>::createElementStartEnd(const std::shared_ptr<Segment>& _element, Point _start, Point _end)
{
    return createElementEnd(_element, _end);
}

template<>
void PathGenerator<Point>::preparePath(std::vector<Point>& _path, Point _start)
{ 
	_path.push_back(_start);
}

template<>
void PathGenerator<Potential_Point>::preparePath(std::vector<Potential_Point>& _path, Point _start)
{ 
    Potential_Point p;
	p.point[0] = _start[0];
	p.point[1] = _start[1];
	
	_path.push_back(p);
}

template<>
Potential_Point PathGenerator<Potential_Point>::createElement(const std::shared_ptr<Segment>& _element)
{
    Potential_Point pt;

    if(_element->planning.Direction == Segment::start_to_end)
    {
        pt.point[0] = _element->getStart()[0];
        pt.point[1] = _element->getStart()[1];
        pt.potential = _element->planning.Potential;
    }
    else
    {
        pt.point[0] = _element->getEnd()[0];
        pt.point[1] = _element->getEnd()[1];
        pt.potential = _element->planning.Potential;
    }

    return pt;
}

template<>
void PathGenerator<Potential_Point>::pushBackPath(const std::vector<std::vector<Potential_Point>>& _actPath, std::vector<Potential_Point>& _path, Potential_Point seg, std::shared_ptr<Segment> originalSegment)
{
    if(originalSegment->planning.Collision == -1)
    {
        segmentNrs_.back().push_back(originalSegment->getIndex());
        _path.push_back(seg);
    }
    else
    {
        int nrIterations = 0;
        int segId = pathQuerry_->findSegNr(originalSegment->planning.Collision, originalSegment->planning.Potential);   //Segment after the collisiton


        if(segId != -1)
        {
            for(int i = segmentNrs_[originalSegment->planning.Collision].size() - 1; i >= 0; i--)
            {
                if(segmentNrs_[originalSegment->planning.Collision][i] == segId)
                {
                    nrIterations = i;
                    break;
                }
            }
        }

        do
        {
            segmentNrs_.back().push_back(originalSegment->getIndex());
            _path.push_back(seg);
        }
        while(_path.size() <= nrIterations + 1);
    }
}



template<>
Potential_Point PathGenerator<Potential_Point>::createElementEnd(const std::shared_ptr<Segment>& _element, Point _end)
{
    Potential_Point p;
	p.point[0] = _end[0];
	p.point[1] = _end[1];
    return p;
}
template<>
Potential_Point PathGenerator<Potential_Point>::createElementStartEnd(const std::shared_ptr<Segment>& _element, Point _start, Point _end)
{
    return createElementEnd(_element, _end);
}





template<>
PathSegment PathGenerator<PathSegment>::createElement(const std::shared_ptr<Segment>& _element)
{
    PathSegment ps;
    if(_element->planning.Direction == Segment::start_to_end)
    {
        ps.segId = _element->getIndex();
        ps.end[0] = _element->getStart()[0];
        ps.end[1] = _element->getStart()[1];
        ps.start[0] = _element->getEnd()[0];
        ps.start[1] = _element->getEnd()[1];
    }
    else
    {
        ps.segId = _element->getIndex();
        ps.end[0] = _element->getEnd()[0];
        ps.end[1] = _element->getEnd()[1];
        ps.start[0] = _element->getStart()[0];
        ps.start[1] = _element->getStart()[1];
    }

    return ps;
}

template<>
PathSegment PathGenerator<PathSegment>::createElementEnd(const std::shared_ptr<Segment>& _element, Point _end)
{
    PathSegment ps = createElement(_element);
    ps.end[0] = _end[0];
    ps.end[1] = _end[1];
    

    return ps;
}

template<>
PathSegment PathGenerator<PathSegment>::createElementStart(const std::shared_ptr<Segment>& _element, Point _start)
{
    PathSegment ps = createElement(_element);
    ps.start[0] = _start[0];
    ps.start[1] = _start[1];

    return ps;
}

template<>
PathSegment PathGenerator<PathSegment>::createElementStartEnd(const std::shared_ptr<Segment>& _element, Point _start, Point _end)
{
    PathSegment ps = createElement(_element);
    ps.start[0] = _start[0];
    ps.start[1] = _start[1];
    ps.end[0] = _end[0];
    ps.end[1] = _end[1];

    return ps;
}

template<>
void PathGenerator<PathSegment>::addPreconditions(PathSegment& _segment, std::shared_ptr<Segment> _segToFind, int _pathNr, const std::vector<std::vector<std::shared_ptr<Segment>>>& _paths)
{
    std::vector<std::pair<int, float>> list = pathQuerry_->getListOfRobotsHigherPrioritizedRobots(_pathNr, _segToFind);
    _segment.preconditions.clear();

    for(const std::pair<int, float>& rob : list)
    {
        bool found = false;

        for(int i = 0; i < _paths[rob.first].size(); i++)
        {
            if(_paths[rob.first][i]->planning.Potential >= rob.second)
            {
                PathPrecondition pc;
                pc.robot = rob.first;
                pc.stepCondition = i;
                _segment.preconditions.push_back(pc);
                found = true;
                break;
            }
        }


    }
}







#endif
