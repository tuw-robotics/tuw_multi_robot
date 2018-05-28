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

#ifndef SVG_PARSER_H
#define SVG_PARSER_H

#include <string>
#include <tuw_serialization/serializer.h>

#include <rapidxml_ns/rapidxml_ns.hpp>
#include <svgpp/policy/xml/rapidxml_ns.hpp>

#include <svgpp/svgpp.hpp>

namespace tuw_graph
{
    class Context
    {
        public:
            void path_move_to(double x, double y, svgpp::tag::coordinate::absolute);
            void path_line_to(double x, double y, svgpp::tag::coordinate::absolute);
            void path_cubic_bezier_to(
                double x1, double y1,
                double x2, double y2,
                double x, double y,
                svgpp::tag::coordinate::absolute);
            void path_quadratic_bezier_to(
                double x1, double y1,
                double x, double y,
                svgpp::tag::coordinate::absolute);
            void path_elliptical_arc_to(
                double rx, double ry, double x_axis_rotation,
                bool large_arc_flag, bool sweep_flag,
                double x, double y,
                svgpp::tag::coordinate::absolute);
            void path_close_subpath();
            void path_exit();

            void on_enter_element(svgpp::tag::element::any);
            void on_exit_element();
            void set(svgpp::tag::attribute::width, double val);
    };
    
    typedef
  boost::mpl::set<
    // SVG Structural Elements
    svgpp::tag::element::svg,
    svgpp::tag::element::g,
    // SVG Shape Elements
    svgpp::tag::element::circle,
    svgpp::tag::element::ellipse,
    svgpp::tag::element::line,
    svgpp::tag::element::path,
    svgpp::tag::element::polygon,
    svgpp::tag::element::polyline,
    svgpp::tag::element::rect
  >::type processed_elements_t;
  
  typedef
    boost::mpl::insert<
    svgpp::traits::shapes_attributes_by_element
    >::type processed_attributes_t;
    
  typedef
    boost::mpl::insert<
    svgpp::traits::shapes_attributes_by_element,
    svgpp::tag::attribute::width
    >::type passthrough_attributes_t;
  
  typedef rapidxml_ns::xml_node<> const * xml_element_t;
  
    class SvgParser
    {
        public:
            void loadSvg(xml_element_t _xml_root_element);
            void loadSvg(std::string _path);
    };
}
#endif // PLANNER_NODE_H

