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

#include <tuw_voronoi_graph/svg_parser.h>
#include <sstream>
#include <rapidxml_ns/rapidxml_ns_utils.hpp>

#include <math.h>

namespace tuw_graph
{
    void Context::on_enter_element(svgpp::tag::element::any)
    {
        std::cout << "on_enter_element" << std::endl;
    }

    void Context::on_exit_element()
    {
        std::cout << "on_exit_element" << std::endl;
    }

    void Context::path_close_subpath()
    {
        std::cout << "path_close_subpath" << std::endl;
    }

    void Context::path_cubic_bezier_to(double x1, double y1, double x2, double y2, double x, double y, svgpp::tag::coordinate::absolute)
    {
        std::cout << "path_cubic_bezier_to" << std::endl;
    }

    void Context::path_elliptical_arc_to(double rx, double ry, double x_axis_rotation, bool large_arc_flag, bool sweep_flag, double x, double y, svgpp::tag::coordinate::absolute)
    {
        std::cout << "path_elliptical_arc_to" << std::endl;
    }

    void Context::path_exit()
    {
        std::cout << "path_exit" << std::endl;
    }

    void Context::path_line_to(double x, double y, svgpp::tag::coordinate::absolute)
    {
        std::cout << "path_line_to " << x << "/" << y << std::endl;
    }

    void Context::path_move_to(double x, double y, svgpp::tag::coordinate::absolute)
    {
        std::cout << "path_move_to" << std::endl;
    }

    void Context::path_quadratic_bezier_to(double x1, double y1, double x, double y, svgpp::tag::coordinate::absolute)
    {
        std::cout << "path_quadratic_bezier_to" << std::endl;
    }


    void Context::set(svgpp::tag::attribute::width, double val)
    {

    }



    void SvgParser::loadSvg(xml_element_t _xml_root_element)
    {
        Context context;
        svgpp::document_traversal<  svgpp::processed_elements<processed_elements_t>, 
                                    svgpp::processed_attributes<processed_attributes_t>
                                    >::load_document(_xml_root_element, context);
    }

    void SvgParser::loadSvg(std::string _path)
    {
        rapidxml_ns::xml_document<> doc;
        try
        {
            rapidxml_ns::file<> xmlFile(_path.c_str()); // Default template is char
            doc.parse<0>(xmlFile.data());

            if(rapidxml_ns::xml_node<> *svg_element = doc.first_node("svg"))
            {
                loadSvg(svg_element);
            }
        }
        catch(std::exception const &e)
        {
            std::cerr << "Error loading SVG: " << _path << " ex: " << e.what() << std::endl;
            return;
        }
    }


}
