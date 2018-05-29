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

#include <tuw_voronoi_graph/dxf_to_graph.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char** argv)
{
    po::options_description description("usage");
    description.add_options()
        ("help,h", "Display help message")
        ("input,i", po::value<std::string>()->default_value("./segments.dxf"), "The path to the file")
        ("output,o", po::value<std::string>()->default_value("./graphs/segments"), "The output directory")
        ("width,w", po::value<float>()->default_value(0.6), "The width of a segments in meters")
        ("length,l", po::value<float>()->default_value(1.0), "The length of a segment in meters");
    
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(description).run(), vm);
    po::notify(vm);
        
    if(vm.count("help")){
        std::cout << description << std::endl;
    }
    else
    {
        std::string inpath = vm["input"].as<std::string>();
        std::string outpath = vm["output"].as<std::string>();
        if(outpath.back() != '/')
            outpath += "/";
        
        float length = vm["length"].as<float>();
        float width = vm["width"].as<float>();
        
     
        std::cout << "\tGenerating Graph from \"" << inpath << "\" with line_width: " << width << " and minimum line_length: " << length << std::endl;
        
        tuw_graph::DxfToGraph dxf2graph;
        dxf2graph.parseGraph(inpath, length, width);
        dxf2graph.serializeGraph(outpath);
        
        std::cout << "\tSaving graph to \"" << outpath << "\"" << std::endl;
    }
    
    
    
    return 0;
}

