/**
 * @copyright
 * Copyright (c) 2017 - SLD Group @ Columbia University. All Rights Reserved.
 *
 * This file is part of Mnemosyne.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file   UGraph.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Methods for representing undirected graphs.
 *
 */
#include "UGraph.hpp"

bool read_compatibility_graph(const std::string& fileName, UGraph& g)
{
   try
   {
      std::map<std::string, UNode> name_to_node;

      YAML::Node config = YAML::LoadFile(fileName.c_str());

      YAML::Node nodes = config["nodes"];
      for (const auto& v : nodes)
      {
         std::string name = v.as<std::string>();
         UNode node = boost::add_vertex(g);
         name_to_node[name] = node;
         g[node].name = name;
      }

      YAML::Node edges = config["edges"];
      for (const auto& e : edges)
      {
         std::string type = e["type"].as<std::string>();
         YAML::Node compatibility = e["compatibility"];
         for (const auto& v1 : compatibility)
         {
            for (const auto& v2 : compatibility)
            {
               std::string source = v1.as<std::string>();
               std::string target = v2.as<std::string>();
               if (name_to_node.find(source) == name_to_node.end())
               {
                  std::cout << "Missing node \"" << source << "\" in edge list" << std::endl;
                  return false;
               }
               if (name_to_node.find(target) == name_to_node.end())
               {
                  std::cout << "Missing node \"" << target << "\" in edge list" << std::endl;
                  return false;
               }
               if (name_to_node[source] >= name_to_node[target] ) continue;
               ULink e; bool b;
               boost::tie(e, b) = boost::add_edge(name_to_node[source], name_to_node[target], g);
               g[e].type = type;
            }
         }
      }
   }
   catch(...)
   {
      std::cout << "ERROR!" << std::endl;
      return false;
   }
   return true;
}
