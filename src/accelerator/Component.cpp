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
 * @file   Component.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Methods used to describe a component that uses a PLM.
 *
 */
#include "Component.hpp"

#include "Array.hpp"
#include "Architecture.hpp"

Component::Component(const std::string& _name) :
   name(_name)
{

}

void Component::print(std::ostream& os) const
{
   os << "Name = " << name << std::endl;
}

std::string Component::get_rdreq_prefix() const
{
   for(const auto& port : rdreq_prefix)
   {
      if (port.find("_request") != std::string::npos)
      {
         std::string sub_id = port.substr(0, port.find("_request"));
         return sub_id;
      }
   }
   return "";
}

std::string Component::get_wrreq_prefix() const
{
   for(const auto& port : wrreq_prefix)
   {
      if (port.find("_request") != std::string::npos)
      {
         std::string sub_id = port.substr(0, port.find("_request"));
         return sub_id;
      }
   }
   return "";
}

std::string Component::get_dmain_prefix() const
{
   for(const auto& port : dmain_prefix)
   {
      if (port.find("_ready") != std::string::npos)
      {
         std::string sub_id = port.substr(0, port.find("_ready"));
         return sub_id;
      }
   }
   return "";
}

std::string Component::get_dmaout_prefix() const
{
   for(const auto& port : dmaout_prefix)
   {
      if (port.find("_ready") != std::string::npos)
      {
         std::string sub_id = port.substr(0, port.find("_ready"));
         return sub_id;
      }
   }
   return "";
}


void Component::parse_interface(const YAML::Node& interface, const std::map<std::string, MemoryWrapperPtr> &buffer_to_wrapper)
{
   for(const auto& p : interface)
   {
      std::string id = p["id"].as<std::string>();
      if (id == "clk" || id == "CLK" || id == "clock")
      {
         clock_name = id;
         continue;
      }
      if (id == "rst" || id == "rst" || id == "reset")
      {
         reset_name = id;
         continue;
      }
      if (id == "conf_done" || id == "init_done")
      {
         conf_done_name = id;
         continue;
      }
      if (id != conf_done_name && (id == "acc_done" || id == "kernel_done" || id == "done" || id.find("_done") != std::string::npos))
      {
         acc_done_name = id;
         continue;
      }
      if (id.find("_valid") != std::string::npos || id.find("_data") != std::string::npos)
      {
         if (p["dir"].as<std::string>() == "input")
            dmain_prefix.insert(id);
         else
            dmaout_prefix.insert(id);
         continue;
      }
      if (id.find("_ready") != std::string::npos)
      {
         if (p["dir"].as<std::string>() == "output")
            dmain_prefix.insert(id);
         else
            dmaout_prefix.insert(id);
         continue;
      }
      if (id.find("_grant") != std::string::npos && p["dir"].as<std::string>() == "input")
      {
         std::string sub_id = id.substr(0, id.find("_grant"));
         if (sub_id == "rd")
            rdreq_prefix.insert(id);
         if (sub_id == "wr")
            wrreq_prefix.insert(id);
         continue;
      }
      if (id.find("_index") != std::string::npos && p["dir"].as<std::string>() == "output")
      {
         std::string sub_id = id.substr(0, id.find("_index"));
         if (sub_id == "rd")
            rdreq_prefix.insert(id);
         if (sub_id == "wr")
            wrreq_prefix.insert(id);
         continue;
      }
      if (id.find("_length") != std::string::npos && p["dir"].as<std::string>() == "output")
      {
         std::string sub_id = id.substr(0, id.find("_length"));
         if (sub_id == "rd")
            rdreq_prefix.insert(id);
         if (sub_id == "wr")
            wrreq_prefix.insert(id);
         continue;
      }
      if (id.find("_request") != std::string::npos && p["dir"].as<std::string>() == "output")
      {
         std::string sub_id = id.substr(0, id.find("_request"));
         if (sub_id == "rd")
            rdreq_prefix.insert(id);
         if (sub_id == "wr")
            wrreq_prefix.insert(id);
         continue;
      }
      if (id.find("ctrl_") != std::string::npos and id.find("___channel_") != std::string::npos)
      {
         std::string buffer = id.substr(0, id.find("__channel_"));
         for(auto& b : buffer_to_wrapper)
         {
            if (buffer.find(b.first) != std::string::npos)
            {
               buffer = b.first;
            }
         }
         darkmem_to_buffer[id] = buffer;
         buffer_to_darkmem[buffer].insert(id);
         continue;
      }

      for(auto& b : buffer_to_wrapper)
      {
         if (id.find(b.first + "_D") != std::string::npos)
         {
            std::string token = b.first + "_D";
            std::string num = id.substr(id.find(token)+token.size(), id.size());
            write_interfaces.insert(name + "_" + b.first + "_CE" + num);
            write_interfaces.insert(name + "_" + b.first + "_A" + num);
            write_interfaces.insert(name + "_" + b.first + "_D" + num);
            write_interfaces.insert(name + "_" + b.first + "_WE" + num);
            write_interfaces.insert(name + "_" + b.first + "_WEM" + num);
         }
         if (id.find(b.first + "_Q") != std::string::npos)
         {
            std::string token = b.first + "_Q";
            std::string num = id.substr(id.find(token)+token.size(), id.size());
            read_interfaces.insert(name + "_" + b.first + "_CE" + num);
            read_interfaces.insert(name + "_" + b.first + "_A" + num);
            read_interfaces.insert(name + "_" + b.first + "_Q" + num);
         }
      }
   }
}

ComponentList::ComponentList(unsigned int _verbosity) :
   verbosity(_verbosity)
{
   buffers = ArrayListPtr(new ArrayList(verbosity));
}

void ComponentList::parse_accelerator_config(const std::string& name, const std::string& input_cgraph)
{
   DEBUG(DBG_VERBOSE, verbosity, "** Parsing compatibility graph file \"" + input_cgraph + "\"..." << std::endl);

   ///compatibility graph
   UGraphPtr c_graph = UGraphPtr(new UGraph);
   if (!read_compatibility_graph(input_cgraph, *c_graph))
      throw std::runtime_error("Error during compatibility graph parsing");
   std::cout << "number of vertices = " << boost::num_vertices(*c_graph) << std::endl;

   BOOST_FOREACH(UNode v, boost::vertices(*c_graph))
   {
      std::string buff_name = name + "_" + boost::get(&ArrayNode::name, *c_graph, v);
      //DEBUG(DBG_VERBOSE, verbosity, "Array name \"" << name << "\"" << std::endl);
      if (id_to_buffer.count(buff_name) != 1)
         throw "mismatch between process and graph";
      node_to_buffer[c_graph][v] = id_to_buffer[buff_name];
      id_to_node[buff_name] = ComponentList::node_t(c_graph, v);
      node_list.push_back(ComponentList::node_t(c_graph, v));
      BOOST_FOREACH(ULink e, boost::out_edges(v, *c_graph))
      {
         //DEBUG(DBG_VERBOSE, verbosity, " -> " << boost::get(&ArrayNode::name, *c_graph, boost::target(e, *c_graph)) << std::endl);
         comp_list[ComponentList::node_t(c_graph, v)].insert(node_t(c_graph, boost::target(e, *c_graph)));
      }
   }
   DEBUG(DBG_VERBOSE, verbosity, "Number of nodes = " << boost::num_vertices(*c_graph) << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, "Number of edges = " << boost::num_edges(*c_graph) << std::endl);
   ///sanity check on the compatibility graph
   if (buffers->db[name].size() != boost::num_vertices(*c_graph))
      throw "mismatch";
   DEBUG(DBG_VERBOSE, verbosity, std::endl);
}

bool ComponentList::parse_config(const std::string& name, const std::string& multiacc_config)
{
   top_name = name;

   try
   {
      DEBUG(DBG_VERBOSE, verbosity, "** Parsing the accelerators' configuration file \"" + multiacc_config + "\"..." << std::endl);
      YAML::Node config = YAML::LoadFile(multiacc_config);
      YAML::Node accelerators = config["accelerator"];
      for(const auto& a_it : accelerators)
      {
         std::string acc_name = a_it["name"].as<std::string>();
         std::string cgraph = a_it["cgraph"].as<std::string>();
         std::string xml_config = a_it["acc_config"].as<std::string>();
         std::string scenario;
         if (a_it["scenario_config"])
            scenario = a_it["scenario_config"].as<std::string>();

         DEBUG(DBG_MINIMUM, verbosity, "** Accelerator \"" + acc_name + "\"" << std::endl);
         //parse_accelerator_config(acc_name, cgraph, xml_config);
         ComponentPtr component(new Component(acc_name));
         buffers->parse_config(acc_name, xml_config, scenario);
         bufferLoad(acc_name);
         parse_accelerator_config(acc_name, cgraph);
         list[acc_name] = component;
      }
      PRINT("Number of accelerators = " << list.size() << std::endl);
   }
   catch(...)
   {
      std::cout << "ERROR!" << std::endl;
      return false;
   }
   return true;
}

void ComponentList::bufferLoad(const std::string& name)
{
   for(unsigned int i = 0; i < buffers->db[name].size(); i++)
   {
       std::string buffer_id = name + "_" + buffers->db[name][i]->name;
       if (id_to_buffer.find(buffer_id) != id_to_buffer.end())
          throw "duplicated buffer \"" + buffer_id + "\"";
       id_to_buffer[buffer_id] = buffers->db[name][i];
   }
}

std::string ComponentList::get_clique_string(const std::set<node_t>& clique)
{
   std::string str;
   bool first = true;
   for(const auto& node : clique)
   {
      ArrayPtr buff = node_to_buffer[std::get<0>(node)][std::get<1>(node)];
      if (!first)
         str += ",";
      str += buff->accelerator + "_" + buff->name;
      first = false;
   }
   return str;
}

bool ComponentList::parse_config(const std::string& name, const std::string& acc_config, const std::string& input_cgraph, const std::string& scenario_config)
{
   top_name = name;
   if (!buffers->parse_config(name, acc_config, scenario_config))
      return false;
   bufferLoad(name);
   ComponentPtr component(new Component(name));
   parse_accelerator_config(name, input_cgraph);
   list[name] = component;
   return true;
}

void ComponentList::create_array(const std::string& name, const unsigned int width, unsigned int height, const std::string& interfaces, const std::string& init_file)
{
   top_name = name;
   ArrayPtr array(new Array);
   array->name = name;
   array->accelerator = name;
   array->width = width;
   array->height = height;
   array->parse_init_file(init_file);

   ProcessPtr proc(new Process);
   proc->name = name;
   boost::char_separator<char> sep(", ");
   boost::tokenizer<boost::char_separator<char>> tokens(interfaces, sep);
   unsigned int idx = 0;
   for(const auto& it : tokens)
   {
      InterfacePtr inter(new Interface);
      if (it == "r")
      {
         proc->r_accesses++;
         array->tot_r_interfaces++;
         inter->type = Interface::R;
      }
      else if (it == "w")
      {
         proc->w_accesses++;
         array->tot_w_interfaces++;
         inter->type = Interface::W;
      }
      else if (it == "rw")
      {
         proc->rw_accesses++;
         array->tot_rw_interfaces++;
         inter->type = Interface::RW;
      }
      else
         throw "wrong definition of interfaces for buffer " + name;
      inter->idx = idx++;
      array->interfaces.push_back(inter);
      proc->interfaces.push_back(inter);
   }
   array->processes.push_back(proc);

   buffers->db[name].push_back(array);

   id_to_buffer[name] = array;
   id_to_node[name] = node_t(0, 0);
   node_to_buffer[0][0] = array;
   graph_to_acc_name[0] = name;
   node_list.push_back(node_t(0, 0));
}
