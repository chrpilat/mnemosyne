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
 * @file   Array.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Methods used to describe a data structure to be stored in the PLM.
 *
 */
#include "Array.hpp"

#include "Scenario.hpp"

#include "Architecture.hpp"
#include "PBlock.hpp"

Array::Array() :
   height(0),
   width(0),
   p_blocks(1),
   s_blocks(1),
   h_block(1),
   tot_w_interfaces(0),
   tot_rw_interfaces(0),
   tot_r_interfaces(0),
   data_partitioning(false),
   merge(1),
   split(1)
{

}

void Array::print(std::ostream &os) const
{
   os << "Data structure: " << std::setw(20) << std::left << accelerator+"_"+name;
   os << std::setw(8) << std::right << height;
   os << "x";
   os << std::setw(4) << std::left << width;
   os << "(W=" << tot_w_interfaces << ";RW=" << tot_rw_interfaces << ";R="  << tot_r_interfaces << ")" << std::endl;
   //interfaces
   os << std::setw(13) << std::left << "  Interface";
   os << std::setw(7) << std::left << "Type";
   os << std::setw(15) << std::left << "Pattern" << std::endl;
   os << "  " << std::string(25, '-') << std::endl;
   for(const auto& intf : interfaces)
   {
      os << std::string(7, ' ') << std::setw(8) << std::left << intf->idx;
      os << std::setw(5) << std::left << Interface::type_to_string(intf->type);
      os << std::setw(15) << std::left << Interface::pattern_to_string(intf->pattern);
      os << std::endl;
   }
   os << "  " << std::string(25, '-') << std::endl;
   //processes
   os << std::setw(32) << std::left << "  Process";
   os << std::setw(5) << std::left << "W";
   os << std::setw(5) << std::left << "RW";
   os << std::setw(5) << std::left << "R" << std::endl;
   os << "  " << std::string(41, '-') << std::endl;
   for(const auto& proc : processes)
   {
      os << std::string(2, ' ') << std::setw(30) << std::left << proc->name;
      os << std::setw(5) << std::left << proc->w_accesses;
      os << std::setw(5) << std::left << proc->rw_accesses;
      os << std::setw(5) << std::left << proc->r_accesses;
      os << std::endl;
   }
   os << "  " << std::string(41, '-') << std::endl;
   //pblocks
   if(pblocks.size())
   {
      os << "  Data partitioning: " << (data_partitioning ? "TRUE" : "FALSE") << std::endl;
      os << "  Data organization: merge = " << merge << " -- split = " << split << std::endl;
      for (unsigned int i = 0; i < pblocks.size(); i++)
      {
         os << "  +p_block: " << std::setw(5) << std::left << pblocks[i]->parallel_id;
         os << std::string(5, '-') << ">";
         os << std::setw(7) << std::right << pblocks[i]->height;
         os << "x";
         os << std::setw(5) << std::left << pblocks[i]->width << std::endl;

         std::cout << pblocks[i]->interfaces.size() << std::endl;
         assert(pblocks[i]->interfaces.size() == 2);
         os << std::setw(50) << std::left << "     Interface [0]";
         os << std::setw(50) << std::left << "     Interface [1]";
         os << std::endl;
         os << std::string(5, ' ');
         os << std::setw(5) << std::left << "Idx";
         os << std::setw(30) << std::left << "Process Signal";
         os << std::setw(5) << "Type";
         os << std::string(10, ' ');
         os << std::setw(5) << std::left << "Idx";
         os << std::setw(30) << std::left << "Process Signal";
         os << std::setw(5) << "Type";
         os << std::endl;
         unsigned int max_interfaces = std::max(pblocks[i]->interfaces[0].size(), pblocks[i]->interfaces[1].size());
         for(unsigned int w = 0; w < max_interfaces; w++)
         {
            if (w >= pblocks[i]->interfaces[0].size())
               os << std::string(50, ' ');
            else
            {
               InterfacePtr intf = std::get<0>(pblocks[i]->interfaces[0][w]);
               unsigned int split_idx = std::get<1>(pblocks[i]->interfaces[0][w]);
               unsigned int data_token = ceil((double)width / (double)split);
               unsigned int lsb = (split_idx)*data_token;
               unsigned int msb = (split_idx+1)*data_token-1;
               msb = std::min(msb, width-1); //this is necessary when the data width is not a multiple of the memory IP width
               os << std::string(5, ' ');
               os << std::setw(5) << std::left << intf->idx;
               std::string process_signal = intf->process_name + "[" + boost::lexical_cast<std::string>(msb) + ":"
                                                                     + boost::lexical_cast<std::string>(lsb) + "]";
               os << std::setw(30) << std::left << process_signal;
               os << std::setw(5) << Interface::type_to_string(intf->type);
               os << std::string(10, ' ');
            }
            if (w >= pblocks[i]->interfaces[1].size())
               os << std::string(50, ' ');
            else
            {
               InterfacePtr intf = std::get<0>(pblocks[i]->interfaces[1][w]);
               unsigned int split_idx = std::get<1>(pblocks[i]->interfaces[1][w]);
               unsigned int data_token = ceil((double)width / (double)split);
               unsigned int lsb = (split_idx)*data_token;
               unsigned int msb = (split_idx+1)*data_token-1;
               msb = std::min(msb, width-1); //this is necessary when the data width is not a multiple of the memory IP width
               os << std::setw(5) << std::left << intf->idx;
               std::string process_signal = intf->process_name + "[" + boost::lexical_cast<std::string>(msb) + ":"
                                                                     + boost::lexical_cast<std::string>(lsb) + "]";
               os << std::setw(30) << std::left << process_signal;
               os << std::setw(5) << Interface::type_to_string(intf->type);
            }
            os << std::endl;
         }
      }
   }
}

unsigned int Array::get_mem_size() const
{
   if (data_partitioning) ///data partitioning
   {
      return ceil((double)height / (double)p_blocks);
   }
   return height; ///data duplication
}

unsigned int Array::get_minimum_bsize(unsigned int size)
{
   unsigned int i = 0;
   while(pow(2, i) < size)
   {
      i++;
   }
   return pow(2, i);
}

ArrayList::ArrayList(unsigned int _verbosity) :
    verbosity(_verbosity)
{

}

bool ArrayList::parse_config(const std::string& acc_name, const std::string& acc_config, const std::string &scenario_config)
{
   try
   {
      ///accelerator configuration
      DEBUG(DBG_VERBOSE, verbosity, "** Parsing the accelerator configuration file \"" + acc_config + "\"..." << std::endl);
      YAML::Node config = YAML::LoadFile(acc_config.c_str());
      DEBUG(DBG_VERBOSE, verbosity, std::string(80, '+') << std::endl);
      YAML::Node arrays = config["arrays"];
      for(const auto& b_it : arrays)
      {
         std::string buffer_id = b_it["name"].as<std::string>();
         ///buffer information
         ArrayPtr buff = ArrayPtr(new Array);
         buff->name = buffer_id;
         buff->accelerator = acc_name;
         buff->height = b_it["height"].as<unsigned int>();
         buff->width = b_it["width"].as<unsigned int>();
         ///check interfaces
         YAML::Node interfaces = b_it["interfaces"];
         unsigned int idx = 0;
         for(const auto& it_it : interfaces)
         {
            std::string it = it_it.as<std::string>();
            InterfacePtr inter(new Interface);
            if (it == "r")
            {
               buff->tot_r_interfaces++;
               inter->type = Interface::R;
            }
            else if (it == "w")
            {
               buff->tot_w_interfaces++;
               inter->type = Interface::W;
            }
            else if (it == "rw")
            {
               buff->tot_rw_interfaces++;
               inter->type = Interface::RW;
            }
            else
               throw std::runtime_error("Wrong interface (\"" + it + "\") for buffer " + buffer_id);
            inter->idx = idx++;
            buff->interfaces.push_back(inter);
         }
         ///sharing information
         YAML::Node sharing = b_it["sharing"];
         for(const auto& s_it : sharing)
         {
            YAML::Node interfaces = s_it["interfaces"];
            std::set<unsigned int> interface_set;
            for(const auto& it_it : interfaces)
            {
               unsigned int idx = it_it.as<unsigned int>();
               for(const auto& i : interface_set)
               {
                  buff->sharing[idx].insert(i);
                  buff->sharing[i].insert(idx);
               }
               interface_set.insert(idx);
            }
         }

         ///process information
         YAML::Node processes = b_it["processes"];
         if (processes)
         {
            for(const auto& p_it : processes)
            {
               ProcessPtr proc(new Process);
               proc->name = p_it["name"].as<std::string>();

               std::string read_pattern;
               std::string write_pattern;
               if (p_it["read_pattern"])
                  read_pattern = p_it["read_pattern"].as<std::string>();
               if (p_it["write_pattern"])
                  write_pattern = p_it["write_pattern"].as<std::string>();

               ///binding of interfaces
               YAML::Node interfaces = p_it["interfaces"];
               std::vector<unsigned int> interface_vec;
               try
               {
                  unsigned int value = interfaces.as<int>();
                  interface_vec.push_back(value);
               }
               catch (const YAML::BadConversion& e)
               {
                  for(const auto& it : interfaces)
                  {
                     unsigned int idx = it.as<unsigned int>();
                     interface_vec.push_back(idx);
                  }
               }
               for(const auto& idx : interface_vec)
               {
                  if (idx >= buff->interfaces.size())
                     throw std::runtime_error("wrong memory interface index for process " + proc->name + " on buffer " + buff->name);
                  if (buff->interfaces[idx]->process_name.size() > 0)
                     throw std::runtime_error("double binding for " + proc->name + "(" + boost::lexical_cast<std::string>(idx) + ") on buffer " + buff->name);
                  buff->interfaces[idx]->process_name = proc->name;
                  switch(buff->interfaces[idx]->type)
                  {
                     case Interface::R:
                        proc->r_accesses++;
                        if (read_pattern.size())
                           buff->interfaces[idx]->pattern = Interface::convert_pattern_string(read_pattern);
                        break;
                     case Interface::W:
                        proc->w_accesses++;
                        if (write_pattern.size())
                           buff->interfaces[idx]->pattern = Interface::convert_pattern_string(write_pattern);
                        break;
                     case Interface::RW:
                        proc->rw_accesses++;
                        ///check
                        if (write_pattern.size())
                           buff->interfaces[idx]->pattern = Interface::convert_pattern_string(write_pattern);
                        if (read_pattern.size())
                           buff->interfaces[idx]->pattern = Interface::convert_pattern_string(read_pattern);
                        break;
                     default:
                        throw std::runtime_error("unknown interface");
                  }
                  proc->interfaces.push_back(buff->interfaces[idx]);
               }
               buff->processes.push_back(proc);
            }
         }
         else
         {
            ///default process
            ProcessPtr proc(new Process);
            proc->name = acc_name;
            for(const auto& it : buff->interfaces)
            {
               it->process_name = acc_name;
               if (it->type == Interface::R)
                  proc->r_accesses++;
               else if (it->type == Interface::W)
                  proc->w_accesses++;
               else if (it->type == Interface::RW)
                  proc->rw_accesses++;
               proc->interfaces.push_back(it);
            }
            buff->processes.push_back(proc);
         }

         ///sanity check for interface binding
         for(const auto& intf : buff->interfaces)
         {
            if (intf->process_name.size() == 0)
               throw std::runtime_error("missing binding for interface " + boost::lexical_cast<std::string>(intf->idx) + " on buffer " + buff->name);
         }

         db[acc_name].push_back(buff);

         if (verbosity < DBG_VERBOSE)
            PRINT("Array name \"" << buff->name << "\" (" << buff->height << "x" << buff->width << ")" << std::endl);
         ///debug information
         DEBUG(DBG_VERBOSE, verbosity, *buff);
         DEBUG(DBG_VERBOSE, verbosity, std::string(80, '+') << std::endl);
      }
      DEBUG(DBG_VERBOSE, verbosity, std::endl);

      if (scenario_config.size())
      {
         DEBUG(DBG_VERBOSE, verbosity, "** Parsing the accelerator' scenario file \"" + scenario_config + "\"..." << std::endl);
         YAML::Node config = YAML::LoadFile(scenario_config);
         YAML::Node scenario_list = config["scenario"];
         for (const auto& s : scenario_list)
         {
            ScenarioPtr scenario(new Scenario);
            if (!scenario->parse_config(s))
               return false;
            DEBUG(DBG_VERBOSE, verbosity, *scenario);
            scenarios.push_back(scenario);
         }
      }
      else
      {
         ScenarioPtr scenario(new Scenario);
         scenario->name = acc_name + "_default";
         scenario->frequency = 1;
         for (unsigned int i = 0; i < db[acc_name].size(); i++)
         {
            Scenario::ScenarioArray buffer(db[acc_name][i]->height, db[acc_name][i]->width);
            scenario->buffers[db[acc_name][i]->name] = buffer;
         }
         DEBUG(DBG_VERBOSE, verbosity, *scenario);
         scenarios.push_back(scenario);
      }
      PRINT("Number of scenarios = " << scenarios.size() << std::endl);
      DEBUG(DBG_VERBOSE, verbosity, std::endl);
   }
   catch(std::exception& e)
   {
      std::cout << "Execution Error: " << e.what() << std::endl;
      return false;
   }
   catch(...)
   {
      std::cout << "ERROR!" << std::endl;
      return false;
   }
   return true;
}
