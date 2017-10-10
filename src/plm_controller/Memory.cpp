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
 * @file   Memory.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Methods used to describe memory IPs.
 *
 */
#include "Memory.hpp"

#include "Architecture.hpp"

Memory::Memory() :
   area(0),
   leakage_active(0),
   leakage_gated(0),
   width(0),
   height(0)
{

}

void Memory::print(std::ostream& os) const
{
   os << std::setw(15) << std::left << type;
   os << std::setw(10) << std::right << height;
   os << std::setw(10) << std::right << width;
   os << std::fixed;
   os.precision(5);
   os << std::setw(15) << std::right << area;
   if (leakage_active)
      os << std::setw(15) << std::right << leakage_active;
   else
      os << std::setw(15) << std::right << "(n.a.)";
   if (leakage_gated)
      os << std::setw(15) << std::right << leakage_gated;
   else
      os << std::setw(15) << std::right << "(n.a.)";
}

void Memory::parse_config(const YAML::Node node)
{
   type = node["type"].as<std::string>();

   v_file = node["file"].as<std::string>();
   if (node["liberty"])
      liberty = node["liberty"].as<std::string>();

   height = node["height"].as<unsigned int>();
   width = node["width"].as<unsigned int>();

   area = node["area"].as<double>();
   if (node["leakage_active"])
      leakage_active = node["leakage_active"].as<double>();
   if (node["leakage_gated"])
      leakage_gated = node["leakage_gated"].as<double>();

   YAML::Node list_interfaces = node["interfaces"];
   unsigned int idx = 0;
   for (const auto& it : list_interfaces)
   {
      std::string intf_id = it.as<std::string>();
      InterfacePtr intf(new Interface);
      if (intf_id == "w")
         intf->type = Interface::W;
      else if (intf_id == "r")
         intf->type = Interface::R;
      else if (intf_id == "rw")
         intf->type = Interface::RW;
      else
         throw std::runtime_error("wrong interface configuration");
      std::string suffix = boost::lexical_cast<std::string>(idx++);
      unsigned int addr_size = ceil((double)log(height) / (double)log(2));
      intf->enable = PortPtr(new Port("CE"+suffix, Port::IN, 1));
      intf->address = PortPtr(new Port("A"+suffix, Port::IN, addr_size));
      intf->ports.push_back(intf->enable);
      intf->ports.push_back(intf->address);
      if (intf->type == Interface::W or intf->type == Interface::RW)
      {
         intf->data_in = PortPtr(new Port("D"+suffix, Port::IN, width));
         intf->write_enable = PortPtr(new Port("WE"+suffix, Port::IN, 1));
         intf->write_mask = PortPtr(new Port("WEM"+suffix, Port::IN, width));
         intf->ports.push_back(intf->data_in);
         intf->ports.push_back(intf->write_enable);
         intf->ports.push_back(intf->write_mask);
      }
      if (intf->type == Interface::W or intf->type == Interface::RW)
      {
         intf->data_out = PortPtr(new Port("Q"+suffix, Port::OUT, width));
         intf->ports.push_back(intf->data_out);
      }
      interfaces.push_back(intf);
   }
}

void Memory::add_port(const PortPtr p)
{
   ports.push_back(p);
   id_to_port[p->id] = p;
}

MemoryPtr Memory::clone()
{
   MemoryPtr mem = MemoryPtr(new Memory);

   mem->id = id;
   mem->type = type;
   mem->v_file = v_file;

   for(unsigned int p = 0; p < ports.size(); p++)
   {
      PortPtr newp = ports[p]->clone();
      mem->add_port(newp);
   }

   mem->area = area;
   mem->leakage_active = leakage_active;
   mem->leakage_gated = leakage_gated;
   mem->width = width;
   mem->height = height;
   mem->liberty = liberty;

   for(auto& intf : interfaces)
   {
      InterfacePtr intm = intf->clone();
      mem->interfaces.push_back(intm);
   }

   return mem;
}

MemoryLibrary::MemoryLibrary(unsigned int verbosity) :
   verbosity(verbosity)
{

}

bool MemoryLibrary::parse_config(const std::string& memlib)
{
   try
   {
      DEBUG(DBG_VERBOSE, verbosity, "\n** Parsing IP configuration file \"" + memlib + "\"" << std::endl);
      YAML::Node config = YAML::LoadFile(memlib);
      if (config["sim_files"])
      {
         YAML::Node sim_files_list = config["sim_files"];
         for(const auto& sf : sim_files_list)
         {
            std::string name = sf["name"].as<std::string>();
            sim_files.push_back(name);
         }
      }
      if (config["sim_libs"])
      {
         YAML::Node sim_libs_list = config["sim_libs"];
         for(const auto& sl : sim_libs_list)
         {
            std::string name = sl["name"].as<std::string>();
            sim_libs.push_back(name);
         }
      }

      YAML::Node memory_ip = config["memory_ip"];
      for(const auto& mip : memory_ip)
      {
         MemoryPtr mem(new Memory);
         mem->parse_config(mip);
         db.push_back(mem);
      }
   }
   catch(...)
   {
      DEBUG(DBG_VERBOSE, verbosity, "ERROR during IP configuration" << std::endl << std::endl);
      return false;
   }
   return true;
}

void MemoryLibrary::printLibrary() const
{
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::left << "Name");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(10) << std::right << "Height");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(10) << std::right << "Width");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::right << "Area");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::right << "Leak(Active)");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::right << "Leak(Gated)");
   DEBUG(DBG_VERBOSE, verbosity, std::endl);
   DEBUG(DBG_VERBOSE, verbosity, std::string(80, '-') << std::endl);
   typedef std::tuple<unsigned int, double, MemoryPtr> mem_t;
   std::list<mem_t> mems;
   for(const auto &m : db)
   {
      mems.push_back(mem_t(m->height, m->area, m));
   }
   mems.sort();
   for(const auto& m2 : mems)
   {
      DEBUG(DBG_VERBOSE, verbosity, *std::get<2>(m2) << std::endl);
   }
   DEBUG(DBG_VERBOSE, verbosity, std::string(80, '-') << std::endl);
   DEBUG(DBG_MINIMUM, verbosity, "Number of available memory IPs = " << db.size() << std::endl);
}
