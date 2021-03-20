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
 * @file   Architecture.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Methods used to describe a PLM controller.
 *
 */
#include "Architecture.hpp"

#include "Array.hpp"
#include "Memory.hpp"

Port::Port()
{

}

Port::Port(std::string _id, dir_t _dir, unsigned int _size) :
   id(_id),
   dir(_dir),
   size(_size)
{

}

Interface::Interface() :
   pattern(UNKNOWN)
{

}

Interface::~Interface()
{

}

Interface::pattern_t Interface::convert_pattern_string(const std::string& str)
{
   if (str == "unknown")
      return UNKNOWN;
   if (str == "none")
      return NONE;
   if (str == "linear")
      return LINEAR;
   throw "Unknown pattern \"" + str + "\"";
   return UNKNOWN;
}

std::string Interface::pattern_to_string(const Interface::pattern_t& pattern)
{
   switch(pattern)
   {
      case NONE:
         return "NONE";
      case LINEAR:
         return "LINEAR";
      case UNKNOWN:
      default:
         return "UNKNOWN";
   }
   return "UNKNOWN";

}

std::string Interface::type_to_string(const Interface::type_t& type)
{
   switch(type)
   {
      case R:
         return "R";
      case W:
         return "W";
      case RW:
         return "RW";
      default:
         return "UNKNOWN";
   }
   return "UNKNOWN";
}

InterfacePtr Interface::clone()
{
   InterfacePtr intf(new Interface);
   intf->idx = idx;
   intf->type = type;
   intf->process_name = process_name;
   if (enable)
   {
      intf->enable = enable->clone();
      intf->ports.push_back(intf->enable);
   }
   if (address)
   {
      intf->address = address->clone();
      intf->ports.push_back(intf->address);
   }
   if (data_in)
   {
      intf->data_in = data_in->clone();
      intf->ports.push_back(intf->data_in);
      intf->write_enable = write_enable->clone();
      intf->ports.push_back(intf->write_enable);
      intf->write_mask = write_mask->clone();
      intf->ports.push_back(intf->write_mask);
   }
   if (data_out)
   {
      intf->data_out = data_out->clone();
      intf->ports.push_back(intf->data_out);
   }
   return intf;
}

Process::Process() :
   r_accesses(0),
   w_accesses(0),
   rw_accesses(0)
{

}

PortPtr Port::clone()
{
   PortPtr p(new Port);
   p->id = id;
   p->size = size;
   p->dir = dir;
   p->binding = binding;
   return p;
}

MemoryWrapper::MemoryWrapper()
{

}

MemoryWrapper::~MemoryWrapper()
{

}

void Port::parse_config(const xmlpp::Node* node)
{
   const xmlpp::Element* port_node = dynamic_cast<const xmlpp::Element*>(node);
   id = port_node->get_attribute_value("id");
   std::string dir_str = port_node->get_attribute_value("dir");
   if (dir_str == "in")
   {
      dir = Port::IN;
   }
   else if (dir_str == "out")
   {
      dir = Port::OUT;
   }
   else
      throw "Malformed port description";
   if (port_node->get_child_text())
      binding = RtlIdentifier::create(port_node->get_child_text()->get_content());
   if (port_node->get_attribute("size"))
      size = boost::lexical_cast<unsigned int>(port_node->get_attribute_value("size"));
   else
      size = 1;
}

