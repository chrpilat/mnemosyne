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
 * @file   module_def.cpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Implementation of the methods for hardware modules.
 *
 */
#include "module_def.hpp"

#include "module_param.hpp"
#include "module_port.hpp"
#include "module_decl.hpp"
#include "module_instance.hpp"
#include "rtl_comment.hpp"

#include <iostream>

ModuleDef::ModuleDef(const std::string& _name) :
   RtlIdentifier(_name)
{

}

ModuleDef::~ModuleDef()
{

}

ModuleDefPtr ModuleDef::create(const std::string& name)
{
   ModuleDefPtr module = ModuleDefPtr(new ModuleDef(name));

   return module;
}

void ModuleDef::add_port(const ModulePortPtr port)
{
   if (!boost::dynamic_pointer_cast<ModulePort>(port))
      throw std::runtime_error("Wrong port");
   if (port_map.find(port->get_name()) != port_map.end())
   {
      port_map[port->get_name()] = port;
      for (unsigned int i = 0; i < ports.size(); i++)
      {
         if (boost::dynamic_pointer_cast<ModulePort>(ports[i])->get_name() == port->get_name()) ports[i] = port;
      }
   }
   else
   {
      port_map[port->get_name()] = port;
      ports.push_back(port);
      portitems.push_back(port);
   }
}

const RtlNodePtr ModuleDef::get_port_by_name(const std::string name) const
{
   if (port_map.find(name) != port_map.end())
      return port_map.find(name)->second;
   return RtlNodePtr();
}

ModulePortPtr ModuleDef::get_positional_port(unsigned int i) const
{
   if (i >= ports.size()) return ModulePortPtr();
   return boost::dynamic_pointer_cast<ModulePort>(ports.at(i));
}

ModuleInstancePtr ModuleDef::add_internal_module(const ModuleDefPtr to_add, const std::string& name)
{
   ModuleInstance::binding_t paramlist;
   ModuleInstance::binding_t portlist;
   ModuleInstancePtr instance = ModuleInstance::create(to_add->get_name(), name, paramlist, portlist);

   items.push_back(instance);

   return instance;
}

void ModuleDef::add_parameter(const RtlNodePtr item)
{
   if (!GetPointer<ModuleParam>(item))
      throw std::runtime_error("Malformed parameter");
   param_map[GetPointer<ModuleParam>(item)->get_name()] = item;
}

void ModuleDef::add_declaration(const RtlNodePtr item)
{
   if (!GetPointer<ModuleDecl>(item) && !GetPointer<RtlComment>(item))
      throw std::runtime_error("Malformed declaration");
   declarations.push_back(item);
}

void ModuleDef::show() const
{
   std::cout << std::string(RtlNode::indent, ' ') << "(" << get_node_name() << ": " << name << ")" << std::endl;
   RtlNode::indent += 2;
   std::cout << std::string(RtlNode::indent, ' ') << "(Interface)" << std::endl;
   RtlNode::indent += 2;
   for (const auto& p : ports)
   {
      std::cout << std::string(RtlNode::indent, ' ') << "(Port: " << GetPointer<RtlIdentifier>(p)->get_name() << ")" << std::endl;
   }
   RtlNode::indent -= 2;
   std::cout << std::string(RtlNode::indent, ' ') << "(Ports)" << std::endl;
   RtlNode::indent += 2;
   for (const auto& p : portitems)
   {
      p->show();
   }
   RtlNode::indent -= 2;
   std::cout << std::string(RtlNode::indent, ' ') << "(Declarations)" << std::endl;
   RtlNode::indent += 2;
   for (const auto& d : declarations)
   {
      d->show();
   }
   RtlNode::indent -= 2;
   std::cout << std::string(RtlNode::indent, ' ') << "(Items)" << std::endl;
   RtlNode::indent += 2;
   for (const auto& it : items)
   {
      it->show();
   }
   RtlNode::indent -= 2;
   RtlNode::indent -= 2;
}
