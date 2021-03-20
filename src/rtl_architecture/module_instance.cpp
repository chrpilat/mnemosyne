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
 * @file   module_instance.cpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Implementation of the methods for module instances.
 *
 */
#include "module_instance.hpp"

ModuleInstance::ModuleInstance(const std::string& name) :
   RtlIdentifier(name)
{

}

ModuleInstance::~ModuleInstance()
{

}

ModuleInstancePtr ModuleInstance::create(const std::string& module_name, const std::string& instance_name, const ModuleInstance::binding_t& paramlist, const ModuleInstance::binding_t& portlist)
{
   ModuleInstancePtr instance = ModuleInstancePtr(new ModuleInstance(instance_name));
   instance->module_name = module_name;
   instance->paramlist = paramlist;
   instance->portlist = portlist;

   return instance;
}

void ModuleInstance::set_port_binding(const std::string& name, const RtlNodePtr bind)
{
   portlist[name] = bind;
   portnamelist.push_back(name);
}

void ModuleInstance::show() const
{
   std::cout << std::string(RtlNode::indent, ' ') << "(" << get_node_name() << ": " << name << " [" << module_name << "])" << std::endl;
   RtlNode::indent += 2;
   if (portnamelist.size())
   {
      for (const auto& p : portnamelist)
      {
         std::cout << std::string(RtlNode::indent, ' ') << "(Port: " << p << ")" << std::endl;
         RtlNode::indent += 2;
         if (portlist.find(p) != portlist.end()) portlist.find(p)->second->show();
         else std::cout << std::string(RtlNode::indent, ' ') << "([no binding])" << std::endl;
         RtlNode::indent -= 2;
      }
   }
   else if (positional_portlist.size())
   {
      for (const auto& p : positional_portlist)
      {
         if (p) p->show();
         else std::cout << std::string(RtlNode::indent, ' ') << "([no binding])" << std::endl;
      }
   }
   RtlNode::indent -= 2;
}
