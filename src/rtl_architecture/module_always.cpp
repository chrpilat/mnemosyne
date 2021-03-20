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
 * @file   module_always.cpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Implementation of the methods for RTL processes.
 *
 */
#include "module_always.hpp"

#include "module_sens.hpp"

ModuleAlways::ModuleAlways()
{

}

ModuleAlways::~ModuleAlways()
{

}

ModuleAlwaysPtr ModuleAlways::create(const std::list<RtlNodePtr>& senslist, const RtlNodePtr statement)
{
   ModuleAlwaysPtr always = ModuleAlwaysPtr(new ModuleAlways);
   always->senslist = senslist;
   always->statement = statement;

   return always;
}

void ModuleAlways::show() const
{
   std::cout << std::string(RtlNode::indent, ' ') << "(" << get_node_name() << ")" << std::endl;
   RtlNode::indent += 2;
   std::cout << std::string(RtlNode::indent, ' ') << "(Sensitivity)" << std::endl;
   RtlNode::indent += 2;
   for (const auto& s : senslist)
   {
      s->show();
   }
   RtlNode::indent -= 2;
   std::cout << std::string(RtlNode::indent, ' ') << "(Body)" << std::endl;
   RtlNode::indent += 2;
   statement->show();
   RtlNode::indent -= 2;
   RtlNode::indent -= 2;
}
