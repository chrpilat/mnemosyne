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
 * @file   module_sens.cpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Implementation of the methods for sensitivity elements.
 *
 */
#include "module_sens.hpp"

#include <iostream>

ModuleSens::ModuleSens()
{

}

ModuleSens::~ModuleSens()
{

}

ModuleSensPtr ModuleSens::create(const RtlNodePtr sig, const ModuleSens::type_t type)
{
   ModuleSensPtr sense = ModuleSensPtr(new ModuleSens);
   sense->sig = sig;
   sense->type = type;

   return sense;
}

void ModuleSens::show() const
{
   std::cout << std::string(RtlNode::indent, ' ') << "(" << get_node_name() << " ";
   switch(type)
   {
      case LEVEL:
         std::cout << "level";
         break;
      case POSEDGE:
         std::cout << "posedge";
         break;
      case NEGEDGE:
         std::cout << "negedge";
         break;
      case ALL:
         std::cout << "all";
         break;
   }
   std::cout << ")" << std::endl;
   RtlNode::indent += 2;
   if (sig) sig->show();
   RtlNode::indent -= 2;
}
