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
 * @file   variable.cpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Implementation of the methods for hardware variables.
 *
 */
#include "variable.hpp"

Variable::Variable(const std::string& name) :
   RtlIdentifier(name)
{

}

Variable::~Variable()
{

}

VariablePtr Variable::create(var_t var_type, const std::string& name, const RtlNodePtr msb, const RtlNodePtr lsb)
{
   VariablePtr variable = VariablePtr(new Variable(name));
   variable->var_type = var_type;
   variable->msb = msb;
   variable->lsb = lsb;

   return variable;
}

void Variable::show() const
{
   std::cout << std::string(RtlNode::indent, ' ') << "(";
   switch (var_type)
   {
      case INTEGER:
         std::cout << "Int: ";
         break;
      case REAL:
         std::cout << "Real: ";
         break;
      case GENVAR:
         std::cout << "Genvar: ";
         break;
   }
   std::cout << name << ")" << std::endl;
   if (msb && lsb)
   {
      RtlNode::indent += 2;
      msb->show();
      lsb->show();
      RtlNode::indent -= 2;
   }
}

