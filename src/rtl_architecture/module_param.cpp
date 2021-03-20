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
 * @file   module_param.cpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Implementation of the methods for module parameters.
 *
 */
#include "module_param.hpp"

#include "binary_value.hpp"
#include "integer_value.hpp"

ModuleParam::ModuleParam(const std::string& name) :
   RtlIdentifier(name),
   local_param(false),
   num_bits(0)
{

}

ModuleParam::~ModuleParam()
{

}

ModuleParamPtr ModuleParam::create(const std::string& name, const RtlNodePtr value, bool sign, bool local_param)
{
   ModuleParamPtr param = ModuleParamPtr(new ModuleParam(name));
   param->value = value;
   param->has_sign = sign;
   param->local_param = local_param;
   if (GetPointer<BinaryValue>(value))
      param->num_bits = GetPointer<BinaryValue>(value)->get_num_bits();

   return param;
}

void ModuleParam::set_width(const RtlNodePtr _msb, RtlNodePtr _lsb)
{
   msb = _msb;
   lsb = _lsb;
   if (GetPointer<IntegerValue>(msb) && GetPointer<IntegerValue>(lsb))
      num_bits = GetPointer<IntegerValue>(msb)->get_value() + 1 - GetPointer<IntegerValue>(lsb)->get_value();
}

void ModuleParam::show() const
{
   std::cout << std::string(RtlNode::indent, ' ') << "(" << get_node_name() << ": " << name;
   if (local_param) std::cout << " [is_local_param]";
   std::cout << ")" << std::endl;
   RtlNode::indent += 2;
   value->show();
   if (msb && lsb)
   {
      std::cout << std::string(RtlNode::indent, ' ') << "(Size)" << std::endl;
      RtlNode::indent += 2;
      msb->show();
      lsb->show();
      RtlNode::indent -= 2;
   }
   RtlNode::indent -= 2;
}
