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
 * @file   module_reg.cpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Implementation of the methods for signal registers.
 *
 */
#include "module_reg.hpp"

#include "integer_value.hpp"

ModuleReg::ModuleReg(const std::string& name) :
   RtlIdentifier(name)
{

}

ModuleReg::~ModuleReg()
{

}

ModuleRegPtr ModuleReg::create(const std::string& name, const RtlNodePtr init)
{
   return ModuleReg::create(name, init, 1, false);
}

ModuleRegPtr ModuleReg::create(const std::string& name, const RtlNodePtr init, unsigned int num_bits, bool has_sign)
{
   ModuleRegPtr reg = ModuleRegPtr(new ModuleReg(name));
   reg->parametric = false;
   reg->init = init;
   reg->num_bits = num_bits;
   reg->has_sign = has_sign;
   reg->msb = IntegerValue::create(num_bits-1);
   reg->lsb = IntegerValue::create(0);

   return reg;
}

ModuleRegPtr ModuleReg::create(const std::string& name, const RtlNodePtr init, const RtlNodePtr msb, const RtlNodePtr lsb, bool sign)
{
   ModuleRegPtr reg = ModuleRegPtr(new ModuleReg(name));
   reg->init = init;
   reg->msb = msb;
   reg->lsb = lsb;
   reg->has_sign = sign;
   reg->parametric = false;
   if (GetPointer<IntegerValue>(msb) && GetPointer<IntegerValue>(lsb))
      reg->num_bits = GetPointer<IntegerValue>(msb)->get_value() + 1 - GetPointer<IntegerValue>(lsb)->get_value();
   else
      reg->parametric = true;

   return reg;
}

void ModuleReg::show() const
{
   std::cout << std::string(RtlNode::indent, ' ') << "(" << get_node_name() << ": " << name << ")" << std::endl;
   RtlNode::indent += 2;
   msb->show();
   lsb->show();
   RtlNode::indent -= 2;
}
