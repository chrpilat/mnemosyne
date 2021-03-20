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
 * @file   module_reg_array.cpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Implementation of the methods for 2D register arrays.
 *
 */
#include "module_reg_array.hpp"

#include "integer_value.hpp"

ModuleRegArray::ModuleRegArray(const std::string& name) :
   ModuleReg(name)
{

}

ModuleRegArray::~ModuleRegArray()
{

}

ModuleRegArrayPtr ModuleRegArray::create(const std::string& name, unsigned int w_bits, unsigned int l_bits, bool has_sign)
{
   ModuleRegArrayPtr reg = ModuleRegArrayPtr(new ModuleRegArray(name));
   reg->parametric = false;
   reg->msb = IntegerValue::create(w_bits-1);
   reg->lsb = IntegerValue::create(0);
   reg->length_msb = IntegerValue::create(l_bits-1);
   reg->length_lsb = IntegerValue::create(0);
   reg->has_sign = has_sign;
   reg->num_bits = w_bits*l_bits;

   return reg;
}

ModuleRegArrayPtr ModuleRegArray::create(const std::string& name, const RtlNodePtr w_msb, const RtlNodePtr w_lsb, const RtlNodePtr l_msb, const RtlNodePtr l_lsb, bool has_sign)
{
   ModuleRegArrayPtr reg = ModuleRegArrayPtr(new ModuleRegArray(name));
   reg->msb = w_msb;
   reg->lsb = w_lsb;
   reg->length_msb = l_msb;
   reg->length_lsb = l_lsb;
   reg->has_sign = has_sign;
   reg->parametric = false;
   if (GetPointer<IntegerValue>(w_msb) && GetPointer<IntegerValue>(w_lsb) && GetPointer<IntegerValue>(l_msb) && GetPointer<IntegerValue>(l_lsb))
      reg->num_bits = (GetPointer<IntegerValue>(w_msb)->get_value() + 1 - GetPointer<IntegerValue>(w_lsb)->get_value())*(GetPointer<IntegerValue>(l_msb)->get_value() + 1 - GetPointer<IntegerValue>(l_lsb)->get_value());
   else
      reg->parametric = true;

   return reg;
}

void ModuleRegArray::show() const
{
   std::cout << std::string(RtlNode::indent, ' ') << "(" << get_node_name() << ": " << name << ")" << std::endl;
   RtlNode::indent += 2;
   msb->show();
   lsb->show();
   length_msb->show();
   length_lsb->show();
   RtlNode::indent -= 2;
}
