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
 * @file   module_wire.cpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Implementation of the methods for wires.
 *
 */
#include "module_wire.hpp"

#include "integer_value.hpp"

ModuleWire::ModuleWire(const std::string& name) :
   RtlIdentifier(name),
   has_sign(false),
   num_bits(0)

{

}

ModuleWire::~ModuleWire()
{

}

ModuleWirePtr ModuleWire::create(const std::string& name)
{
   return ModuleWire::create(name, 1, false);
}

ModuleWirePtr ModuleWire::create(const std::string& name, unsigned int num_bits, bool has_sign)
{
   ModuleWirePtr wire = ModuleWirePtr(new ModuleWire(name));
   wire->parametric = false;
   wire->num_bits = num_bits;
   wire->has_sign = has_sign;
   wire->msb = IntegerValue::create(num_bits-1);
   wire->lsb = IntegerValue::create(0);

   return wire;
}

ModuleWirePtr ModuleWire::create(const std::string& name, const RtlNodePtr msb, const RtlNodePtr lsb, bool sign)
{
   ModuleWirePtr wire = ModuleWirePtr(new ModuleWire(name));
   wire->parametric = false;
   wire->msb = msb;
   wire->lsb = lsb;
   wire->has_sign = sign;
   if (boost::static_pointer_cast<IntegerValue>(msb) && boost::static_pointer_cast<IntegerValue>(lsb))
      wire->num_bits = boost::static_pointer_cast<IntegerValue>(msb)->get_value() + 1 - boost::static_pointer_cast<IntegerValue>(lsb)->get_value();
   else
      wire->parametric = true;

   return wire;
}

void ModuleWire::show() const
{
   std::cout << std::string(RtlNode::indent, ' ') << "(" << get_node_name() << ": " << name << ")" << std::endl;
   RtlNode::indent += 2;
   msb->show();
   lsb->show();
   RtlNode::indent -= 2;
}
