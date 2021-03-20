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
 * @file   module_port.cpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Implementation of the methods for module ports.
 *
 */
#include "module_port.hpp"

#include "integer_value.hpp"

#include <iostream>

ModulePort::ModulePort(const std::string& name, port_type type, bool sign, data_type d_type) :
   RtlIdentifier(name),
   type(type),
   has_sign(sign),
   d_type(d_type),
   num_bits(1)
{

}

ModulePort::~ModulePort()
{

}

ModulePortPtr ModulePort::create(const std::string& name, port_type type, data_type d_type)
{
   return ModulePortPtr(new ModulePort(name, type, false, d_type));
}

ModulePortPtr ModulePort::create(const std::string& name, const RtlNodePtr msb, const RtlNodePtr lsb, port_type type, bool sign, data_type d_type)
{
   ModulePortPtr port = ModulePort::create(name, type, d_type);
   port->msb = msb;
   port->lsb = lsb;
   port->has_sign = sign;
   if (boost::static_pointer_cast<IntegerValue>(msb) && boost::static_pointer_cast<IntegerValue>(lsb))
      port->num_bits = boost::static_pointer_cast<IntegerValue>(msb)->get_value() + 1 - boost::static_pointer_cast<IntegerValue>(lsb)->get_value();

   return port;
}

ModulePortPtr ModulePort::create(const std::string& name, unsigned int num_bits, port_type type, bool sign, data_type d_type)
{
   return ModulePort::create(name, IntegerValue::create(num_bits-1), IntegerValue::create(0), type, sign, d_type);
}

ModulePort::port_type ModulePort::get_port_direction(const std::string& direction)
{
   if (direction == "input")
      return ModulePort::IN;
   else if (direction == "output")
      return ModulePort::OUT;
   else if (direction == "inout")
      return ModulePort::INOUT;
   else
      throw std::runtime_error("Unknown port direction: " + direction);
   return ModulePort::IN;
}

ModulePort::port_type ModulePort::get_port_direction() const
{
   return type;
}

bool ModulePort::is_signed() const
{
   return has_sign;
}

void ModulePort::show() const
{
   std::cout << std::string(RtlNode::indent, ' ') << "(";
   switch (type)
   {
      case IN:
         std::cout << "Input: ";
         break;
      case OUT:
         std::cout << "Output: ";
         break;
      case INOUT:
         std::cout << "Inout: ";
         break;
   }
   std::cout << name << " [";
   switch (type)
   {
      case DATA:
         std::cout << "data";
         break;
      case CONTROL:
         std::cout << "control";
         break;
      case PROTOCOL:
         std::cout << "protocol";
         break;
      case CLK:
         std::cout << "clock";
         break;
      case RST:
         std::cout << "reset";
         break;
      case IRQ:
         std::cout << "irq";
         break;
   }
   std::cout << "])" << std::endl;
   if (msb && lsb)
   {
      RtlNode::indent += 2;
      msb->show();
      lsb->show();
      RtlNode::indent -= 2;
   }
}

