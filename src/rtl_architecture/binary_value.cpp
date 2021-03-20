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
 * @file   binary_value.cpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Implementation of the methods for binary values.
 *
 */
#include "binary_value.hpp"

#include <boost/lexical_cast.hpp>

BinaryValue::BinaryValue()
{

}

BinaryValue::~BinaryValue()
{

}

BinaryValuePtr BinaryValue::create(type_t type, unsigned int size, const std::string& string_value)
{
   BinaryValuePtr value = BinaryValuePtr(new BinaryValue);
   value->type = type;
   value->num_bits = size;
   value->string_value = string_value;

   return value;
}

BinaryValuePtr BinaryValue::create(const std::string& string_value)
{
   BinaryValuePtr value = BinaryValuePtr(new BinaryValue);
   if (string_value.find("'h") != std::string::npos)
   {
      value->type = HEX;
      std::string size_str = string_value.substr(0, string_value.find("'h"));
      std::string bin_str = string_value.substr(string_value.find("'h")+2,std::string::npos);
      value->num_bits = boost::lexical_cast<unsigned int>(size_str);
      value->string_value = bin_str;
   }
   else if (string_value.find("'b") != std::string::npos)
   {
      value->type = BIN;
      std::string size_str = string_value.substr(0, string_value.find("'b"));
      std::string bin_str = string_value.substr(string_value.find("'b")+2,std::string::npos);
      value->num_bits = boost::lexical_cast<unsigned int>(size_str);
      value->string_value = bin_str;
   }
   else if (string_value.find("'d") != std::string::npos)
   {
      value->type = DEC;
      std::string size_str = string_value.substr(0, string_value.find("'d"));
      std::string bin_str = string_value.substr(string_value.find("'d")+2,std::string::npos);
      value->num_bits = boost::lexical_cast<unsigned int>(size_str);
      value->string_value = bin_str;
   }
   else
   {
      value->type = BIN;
      value->num_bits = string_value.size();
      value->string_value = string_value;
   }

   return value;
}

unsigned int BinaryValue::get_num_bits() const
{
   return num_bits;
}

void BinaryValue::show() const
{
   if (type == BIN)
      std::cout << std::string(RtlNode::indent, ' ') << "(Bin: ";
   else if (type == HEX)
      std::cout << std::string(RtlNode::indent, ' ') << "(Hex: ";
   else if (type == DEC)
      std::cout << std::string(RtlNode::indent, ' ') << "(Dec: ";
   std::cout << string_value <<  " [" << num_bits << " bits])" << std::endl;
}
