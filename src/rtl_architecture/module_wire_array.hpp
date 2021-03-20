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
 * @file   module_wire_array.hpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Class to represent 2D arrays
 *
 */
#ifndef _MODULE_WIRE_ARRAY_
#define _MODULE_WIRE_ARRAY_

#include "module_wire.hpp"

FORWARD_DECL(ModuleWireArray);

class ModuleWireArray : public ModuleWire
{
   protected:
      RtlNodePtr length_msb;

      RtlNodePtr length_lsb;

   public:
      ModuleWireArray(const std::string& name);

      virtual ~ModuleWireArray();

      static ModuleWireArrayPtr create(const std::string& name, unsigned int w_bits, unsigned int l_bits, bool has_sign);

      static ModuleWireArrayPtr create(const std::string& name, const RtlNodePtr w_msb, const RtlNodePtr w_lsb, const RtlNodePtr l_msb, const RtlNodePtr l_lsb, bool has_sign);

      std::tuple<RtlNodePtr, RtlNodePtr> get_wire_length() const { return std::tuple<RtlNodePtr, RtlNodePtr>(length_msb, length_lsb); }

      virtual const std::string get_node_name() const { return "ModuleWireArray"; }

      virtual void show() const;
};

#endif
