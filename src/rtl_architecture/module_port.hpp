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
 * @file   module_port.hpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Class to represent module ports
 *
 */
#ifndef _MODULE_PORT_
#define _MODULE_PORT_

#include "rtl_identifier.hpp"

FORWARD_DECL(ModulePort);
FORWARD_DECL(RtlNode);

class ModulePort : public RtlIdentifier
{
   public:
      typedef enum
      {
         IN,
         OUT,
         INOUT
      } port_type;

      typedef enum
      {
         DATA,
         CONTROL,
         PROTOCOL,
         CLK,
         RST,
         IRQ
      } data_type;

   protected:
      RtlNodePtr msb;

      RtlNodePtr lsb;

      port_type type;

      bool has_sign;

      data_type d_type;

      unsigned int num_bits;

   public:
      ModulePort(const std::string& name, port_type type, bool sign, data_type d_type);

      virtual ~ModulePort();

      static ModulePortPtr create(const std::string& name, const RtlNodePtr msb, const RtlNodePtr lsb, port_type type, bool sign, data_type d_type);

      static ModulePortPtr create(const std::string& name, port_type type, data_type d_type);

      static ModulePortPtr create(const std::string& name, unsigned int num_bits, port_type type, bool sign, data_type d_type);

      std::tuple<RtlNodePtr, RtlNodePtr> get_port_width() const { return std::tuple<RtlNodePtr, RtlNodePtr>(msb, lsb); }

      static port_type get_port_direction(const std::string& direction);

      bool is_signed() const;

      unsigned int get_port_size() const { return num_bits; }

      port_type get_port_direction() const;

      virtual const std::string get_node_name() const { return "ModulePort"; }

      virtual void show() const;
};

#endif
