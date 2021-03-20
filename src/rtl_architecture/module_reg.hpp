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
 * @file   module_reg.hpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Class to represent signal registers
 *
 */
#ifndef _MODULE_REG_
#define _MODULE_REG_

#include "rtl_identifier.hpp"

FORWARD_DECL(ModuleReg);

class ModuleReg : public RtlIdentifier
{
   protected:
      RtlNodePtr msb;

      RtlNodePtr lsb;

      RtlNodePtr init;

      bool has_sign;

      unsigned int num_bits;

      bool parametric;

   public:
      ModuleReg(const std::string& name);

      virtual ~ModuleReg();

      static ModuleRegPtr create(const std::string& name, const RtlNodePtr init);

      static ModuleRegPtr create(const std::string& name, const RtlNodePtr init, unsigned int num_bits, bool has_sign);

      static ModuleRegPtr create(const std::string& name, const RtlNodePtr init, const RtlNodePtr msb, const RtlNodePtr lsb, bool sign);

      std::tuple<RtlNodePtr, RtlNodePtr> get_reg_width() const { return std::tuple<RtlNodePtr, RtlNodePtr>(msb, lsb); }

      bool is_signed() const { return has_sign; }

      bool is_parametric() const { return parametric; }

      unsigned int get_reg_size() const { return num_bits; }

      void set_init(const RtlNodePtr _init) { init = _init; }

      virtual const std::string get_node_name() const { return "ModuleReg"; }

      virtual void show() const;
};

#endif
