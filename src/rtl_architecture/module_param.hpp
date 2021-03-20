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
 * @file   module_param.hpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Class to represent module parameters
 *
 */
#ifndef _MODULE_PARAM_
#define _MODULE_PARAM_

#include "rtl_identifier.hpp"

FORWARD_DECL(ModuleParam);

class ModuleParam : public RtlIdentifier
{
   protected:
      RtlNodePtr msb;

      RtlNodePtr lsb;

      RtlNodePtr value;

      bool has_sign;

      bool local_param;

      unsigned int num_bits;

   public:
      ModuleParam(const std::string& name);

      virtual ~ModuleParam();

      static ModuleParamPtr create(const std::string& name, const RtlNodePtr value, bool sign, bool local_param);

      virtual const std::string get_node_name() const { return "ModuleParam"; }

      void set_width(const RtlNodePtr msb, RtlNodePtr lsb);

      std::tuple<RtlNodePtr, RtlNodePtr> get_width() const { return std::tuple<RtlNodePtr, RtlNodePtr>(msb, lsb); }

      bool is_signed() const { return has_sign; }

      unsigned int get_size() const { return num_bits; }

      const RtlNodePtr get_value() const { return value; }

      bool is_local_param() const { return local_param; }

      virtual void show() const;
};

#endif
