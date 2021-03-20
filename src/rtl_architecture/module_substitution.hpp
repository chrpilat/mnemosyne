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
 * @file   module_substitution.hpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Class to represent hardware substitution elements
 *
 */
#ifndef _MODULE_SUBSTITUTION_
#define _MODULE_SUBSTITUTION_

#include "rtl_node.hpp"

FORWARD_DECL(ModuleSubstitution);

class ModuleSubstitution : public RtlNode
{
   protected:
      bool blocking;

      RtlNodePtr lvalue;

      RtlNodePtr rvalue;

      RtlNodePtr ldelay;

      RtlNodePtr rdelay;

   public:
      ModuleSubstitution();

      virtual ~ModuleSubstitution();

      static ModuleSubstitutionPtr create(const RtlNodePtr lvalue, const RtlNodePtr rvalue, bool blocking);

      void set_ldelay(const RtlNodePtr lvalue) { ldelay = lvalue; }

      void set_rdelay(const RtlNodePtr rvalue) { rdelay = rvalue; }

      const bool is_blocking() const { return blocking; }

      const RtlNodePtr get_lvalue() const { return lvalue; }

      const RtlNodePtr get_rvalue() const { return rvalue; }

      virtual const std::string get_node_name() const { return "ModuleSubstitution"; }

      virtual void show() const;
};

#endif
