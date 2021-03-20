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
 * @file   module_function.hpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Class to represent hardware functions
 *
 */
#ifndef _MODULE_FUNCTION_
#define _MODULE_FUNCTION_

#include "rtl_node.hpp"

FORWARD_DECL(ModuleFunction);

#include <list>

class ModuleFunction : public RtlNode
{
   protected:
      RtlNodePtr name;

      RtlNodePtr ret_msb;

      RtlNodePtr ret_lsb;

      std::list<RtlNodePtr> declarations;

      RtlNodePtr statement;

   public:
      ModuleFunction();

      virtual ~ModuleFunction();

      static ModuleFunctionPtr create(const RtlNodePtr name, const RtlNodePtr ret_msb, const RtlNodePtr ret_lsb, const std::list<RtlNodePtr>& declarations, const RtlNodePtr statement);

      const RtlNodePtr get_name() const { return name; }

      std::tuple<RtlNodePtr, RtlNodePtr> get_ret_width() const { return std::tuple<RtlNodePtr, RtlNodePtr>(ret_msb, ret_lsb); }

      const std::list<RtlNodePtr> get_declarations() const { return declarations; }

      const RtlNodePtr get_statement() const { return statement; }

      virtual const std::string get_node_name() const { return "ModuleFunction"; }

      virtual void show() const;
};

#endif
