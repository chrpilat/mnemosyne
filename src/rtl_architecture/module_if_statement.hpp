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
 * @file   module_if_statement.hpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Class to represent hardware if statements
 *
 */
#ifndef _MODULE_IF_STATEMENT_
#define _MODULE_IF_STATEMENT_

#include "rtl_node.hpp"

FORWARD_DECL(ModuleIfStatement);

class ModuleIfStatement : public RtlNode
{
   protected:
      RtlNodePtr cond;

      RtlNodePtr true_statement;

      RtlNodePtr false_statement;

   public:
      ModuleIfStatement();

      virtual ~ModuleIfStatement();

      static ModuleIfStatementPtr create(const RtlNodePtr cond, const RtlNodePtr true_statement, const RtlNodePtr false_statement);

      const RtlNodePtr get_cond() const { return cond; }

      const RtlNodePtr get_true_statement() const { return true_statement; }

      const RtlNodePtr get_false_statement() const { return false_statement; }

      virtual const std::string get_node_name() const { return "ModuleIfStatement"; }

      virtual void show() const;
};

#endif
