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
 * @file   variable.hpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Class to represent hardware variables
 *
 */
#ifndef _VARIABLE_HPP_
#define _VARIABLE_HPP_

#include "rtl_identifier.hpp"

FORWARD_DECL(Variable);

class Variable : public RtlIdentifier
{
   public:
      typedef enum
      {
         INTEGER = 0,
         REAL,
         GENVAR,
      } var_t;

   protected:
      var_t var_type;

      RtlNodePtr msb;

      RtlNodePtr lsb;

   public:
      Variable(const std::string& name);

      virtual ~Variable();

      static VariablePtr create(var_t var_type, const std::string& name, const RtlNodePtr msb, const RtlNodePtr lsb);

      const var_t get_var_type() const { return var_type; }

      std::tuple<RtlNodePtr, RtlNodePtr> get_var_width() const { return std::tuple<RtlNodePtr, RtlNodePtr>(msb, lsb); }

      virtual const std::string get_node_name() const { return "Variable"; }

      virtual void show() const;
};

#endif
