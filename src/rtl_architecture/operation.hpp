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
 * @file   operation.hpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Class to represent RTL operations
 *
 */
#ifndef _OPERATION_HPP_
#define _OPERATION_HPP_

#include "rtl_identifier.hpp"

FORWARD_DECL(Operation);

class Operation : public RtlIdentifier
{
   public:
      typedef enum
      {
         UNKNOWN_OP  = 0,
         UPLUS       = 1,
         UMINUS      = 2,
         ULNOT       = 3,
         UNOT        = 4,
         UAND        = 5,
         UNAND       = 6,
         UOR         = 7,
         UNOR        = 8,
         UXOR        = 9,
         UXNOR       = 10,
         POWER       = 11,
         MULT        = 12,
         DIVIDE      = 13,
         MOD         = 14,
         PLUS        = 15,
         MINUS       = 16,
         SLL         = 17,
         SRL         = 18,
         SLA         = 19,
         SRA         = 20,
         LESSTHAN    = 21,
         GREATERTHAN = 22,
         LESSEQ      = 23,
         GREATEREQ   = 24,
         EQ          = 25,
         NOTEQ       = 26,
         EQL         = 27,
         NOTEQL      = 28,
         AND         = 29,
         XOR         = 30,
         XNOR        = 31,
         OR          = 32,
         LAND        = 33,
         LOR         = 34,
         COND        = 35,
      } op_t;

      op_t op_type;

   public:
      Operation(const std::string& name, const Operation::op_t op_type);

      virtual ~Operation();

      static Operation::op_t get_op_type(const std::string& op_string);

      static std::string get_op_string(Operation::op_t op_type);

      static unsigned int get_level(Operation::op_t op_type);

      const op_t get_op_type() const { return op_type; }

      virtual const std::string get_node_name() const { return "Operation"; }
};

#endif
