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
 * @file   operation.cpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Implementation of the methods for RTL operations.
 *
 */
#include "operation.hpp"

Operation::Operation(const std::string& name, const Operation::op_t op_type) :
   RtlIdentifier(name),
   op_type(op_type)
{

}

Operation::~Operation()
{

}

Operation::op_t Operation::get_op_type(const std::string& op_string)
{
   if (op_string == "Uplus")
      return Operation::UPLUS;
   if (op_string == "Uminus")
      return Operation::UMINUS;
   if (op_string == "Ulnot")
      return Operation::ULNOT;
   if (op_string == "Unot")
      return Operation::UNOT;
   if (op_string == "Uand")
      return Operation::UAND;
   if (op_string == "Unand")
      return Operation::UNAND;
   if (op_string == "Uor")
      return Operation::UOR;
   if (op_string == "Unor")
      return Operation::UNOR;
   if (op_string == "Uxor")
      return Operation::UXOR;
   if (op_string == "Uxnor")
      return Operation::UXNOR;
   if (op_string == "Power")
      return Operation::POWER;
   if (op_string == "Mult")
      return Operation::MULT;
   if (op_string == "Divide")
      return Operation::DIVIDE;
   if (op_string == "Mod")
      return Operation::MOD;
   if (op_string == "Plus")
      return Operation::PLUS;
   if (op_string == "Minus")
      return Operation::MINUS;
   if (op_string == "Sll")
      return Operation::SLL;
   if (op_string == "Srl")
      return Operation::SRL;
   if (op_string == "Sla")
      return Operation::SLA;
   if (op_string == "Sra")
      return Operation::SRA;
   if (op_string == "LessThan")
      return Operation::LESSTHAN;
   if (op_string == "GreaterThan")
      return Operation::GREATERTHAN;
   if (op_string == "LessEq")
      return Operation::LESSEQ;
   if (op_string == "GreaterEq")
      return Operation::GREATEREQ;
   if (op_string == "Eq")
      return Operation::EQ;
   if (op_string == "NotEq")
      return Operation::NOTEQ;
   if (op_string == "Eql")
      return Operation::EQL;
   if (op_string == "NotEql")
      return Operation::NOTEQL;
   if (op_string == "And")
      return Operation::AND;
   if (op_string == "Xor")
      return Operation::XOR;
   if (op_string == "Xnor")
      return Operation::XNOR;
   if (op_string == "Or")
      return Operation::OR;
   if (op_string == "Land")
      return Operation::LAND;
   if (op_string == "Lor")
      return Operation::LOR;
   throw std::runtime_error("unknown operation string \"" + op_string + "\"");
   return Operation::UNKNOWN_OP;
}

std::string Operation::get_op_string(Operation::op_t op_type)
{
   switch ((unsigned int)op_type)
   {
      case UPLUS:
         return "Uplus";
      case UMINUS:
         return "Uminus";
      case ULNOT:
         return "Ulnot";
      case UNOT:
         return "Unot";
      case UAND:
         return "Uand";
      case UNAND:
         return "Unand";
      case UOR:
         return "Uor";
      case UNOR:
         return "Unor";
      case UXOR:
         return "Uxor";
      case UXNOR:
         return "Uxnor";
      case POWER:
         return "Power";
      case MULT:
         return "Mult";
      case DIVIDE:
         return "Divide";
      case MOD:
         return "Mod";
      case PLUS:
         return "Plus";
      case MINUS:
         return "Minus";
      case SLL:
         return "Sll";
      case SRL:
         return "Srl";
      case SLA:
         return "Sla";
      case SRA:
         return "Sra";
      case LESSTHAN:
         return "LessThan";
      case GREATERTHAN:
         return "GreaterThan";
      case LESSEQ:
         return "LessEq";
      case GREATEREQ:
         return "GreaterEq";
      case EQ:
         return "Eq";
      case NOTEQ:
         return "NotEq";
      case EQL:
         return "Eql";
      case NOTEQL:
         return "NotEql";
      case AND:
         return "And";
      case XOR:
         return "Xor";
      case XNOR:
         return "Xnor";
      case OR:
         return "Or";
      case LAND:
         return "Land";
      case LOR:
         return "Lor";
      default:
         throw std::runtime_error("unknown operation type: " + STR(op_type));
   }
   return "<unknown>";
}

unsigned int Operation::get_level(Operation::op_t op_type)
{
   switch (op_type)
   {
      case UPLUS:
      case UMINUS:
      case ULNOT:
      case UNOT:
      case UAND:
      case UNAND:
      case UOR:
      case UNOR:
      case UXOR:
      case UXNOR:
         return 1;
      case POWER:
      case MULT:
      case DIVIDE:
      case MOD:
         return 2;
      case PLUS:
      case MINUS:
         return 3;
      case SLL:
      case SRL:
      case SLA:
      case SRA:
         return 4;
      case LESSTHAN:
      case GREATERTHAN:
      case LESSEQ:
      case GREATEREQ:
         return 5;
      case EQ:
      case NOTEQ:
      case EQL:
      case NOTEQL:
         return 6;
      case AND:
      case XOR:
      case XNOR:
         return 7;
      case OR:
         return 8;
      case LAND:
         return 9;
      case LOR:
         return 10;
      default:
         return 11;
   }

   return 0;
}
