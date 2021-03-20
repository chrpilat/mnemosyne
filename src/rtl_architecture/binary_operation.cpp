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
 * @file   binary_operation.cpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Implementation of the methods for binary operations.
 *
 */
#include "binary_operation.hpp"

BinaryOperation::BinaryOperation(const std::string& name, const Operation::op_t op_type) :
   Operation(name, op_type)
{

}

BinaryOperation::~BinaryOperation()
{

}

OperationPtr BinaryOperation::create(const Operation::op_t op_type, const RtlNodePtr lvalue, const RtlNodePtr rvalue)
{
   BinaryOperationPtr operation = BinaryOperationPtr(new BinaryOperation("", op_type));
   switch (op_type)
   {
      case POWER:
      case MULT:
      case DIVIDE:
      case MOD:
      case PLUS:
      case MINUS:
      case SLL:
      case SRL:
      case SRA:
      case LESSTHAN:
      case GREATERTHAN:
      case LESSEQ:
      case GREATEREQ:
      case EQ:
      case NOTEQ:
      case EQL:
      case NOTEQL:
      case AND:
      case XOR:
      case XNOR:
      case OR:
      case LAND:
      case LOR:
         break;
      default:
         throw std::runtime_error("Malformed binary operation: (" + STR(operation->op_type) + ")");
   }
   operation->op0 = lvalue;
   operation->op1 = rvalue;

   return operation;
}

OperationPtr BinaryOperation::create(const std::string& str, const RtlNodePtr lvalue, const RtlNodePtr rvalue)
{
   return BinaryOperation::create(Operation::get_op_type(str), lvalue, rvalue);
}

void BinaryOperation::show() const
{
   std::cout << std::string(RtlNode::indent, ' ') << "(" << Operation::get_op_string(get_op_type()) << ")" << std::endl;
   RtlNode::indent += 2;
   op0->show();
   op1->show();
   RtlNode::indent -= 2;
}

