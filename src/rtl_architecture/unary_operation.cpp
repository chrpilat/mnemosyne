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
 * @file   unary_operation.cpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Implementation of the methods for unary RTL operations.
 *
 */
#include "unary_operation.hpp"

UnaryOperation::UnaryOperation(const std::string& name, const Operation::op_t op_type) :
   Operation(name, op_type)
{

}

UnaryOperation::~UnaryOperation()
{

}

OperationPtr UnaryOperation::create(const Operation::op_t op_type, const std::string& name, const RtlNodePtr rvalue)
{
   UnaryOperationPtr operation = UnaryOperationPtr(new UnaryOperation(name, op_type));
   operation->op = rvalue;

   return operation;
}

OperationPtr UnaryOperation::create(const std::string& str, const std::string& name, const RtlNodePtr rvalue)
{
   return UnaryOperation::create(Operation::get_op_type(str), name, rvalue);
}

OperationPtr UnaryOperation::create(const Operation::op_t op_type, const RtlNodePtr rvalue)
{
   return UnaryOperation::create(op_type, "", rvalue);
}

void UnaryOperation::show() const
{
   std::cout << std::string(RtlNode::indent, ' ') << "(" << Operation::get_op_string(get_op_type()) << ")" << std::endl;
   RtlNode::indent += 2;
   op->show();
   RtlNode::indent -= 2;
}
