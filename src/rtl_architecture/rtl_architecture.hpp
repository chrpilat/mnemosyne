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
 * @file   rtl_architecture.hpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Wrapper file for all RTL classes
 *
 */
#ifndef _RTL_ARCHITECTURE_
#define _RTL_ARCHITECTURE_

#include "binary_operation.hpp"
#include "binary_value.hpp"
#include "concat.hpp"

#include "module_block.hpp"
#include "module_if_statement.hpp"
#include "module_single_statement.hpp"
#include "rtl_identifier.hpp"
#include "module_call.hpp"
#include "module_initial.hpp"
#include "module_substitution.hpp"
#include "rtl_node.hpp"
#include "module_case.hpp"
#include "module_instance.hpp"
#include "module_wait.hpp"
#include "rtl_strings.hpp"
#include "rtl_comment.hpp"
#include "cond_operation.hpp"
#include "module_case_statement.hpp"
#include "module_param.hpp"
#include "module_while_statement.hpp"
#include "string_value.hpp"
#include "delay.hpp"
#include "module_decl.hpp"
#include "module_port.hpp"
#include "module_wire.hpp"
#include "module_wire_array.hpp"
#include "unary_operation.hpp"
#include "float_value.hpp"
#include "module_def.hpp"
#include "module_reg.hpp"
#include "operation.hpp"
#include "variable.hpp"
#include "integer_value.hpp"
#include "module_for_statement.hpp"
#include "module_reg_array.hpp"
#include "partselect.hpp"
#include "module_always.hpp"
#include "module_function.hpp"
#include "module_repeat.hpp"
#include "pointer.hpp"
#include "module_assign.hpp"
#include "module_generate.hpp"
#include "module_sens.hpp"

#endif
