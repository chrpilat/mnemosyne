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
 * @file   module_instance.hpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Class to represent module instances
 *
 */
#ifndef _MODULE_INSTANCE_
#define _MODULE_INSTANCE_

#include "rtl_identifier.hpp"

FORWARD_DECL(ModuleInstance);

#include <map>
#include <vector>

class ModuleInstance : public RtlIdentifier
{
   public:
      typedef std::map<std::string, RtlNodePtr> binding_t;

   protected:
      std::string module_name;

      std::vector<std::string> portnamelist;

      binding_t paramlist;

      binding_t portlist;

      std::vector<RtlNodePtr> positional_portlist;

   public:
      ModuleInstance(const std::string& name);

      virtual ~ModuleInstance();

      static ModuleInstancePtr create(const std::string& module_name, const std::string& instance_name, const binding_t& paramlist, const binding_t& portlist);

      const std::string get_module_name() const { return module_name; }

      void set_port_binding(const std::string& name, const RtlNodePtr bind);

      const binding_t get_port_binding() const { return portlist; }

      const std::vector<std::string> get_portname_list() const { return portnamelist; }

      const binding_t get_paramlist() const { return paramlist; }

      virtual const std::string get_node_name() const { return "ModuleInstance"; }

      virtual void show() const;
};

#endif
