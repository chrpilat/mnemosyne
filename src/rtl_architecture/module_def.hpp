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
 * @file   module_def.hpp
 * @author Christian Pilato <christian.pilato@polimi.it>
 *
 * @brief  Class to represent hardware modules
 *
 */
#ifndef _MODULE_DEF_
#define _MODULE_DEF_

#include "rtl_identifier.hpp"
#include <map>
#include <vector>

FORWARD_DECL(ModuleDef);
FORWARD_DECL(ModulePort);
FORWARD_DECL(ModuleInstance);

class ModuleDef : public RtlIdentifier
{
   protected:
      std::map<std::string, RtlNodePtr> param_map;

      std::map<std::string, RtlNodePtr> port_map;

      std::vector<RtlNodePtr> ports;
      std::list<RtlNodePtr> portitems;

      std::vector<RtlNodePtr> declarations;

      std::vector<RtlNodePtr> items;

      /// module description
      std::string description;
      /// copyright description
      std::string copyright;
      /// list of authors
      std::string authors;
      /// license type associated with the module
      std::string license;

   public:
      ModuleDef(const std::string& _name);

      virtual ~ModuleDef();

      static ModuleDefPtr create(const std::string& name);

      const RtlNodePtr get_port_by_name(const std::string name) const;

      const std::vector<RtlNodePtr> get_ports() const { return ports; }

      const std::list<RtlNodePtr> get_portitems() const { return portitems; }

      const std::map<std::string, RtlNodePtr> get_param_map() const { return param_map; }

      const std::vector<RtlNodePtr> get_declarations() const { return declarations; }

      const std::vector<RtlNodePtr> get_items() const { return items; }

      unsigned int get_num_ports() const { return ports.size(); }

      ModulePortPtr get_positional_port(unsigned int i) const;

      void set_description(const std::string& d) { description = d; }

      const std::string get_description() const { return description; }

      void set_copyright(const std::string& c) { copyright = c; }

      const std::string get_copyright() const { return copyright; }

      void set_authors(const std::string& a) { authors = a; }

      const std::string get_authors() const { return authors; }

      void set_license(const std::string& l) { license = l; }

      const std::string get_license() const { return license; }

      ModuleInstancePtr add_internal_module(const ModuleDefPtr module, const std::string& instance_name);

      void add_port(const ModulePortPtr port);

      void add_portitem(const RtlNodePtr item) { portitems.push_back(item); }

      void add_item(const RtlNodePtr item) { items.push_back(item); }

      void add_parameter(const RtlNodePtr item);

      void add_declaration(const RtlNodePtr item);

      virtual const std::string get_node_name() const { return "ModuleDef"; }

      virtual void show() const;
};

#endif
