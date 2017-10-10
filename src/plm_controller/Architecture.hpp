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
 * @file   Architecture.hpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Classes used to describe a PLM controller.
 *
 */
#ifndef _ARCH_HPP_
#define _ARCH_HPP_

#include "utils.hpp"

FORWARD_DECL(Array);
FORWARD_DECL(Interface);
FORWARD_DECL(Memory);
FORWARD_DECL(MemoryLibrary);
FORWARD_DECL(Port);

struct Port
{
   typedef enum
   {
      IN = 0,
      OUT = 1
   } dir_t;

   std::string id;
   unsigned int size;
   dir_t dir;
   std::string binding;

   void parse_config(const xmlpp::Node* node);
   PortPtr clone();
   Port();
   Port(std::string id, dir_t dir, unsigned int size);
};

struct Interface
{
   typedef enum
   {
      UNKNOWN = 0,
      NONE,
      LINEAR,
   } pattern_t;

   pattern_t pattern;

   static
   pattern_t convert_pattern_string(const std::string& str);

   static
   std::string pattern_to_string(const pattern_t& pattern);

   typedef enum
   {
      R = 0,
      W = 1,
      RW = 2
   } type_t;

   static
   std::string type_to_string(const type_t& type);

   std::string process_name;
   unsigned int idx;

   type_t type;
   std::vector<PortPtr> ports;
   std::map<std::string, PortPtr> id_to_port;

   PortPtr enable;
   PortPtr address;
   PortPtr write_enable;
   PortPtr data_in;
   PortPtr write_mask;
   PortPtr data_out;

   Interface();

   ~Interface();

   InterfacePtr clone();
};
typedef boost::shared_ptr<Interface> InterfacePtr;

struct Process
{
   std::string name;
   std::vector<InterfacePtr> interfaces;
   std::vector<InterfacePtr> read_interfaces;
   std::vector<InterfacePtr> write_interfaces;

   unsigned int r_accesses;
   unsigned int w_accesses;
   unsigned int rw_accesses;

   Process();
};
typedef boost::shared_ptr<Process> ProcessPtr;

struct Wire
{
   std::string id;
   unsigned int size;
   std::string binding;
   bool is_buffered;
};
typedef boost::shared_ptr<Wire> WirePtr;

struct MemoryWrapper
{
   std::string id;

   std::vector<MemoryPtr> banks;
   std::map<ArrayPtr, std::map<InterfacePtr, WirePtr> > merge_selector;
   ///map between <buffer, parallel_split_block, <vector of wires for each parallel block> >
   std::map<ArrayPtr, std::map<InterfacePtr, std::vector<WirePtr> > > split_selector;

   std::set<std::string> files;
   std::map<std::string, MemoryPtr> id_to_bank;

   std::map<std::string, PortPtr> id_to_port;
   std::vector<ArrayPtr> buffers;
   std::map<ArrayPtr, unsigned int> buffer_offset;

   std::vector<WirePtr> wires;

   unsigned int space;

   MemoryWrapper();

   ~MemoryWrapper();

   PortPtr parse_port(const xmlpp::Node* node);
};
typedef boost::shared_ptr<MemoryWrapper> MemoryWrapperPtr;

#endif
