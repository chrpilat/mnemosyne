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
 * @file   Memory.hpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Classes used to describe memory IPs.
 *
 */
#ifndef _MEMORY_HPP_
#define _MEMORY_HPP_

#include "utils.hpp"

FORWARD_DECL(Interface);
FORWARD_DECL(Wire);
FORWARD_DECL(Port);
FORWARD_DECL(Memory);
FORWARD_DECL(MemoryLibrary);

struct Memory
{
   ///name of the instance in case of allocated bank
   std::string id;
   ///name of the component
   std::string type;
   std::string v_file;
   std::vector<PortPtr> ports;
   std::map<std::string, PortPtr> id_to_port;
   std::vector<InterfacePtr> interfaces;

   ///physical information
   double area;
   double leakage_active;
   double leakage_gated;
   unsigned int width;
   unsigned int height;
   std::string liberty;

   std::map<unsigned int, std::map<InterfacePtr, WirePtr> > selector;

   Memory();

   void parse_config(const YAML::Node node);

   MemoryPtr clone();

   void add_port(const PortPtr);

   void print(std::ostream& os) const;

   friend std::ostream& operator<<(std::ostream& os, const Memory& mem)
   {
      mem.print(os);
      return os ;
   }
};

struct MemoryLibrary
{
   ///verbosity level of the class
   unsigned int verbosity;

   ///set of simulation files
   std::vector<std::string> sim_files;
   ///set of simulation libraries
   std::vector<std::string> sim_libs;

   ///archive of memory IPs
   std::vector<MemoryPtr> db;

   /**
    * Constructor
    */
   MemoryLibrary(unsigned int verbosity);

   /**
    * Print the current status of the library
    */
   void printLibrary() const;

   /**
    * Parse the description of the library
    */
   bool parse_config(const std::string& memlib);
};

#endif
