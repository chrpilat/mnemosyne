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
 * @file   Array.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Class to describe a data structure (array) to be stored in PLM.
 *
 */
#ifndef _ARRAY_HPP_
#define _ARRAY_HPP_

#include "utils.hpp"

FORWARD_DECL(Interface);
FORWARD_DECL(Memory);
FORWARD_DECL(PBlock);
FORWARD_DECL(Process);
FORWARD_DECL(Scenario);

/**
 * Data structure to represent a buffer to be stored in the PLM.
 */
struct Array
{
   //! Name of the buffer.
   std::string name;
   //! Identifier of the accelerator requiring the buffer.
   std::string accelerator;

   //! Number of elements to be stored.
   unsigned int height;
   //! Bitwidth of the buffer.
   unsigned int width;
   //! Initialization file (if any)
   std::string init_file;
   //! list of initialization values
   std::vector<std::string> init_values;

   //! Number of parallel blocks to ensure enough bandwidth for parallel accesses
   unsigned p_blocks;
   //! Vector of parallel blocks resulting from the implementation
   std::vector<PBlockPtr> pblocks;
   //! Number of serial blocks to ensure enough storage capability
   unsigned s_blocks;
   //! Height of the single block
   unsigned int h_block;

   //! Vector of all interfaces
   std::vector<InterfacePtr> interfaces;
   //! Vector of write interfaces
   std::vector<InterfacePtr> write_interfaces;
   //! Vector of read interfaces
   std::vector<InterfacePtr> read_interfaces;
   //! Total number of write interfaces
   unsigned tot_w_interfaces;
   //! Total number of read/write interfaces
   unsigned tot_rw_interfaces;
   //! Total number of read interfaces
   unsigned tot_r_interfaces;

   //! Vector of processes accessing the buffer
   std::vector<ProcessPtr> processes;

   std::map<unsigned int, std::set<unsigned int> > sharing;

   unsigned w_block_size;

   bool data_partitioning;
   unsigned int merge;
   unsigned int split;

   std::map<unsigned int, std::vector<unsigned int> > block_configuration;

   Array();

   void print(std::ostream& os) const;

   friend std::ostream& operator<<(std::ostream& os, const Array& b)
   {
      b.print(os);
      return os ;
   }

   unsigned int get_mem_size() const;

   static unsigned int get_minimum_bsize(unsigned int size);

   void parse_init_file(const std::string& init_file);
};
typedef boost::shared_ptr<Array> ArrayPtr;

struct ArrayList
{
    unsigned int verbosity;

    ///archive of buffers to be stored
    std::map<std::string, std::vector<ArrayPtr> > db;

    ///accelerators' scenarios
    std::vector<ScenarioPtr> scenarios;

    /**
     * @brief Constructor
     * @param verbosity is the verbosity level of the class
     */
    ArrayList(unsigned int verbosity);

    bool parse_config(const std::string& acc_name, const std::string& acc_config, const std::string& scenario_config);
};

#endif
