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
 * @file   Allocation.hpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Class to perform system-level memory allocation.
 *
 */
#ifndef _ALLOCATION_HPP_
#define _ALLOCATION_HPP_

#include "utils.hpp"

FORWARD_DECL(Array);
FORWARD_DECL(Interface);
FORWARD_DECL(Memory);
FORWARD_DECL(MemoryLibrary);
FORWARD_DECL(ArrayList);
FORWARD_DECL(ComponentList);
FORWARD_DECL(MemoryWrapper);
FORWARD_DECL(Process);
FORWARD_DECL(Partition);
FORWARD_DECL(Scenario);
FORWARD_DECL(Tag);

#include "Functor.hpp"
#include "Component.hpp"

class Allocation
{
   public:

      ///available optimizations
      const static unsigned int ADDRESS_SPACE = 1 << 0;
      const static unsigned int INTERFACE     = 1 << 1;
      const static unsigned int COLORING      = 1 << 2;

   protected:

      //compose output id
      bool compose_id;

      //verbosity level
      unsigned int verbosity;
      //output directory
      std::string out_dir;
      //name of the accelerator
      std::string accelerator_name;

      //available memory IPs
      MemoryLibraryPtr mem_library;

      //list of components accessing the PLM
      ComponentListPtr components;

      //vector of buffers to be allocated
      ArrayListPtr buffers;

      ///<bank_id, port>
      typedef std::tuple<unsigned int, unsigned int> mem_port_t;
      ///<buffer, process_interface, bank_id, port, parallel_block, serial_block>
      typedef std::tuple<ArrayPtr, InterfacePtr, unsigned int, unsigned int, unsigned int, unsigned int> binding_t;
      typedef std::vector<binding_t> merge_t;
      typedef std::vector<binding_t> split_t;
      typedef std::set<ComponentList::node_t> clique;

      ///???
      typedef std::tuple<unsigned int, MemoryPtr, unsigned int, unsigned int> split_bind_t;

      std::string get_partition_prefix(const std::set<ComponentList::node_t>& c);

      void determine_data_allocation(const ArrayPtr buff, unsigned int active_sharing);

      /**
       * @brief Print general statistics about the generated solution
       * @param opt_functor is the optimization functor that contains the final solution
       */
      void print_statistics(const FunctorPtr opt_functor);

      /**
       * @brief Print results in XML
       * @param opt_functor is the optimization functor that contains the final solution
       */
      void print_xml_statistics(const FunctorPtr opt_functor);

      /**
       * @brief Perform the partitioning of the compatibility graph into disjoint cliques
       * @param opt_functor is the optimization functor to contain the final solution
       * @param valid_partitions is the set of admissible clique
       * @param computed_partitions is the characterization of each clique
       */
      void perform_clique_partitioning(const FunctorPtr opt_functor, std::map<unsigned int, clique>& valid_partitions, std::map<clique, PartitionPtr>& computed_partitions);

      MemoryWrapperPtr create_architecture(const PartitionPtr partition);

      void create_memory_interfaces(const PartitionPtr partition, const MemoryWrapperPtr arch);
      /**
       * Creates the selectors for merge and multiple banks situations
       */
      void createReadSelectors(const PartitionPtr partition, const MemoryWrapperPtr arch, const ArrayPtr buff, const std::string& prefix);

      /**
       * Creates the RTL architecture associated with each buffer of a wrapper
       */
      void createArrayArchitecture(const PartitionPtr partition, const MemoryWrapperPtr arch, const ArrayPtr buff, const std::string& prefix);

      void set_default_binding(const MemoryPtr bank, unsigned int mem_intf_idx);
      std::string get_activation(const PartitionPtr partition, const MemoryPtr bank, unsigned int mem_intf, const binding_t& bind, const std::string &prefix);
      void set_ce_binding(const InterfacePtr mem_intf, unsigned int mem_intf_idx, std::string activation);
      void set_address_binding(const PartitionPtr partition, const ArrayPtr buff, const InterfacePtr proc_intf, const MemoryPtr bank, unsigned int mem_intf_idx, std::string activation, const std::string& prefix);

public:

      /**
       * Constructor
       */
      Allocation(unsigned int verbosity,
                 const std::string& out_dir, const std::string& accelerator_name,
                 const MemoryLibraryPtr mem_library, const ComponentListPtr components);

      /**
       * Destructor
       */
      ~Allocation();

      /**
       * @brief Set the new value for compose id
       * @param new_value is the new value
       */
      void set_compose_id(bool new_value);

      /**
       * Determines the memory sub-system
       */
      void perform_allocation(const FunctorPtr opt_functor, unsigned int active_sharing, std::map<std::string, MemoryWrapperPtr>& wrappers);

};

typedef boost::shared_ptr<Allocation> AllocationPtr;

#endif
