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
 * @file   AreaHeuristic.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Implementation of the methods for the area-optimization functor.
 *
 */
#include "AreaHeuristic.hpp"

#include "Memory.hpp"

#include "Array.hpp"
#include "PBlock.hpp"
#include "Component.hpp"
#include "Partition.hpp"

AreaHeuristic::AreaHeuristic(unsigned int verbosity, const std::string& out_dir, const MemoryLibraryPtr mem_library, const ComponentListPtr components) :
   Functor(verbosity, out_dir, mem_library, components)
{

}

AreaHeuristic::~AreaHeuristic()
{

}

PartitionPtr AreaHeuristic::compute_clique_cost(const std::set<ComponentList::node_t> &clique)
{
   PartitionPtr partition(new Partition);

   std::list<std::tuple<unsigned int, ComponentList::node_t> > ordered_list;
   ///determine the ordered list of buffers based on the number of parallel blocks
   double totaldifference = 0;
   DEBUG(DBG_MINIMUM, verbosity, std::setw(32) << std::left << "  Array");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << "PBlocks");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << "PB H");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << "PB W");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << "Merge");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << "Split");
   DEBUG(DBG_MINIMUM, verbosity, std::endl);
   DEBUG(DBG_MINIMUM, verbosity, "  " << std::string(50, '-') << std::endl);
   for(const auto& node : clique)
   {
      ArrayPtr buff = components->node_to_buffer[std::get<0>(node)][std::get<1>(node)];
      DEBUG(DBG_MINIMUM, verbosity, "  " << std::setw(30) << std::left << buff->accelerator + "_" + buff->name);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << buff->p_blocks);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << buff->pblocks[0]->height);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << buff->pblocks[0]->width);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << buff->merge);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << buff->split);
      DEBUG(DBG_MINIMUM, verbosity, std::endl);
      ordered_list.push_back(std::tuple<unsigned int, ComponentList::node_t>(buff->p_blocks, node));
   }
   DEBUG(DBG_MINIMUM, verbosity, "  " << std::string(50, '-') << std::endl);

   ordered_list.sort();
   ///to have the list with the number of parallel blocks in decreasing order
   ordered_list.reverse();

   std::map<ArrayPtr, unsigned int> buff_s_blocks;
   std::map<ArrayPtr, unsigned int> buff_data_per_pblock;

   ///get initial configuration
   unsigned int B = std::get<0>(ordered_list.front());
   UGraphPtr cgraph = std::get<0>(std::get<1>(ordered_list.front()));
   UNode v = std::get<1>(std::get<1>(ordered_list.front()));
   const ArrayPtr buff = components->node_to_buffer[cgraph][v];
   unsigned int W = buff->pblocks[0]->width;
   unsigned int Size = buff->pblocks[0]->height;
   //DEBUG(DBG_VERBOSE, verbosity, "-initial configuration = " << B << "x(" << Size << "x" << W << ")" << std::endl);

   DEBUG(DBG_MINIMUM, verbosity, std::setw(32) << std::left << "  Array");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << "Buff H");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << "Buff W");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << "PB H");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << "PB W");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << "Num");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << "Mem H");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << "Mem W");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << "P Blocks");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << "S Blocks");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << "Offset");
   DEBUG(DBG_MINIMUM, verbosity, std::endl);
   DEBUG(DBG_MINIMUM, verbosity, "  " << std::string(50, '-') << std::endl);
   ///determine the configuration of the banks to accomodate all data structures
   for(const auto& p : ordered_list)
   {
      unsigned int num = std::get<0>(p);
      UGraphPtr cgraph = std::get<0>(std::get<1>(p));
      UNode v = std::get<1>(std::get<1>(p));
      ArrayPtr buff = components->node_to_buffer[cgraph][v];
      DEBUG(DBG_MINIMUM, verbosity, "  " << std::setw(30) << std::left << buff->accelerator + "_" + buff->name);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << buff->height);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << buff->width);
      ///computing offsets
      unsigned int offset = 0;
      for(const auto& node2 : clique)
      {
         UGraphPtr cgraph_tgt = std::get<0>(node2);
         UNode tgt = std::get<1>(node2);
         //DEBUG(DBG_VERBOSE, verbosity, "---compatible with buffer = " << buff2->accelerator + "_" + buff2->name << std::endl);
         if (tgt == v) continue;
         ArrayPtr buff_tgt = components->node_to_buffer[cgraph_tgt][tgt];
         if (cgraph == cgraph_tgt)
         {
            std::pair<ULink, bool> edge = boost::edge(v, tgt, *cgraph_tgt);
            if (edge.second)
            {
               std::string type = boost::get(&Arc::type, *cgraph, edge.first);
               //DEBUG(DBG_VERBOSE, verbosity, " - type = " << type << std::endl);
               if (type == "b" and partition->buffer_offset.find(buff_tgt) != partition->buffer_offset.end()) ///sharing of ports, but different memory space
               {
                  offset += Array::get_minimum_bsize(buff_tgt->get_mem_size()); ///to have the new buffer aligned to a power of 2.
               }
            }
         }
      }
      unsigned int N = ceil((double)B / (double)num);
      buff_data_per_pblock[buff] = buff->pblocks[0]->height;
      if (buff->data_partitioning) //data partitioning
      {
         if (offset + buff_data_per_pblock[buff] > (Size * N))
            Size = offset + ceil((double)components->node_to_buffer[cgraph][v]->height / (double)(B * buff->merge));
      }
      else //data duplication
      {
         if (buff_data_per_pblock[buff] + offset > (Size * N))
            Size = offset + ceil((double)buff_data_per_pblock[buff] / (double)(N));
      }
      buff_s_blocks[buff] = 1;

      DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << buff_data_per_pblock[buff]);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << buff->pblocks[0]->width);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << B);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << Size);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(7) << std::left << W);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << buff->p_blocks);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << buff_s_blocks[buff]);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << offset);
      DEBUG(DBG_MINIMUM, verbosity, std::endl);
      partition->buffer_offset[buff] = offset;
   }

   ///check how the current configuration can be accomodated by available memory IPs
   typedef std::tuple<double, unsigned int, unsigned int, MemoryPtr> agg_mem_t;
   std::list<agg_mem_t> aggr_mems;
   for(const auto &m : mem_library->db)
   {
      unsigned int num = (unsigned int)ceil((double)(Size) / double(m->height));
      if (buff->pblocks[0]->width > m->width)
      {
         continue;
      }
      double area = m->area * num;
      aggr_mems.push_back(agg_mem_t(area, num, m->width, m));
   }
   if (aggr_mems.size() == 0)
      throw "no available memory IPs";
   aggr_mems.sort();
   partition->memory = std::get<3>(aggr_mems.front());
   unsigned int factor = std::get<1>(aggr_mems.front());
   ///each block is split into smaller physical banks
   partition->num_banks = factor * B;
   std::cout << std::fixed;
   std::cout.precision(5);
   double area = partition->num_banks * partition->memory->area;
   DEBUG(DBG_MINIMUM, verbosity, "Final architecture: " << partition->num_banks << "x(" << partition->memory->height << "x" << partition->memory->width << ") = " << area << std::endl);

   DEBUG(DBG_MINIMUM, verbosity, std::setw(32) << std::left << "  Array");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << "Offset");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << "P Banks");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << "S Banks");
   DEBUG(DBG_MINIMUM, verbosity, std::setw(15) << std::left << "Block Data");
   DEBUG(DBG_MINIMUM, verbosity, std::endl);
   DEBUG(DBG_MINIMUM, verbosity, "  " << std::string(50, '-') << std::endl);
   for(const auto& bit : partition->buffer_offset)
   {
      const ArrayPtr buff = bit.first;
      unsigned int p_blocks = buff->p_blocks;
      unsigned int s_blocks = ((unsigned int)ceil((double)(buff_data_per_pblock[buff]) / double(partition->memory->height)));
      DEBUG(DBG_MINIMUM, verbosity, "  " << std::setw(30) << std::left << buff->accelerator + "_" + buff->name);
      if (partition->buffer_offset[buff] >= partition->memory->height)
      {
         partition->buffer_offset[buff] = partition->buffer_offset[buff] % partition->memory->height;
      }
      DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << partition->buffer_offset[buff]);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << p_blocks);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(10) << std::left << s_blocks);
      DEBUG(DBG_MINIMUM, verbosity, std::setw(15) << std::left << buff_data_per_pblock[buff]);
      DEBUG(DBG_MINIMUM, verbosity, std::endl);

      unsigned int num_banks = p_blocks * s_blocks;
      for(unsigned int i = 0; i < num_banks; i++)
      {
         unsigned int block = i % p_blocks;
         partition->buffer_configuration[buff][block].push_back(i);
      }
      if (verbosity >= DBG_VERBOSE)
      {
         for(unsigned int i = 0; i < p_blocks; i++)
         {
            std::cout << "  PBlock = " << i << " -- Banks = {";
            for(unsigned int b = 0; b < partition->buffer_configuration[buff][i].size(); b++)
            {
               if (b > 0) std::cout << ",";
               std::cout << partition->buffer_configuration[buff][i][b];
            }
            std::cout << "}" << std::endl;
         }
      }
   }
   partition->cost = partition->memory->area*(double)(partition->num_banks);
   return partition;
}
