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
 * @file   Allocation.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Implementation of the methods to perform memory allocation.
 *
 */
#include "Allocation.hpp"

#include "Architecture.hpp"
#include "PBlock.hpp"
#include "Tag.hpp"
#include "Partition.hpp"
#include "Memory.hpp"

#include "Array.hpp"
#include "Component.hpp"
#include "UGraph.hpp"
#include "Scenario.hpp"

#define GLPK_FILE "solver.mod"

static
unsigned int get_mcm(unsigned int a, unsigned int b)
{
   unsigned mult_a = a;
   unsigned mult_b = b;
   while (mult_a != mult_b)
   {
      while (mult_a < mult_b)  mult_a += a;
      while (mult_b < mult_a)  mult_b += b;
   }
   return mult_a;
}

Allocation::Allocation(unsigned int _verbosity, const std::string& _out_dir,
   const std::string& _accelerator_name,
   const MemoryLibraryPtr _mem_library, const ComponentListPtr _components) :
   compose_id(true),
   verbosity(_verbosity),
   out_dir(_out_dir),
   accelerator_name(_accelerator_name),
   mem_library(_mem_library),
   components(_components),
   buffers(components->buffers)
{

}

Allocation::~Allocation()
{

}

void Allocation::set_compose_id(bool new_value)
{
   compose_id = new_value;
}

void Allocation::determine_data_allocation(const ArrayPtr buff, unsigned int active_sharing)
{
   DEBUG(DBG_MINIMUM, verbosity, "Data structure: " << buff->accelerator + "_" + buff->name << std::endl);
   Interface::pattern_t write_type = Interface::NONE, read_type = Interface::NONE;
   for (const auto& proc : buff->processes)
   {
      for (const auto& proc_if : proc->interfaces)
      {
         if (proc_if->type == Interface::R or proc_if->type == Interface::RW)
         {
            buff->read_interfaces.push_back(proc_if);
            if (read_type == Interface::NONE)
               read_type = (proc->r_accesses == 1 ? Interface::NONE : proc_if->pattern);
            if (read_type != proc_if->pattern)
               read_type = Interface::UNKNOWN;
         }
         if (proc_if->type == Interface::W or proc_if->type == Interface::RW)
         {
            buff->write_interfaces.push_back(proc_if);
            if (write_type == Interface::NONE)
               write_type = (proc->w_accesses == 1 or proc->rw_accesses == 1 ? Interface::NONE : proc_if->pattern);
            if (write_type != proc_if->pattern)
               write_type = Interface::UNKNOWN;
         }
      }
   }

   if (buff->interfaces.size() == 0)
       throw "No interfaces";
   std::map<unsigned int, std::vector<InterfacePtr> > write_interface_coloring;
   std::map<unsigned int, std::vector<InterfacePtr> > read_interface_coloring;
   std::map<InterfacePtr, unsigned int> inverse_color;
   bool rw_right = (buff->interfaces[0]->type == Interface::W);
   //coloring of interfaces
   for (const auto& proc : buff->processes)
   {
      for (const auto& proc_if : proc->interfaces)
      {
         if (proc_if->type == Interface::W or (!rw_right and proc_if->type == Interface::RW))
         {
            proc->write_interfaces.push_back(proc_if);
            unsigned color = 0;
            bool found = false;
            do
            {
               found = false;
               for (const auto& intf : write_interface_coloring[color])
               {
                  if (intf->process_name == proc_if->process_name)
                     found = true;
                  else
                  {
                     bool share = false;
                     for (auto& sh : buff->sharing[proc_if->idx])
                     {
                        if (sh == intf->idx)
                        {
                           share = true;
                        }
                     }
                     if (!share)
                        found = true;
                  }

               }
               if (color == write_interface_coloring.size())
               {
                  break;
               }
               if (found) color++;
            }
            while (found);
            //std::cout << proc_if->idx << "(" << proc_if->process_name << ") - " << Interface::type_to_string(proc_if->type) << " assigned to color: " << color << std::endl;
            write_interface_coloring[color].push_back(proc_if);
         }
         if (proc_if->type == Interface::R or (rw_right and proc_if->type == Interface::RW))
         {
            proc->read_interfaces.push_back(proc_if);
            unsigned color = 0;
            bool found = false;
            do
            {
               found = false;
               for (const auto& intf : read_interface_coloring[color])
               {
                  if (intf->process_name == proc_if->process_name or (active_sharing & COLORING) == 0)
                     found = true;
                  else
                  {
                     bool share = false;
                     for (auto& sh : buff->sharing[proc_if->idx])
                     {
                        if (sh == intf->idx)
                        {
                           share = true;
                        }
                     }
                     if (!share)
                        found = true;
                  }
               }
               if (color == read_interface_coloring.size())
               {
                  break;
               }
               if (found) color++;
            }
            while (found);
            read_interface_coloring[color].push_back(proc_if);
            inverse_color[proc_if] = color;
         }
      }
   }

   unsigned int r_ports = read_interface_coloring.size();
   unsigned int w_ports = write_interface_coloring.size();
   DEBUG(DBG_VERBOSE, verbosity, "  Required number of ports: W = " << std::setw(5) << std::left << w_ports << std::setw(10) << std::left << Interface::pattern_to_string(write_type) << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, std::string(28, ' '));
   DEBUG(DBG_VERBOSE, verbosity, "R = " << std::setw(5) << std::left << r_ports << std::setw(10) << std::left << Interface::pattern_to_string(read_type) << std::endl);
   ///this is the size of the write block (i.e. how many parallel blocks are required to implement the writes)
   buff->w_block_size = w_ports;

   if (buff->w_block_size > 1 and write_type != Interface::LINEAR)
   {
      throw "It is not possible to create a memory wrapper for \"" + buff->name + "\" with more than 2 writes with unknown pattern";
   }

   ///definition parallel blocks
   buff->p_blocks = read_type == Interface::LINEAR ? get_mcm(w_ports, r_ports) : (w_ports * r_ports);
   DEBUG(DBG_VERBOSE, verbosity, "  Number of PBlocks: " << std::setw(10) << std::left << buff->p_blocks);
   DEBUG(DBG_VERBOSE, verbosity, std::endl);
   buff->data_partitioning = (write_type == Interface::LINEAR or read_type == Interface::LINEAR);
   ///creation of parallel blocks
   for (unsigned int i = 0; i < buff->p_blocks; i++)
   {
      PBlockPtr pb(new PBlock);
      pb->parallel_id = i;
      unsigned int scaling_factor = read_type == Interface::LINEAR ? buff->p_blocks : buff->w_block_size;
      pb->height = (unsigned)ceil((double)buff->height / (double)(scaling_factor));
      pb->width = buff->width;
      buff->pblocks.push_back(pb);
   }
   ///definition of interfaces connection
   for (const auto& proc : buff->processes)
   {
      if (proc->write_interfaces.size() == 0) continue;
      for (unsigned int i = 0; i < buff->p_blocks; )
      {
         for (unsigned int m = 0; m < buff->w_block_size; m++)
         {
            InterfacePtr w_intf = proc->write_interfaces[m % proc->write_interfaces.size()];
            buff->pblocks[i++]->interfaces[0].push_back(PBlock::split_t(w_intf, 0));
         }
      }
   }
   unsigned int factor = buff->p_blocks / r_ports;
   if (!buff->data_partitioning)
   {
      for (unsigned int i = 0; i < buff->p_blocks; i++)
      {
         for (unsigned int m = 0; m < buff->read_interfaces.size(); m++)
         {
            InterfacePtr r_intf = buff->read_interfaces[m];
            if ((i / factor) % buff->read_interfaces.size() == inverse_color[r_intf] and (rw_right or r_intf->type == Interface::R))
               buff->pblocks[i]->interfaces[1].push_back(PBlock::split_t(r_intf, 0));
         }
      }
   }
   else
   {
      for (const auto& proc : buff->processes)
      {
         if (proc->read_interfaces.size() == 0) continue;
         for (unsigned int i = 0; i < buff->p_blocks; i++)
         {
            for (unsigned int m = 0; m < proc->read_interfaces.size(); m++)
            {
               InterfacePtr r_intf = proc->read_interfaces[m];
               if ((i % proc->read_interfaces.size() == inverse_color[r_intf]) and (rw_right or r_intf->type == Interface::R))
                  buff->pblocks[i]->interfaces[1].push_back(PBlock::split_t(r_intf, 0));
            }
         }
      }
   }
   DEBUG(DBG_VERBOSE, verbosity, *buff);
   DEBUG(DBG_VERBOSE, verbosity, std::string(35, '+') << std::endl);
   ///definition of merging/splitting possibilities based on the technology
   std::vector<unsigned> merge_candidates;
   for (unsigned int i = 1; i <= buff->w_block_size; i++)
   {
      if (buff->w_block_size % i == 0)
         merge_candidates.push_back(i);
   }
   std::list<std::tuple<double, unsigned, unsigned, unsigned, MemoryPtr> > configurations;
   for (std::vector<unsigned>::const_iterator i = merge_candidates.begin(); i != merge_candidates.end(); ++i)
   {
      DEBUG(DBG_VERBOSE, verbosity, "Merge factor: " << *i << std::endl);
      DEBUG(DBG_VERBOSE, verbosity, std::setw(7) << std::left << "  MemH");
      DEBUG(DBG_VERBOSE, verbosity, std::setw(5) << std::left << "MemW");
      DEBUG(DBG_VERBOSE, verbosity, std::setw(10) << std::left << "ReshapeW");
      DEBUG(DBG_VERBOSE, verbosity, std::setw(10) << std::left << "SplitF");
      DEBUG(DBG_VERBOSE, verbosity, std::setw(10) << std::left << "MergeF");
      DEBUG(DBG_VERBOSE, verbosity, std::setw(8) << std::left << "Scale");
      DEBUG(DBG_VERBOSE, verbosity, std::setw(10) << std::left << "ReshapeH");
      DEBUG(DBG_VERBOSE, verbosity, std::setw(10) << std::left << "PBlocks");
      DEBUG(DBG_VERBOSE, verbosity, std::setw(8) << std::left << "PBanks");
      DEBUG(DBG_VERBOSE, verbosity, std::setw(8) << std::left << "SBanks");
      DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::right << "Area    ");
      DEBUG(DBG_VERBOSE, verbosity, std::endl);
      DEBUG(DBG_VERBOSE, verbosity, "  " << std::string(99, '-') << std::endl);
      for(const auto &mem : mem_library->db)
      {
         DEBUG(DBG_VERBOSE, verbosity, std::string(2, ' ') << std::setw(5) << std::left << mem->height);
         DEBUG(DBG_VERBOSE, verbosity, std::setw(5) << std::left << mem->width);

         unsigned int reshaped_width = *i * buff->width;
         DEBUG(DBG_VERBOSE, verbosity, std::setw(10) << std::left << reshaped_width);

         unsigned int p_banks = buff->p_blocks;
         unsigned int split_factor = 1;
         unsigned int merge_factor = 1;
         if (*i == 1 and reshaped_width > mem->width)
         {
            split_factor = (unsigned)ceil((double)reshaped_width / (double)mem->width);
            p_banks *= split_factor;
         }
         if (*i > 1 and mem->width >= reshaped_width)
         {
            merge_factor = *i;
            unsigned int updated_w_block = buff->w_block_size / merge_factor;
            p_banks = (read_type == Interface::LINEAR ? get_mcm(updated_w_block, r_ports) : (updated_w_block * r_ports));
         }
         DEBUG(DBG_VERBOSE, verbosity, std::setw(10) << std::left << split_factor);
         DEBUG(DBG_VERBOSE, verbosity, std::setw(10) << std::left << merge_factor);

         unsigned int scaling_factor = read_type == Interface::LINEAR ? buff->p_blocks : buff->w_block_size;
         DEBUG(DBG_VERBOSE, verbosity, std::setw(8) << std::left << scaling_factor);

         unsigned int reshaped_height = (unsigned)ceil((double)buff->height / (double)(scaling_factor));
         DEBUG(DBG_VERBOSE, verbosity, std::setw(10) << std::left << reshaped_height);
         DEBUG(DBG_VERBOSE, verbosity, std::setw(10) << std::left << buff->p_blocks);

         unsigned s_banks = (unsigned)ceil((double)reshaped_height / (double)mem->height);
         double area = mem->area * double(p_banks * s_banks);
         DEBUG(DBG_VERBOSE, verbosity, std::setw(8) << std::left << p_banks);
         DEBUG(DBG_VERBOSE, verbosity, std::setw(8) << std::left << s_banks);
         DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::right << area);

         configurations.push_back(std::tuple<double, unsigned, unsigned, unsigned, MemoryPtr>(area, p_banks, split_factor, merge_factor, mem));

         DEBUG(DBG_VERBOSE, verbosity, std::endl);
      }
   }
   DEBUG(DBG_VERBOSE, verbosity, "  " << std::string(99, '-') << std::endl);
   configurations.sort();

   unsigned p_banks = std::get<1>(configurations.front());
   unsigned split_factor = std::get<2>(configurations.front());
   unsigned merge_factor = std::get<3>(configurations.front());
   MemoryPtr mem = std::get<4>(configurations.front());
   DEBUG(DBG_VERBOSE, verbosity, "Solution:" << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, "--Merge Factor   = " << merge_factor << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, "--Split Factor   = " << split_factor << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, "--Parallel Banks = " << p_banks << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, "--Memory IP      = " << mem->height << "x" << mem->width << std::endl);
   assert(!(merge_factor > 1 and split_factor > 1));

   if (split_factor > 1)
   {
      buff->split = split_factor;
      buff->w_block_size = buff->w_block_size * buff->split;
      std::vector<PBlockPtr> new_blocks;
      for (unsigned int i = 0; i < buff->p_blocks; i++)
      {
         for (unsigned int spl = 0; spl < split_factor; spl++)
         {
            PBlockPtr pb(new PBlock);
            pb->parallel_id = i * split_factor + spl;
            new_blocks.push_back(pb);
            pb->height = buff->pblocks[i]->height;
            pb->width = ceil((double)buff->pblocks[i]->width / (double)split_factor);
            for (const auto& idx : buff->pblocks[i]->interfaces)
            {
               for (const auto& it : idx.second)
               {
                  InterfacePtr intf = std::get<0>(it);
                  pb->interfaces[idx.first].push_back(PBlock::split_t(intf, spl));
               }
            }
         }
      }
      buff->pblocks = new_blocks;
      buff->p_blocks = p_banks;
   }

   if (merge_factor > 1)
   {
      buff->merge = merge_factor;
      buff->w_block_size = buff->w_block_size / buff->merge;
      std::vector<PBlockPtr> new_blocks;
      for (unsigned int i = 0; i < p_banks; i++)
      {
         PBlockPtr pb(new PBlock);
         pb->parallel_id = i;
         new_blocks.push_back(pb);
         pb->height = buff->pblocks[0]->height;
         pb->width = buff->pblocks[i]->width * merge_factor;
         for (unsigned int spl = 0; spl < merge_factor; spl++)
         {
            for (const auto& idx : buff->pblocks[i * merge_factor + spl]->interfaces)
            {
               for (const auto& it : idx.second)
               {
                  InterfacePtr intf = std::get<0>(it);
                  PBlock::split_t tp(intf, 0);
                  if (std::find(pb->interfaces[idx.first].begin(), pb->interfaces[idx.first].end(), tp) == pb->interfaces[idx.first].end())
                  {
                     pb->interfaces[idx.first].push_back(tp);
                  }
               }
            }
         }
      }
      buff->pblocks = new_blocks;
      buff->p_blocks = p_banks;
   }
   DEBUG(DBG_VERBOSE, verbosity, "** Resulting buffer implementation:" << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, *buff);
   DEBUG(DBG_VERBOSE, verbosity, std::string(80, '+') << std::endl);
}

void Allocation::perform_clique_partitioning(const FunctorPtr opt_functor, std::map<unsigned int, clique>& valid_partitions, std::map<clique, PartitionPtr>& computed_partitions)
{
   std::map<std::string, clique> id_to_clique;
   ///creating ILP formulation for solving the clique covering problem
   if (!boost::filesystem::exists(out_dir)) boost::filesystem::create_directories(out_dir);
   DEBUG(DBG_VERBOSE, verbosity, "** Saving GLPK problem: \"" << out_dir << "/" << std::string(GLPK_FILE) << "\"..." << std::endl);
   std::ofstream os((out_dir + "/" GLPK_FILE).c_str(), std::ios_base::out);
   os << "#problem formulation" << std::endl;
   //variables
   os << "#variables" << std::endl;
   unsigned int id = 0;
   for (auto& c : valid_partitions)
   {
      os << "#Partition {";
      bool is_first = true;
      for (auto& j : c.second)
      {
         if (!is_first) os << ", ";
         is_first = false;
         ArrayPtr buff = components->node_to_buffer[std::get<0>(j)][std::get<1>(j)];
         os << buff->accelerator + "_" + buff->name;
      }
      os << "}" << std::endl;
      std::string str = accelerator_name + "_" + boost::lexical_cast<std::string>(id++);
      id_to_clique[str] = c.second;
      os << "var " << str << " binary;" << std::endl;
   }
   //objective function
   os << "#objective function" << std::endl;
   os << "minimize obj: ";
   bool first = true;
   id = 0;
   DEBUG(DBG_VERBOSE, verbosity, std::string(80, '+') << std::endl);
   for (auto& c : id_to_clique)
   {
      if (!first) os << " + ";
      first = false;
      PartitionPtr sol = computed_partitions[c.second];
      os << std::fixed;
      os << std::setprecision(5);
      os << "(" << sol->cost << ") * " << c.first;
      DEBUG(DBG_VERBOSE, verbosity, std::string(80, '+') << std::endl);
   }
   os << ";" << std::endl;
   //constraints
   os << "#constraints" << std::endl;
   id = 0;
   std::map<unsigned int, std::set<std::string> > clique_to_buffers;
   std::map<std::string, std::set<unsigned int> > buffer_to_cliques;
   for (auto& c : valid_partitions)
   {
      for (auto& j : c.second)
      {
         ArrayPtr buff = components->node_to_buffer[std::get<0>(j)][std::get<1>(j)];
         clique_to_buffers[id].insert(buff->accelerator + "_" + buff->name);
         buffer_to_cliques[buff->accelerator + "_" + buff->name].insert(id);
      }
      id++;
   }
   unsigned int numc = 0;
   for (auto& b : buffer_to_cliques)
   {
      os << "#covering of " << b.first << std::endl;
      os << "s.t. c" << numc++ << ": ";
      bool is_first = true;
      for (auto& c : b.second)
      {
         std::string str = accelerator_name + "_" + boost::lexical_cast<std::string>(c);
         if (!is_first) os << " + ";
         is_first = false;
         os << str;
      }
      os << " == 1;" << std::endl;
   }
   //solution
   os << "solve;" << std::endl;
   os << "#display results" << std::endl;
   first = true;
   os << "display ";
   id = 0;
   for (auto& c : valid_partitions)
   {
      std::string str = accelerator_name + "_" + boost::lexical_cast<std::string>(id++);
      if (!first) os << ", ";
      first = false;
      os << str;
   }
   os << ";" << std::endl;
   os << "end;" << std::endl;
   os.close();
   DEBUG(DBG_VERBOSE, verbosity, std::endl);

   ///solving the formulation
   DEBUG(DBG_VERBOSE, verbosity, "** Executing the ILP solver..." << std::endl);
   std::string command = "glpsol --math " + out_dir + "/" + GLPK_FILE + " -o " + out_dir + "/solver.result --log " + out_dir + "/solver.log >& " + out_dir + "/glpk.log";
   std::cerr << command << std::endl;
   system(command.c_str());
   DEBUG(DBG_VERBOSE, verbosity, std::endl);
   DEBUG(DBG_VERBOSE, verbosity, "** Reading glpk log = " + out_dir + "/solver.log" << std::endl);
   if (!boost::filesystem::exists(out_dir + "/solver.log"))
      throw std::runtime_error("problem not correctly solved!");      
   std::ifstream fileos ((out_dir + "/solver.log").c_str());
   DEBUG(DBG_VERBOSE, verbosity, std::endl);

   ///creating wrapper architecture
   std::string line;
   DEBUG(DBG_MINIMUM, verbosity, "** Creating wrapper architectures..." << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, std::string(80, '+') << std::endl);
   if (fileos.is_open())
   {
      while (fileos.good())
      {
         getline(fileos,line);
         boost::trim(line);
         if (line.size() == 0) continue;
         if (line.find(".val = 1") == std::string::npos) continue;
         std::string var = line.substr(0, line.find(".val = 1"));

         std::string arch_id = "plm_unit";
         PartitionPtr p = computed_partitions[id_to_clique[var]];
         std::string buffers;
         for(const auto& node : id_to_clique[var])
         {
            ArrayPtr buff = components->node_to_buffer[std::get<0>(node)][std::get<1>(node)];
            if (arch_id.size()) arch_id += "__";
            arch_id += buff->accelerator + "_" + buff->name;
            if (buffers.size()) buffers += ",";
            buffers += buff->accelerator + "_" + buff->name;
         }
         DEBUG(DBG_VERBOSE, verbosity, "Wrapper: \"" << arch_id << "\"" << std::endl);
         DEBUG(DBG_VERBOSE, verbosity, std::setw(25) << std::left << "  Arrays: " << std::setw(5) << std::left << id_to_clique[var].size() << " -- {" << buffers << "}" << std::endl);

         MemoryWrapperPtr arch = create_architecture(p);
         opt_functor->wrappers[var] = arch;
         if (compose_id)
            opt_functor->wrappers[var]->id = arch_id;
         else
            opt_functor->wrappers[var]->id = accelerator_name;
         DEBUG(DBG_VERBOSE, verbosity, std::string(80, '+') << std::endl);
      }
   }
}

void Allocation::print_statistics(const FunctorPtr opt_functor)
{
   double local_data = 0;
   for(const auto& buff : components->id_to_buffer)
   {
      local_data += buff.second->width * buff.second->height;
   }

   ///final statistics
   double mem_area = 0;
   double plm_size = 0;
   std::cout << std::fixed;
   std::cout.precision(5);
   DEBUG(DBG_VERBOSE, verbosity, "-----" << std::endl);
   std::map<MemoryWrapperPtr, double> wrapper_area;
   for (auto& w : opt_functor->wrappers)
   {
      MemoryWrapperPtr arch = w.second;
      std::string buffers;
      for (auto& b : arch->buffers)
      {
         if (buffers.size()) buffers += ",";
         buffers += b->name;
      }
      if (arch->banks.size() == 0) throw "empty wrapper";
      PRINT("Wrapper = " << arch->id << " {" << buffers << "}" << std::endl);
      const MemoryPtr bank = arch->banks[0];
      PRINT("---Number of banks = " << arch->banks.size() << "x(" << bank->height << "x" << bank->width << ")" << std::endl);
      wrapper_area[arch] = (double)(bank->area * arch->banks.size());
      mem_area += wrapper_area[arch];
      plm_size += (bank->width * bank->height) * arch->banks.size();
      PRINT("---Area of the banks = " << bank->area * arch->banks.size() << std::endl);
      DEBUG(DBG_VERBOSE, verbosity, "-----" << std::endl);
   }
   PRINT("Total memory area = " << (double)mem_area << std::endl);
   PRINT("Local data        = " << (double)(local_data / (double)8) / (double)1024 << " KB" << std::endl);
   PRINT("Size of PLM       = " << (double)(plm_size / (double)8) / (double)1024 << " KB ");
   if (plm_size == local_data)
   {
      PRINT("(==)" << std::endl);
   }
   else
   {
      double difference = (plm_size-local_data)*100.0/local_data;
      PRINT("(");
      if (difference > 0) PRINT("+");
      PRINT(difference << "%)" << std::endl);
   }
}

void Allocation::print_xml_statistics(const FunctorPtr opt_functor)
{
   ///final statistics
   double mem_area = 0;
   std::map<MemoryWrapperPtr, double> wrapper_area;
   for (auto& w : opt_functor->wrappers)
   {
      MemoryWrapperPtr arch = w.second;
      const MemoryPtr bank = arch->banks[0];
      double area = (double)(bank->area * arch->banks.size());
      wrapper_area[arch] = area;
      mem_area += area;
   }

   xmlpp::Document* doc = new xmlpp::Document();
   xmlpp::Node* root_node = doc->create_root_node("mnemosyne_results");
   xmlpp::Element* wr_node = root_node->add_child("memory_wrappers");
   wr_node->set_attribute("number", boost::lexical_cast<std::string>(opt_functor->wrappers.size()));
   for (auto& w : opt_functor->wrappers)
   {
      xmlpp::Element* w_node = wr_node->add_child("memory_wrapper");
      MemoryWrapperPtr arch = w.second;
      w_node->set_attribute("area", boost::lexical_cast<std::string>(wrapper_area[arch]));
   }
   wr_node->set_attribute("area", boost::lexical_cast<std::string>((double)mem_area));
   doc->write_to_file_formatted("./results.xml");
}

void Allocation::perform_allocation(const FunctorPtr opt_functor, unsigned int active_sharing, std::map<std::string, MemoryWrapperPtr>& wrappers)
{
   opt_functor->accelerator_name = accelerator_name;

   DEBUG(DBG_MINIMUM, verbosity, "** Determining data allocation for the buffers..." << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, std::string(80, '+') << std::endl);
   for(const auto& component : buffers->db)
   {
      for(const auto& buff : component.second)
      {
         ///DISTRIBUTION/DUPLICATION: used to determine the number of parallel banks;
         ///if data access patterns are unknown or not predictable, data needs to be duplicated
         determine_data_allocation(buff, active_sharing);
      }
   }
   DEBUG(DBG_VERBOSE, verbosity, std::endl);

   DEBUG(DBG_MINIMUM, verbosity, "** mnemosyne: memory optimization..." << std::endl);
   ///check for active sharing options
   if (active_sharing & ADDRESS_SPACE) DEBUG(DBG_VERBOSE, verbosity,"(ADDRESS SPACE)");
   if (active_sharing & INTERFACE) DEBUG(DBG_VERBOSE, verbosity,"(INTERFACE)");
   if (active_sharing & COLORING) DEBUG(DBG_VERBOSE, verbosity,"(COLORING)");
   if (active_sharing == 0) DEBUG(DBG_VERBOSE, verbosity,"(no optimizations)");
   DEBUG(DBG_VERBOSE, verbosity, std::endl);

   std::map<unsigned int, clique> partitions;
   DEBUG(DBG_MINIMUM, verbosity, "** Enumerate cliques (" << components->node_list.size() << " nodes)..." << std::endl);
   for(unsigned int i = 0; i < components->node_list.size(); i++)
   {
      for (auto& c : partitions)
      {
         std::set<ComponentList::node_t> new_c = c.second;
         bool comp = true;
         for (auto& j : c.second)
         {
            if (std::get<0>(components->node_list[i]) == std::get<0>(j) and components->comp_list[j].find(components->node_list[i]) == components->comp_list[j].end() or
                ((active_sharing & (ADDRESS_SPACE | INTERFACE)) == 0))
            {
               comp = false;
            }
         }
         if (!comp) continue;
         new_c.insert(components->node_list[i]);
         unsigned int clique_id = partitions.size();
         partitions[clique_id] = new_c;
      }
      std::set<ComponentList::node_t> single_clique;
      single_clique.insert(components->node_list[i]);
      unsigned int clique_id = partitions.size();
      partitions[clique_id] = single_clique;
   }
   if (partitions.size() == 0) throw std::runtime_error("No cliques have been created");

   std::map<unsigned int, clique> valid_partitions;
   DEBUG(DBG_MINIMUM, verbosity, "** Clique post-processing to eliminate unfeasible ones..." << std::endl);
   for(const auto& c : partitions)
   {
      unsigned int num_interface_compatibilities = 0;
      for (const auto& node : c.second)
      {
         for(const auto& node2 : c.second)
         {
            if (node == node2) break;
            if (std::get<0>(node2) == std::get<0>(node))
            {
               std::pair<ULink, bool> edge = boost::edge(std::get<1>(node), std::get<1>(node2), *std::get<0>(node2));
               if (edge.second)
               {
                  std::string type = boost::get(&Arc::type, *(std::get<0>(node2)), edge.first);
                  if (type == "b") num_interface_compatibilities++;
               }
            }
         }
      }
      if (num_interface_compatibilities > 1) continue;
      if (num_interface_compatibilities == 1 and ((active_sharing & INTERFACE) == 0)) continue;
      unsigned clique_num = valid_partitions.size();
      valid_partitions[clique_num] = c.second;
      DEBUG(DBG_VERBOSE, verbosity, "  Arrays: " << std::setw(3) << std::left << c.second.size());
      DEBUG(DBG_VERBOSE, verbosity, "  -- {" << components->get_clique_string(c.second) << "}" << std::endl);
   }
   if (valid_partitions.size() == 0) throw std::runtime_error("No admissible cliques have been created");

   DEBUG(DBG_MINIMUM, verbosity, "Number of admissible cliques: " << valid_partitions.size() << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, std::endl);

   std::map<clique, PartitionPtr> computed_partitions;
   DEBUG(DBG_MINIMUM, verbosity, "** Characterizing cliques..." << std::endl);
   ///characterizing clique cost
   for (auto& c : valid_partitions)
   {
      DEBUG(DBG_MINIMUM, verbosity, "Computing cost of clique {" + components->get_clique_string(c.second) << "}" << std::endl);
      PartitionPtr sol = opt_functor->compute_clique_cost(c.second);
      if (!sol) throw std::runtime_error("Clique not characterized");
      computed_partitions[c.second] = sol;
      DEBUG(DBG_VERBOSE, verbosity, std::string(80, '+') << std::endl);
   }

   DEBUG(DBG_MINIMUM, verbosity, "** Solving graph partitioning..." << std::endl);
   perform_clique_partitioning(opt_functor, valid_partitions, computed_partitions);
   PRINT("Number of generated wrappers = " << opt_functor->wrappers.size() << std::endl);

   print_statistics(opt_functor);
   if (verbosity > DBG_NONE)
     print_xml_statistics(opt_functor);
}

void Allocation::set_ce_binding(const InterfacePtr mem_intf, unsigned int mem_intf_idx, std::string activation)
{
   //default value
   if (mem_intf->enable->binding.size() == 0) mem_intf->enable->binding = "1'b0";
   mem_intf->enable->binding = activation + " ? 1'b1 : (" + mem_intf->enable->binding + ")";
   DEBUG(DBG_VERBOSE, verbosity, "     CE[" << mem_intf_idx << "] = { " << activation << " }" << std::endl);
}

void Allocation::set_address_binding(const PartitionPtr partition, const ArrayPtr buff, const InterfacePtr proc_intf, const MemoryPtr bank, unsigned int mem_intf_idx, std::string activation, const std::string& prefix)
{
   TagPtr tag = partition->buffer_tag[buff];
   ///address signal
   unsigned int a_lsb = tag->ptag_size + tag->mtag_size;
   unsigned int a_msb = tag->ptag_size + tag->mtag_size;
   if (!partition->buffer_offset[buff])
      a_msb += tag->bank_size-1;
   else
      a_msb = tag->buff_size-1;
   assert(a_msb >= a_lsb);
   std::string padding_str;
   if (a_msb >= tag->buff_size and partition->buffer_offset[buff] == 0)
   {
      unsigned int padd_num = (a_msb + 1 - tag->buff_size);
      padding_str = "{" + boost::lexical_cast<std::string>(padd_num) + "{1'b0}}";
      a_msb = tag->buff_size-1;
   }
   if (tag->bank_size > tag->buff_size)
   {
      unsigned int padd_num = (tag->bank_size - tag->buff_size);
      padding_str = "{" + boost::lexical_cast<std::string>(padd_num) + "{1'b0}}";
   }

   unsigned int buff_size = ceil((double)log(buff->height) / (double)log(2));
   std::string abit_cast;
   if (buff_size > 1)
      abit_cast = "[" + boost::lexical_cast<std::string>(a_msb) + ":" + boost::lexical_cast<std::string>(a_lsb) + "]";
   if (partition->buffer_offset[buff]) abit_cast += " + " + boost::lexical_cast<std::string>(partition->buffer_offset[buff]);

   unsigned int bank_size = ceil((double)log(bank->height) / (double)log(2));
   std::string str_buffer;
   if (buff_size > 0)
   {
      if (padding_str.size()) str_buffer = "{" + padding_str + ",";
      str_buffer += prefix + "A" + boost::lexical_cast<std::string>(proc_intf->idx) + abit_cast;
      if (padding_str.size()) str_buffer += "}";
   }
   else
   {
      str_buffer = "{" + boost::lexical_cast<std::string>(bank_size) + "{1'b0}}";
   }

   const InterfacePtr mem_intf = bank->interfaces[mem_intf_idx];
   //address
   if (mem_intf->address->binding.size() == 0) mem_intf->address->binding = "{" + boost::lexical_cast<std::string>(bank_size) + "{1'b0}}";
   mem_intf->address->binding = activation + " ? " + str_buffer + " : (" + mem_intf->address->binding + ")";
   DEBUG(DBG_VERBOSE, verbosity, "     A[" << mem_intf_idx << "] = { " << str_buffer << " }" << std::endl);
}

std::string Allocation::get_activation(const PartitionPtr partition, const MemoryPtr bank, unsigned int mem_intf_idx, const binding_t& bind, const std::string& prefix)
{
   const ArrayPtr buff = std::get<0>(bind);
   InterfacePtr proc_intf = std::get<1>(bind);
   unsigned int p = std::get<4>(bind);
   unsigned int s = std::get<5>(bind);

   DEBUG(DBG_VERBOSE, verbosity, "  +Bank = {" << bank->id << "}[" << mem_intf_idx << "] <--> Proc Interface = {" << proc_intf->idx << " -- " << Interface::type_to_string(proc_intf->type) <<  "} (P = {" << p << "} :: S = {" << s << "})" << std::endl);

   TagPtr tag = partition->buffer_tag[buff];

   ///activation function
   std::string activation = prefix + "CE" + boost::lexical_cast<std::string>(proc_intf->idx);
   if (tag->stag_size > 0)
   {
      unsigned int s_lsb = (buff->data_partitioning ? 0 : tag->ptag_size) + tag->mwtag_size + tag->mtag_size + tag->bank_size;
      unsigned int s_msb = tag->buff_size - 1;
      assert(s_msb >= s_lsb);
      std::string sbit_cast = "[" + boost::lexical_cast<std::string>(s_msb) + ":" + boost::lexical_cast<std::string>(s_lsb) + "]";
      activation += " & " + prefix + "A" + boost::lexical_cast<std::string>(proc_intf->idx) + sbit_cast + " == " + boost::lexical_cast<std::string>(s);
   }
   if (tag->ptag_size > 0)
   {
      unsigned int s_lsb = tag->mtag_size;
      unsigned int s_msb = tag->mtag_size + tag->ptag_size - 1;
      assert(s_msb >= s_lsb);
      std::string sbit_cast = "[" + boost::lexical_cast<std::string>(s_msb) + ":" + boost::lexical_cast<std::string>(s_lsb) + "]";
      activation += " & " + prefix + "A" + boost::lexical_cast<std::string>(proc_intf->idx) + sbit_cast + " == " + boost::lexical_cast<std::string>(p);
   }
   if (tag->mwtag_size > 0)
   {
      unsigned int s_lsb = 0;
      unsigned int s_msb = tag->mwtag_size - 1;
      assert(s_msb >= s_lsb);
      std::string sbit_cast = "[" + boost::lexical_cast<std::string>(s_msb) + ":" + boost::lexical_cast<std::string>(s_lsb) + "]";
      activation += " & " + prefix + "A" + boost::lexical_cast<std::string>(proc_intf->idx) + sbit_cast + " == " + boost::lexical_cast<std::string>(p % (buff->w_block_size / buff->split));
   }
   DEBUG(DBG_VERBOSE, verbosity, "     activation = { " << activation << " }" << std::endl);
   return activation;
}

void Allocation::set_default_binding(const MemoryPtr bank, unsigned int mem_intf_idx)
{
   const InterfacePtr mem_intf = bank->interfaces[mem_intf_idx];
   unsigned int bank_size = ceil((double)log(bank->height) / (double)log(2));
   //enable
   if (mem_intf->enable->binding.size() == 0) mem_intf->enable->binding = "1'b0";
   //address
   if (mem_intf->address->binding.size() == 0) mem_intf->address->binding = "{" + boost::lexical_cast<std::string>(bank_size) + "{1'b0}}";
   //data_in
   if (mem_intf->data_in->binding.size() == 0)
   {
      mem_intf->data_in->binding = "1'b0";
      if (bank->width > 1) mem_intf->data_in->binding = "{" + boost::lexical_cast<std::string>(bank->width) + "{" + mem_intf->data_in->binding + "}}";
   }
   //write enable
   if (mem_intf->write_enable->binding.size() == 0) mem_intf->write_enable->binding = "1'b0";
   //write mask
   if (mem_intf->write_mask->binding.size() == 0)
   {
      mem_intf->write_mask->binding = "1'b0";
      if (bank->width > 1) mem_intf->write_mask->binding = "{" + boost::lexical_cast<std::string>(bank->width) + "{" + mem_intf->write_mask->binding + "}}";
   }
}

void Allocation::createReadSelectors(const PartitionPtr partition, const MemoryWrapperPtr arch, const ArrayPtr buff, const std::string& prefix)
{
   DEBUG(DBG_VERBOSE, verbosity, "** Creating read selectors..." << std::endl);
   for(const auto& intf : buff->read_interfaces)
   {
      DEBUG(DBG_VERBOSE, verbosity, " Read interface idx = " << intf->idx << std::endl);

      if (buff->merge > 1)
      {
         WirePtr wire(new Wire);
         wire->id = "sel_merge_" + prefix + "Q" + boost::lexical_cast<std::string>(intf->idx);
         wire->size = partition->buffer_tag[buff]->mtag_size;
         wire->is_buffered = true;
         arch->wires.push_back(wire);
         arch->merge_selector[buff][intf] = wire;
         DEBUG(DBG_VERBOSE, verbosity, std::setw(30) << std::left << "  +Created merge selector" << " = ");
         DEBUG(DBG_VERBOSE, verbosity, std::setw(40) << std::left << wire->id);
         DEBUG(DBG_VERBOSE, verbosity, " -- size = " << wire->size << std::endl);
      }

      unsigned int size = 1;
      if (partition->num_banks > 1)
         size = ceil(log((double)partition->num_banks) / log((double)2));
      if (buff->split > 1)
      {
         for(unsigned int num = 0; num < buff->split; num++)
         {
            WirePtr wire(new Wire);
            wire->id = "sel_split_" + prefix + "Q" + boost::lexical_cast<std::string>(intf->idx) + "_s" + boost::lexical_cast<std::string>(num);
            wire->size = size;
            wire->is_buffered = true;
            arch->wires.push_back(wire);
            arch->split_selector[buff][intf].push_back(wire);
            std::string num_str = boost::lexical_cast<std::string>(num);
            DEBUG(DBG_VERBOSE, verbosity, std::setw(30) << std::left << "  +Created split selector (" << num_str + ")" << " = ");
            DEBUG(DBG_VERBOSE, verbosity, std::setw(40) << std::left << wire->id);
            DEBUG(DBG_VERBOSE, verbosity, " -- size = " << wire->size << std::endl);
         }
      }
      else
      {
         WirePtr wireq0(new Wire);
         wireq0->id = "sel_Q0_" + prefix + boost::lexical_cast<std::string>(intf->idx);
         wireq0->size = size;
         wireq0->is_buffered = true;
         arch->wires.push_back(wireq0);
         DEBUG(DBG_VERBOSE, verbosity, std::setw(30) << std::left << "  +Created std selector [0]" << " = ");
         DEBUG(DBG_VERBOSE, verbosity, std::setw(40) << std::left << wireq0->id);
         DEBUG(DBG_VERBOSE, verbosity, " -- size = " << wireq0->size << std::endl);

         WirePtr wireq1(new Wire);
         wireq1->id = "sel_Q1_" + prefix + boost::lexical_cast<std::string>(intf->idx);
         wireq1->size = size;
         wireq1->is_buffered = true;
         arch->wires.push_back(wireq1);
         DEBUG(DBG_VERBOSE, verbosity, std::setw(30) << std::left << "  +Created std selector [1]" << " = ");
         DEBUG(DBG_VERBOSE, verbosity, std::setw(40) << std::left << wireq1->id);
         DEBUG(DBG_VERBOSE, verbosity, " -- size = " << wireq1->size << std::endl);

         for(unsigned int num = 0; num < partition->num_banks; num++)
         {
            arch->banks[num]->selector[0][intf] = wireq0;
            arch->banks[num]->selector[1][intf] = wireq1;
         }
      }
   }
}

void Allocation::create_memory_interfaces(const PartitionPtr partition, const MemoryWrapperPtr arch)
{
   ///creating ports for all interfaces
   for (const auto &bit : partition->buffer_offset)
   {
      const ArrayPtr b = bit.first;
      arch->buffers.push_back(b);
      arch->buffer_offset[b] = bit.second;
      std::string prefix;
      if (buffers->db.size() > 1 or buffers->db[b->accelerator].size() > 1)
         prefix = b->accelerator + "_" + b->name + "_";
      ///creates the ports for each interface
      for(const auto& intf : b->interfaces)
      {
         ///size of address provided by HLS tool is detemined by the size of the buffer
         unsigned int size = ceil(log((double)b->height) / log((double)2));
         std::string suffix = boost::lexical_cast<std::string>(intf->idx);
         intf->enable = PortPtr(new Port(prefix+"CE"+suffix, Port::IN, 1));
         intf->ports.push_back(intf->enable);
         intf->address = PortPtr(new Port(prefix+"A"+suffix, Port::IN, size));
         intf->ports.push_back(intf->address);
         if (intf->type == Interface::W or intf->type == Interface::RW)
         {
            intf->data_in = PortPtr(new Port(prefix+"D"+suffix, Port::IN, b->width));
            intf->ports.push_back(intf->data_in);
            intf->write_enable = PortPtr(new Port(prefix+"WE"+suffix, Port::IN, 1));
            intf->ports.push_back(intf->write_enable);
            intf->write_mask = PortPtr(new Port(prefix+"WEM"+suffix, Port::IN, b->width));
            intf->ports.push_back(intf->write_mask);
         }
         if (intf->type == Interface::R or intf->type == Interface::RW)
         {
            intf->data_out = PortPtr(new Port(prefix+"Q"+suffix, Port::OUT, b->width));
            intf->ports.push_back(intf->data_out);
         }
      }
   }
}

void Allocation::createArrayArchitecture(const PartitionPtr partition, const MemoryWrapperPtr arch, const ArrayPtr buff, const std::string& prefix)
{
   DEBUG(DBG_VERBOSE, verbosity, "Binding buffer = " << std::setw(25) << std::left << prefix);
   DEBUG(DBG_VERBOSE, verbosity, buff->height << "x" << buff->width << " -- ");
   DEBUG(DBG_VERBOSE, verbosity, (buff->data_partitioning ? "DISTRIBUTED" : "NOT DISTRIBUTED") << std::endl);

   partition->buffer_tag[buff] = TagPtr(new Tag(partition, buff, verbosity));
   TagPtr tag = partition->buffer_tag[buff];

   ///create selectors for read interfaces
   createReadSelectors(partition, arch, buff, prefix);

   unsigned int mem_idx = (buff->interfaces[0]->type == Interface::W) ? 1 : 0;
   if (buff->merge > 1)
   {
      for (const auto& buff_intf : buff->read_interfaces)
      {
         DEBUG(DBG_VERBOSE, verbosity, " Read interface idx = " << buff_intf->idx << std::endl);
         DEBUG(DBG_VERBOSE, verbosity, "  +Merge selector = " << arch->merge_selector[buff][buff_intf]->id << std::endl);
         std::string bind_str;
         if (arch->banks.size() == 1)
            bind_str = arch->banks[0]->id + "_Q" + boost::lexical_cast<std::string>(mem_idx);
         else
         {
            bind_str = "Q" + boost::lexical_cast<std::string>(mem_idx) + "_out";
            bind_str += "[sel_Q1_" + prefix + boost::lexical_cast<std::string>(buff_intf->idx) + "]";
         }
         for(unsigned int i = 0; i < buff->merge; i++)
         {
            if (buff_intf->data_out->binding.size() == 0)
               buff_intf->data_out->binding = "{" + boost::lexical_cast<std::string>(buff->width) + "{1'b0}}";
            unsigned int lsb = buff->width * i;
            unsigned int msb = (buff->width * (i+1))-1;
            assert(msb >= lsb);
            std::string bit_cast = "[" + boost::lexical_cast<std::string>(msb) + ":" + boost::lexical_cast<std::string>(lsb) + "]";
            DEBUG(DBG_VERBOSE, verbosity, "  +Merge (" << i << ") = " << bind_str + bit_cast << std::endl);
            buff_intf->data_out->binding = arch->merge_selector[buff][buff_intf]->id + " == " + boost::lexical_cast<std::string>(i) + " ? " + bind_str + bit_cast + " : (" + buff_intf->data_out->binding + ")";
         }

         if (arch->merge_selector[buff][buff_intf]->binding.size() == 0)
         {
            arch->merge_selector[buff][buff_intf]->binding = "{" + boost::lexical_cast<std::string>(arch->merge_selector[buff][buff_intf]->size) + "{1'b0}}";
         }

         TagPtr tag = partition->buffer_tag[buff];
         unsigned int lsb = 0;
         unsigned int msb = tag->mtag_size-1;
         assert(msb >= lsb);
         std::string bit_cast = "[" + boost::lexical_cast<std::string>(msb) + ":" + boost::lexical_cast<std::string>(lsb) + "]";
         std::string activation = prefix + "CE" + boost::lexical_cast<std::string>(buff_intf->idx);
         arch->merge_selector[buff][buff_intf]->binding = activation + " ? "+ prefix + "A" + boost::lexical_cast<std::string>(buff_intf->idx) + bit_cast + " : (" + arch->merge_selector[buff][buff_intf]->binding + ")";
      }
   }

   if (buff->split > 1)
   {
      std::map<InterfacePtr, std::vector<std::tuple<PBlock::split_t, unsigned int> > > read_split_interfaces;
      for(unsigned int p = 0; p < buff->p_blocks; p++)
      {
         PBlockPtr pb = buff->pblocks[p];
         for (const auto& idx : pb->interfaces)
         {
            for (const auto& it : idx.second)
            {
               const InterfacePtr buff_intf = std::get<0>(it);
               if (buff_intf->type == Interface::R or buff_intf->type == Interface::RW)
                  read_split_interfaces[buff_intf].push_back(std::tuple<PBlock::split_t, unsigned int>(it, p));
            }
         }
      }
      for (const auto& buff_intf : read_split_interfaces)
      {
         std::string align_str;
         for (const auto& it : buff_intf.second)
         {
            unsigned int s_idx = std::get<1>(std::get<0>(it));
            if (align_str.size())
               align_str = "," + align_str;
            std::string slice;
            if (s_idx+1 == buff->split)
            {
               if ((s_idx+1) * arch->banks[0]->width > buff->width)
               {
                  slice = "[" + boost::lexical_cast<std::string>(buff->width - (s_idx) * arch->banks[0]->width - 1) + ":0]";
               }
            }
            align_str = "Q" + boost::lexical_cast<std::string>(1) + "_out[" + arch->split_selector[buff][buff_intf.first][s_idx]->id + "]" + slice + align_str;
            buff_intf.first->data_out->binding = "{" + align_str + "}";
         }
      }
   }

   DEBUG(DBG_VERBOSE, verbosity, std::string(50, '-') << std::endl);
   ///ASSIGN interfaces
   for(unsigned int p = 0; p < buff->p_blocks; p++)
   {
      PBlockPtr pb = buff->pblocks[p];
      for(unsigned int s = 0; s < partition->buffer_configuration[buff][p].size(); s++)
      {
         unsigned int bank_id = partition->buffer_configuration[buff][p][s];
         assert(arch->banks.size() > bank_id);
         MemoryPtr bank = arch->banks[bank_id];
         DEBUG(DBG_VERBOSE, verbosity, "PBlock = " << p << " :: ");
         DEBUG(DBG_VERBOSE, verbosity, "SBlock = " << s << " :: ");
         DEBUG(DBG_VERBOSE, verbosity, "BankId = " << bank_id << std::endl);

         for (const auto& idx : pb->interfaces)
         {
            set_default_binding(bank, idx.first);
         }

         ///connect write interfaces
         if (buff->merge > 1)
         {
            for (const auto& idx : pb->interfaces)
            {
               std::map<std::string, std::vector<PBlock::split_t> > write_merge_interfaces, read_merge_interfaces;
               for (const auto& it : idx.second)
               {
                  const InterfacePtr buff_intf = std::get<0>(it);
                  if (buff_intf->type == Interface::W)
                     write_merge_interfaces[buff_intf->process_name].push_back(it);
                  else if (buff_intf->type == Interface::R)
                     read_merge_interfaces[buff_intf->process_name].push_back(it);
               }
               ///START of merge WRITE
               for (const auto& m : write_merge_interfaces)
               {
                  std::string data_in, data_we, data_wmask;
                  std::string activation;
                  for (const auto& it : m.second)
                  {
                     const InterfacePtr buff_intf = std::get<0>(it);
                     std::string intf_idx = boost::lexical_cast<std::string>(buff_intf->idx);
                     binding_t bind(buff, buff_intf, bank_id, idx.first, p, s);
                     std::string l_activation = get_activation(partition, bank, idx.first, bind, prefix);
                     ///chip enable
                     set_ce_binding(bank->interfaces[idx.first], idx.first, l_activation);
                     ///address
                     set_address_binding(partition, buff, buff_intf, bank, idx.first, l_activation, prefix);
                     if (data_in.size())
                        data_in = "," + data_in;
                     if (data_we.size())
                        data_we = " | " + data_we;
                     if (data_wmask.size())
                        data_wmask = "," + data_wmask;
                     data_in = prefix + "D" + intf_idx + data_in;
                     data_we = prefix + "WE" + intf_idx + data_we;
                     data_wmask = prefix + "WEM" + intf_idx + data_wmask;
                     if (activation.size())
                        activation += " || ";
                     activation += l_activation;
                  }
                  const InterfacePtr mem_intf = bank->interfaces[idx.first];
                  DEBUG(DBG_VERBOSE, verbosity, "     D[" << idx.first << "] = { " << data_in << " }" << std::endl);
                  DEBUG(DBG_VERBOSE, verbosity, "     WE[" << idx.first << "] = { " << data_we << " }" << std::endl);
                  DEBUG(DBG_VERBOSE, verbosity, "     WEM[" << idx.first << "] = { " << data_wmask << " }" << std::endl);
                  mem_intf->data_in->binding = activation + " ? {" + data_in + "} : (" + mem_intf->data_in->binding + ")";
                  mem_intf->write_enable->binding = activation + " ? (" + data_we + ") : (" + mem_intf->write_enable->binding + ")";
                  mem_intf->write_mask->binding = activation + " ? {" + data_wmask + "} : (" + mem_intf->write_mask->binding + ")";
               }
               ///END of merge WRITE

               ///START of merge READ
               for (const auto& m : read_merge_interfaces)
               {
                  for (const auto& it : m.second)
                  {
                     const InterfacePtr buff_intf = std::get<0>(it);
                     binding_t bind(buff, buff_intf, bank_id, idx.first, p, s);
                     std::string activation = get_activation(partition, bank, idx.first, bind, prefix);

                     set_ce_binding(bank->interfaces[idx.first], idx.first, activation);
                     set_address_binding(partition, buff, buff_intf, bank, idx.first, activation, prefix);

                     if (partition->num_banks > 1)
                     {
                        if (bank->selector[idx.first][buff_intf]->binding.size() == 0)
                           bank->selector[idx.first][buff_intf]->binding = "{" + boost::lexical_cast<std::string>(bank->selector[idx.first][buff_intf]->size) + "{1'b0}}";
                        bank->selector[idx.first][buff_intf]->binding = activation + " ? " + boost::lexical_cast<std::string>(bank_id) + " : (" + bank->selector[idx.first][buff_intf]->binding + ")";
                     }
                  }
               }
               ///END of merge READ
            }
         }
         else //not merge
         {
            for (const auto& idx : pb->interfaces)
            {
               for (const auto& it : idx.second)
               {
                  const InterfacePtr buff_intf = std::get<0>(it);
                  unsigned int mem_intf_idx = idx.first;
                  binding_t bind(buff, buff_intf, bank_id, idx.first, p, s);
                  if (buff_intf->type == Interface::W or buff_intf->type == Interface::RW)
                  {
                     ///<buffer, process_interface, bank_id, port, parallel_block, serial_block>
                     unsigned int s_idx = std::get<1>(it);
                     const InterfacePtr mem_intf = bank->interfaces[mem_intf_idx];
                     set_default_binding(bank, mem_intf_idx);

                     std::string activation = get_activation(partition, bank, mem_intf_idx, bind, prefix);
                     set_ce_binding(bank->interfaces[mem_intf_idx], mem_intf_idx, activation);
                     set_address_binding(partition, buff, buff_intf, bank, mem_intf_idx, activation, prefix);

                     std::string dinbit_cast;
                     unsigned int din_lsb = 0, din_msb = 0;
                     if (buff->width > 1)
                     {
                        ///datain
                        din_lsb = s_idx * bank->width;
                        din_msb = (s_idx+1) * bank->width - 1;
                        if (din_msb > buff->width-1)
                        {
                           din_msb = buff->width - 1;
                        }
                        dinbit_cast = "[" + boost::lexical_cast<std::string>(din_msb) + ":" + boost::lexical_cast<std::string>(din_lsb) + "]";
                     }
                     std::string binding = prefix + "D" + boost::lexical_cast<std::string>(buff_intf->idx) + dinbit_cast;
                     if (din_msb+1 < bank->width*(s_idx+1)) //data padding
                     {
                        binding = "{{" + boost::lexical_cast<std::string>(bank->width*(s_idx+1)-(din_msb+1)) + "{1'b0}}," + binding + "}";
                     }
                     mem_intf->data_in->binding = activation + " ? " + binding + " : (" + mem_intf->data_in->binding + ")";
                     mem_intf->write_enable->binding = activation + " ? " + prefix + "WE" + boost::lexical_cast<std::string>(buff_intf->idx) + " : (" + mem_intf->write_enable->binding + ")";
                     binding = prefix + "WEM" + boost::lexical_cast<std::string>(buff_intf->idx) + dinbit_cast;
                     if (din_msb+1 < bank->width*(s_idx+1))
                     {
                        binding = "{{" + boost::lexical_cast<std::string>(bank->width*(s_idx+1)-(din_msb+1)) + "{1'b0}}," + binding + "}";
                     }
                     mem_intf->write_mask->binding = activation + " ? " + binding + " : (" + mem_intf->write_mask->binding + ")";
                  }
               }
            }

            ///all interfaces
            for (const auto& idx : pb->interfaces)
            {
               for (const auto& it : idx.second)
               {
                  const InterfacePtr buff_intf = std::get<0>(it);
                  binding_t bind(buff, buff_intf, bank_id, idx.first, p, s);

                  if (buff_intf->type == Interface::R or buff_intf->type == Interface::RW)
                  {
                     std::string activation = get_activation(partition, bank, idx.first, bind, prefix);
                     set_ce_binding(bank->interfaces[idx.first], idx.first, activation);
                     set_address_binding(partition, buff, buff_intf, bank, idx.first, activation, prefix);
                  }
               }
            }

            if (buff->split > 1)
            {
               unsigned int i = (p % buff->split);

               for (const auto& buff_intf : buff->read_interfaces)
               {
                  bool found = false;
                  for (const auto& idx : pb->interfaces)
                  {
                     for (const auto& it : idx.second)
                     {
                        if (buff_intf == std::get<0>(it)) found = true;
                     }
                  }
                  if (!found) continue;
                  std::string local_act = prefix + "CE" + boost::lexical_cast<std::string>(buff_intf->idx);
                  if (arch->split_selector[buff][buff_intf][i]->binding.size() == 0)
                  {
                     if (arch->split_selector[buff][buff_intf][i]->size == 1)
                        arch->split_selector[buff][buff_intf][i]->binding = "1'b0";
                     else
                        arch->split_selector[buff][buff_intf][i]->binding = "{" + boost::lexical_cast<std::string>(arch->split_selector[buff][buff_intf][i]->size) + "{1'b0}}";
                  }
                  if (partition->buffer_configuration[buff][p].size() > 1)
                  {
                     TagPtr tag = partition->buffer_tag[buff];

                     unsigned int s_lsb = tag->ptag_size + tag->bank_size;
                     unsigned int s_msb = tag->buff_size - 1;
                     assert(s_msb >= s_lsb);
                     std::string sbit_cast = "[" + boost::lexical_cast<std::string>(s_msb) + ":" + boost::lexical_cast<std::string>(s_lsb) + "]";
                     local_act += " && " + prefix + "A" + boost::lexical_cast<std::string>(buff_intf->idx) + sbit_cast + " == " + boost::lexical_cast<std::string>(s);
                  }
                  arch->split_selector[buff][buff_intf][i]->binding = local_act + " ? "+ boost::lexical_cast<std::string>(bank_id) + " : (" + arch->split_selector[buff][buff_intf][i]->binding + ")";
               }
            }
            else // not split
            {
               for (const auto& idx : pb->interfaces)
               {
                  for (const auto& it : idx.second)
                  {
                     const InterfacePtr buff_intf = std::get<0>(it);
                     if (buff_intf->type != Interface::R and buff_intf->type != Interface::RW)
                        continue;
                     binding_t bind(buff, buff_intf, bank_id, idx.first, p, s);

                     std::string bind_str;
                     if (partition->num_banks > 1)
                     {
                        std::string activation = get_activation(partition, bank, idx.first, bind, prefix);
                        if (bank->selector[idx.first][buff_intf]->binding.size() == 0)
                           bank->selector[idx.first][buff_intf]->binding = "{" + boost::lexical_cast<std::string>(bank->selector[idx.first][buff_intf]->size) + "{1'b0}}";
                        bank->selector[idx.first][buff_intf]->binding = activation + " ? " + boost::lexical_cast<std::string>(bank_id) + " : (" + bank->selector[idx.first][buff_intf]->binding + ")";

                        bind_str = "Q" + boost::lexical_cast<std::string>(idx.first) + "_out[sel_Q1_" + prefix + boost::lexical_cast<std::string>(buff_intf->idx) + "]";
                        buff_intf->data_out->binding = bind_str;
                        if (bank->width > buff->width)
                           buff_intf->data_out->binding += "[" + boost::lexical_cast<std::string>(buff->width-1) + ":0]";
                     }
                     else
                     {
                        bind_str = bank->id + "_" + bank->interfaces[idx.first]->data_out->id;
                        buff_intf->data_out->binding = bind_str;
                        if (bank->width > buff->width)
                           buff_intf->data_out->binding += "[" + boost::lexical_cast<std::string>(buff->width-1) + ":0]";
                     }
                  }
               }
            }
         }
         DEBUG(DBG_VERBOSE, verbosity, std::string(50, '-') << std::endl);
      }
   }
}


MemoryWrapperPtr Allocation::create_architecture(const PartitionPtr partition)
{
   MemoryWrapperPtr arch(new MemoryWrapper);

   ///create memory interfaces for the processes
   create_memory_interfaces(partition, arch);

   MemoryPtr target_mem = partition->memory;
   arch->files.insert(target_mem->v_file);
   ///create the banks
   for(unsigned int num = 0; num < partition->num_banks; num++)
   {
      MemoryPtr bank = target_mem->clone();
      bank->id = "mem_" + boost::lexical_cast<std::string>(num);
      arch->banks.push_back(bank);
   }
   DEBUG(DBG_VERBOSE, verbosity, "  Wrapper configuration: " << partition->num_banks << " x " << partition->memory->type << " (" << partition->memory->height << "x" << partition->memory->width << ")" << std::endl);

   for (const auto &bit : partition->buffer_offset)
   {
      const ArrayPtr buff = bit.first;
      std::string prefix;
      if (buffers->db.size() > 1 or buffers->db[buff->accelerator].size() > 1)
         prefix = buff->accelerator + "_" + buff->name + "_";
      createArrayArchitecture(partition, arch, buff, prefix);
   }

   return arch;
}
