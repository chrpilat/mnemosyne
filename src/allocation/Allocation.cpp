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

#include "rtl_architecture.hpp"

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
            if (arch_id.size()) arch_id += "_";
            arch_id += buff->accelerator + "_" + buff->name;
            if (buffers.size()) buffers += ",";
            buffers += buff->accelerator + "_" + buff->name;
         }
         DEBUG(DBG_VERBOSE, verbosity, "Wrapper: \"" << arch_id << "\"" << std::endl);
         DEBUG(DBG_VERBOSE, verbosity, std::setw(25) << std::left << "  Arrays: " << std::setw(5) << std::left << id_to_clique[var].size() << " -- {" << buffers << "}" << std::endl);

         std::string arch_name = arch_id;
         if (!compose_id)
            arch_name = accelerator_name;
         MemoryWrapperPtr arch = create_architecture(p, arch_name);
         opt_functor->wrappers[var] = arch;
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
            if ((std::get<0>(components->node_list[i]) == std::get<0>(j) and components->comp_list[j].find(components->node_list[i]) == components->comp_list[j].end()) or
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

void Allocation::set_ce_binding(const InterfacePtr mem_intf, unsigned int mem_intf_idx, RtlNodePtr activation)
{
   //default value
   if (mem_intf->enable->binding == NULL) mem_intf->enable->binding = BinaryValue::create("1'b0");//"1'b0";
   //mem_intf->enable->binding = activation + " ? 1'b1 : (" + mem_intf->enable->binding + ")";
   mem_intf->enable->binding = CondOperation::create(activation, BinaryValue::create("1'b1"),  mem_intf->enable->binding);
   DEBUG(DBG_VERBOSE, verbosity, "     CE[" << mem_intf_idx << "] = { " << activation << " }" << std::endl);
}

void Allocation::set_address_binding(const PartitionPtr partition, const ArrayPtr buff, const InterfacePtr proc_intf, const MemoryPtr bank, unsigned int mem_intf_idx, RtlNodePtr activation, const std::string& prefix)
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
   RtlNodePtr padding;
   if (a_msb >= tag->buff_size and partition->buffer_offset[buff] == 0)
   {
      unsigned int padd_num = (a_msb + 1 - tag->buff_size);
      padding = ModuleRepeat::create(BinaryValue::create("1'b0"), IntegerValue::create(padd_num));
      a_msb = tag->buff_size-1;
   }
   if (tag->bank_size > tag->buff_size)
   {
      unsigned int padd_num = (tag->bank_size - tag->buff_size);
      padding = ModuleRepeat::create(BinaryValue::create("1'b0"), IntegerValue::create(padd_num));
   }

   unsigned int buff_size = ceil((double)log(buff->height) / (double)log(2));
   //std::string abit_cast;
   //if (buff_size > 1)
   //   abit_cast = "[" + boost::lexical_cast<std::string>(a_msb) + ":" + boost::lexical_cast<std::string>(a_lsb) + "]";
   //if (partition->buffer_offset[buff]) abit_cast += " + " + boost::lexical_cast<std::string>(partition->buffer_offset[buff]);

   unsigned int bank_size = ceil((double)log(bank->height) / (double)log(2));
   RtlNodePtr str_buffer;
   if (buff_size > 0)
   {
      //if (padding_str.size()) str_buffer = "{" + padding_str + ",";
      //str_buffer += prefix + "A" + boost::lexical_cast<std::string>(proc_intf->idx) + abit_cast;
      //if (padding_str.size()) str_buffer += "}";
      str_buffer = RtlIdentifier::create(prefix + "A" + boost::lexical_cast<std::string>(proc_intf->idx));
      if (buff_size > 1){
         str_buffer = Partselect::create(str_buffer, IntegerValue::create(a_msb), IntegerValue::create(a_lsb));
      }

      if (partition->buffer_offset[buff]) {
         str_buffer = BinaryOperation::create(Operation::PLUS, str_buffer, IntegerValue::create(partition->buffer_offset[buff]));
      }
      if (padding != NULL) {
         std::list<RtlNodePtr> items;
         items.push_back(padding);
         items.push_back(str_buffer);
         str_buffer = Concat::create(items, false);
      }
   }
   else
   {
      str_buffer = ModuleRepeat::create(BinaryValue::create("1'b0"), IntegerValue::create(bank_size));
      //str_buffer = "{" + boost::lexical_cast<std::string>(bank_size) + "{1'b0}}";
   }

   const InterfacePtr mem_intf = bank->interfaces[mem_intf_idx];
   //address
   if (mem_intf->address->binding == NULL) mem_intf->address->binding = ModuleRepeat::create(BinaryValue::create("1'b0"), IntegerValue::create(bank_size));
   //mem_intf->address->binding = activation + " ? " + str_buffer + " : (" + mem_intf->address->binding + ")";
   mem_intf->address->binding = CondOperation::create(activation, str_buffer, mem_intf->address->binding);
   DEBUG(DBG_VERBOSE, verbosity, "     A[" << mem_intf_idx << "] = { " << str_buffer << " }" << std::endl);
}

RtlNodePtr Allocation::get_activation(const PartitionPtr partition, const MemoryPtr bank, unsigned int mem_intf_idx, const binding_t& bind, const std::string& prefix)
{
   const ArrayPtr buff = std::get<0>(bind);
   InterfacePtr proc_intf = std::get<1>(bind);
   unsigned int p = std::get<4>(bind);
   unsigned int s = std::get<5>(bind);

   DEBUG(DBG_VERBOSE, verbosity, "  +Bank = {" << bank->id << "}[" << mem_intf_idx << "] <--> Proc Interface = {" << proc_intf->idx << " -- " << Interface::type_to_string(proc_intf->type) <<  "} (P = {" << p << "} :: S = {" << s << "})" << std::endl);

   TagPtr tag = partition->buffer_tag[buff];

   ///activation function
   RtlNodePtr activation = RtlIdentifier::create(prefix + "CE" + boost::lexical_cast<std::string>(proc_intf->idx));
   if (tag->stag_size > 0)
   {
      unsigned int s_lsb = (buff->data_partitioning ? 0 : tag->ptag_size) + tag->mwtag_size + tag->mtag_size + tag->bank_size;
      unsigned int s_msb = tag->buff_size - 1;
      assert(s_msb >= s_lsb);
      //std::string sbit_cast = "[" + boost::lexical_cast<std::string>(s_msb) + ":" + boost::lexical_cast<std::string>(s_lsb) + "]";
      //activation += " & " + prefix + "A" + boost::lexical_cast<std::string>(proc_intf->idx) + sbit_cast + " == " + boost::lexical_cast<std::string>(s);
      activation = BinaryOperation::create(Operation::AND, 
                      activation, 
		      BinaryOperation::create(Operation::EQ,
			 Partselect::create(RtlIdentifier::create(prefix + "A" + boost::lexical_cast<std::string>(proc_intf->idx)),
		            IntegerValue::create(s_msb),
					    IntegerValue::create(s_lsb)),
					      IntegerValue::create(s)));
   }
   if (tag->ptag_size > 0)
   {
      unsigned int s_lsb = tag->mtag_size;
      unsigned int s_msb = tag->mtag_size + tag->ptag_size - 1;
      assert(s_msb >= s_lsb);
      //std::string sbit_cast = "[" + boost::lexical_cast<std::string>(s_msb) + ":" + boost::lexical_cast<std::string>(s_lsb) + "]";
      //activation += " & " + prefix + "A" + boost::lexical_cast<std::string>(proc_intf->idx) + sbit_cast + " == " + boost::lexical_cast<std::string>(p);
      activation = BinaryOperation::create(Operation::AND, 
                      activation, 
		      BinaryOperation::create(Operation::EQ,
			 Partselect::create(RtlIdentifier::create(prefix + "A" + boost::lexical_cast<std::string>(proc_intf->idx)),
		            IntegerValue::create(s_msb),
			    IntegerValue::create(s_lsb)),
					      IntegerValue::create(p)));
   }
   if (tag->mwtag_size > 0)
   {
      unsigned int s_lsb = 0;
      unsigned int s_msb = tag->mwtag_size - 1;
      assert(s_msb >= s_lsb);
      //std::string sbit_cast = "[" + boost::lexical_cast<std::string>(s_msb) + ":" + boost::lexical_cast<std::string>(s_lsb) + "]";
      //activation += " & " + prefix + "A" + boost::lexical_cast<std::string>(proc_intf->idx) + sbit_cast + " == " + boost::lexical_cast<std::string>(p % (buff->w_block_size / buff->split));
      activation = BinaryOperation::create(Operation::AND, 
                      activation, 
		      BinaryOperation::create(Operation::EQ,
			 Partselect::create(RtlIdentifier::create(prefix + "A" + boost::lexical_cast<std::string>(proc_intf->idx)),
		            IntegerValue::create(s_msb),
			    IntegerValue::create(s_lsb)),
					      IntegerValue::create(p % (buff->w_block_size / buff->split))));
   }
   DEBUG(DBG_VERBOSE, verbosity, "     activation = { " << activation << " }" << std::endl);
   return activation;
}

void Allocation::set_default_binding(const MemoryPtr bank, unsigned int mem_intf_idx)
{
   const InterfacePtr mem_intf = bank->interfaces[mem_intf_idx];
   unsigned int bank_size = ceil((double)log(bank->height) / (double)log(2));
   //enable
   if (mem_intf->enable->binding == NULL) mem_intf->enable->binding = BinaryValue::create("1'b0");
   //address
   if (mem_intf->address->binding == NULL) mem_intf->address->binding = ModuleRepeat::create(BinaryValue::create("1'b0"), IntegerValue::create(bank_size));
   //data_in
   if (mem_intf->data_in->binding == NULL)
   {
      mem_intf->data_in->binding = BinaryValue::create("1'b0");
      if (bank->width > 1) mem_intf->data_in->binding = ModuleRepeat::create( mem_intf->data_in->binding, IntegerValue::create(bank->width));
   }
   //write enable
   if (mem_intf->write_enable->binding == NULL) mem_intf->write_enable->binding = BinaryValue::create("1'b0");
   //write mask
   if (mem_intf->write_mask->binding == NULL)
   {
      mem_intf->write_mask->binding = BinaryValue::create("1'b0");
      if (bank->width > 1) mem_intf->write_mask->binding = ModuleRepeat::create( mem_intf->write_mask->binding, IntegerValue::create(bank->width));
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
         std::string comment = "buffer: " + b->name + " - process: " + intf->process_name  + " - interface: ";
         if (intf->type == Interface::R)
            comment += "R";
         else if (intf->type == Interface::W)
            comment += "W";
         arch->module->add_portitem(RtlComment::create(comment));

         ///size of address provided by HLS tool is detemined by the size of the buffer
         unsigned int size = ceil(log((double)b->height) / log((double)2));
         std::string suffix = boost::lexical_cast<std::string>(intf->idx);
         arch->module->add_port(ModulePort::create(prefix+"CE"+suffix, ModulePort::IN, ModulePort::CONTROL));
         arch->module->add_port(ModulePort::create(prefix+"A"+suffix, size, ModulePort::IN, false, ModulePort::CONTROL));
         
         intf->enable = PortPtr(new Port(prefix+"CE"+suffix, Port::IN, 1));
         intf->ports.push_back(intf->enable);
         intf->address = PortPtr(new Port(prefix+"A"+suffix, Port::IN, size));
         intf->ports.push_back(intf->address);
         if (intf->type == Interface::W or intf->type == Interface::RW)
         {
            intf->data_in = PortPtr(new Port(prefix+"D"+suffix, Port::IN, b->width));
            arch->module->add_port(ModulePort::create(prefix+"D"+suffix, b->width, ModulePort::IN, false, ModulePort::DATA));
            intf->ports.push_back(intf->data_in);
            intf->write_enable = PortPtr(new Port(prefix+"WE"+suffix, Port::IN, 1));
            arch->module->add_port(ModulePort::create(prefix+"WE"+suffix, ModulePort::IN, ModulePort::CONTROL));
            intf->ports.push_back(intf->write_enable);
            intf->write_mask = PortPtr(new Port(prefix+"WEM"+suffix, Port::IN, b->width));
            arch->module->add_port(ModulePort::create(prefix+"WEM"+suffix, b->width, ModulePort::IN, false, ModulePort::CONTROL));
            intf->ports.push_back(intf->write_mask);
         }
         if (intf->type == Interface::R or intf->type == Interface::RW)
         {
            intf->data_out = PortPtr(new Port(prefix+"Q"+suffix, Port::OUT, b->width));
            arch->module->add_port(ModulePort::create(prefix+"Q"+suffix, b->width, ModulePort::OUT, false, ModulePort::DATA));
            intf->ports.push_back(intf->data_out);
         }
      }
   }
}

std::string dec2hex(unsigned int value, unsigned int digits)
{
   std::stringstream ss;
   ss << std::hex << value;
   std::string res(ss.str());
   if (res.size() < digits)
   {
      std::string padding(digits - res.size(), '0');
      res = padding + res;      
   }
   for (unsigned int i = 0; i < res.size(); i++)
   {
      res[i] = toupper(res[i]);
   }
   return res;
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
         RtlNodePtr bind_str;
         if (arch->banks.size() == 1)
            bind_str = RtlIdentifier::create(arch->banks[0]->id + "_Q" + boost::lexical_cast<std::string>(mem_idx));
         else
         {
            bind_str = RtlIdentifier::create("Q" + boost::lexical_cast<std::string>(mem_idx) + "_out");
            bind_str = Pointer::create(bind_str, RtlIdentifier::create("sel_Q1_" + prefix + boost::lexical_cast<std::string>(buff_intf->idx)));
         }
         for(unsigned int i = 0; i < buff->merge; i++)
         {
            if (buff_intf->data_out->binding == NULL)
               buff_intf->data_out->binding = ModuleRepeat::create(BinaryValue::create("1'b0"), IntegerValue::create(buff->width));
            unsigned int lsb = buff->width * i;
            unsigned int msb = (buff->width * (i+1))-1;
            assert(msb >= lsb);
            std::string bit_cast = "[" + boost::lexical_cast<std::string>(msb) + ":" + boost::lexical_cast<std::string>(lsb) + "]";
            DEBUG(DBG_VERBOSE, verbosity, "  +Merge (" << i << ") = " << bind_str << bit_cast << std::endl);
            //buff_intf->data_out->binding = arch->merge_selector[buff][buff_intf]->id + " == " + boost::lexical_cast<std::string>(i) + " ? " + bind_str + bit_cast + " : (" + buff_intf->data_out->binding + ")";
            buff_intf->data_out->binding = CondOperation::create( 
                  //RtlIdentifier::create(arch->merge_selector[buff][buff_intf]->id + " == " + boost::lexical_cast<std::string>(i)), 
                  BinaryOperation::create(Operation::EQ, RtlIdentifier::create(arch->merge_selector[buff][buff_intf]->id), IntegerValue::create(i)),
                  //RtlIdentifier::create( bind_str + bit_cast), 
                  Partselect::create(bind_str, IntegerValue::create(msb), IntegerValue::create(lsb)),
                  buff_intf->data_out->binding);
         }

         if (arch->merge_selector[buff][buff_intf]->binding == NULL)
         {
            arch->merge_selector[buff][buff_intf]->binding = ModuleRepeat::create(BinaryValue::create("1'b0"), IntegerValue::create(arch->merge_selector[buff][buff_intf]->size));
         }

         TagPtr tag = partition->buffer_tag[buff];
         unsigned int lsb = 0;
         unsigned int msb = tag->mtag_size-1;
         assert(msb >= lsb);
         //std::string bit_cast = "[" + boost::lexical_cast<std::string>(msb) + ":" + boost::lexical_cast<std::string>(lsb) + "]";
         RtlNodePtr activation = RtlIdentifier::create(prefix + "CE" + boost::lexical_cast<std::string>(buff_intf->idx));
         arch->merge_selector[buff][buff_intf]->binding = CondOperation::create(activation, Partselect::create(RtlIdentifier::create(prefix + "A" + boost::lexical_cast<std::string>(buff_intf->idx)), IntegerValue::create(msb), IntegerValue::create(lsb)), arch->merge_selector[buff][buff_intf]->binding);
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
         //std::string align_str;
         std::list<RtlNodePtr> align_list;
         for (const auto& it : buff_intf.second)
         {
            unsigned int s_idx = std::get<1>(std::get<0>(it));
            //if (align_str.size())
            //   align_str = "," + align_str;
            std::string slice;
            RtlNodePtr align_item;
            align_item = Pointer::create(RtlIdentifier::create("Q" + boost::lexical_cast<std::string>(1) + "_out"), RtlIdentifier::create(arch->split_selector[buff][buff_intf.first][s_idx]->id));
            if (s_idx+1 == buff->split)
            {
               if ((s_idx+1) * arch->banks[0]->width > buff->width)
               {
                  align_item = Partselect::create(align_item, IntegerValue::create(buff->width - (s_idx) * arch->banks[0]->width - 1), IntegerValue::create(0));
                  //slice = "[" + boost::lexical_cast<std::string>(buff->width - (s_idx) * arch->banks[0]->width - 1) + ":0]";
               }
            }
            align_list.push_front(align_item);
         }
         buff_intf.first->data_out->binding = Concat::create(align_list, false);
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

         if (buff->init_values.size())
         {
            unsigned int start_value = s * bank->height;
            unsigned int end_value = (s+1) * bank->height;
            DEBUG(DBG_VERBOSE, verbosity, "Init" << std::endl);
            for (unsigned int it = start_value; it < end_value; it++)
            {
               std::string value = buff->init_values[it];
               //DEBUG(DBG_VERBOSE, verbosity, "  " << value << std::endl);
               if (value.size() < bank->width)
               {
                  std::string padding(bank->width - value.size(), '0');
                  value = value + padding;      
               }
               bank->init_values.push_back(value);
            }
            unsigned int values = 256/bank->width;
            for (unsigned int a = 0; a < ceil(bank->init_values.size()/values); a++)
            {
               std::string hexaddress = dec2hex(a, 2);
               std::string tot_value;
               std::string hex_value;
               for (unsigned int val = a*values; val < (a+1)*values; val++)
               {
                  std::string value;
                  if (val >= bank->init_values.size())
                     value = std::string(bank->width, '0');
                  else
                     value = bank->init_values[val];
                  value = std::string(value.rbegin(), value.rend());
                  tot_value = value + tot_value;
               }
               for (size_t j = 0; j < tot_value.size(); j += 4)
               {
                  std::string tmp = tot_value.substr(j, 4);
                  if      (!tmp.compare("0000")) hex_value += "0";
                  else if (!tmp.compare("0001")) hex_value += "1";
                  else if (!tmp.compare("0010")) hex_value += "2";
                  else if (!tmp.compare("0011")) hex_value += "3";
                  else if (!tmp.compare("0100")) hex_value += "4";
                  else if (!tmp.compare("0101")) hex_value += "5";
                  else if (!tmp.compare("0110")) hex_value += "6";
                  else if (!tmp.compare("0111")) hex_value += "7";
                  else if (!tmp.compare("1000")) hex_value += "8";
                  else if (!tmp.compare("1001")) hex_value += "9";
                  else if (!tmp.compare("1010")) hex_value += "A";
                  else if (!tmp.compare("1011")) hex_value += "B";
                  else if (!tmp.compare("1100")) hex_value += "C";
                  else if (!tmp.compare("1101")) hex_value += "D";
                  else if (!tmp.compare("1110")) hex_value += "E";
                  else if (!tmp.compare("1111")) hex_value += "F";
                  else continue;
               }
               bank->param_map["INIT_"+hexaddress] = BinaryValue::create(BinaryValue::HEX, 256, hex_value);
            }
         }

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
                  //std::string data_in, data_we, data_wmask;
                  std::list<RtlNodePtr> data_in_items, data_wmask_items;
                  RtlNodePtr data_in, data_we, data_wmask;
                  RtlNodePtr activation;
                  for (const auto& it : m.second)
                  {
                     const InterfacePtr buff_intf = std::get<0>(it);
                     std::string intf_idx = boost::lexical_cast<std::string>(buff_intf->idx);
                     binding_t bind(buff, buff_intf, bank_id, idx.first, p, s);
                     RtlNodePtr data_in_item, data_we_item, data_wmask_item;
                     RtlNodePtr l_activation = get_activation(partition, bank, idx.first, bind, prefix);
                     ///chip enable
                     set_ce_binding(bank->interfaces[idx.first], idx.first, l_activation);
                     ///address
                     set_address_binding(partition, buff, buff_intf, bank, idx.first, l_activation, prefix);
                     //if (data_in.size())
                     //   data_in = "," + data_in;
                     //if (data_we.size())
                     //   data_we = " | " + data_we;
                     //if (data_wmask.size())
                     //   data_wmask = "," + data_wmask;
                     data_in_item = RtlIdentifier::create(prefix + "D" + intf_idx);
                     data_we_item = RtlIdentifier::create(prefix + "WE" + intf_idx);
                     data_wmask_item = RtlIdentifier::create(prefix + "WEM" + intf_idx);

                     data_in_items.push_front(data_in_item);
                     if (data_we == NULL) {
                        data_we = data_we_item;
                     } else {
                        data_we = BinaryOperation::create(Operation::OR, data_we_item, data_we);
                     }
                     data_wmask_items.push_front(data_wmask_item);
                     //if (activation != NULL)
                     //   activation += " || ";
                     //activation += l_activation;
                     if (activation == NULL) {
                        activation = l_activation;
                     } else { 
                        activation = BinaryOperation::create(Operation::LOR, activation, l_activation);
                     }
                  }
                  const InterfacePtr mem_intf = bank->interfaces[idx.first];
                  data_in = Concat::create(data_in_items, false);
                  data_wmask = Concat::create(data_wmask_items, false);
                  DEBUG(DBG_VERBOSE, verbosity, "     D[" << idx.first << "] = { " << data_in << " }" << std::endl);
                  DEBUG(DBG_VERBOSE, verbosity, "     WE[" << idx.first << "] = { " << data_we << " }" << std::endl);
                  DEBUG(DBG_VERBOSE, verbosity, "     WEM[" << idx.first << "] = { " << data_wmask << " }" << std::endl);
                  mem_intf->data_in->binding = CondOperation::create(activation, data_in, mem_intf->data_in->binding);
                  mem_intf->write_enable->binding = CondOperation::create(activation, data_we, mem_intf->write_enable->binding);
                  mem_intf->write_mask->binding = CondOperation::create(activation, data_wmask, mem_intf->write_mask->binding);
               }
               ///END of merge WRITE

               ///START of merge READ
               for (const auto& m : read_merge_interfaces)
               {
                  for (const auto& it : m.second)
                  {
                     const InterfacePtr buff_intf = std::get<0>(it);
                     binding_t bind(buff, buff_intf, bank_id, idx.first, p, s);
                     RtlNodePtr activation = get_activation(partition, bank, idx.first, bind, prefix);

                     set_ce_binding(bank->interfaces[idx.first], idx.first, activation);
                     set_address_binding(partition, buff, buff_intf, bank, idx.first, activation, prefix);

                     if (partition->num_banks > 1)
                     {
                        if (bank->selector[idx.first][buff_intf]->binding == NULL)
                           bank->selector[idx.first][buff_intf]->binding = ModuleRepeat::create(BinaryValue::create("1'b0"), IntegerValue::create(bank->selector[idx.first][buff_intf]->size));
                        bank->selector[idx.first][buff_intf]->binding = CondOperation::create(activation, RtlIdentifier::create(boost::lexical_cast<std::string>(bank_id)),  bank->selector[idx.first][buff_intf]->binding);
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

                     RtlNodePtr activation = get_activation(partition, bank, mem_intf_idx, bind, prefix);
                     set_ce_binding(bank->interfaces[mem_intf_idx], mem_intf_idx, activation);
                     set_address_binding(partition, buff, buff_intf, bank, mem_intf_idx, activation, prefix);

                     RtlNodePtr d_binding = RtlIdentifier::create(prefix + "D" + boost::lexical_cast<std::string>(buff_intf->idx));
                     RtlNodePtr wem_binding = RtlIdentifier::create(prefix + "WEM" + boost::lexical_cast<std::string>(buff_intf->idx));
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
                        d_binding = Partselect::create(d_binding, IntegerValue::create(din_msb), IntegerValue::create(din_lsb));
                        wem_binding = Partselect::create(wem_binding, IntegerValue::create(din_msb), IntegerValue::create(din_lsb));
                        //dinbit_cast = "[" + boost::lexical_cast<std::string>(din_msb) + ":" + boost::lexical_cast<std::string>(din_lsb) + "]";
                     }
                     if (din_msb+1 < bank->width*(s_idx+1)) //data padding
                     {
                        std::list<RtlNodePtr> d_items, wem_items;
                        RtlNodePtr padding = ModuleRepeat::create(BinaryValue::create("1'b0"), IntegerValue::create(bank->width*(s_idx+1)-(din_msb+1)));
                        d_items.push_back(padding);
                        d_items.push_back(d_binding);
                        d_binding = Concat::create(d_items, false);
                        wem_items.push_back(padding);
                        wem_items.push_back(wem_binding);
                        wem_binding = Concat::create(wem_items, false);
                        //binding = "{{" + boost::lexical_cast<std::string>(bank->width*(s_idx+1)-(din_msb+1)) + "{1'b0}}," + binding + "}";
                     }
                     mem_intf->data_in->binding = CondOperation::create(activation, d_binding, mem_intf->data_in->binding);
                     mem_intf->write_enable->binding = CondOperation::create(activation, RtlIdentifier::create(prefix + "WE" + boost::lexical_cast<std::string>(buff_intf->idx)),  mem_intf->write_enable->binding);
                     mem_intf->write_mask->binding = CondOperation::create(activation, wem_binding, mem_intf->write_mask->binding);
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
                     RtlNodePtr activation = get_activation(partition, bank, idx.first, bind, prefix);
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
                  RtlNodePtr local_act = RtlIdentifier::create(prefix + "CE" + boost::lexical_cast<std::string>(buff_intf->idx));
                  if (arch->split_selector[buff][buff_intf][i]->binding == NULL)
                  {
                     if (arch->split_selector[buff][buff_intf][i]->size == 1)
                        arch->split_selector[buff][buff_intf][i]->binding = BinaryValue::create("1'b0");
                     else
                        arch->split_selector[buff][buff_intf][i]->binding = ModuleRepeat::create(BinaryValue::create("1'b0"), IntegerValue::create(arch->split_selector[buff][buff_intf][i]->size));
                  }
                  if (partition->buffer_configuration[buff][p].size() > 1)
                  {
                     TagPtr tag = partition->buffer_tag[buff];

                     unsigned int s_lsb = tag->ptag_size + tag->bank_size;
                     unsigned int s_msb = tag->buff_size - 1;
                     assert(s_msb >= s_lsb);
                     //std::string sbit_cast = "[" + boost::lexical_cast<std::string>(s_msb) + ":" + boost::lexical_cast<std::string>(s_lsb) + "]";
                     //local_act += " && " + prefix + "A" + boost::lexical_cast<std::string>(buff_intf->idx) + sbit_cast + " == " + boost::lexical_cast<std::string>(s);
                     local_act = BinaryOperation::create(Operation::LAND, 
               			 local_act, 
               			 BinaryOperation::create(Operation::EQ, 
							 Partselect::create(RtlIdentifier::create(prefix + "A" + boost::lexical_cast<std::string>(buff_intf->idx)),
									    IntegerValue::create(s_msb),
									    IntegerValue::create(s_lsb)),
							 RtlIdentifier::create(boost::lexical_cast<std::string>(s))));
                  }
                  arch->split_selector[buff][buff_intf][i]->binding = CondOperation::create(local_act, RtlIdentifier::create(boost::lexical_cast<std::string>(bank_id)), arch->split_selector[buff][buff_intf][i]->binding);

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

                     if (partition->num_banks > 1)
                     {
                        RtlNodePtr activation = get_activation(partition, bank, idx.first, bind, prefix);
                        if (bank->selector[idx.first][buff_intf]->binding == NULL)
                           bank->selector[idx.first][buff_intf]->binding = ModuleRepeat::create(BinaryValue::create("1'b0"), IntegerValue::create(bank->selector[idx.first][buff_intf]->size));
                        bank->selector[idx.first][buff_intf]->binding = CondOperation::create(activation, IntegerValue::create(bank_id),  bank->selector[idx.first][buff_intf]->binding);

                        buff_intf->data_out->binding = Pointer::create(RtlIdentifier::create("Q" + boost::lexical_cast<std::string>(idx.first) + "_out"),RtlIdentifier::create("sel_Q1_" + prefix + boost::lexical_cast<std::string>(buff_intf->idx)));
                        if (bank->width > buff->width)
                           //buff_intf->data_out->binding += "[" + boost::lexical_cast<std::string>(buff->width-1) + ":0]";
                           buff_intf->data_out->binding = Partselect::create(buff_intf->data_out->binding, IntegerValue::create(buff->width-1), IntegerValue::create(0));
                     }
                     else
                     {
                        buff_intf->data_out->binding = RtlIdentifier::create(bank->id + "_" + bank->interfaces[idx.first]->data_out->id);
                        if (bank->width > buff->width)
                           buff_intf->data_out->binding = Partselect::create(buff_intf->data_out->binding, IntegerValue::create(buff->width-1), IntegerValue::create(0));
                     }
                  }
               }
            }
         }
         DEBUG(DBG_VERBOSE, verbosity, std::string(50, '-') << std::endl);
      }
   }
}


MemoryWrapperPtr Allocation::create_architecture(const PartitionPtr partition, const std::string name)
{
   MemoryWrapperPtr arch(new MemoryWrapper);
   arch->id = name;
   arch->module = ModuleDef::create(name);
   arch->module->add_port(ModulePort::create("CLK", ModulePort::IN, ModulePort::CLK));

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

   if (arch->banks.size() > 1)
   {
      MemoryPtr bank = arch->banks[0];
      std::list<RtlNodePtr> declarations;
      declarations.push_back(RtlComment::create("signals from banks"));
      declarations.push_back(ModuleWireArray::create("Q0_out", bank->width, arch->banks.size(), false));
      declarations.push_back(ModuleWireArray::create("Q1_out", bank->width, arch->banks.size(), false));

      declarations.push_back(RtlComment::create("signals to banks"));
      for(const auto &b : arch->banks)
      {
         for(const auto &i : b->interfaces)
         {
            for(const auto &p : i->ports)
            {
               declarations.push_back(ModuleWire::create(b->id + "_" + p->id, p->size, false));
            }
         }
      }
      arch->module->add_declaration(ModuleDecl::create(declarations));
   }
   
   if (arch->wires.size())
   {
      std::list<RtlNodePtr> declarations;
      declarations.push_back(RtlComment::create("additional wires"));
      bool has_buffered = false;
      for(const auto &w : arch->wires)
      {
         if (w->binding == NULL) continue;
         unsigned int w_size = boost::lexical_cast<unsigned int>(w->size);
         if (w->is_buffered)
         {
            declarations.push_back(ModuleReg::create(w->id, RtlNodePtr(), w_size, false));
         }
         else
         {
            declarations.push_back(ModuleWire::create(w->id, w_size, false));
         }
      }
      arch->module->add_declaration(ModuleDecl::create(declarations));

      std::list<RtlNodePtr> statements;
      for(const auto &w : arch->wires)
      {
         if (w->binding == NULL) continue;
         if (w->is_buffered)
            statements.push_back(ModuleSubstitution::create(RtlIdentifier::create(w->id), w->binding, false));
      }
      if (statements.size())
      {
         std::list<RtlNodePtr> senslist;
         senslist.push_back(ModuleSens::create(RtlIdentifier::create("CLK"), ModuleSens::POSEDGE));
         arch->module->add_item(ModuleAlways::create(senslist, ModuleBlock::create(statements)));
      }
   }
   
   if (arch->banks.size() > 1)
   {
      for(unsigned int num = 0; num < arch->banks.size(); num++)
      {
         arch->module->add_item(RtlComment::create("signals for bank: " + arch->banks[num]->id));
         for(const auto &int_it : arch->banks[num]->interfaces)
         {
            for(const auto &p : int_it->ports)
            {
               if (p->dir == Port::IN and (p->binding != NULL))
               {
                  arch->module->add_item(ModuleAssign::create(RtlIdentifier::create(arch->banks[num]->id + "_" + p->id), p->binding));
               }
            }
         }
         arch->module->add_item(RtlComment::create("output selection for bank: " + arch->banks[num]->id));
         arch->module->add_item(ModuleAssign::create(Pointer::create(RtlIdentifier::create("Q0_out"), IntegerValue::create(num)), RtlIdentifier::create(arch->banks[num]->id + "_Q0")));
         arch->module->add_item(ModuleAssign::create(Pointer::create(RtlIdentifier::create("Q1_out"), IntegerValue::create(num)), RtlIdentifier::create(arch->banks[num]->id + "_Q1")));
      }
   }
   
   arch->module->add_item(RtlComment::create("assigns to output ports"));
   for(const auto &buff : arch->buffers)
   {
      for(const auto &proc : buff->processes)
      {
         for(const auto &int_it : proc->interfaces)
         {
            for(const auto &p : int_it->ports)
            {
               if (p->dir == Port::OUT and (p->binding != NULL))
	       {
                  arch->module->add_item(ModuleAssign::create(RtlIdentifier::create(p->id), p->binding));
	       }
            }
         }
      }
   }
   

   arch->module->add_item(RtlComment::create("definitions of the banks"));
#if 0
   for(const auto &b : arch->banks)
   {
      for(const auto &p : b->ports)
      {
         *os << "  wire ";
         unsigned int p_size = boost::lexical_cast<unsigned int>(p->size);
         if (p_size > 1)
            *os << "[" << p_size-1 << ":0] ";
         *os << b->id + "_" + p->id + "_float;" << std::endl;
         p->binding = b->id + "_" + p->id + "_float";
      }
   }
#endif
   for(unsigned int num = 0; num < arch->banks.size(); num++)
   {
      MemoryPtr b = arch->banks[num];
      ModuleInstancePtr bank = ModuleInstance::create(b->type, b->id, b->param_map, ModuleInstance::binding_t());
      bank->set_port_binding("CLK", RtlIdentifier::create("CLK"));
      for(const auto &i : b->interfaces)
      {
         for(const auto &p : i->ports)
         {
            bank->set_port_binding(p->id, RtlIdentifier::create(b->id + "_" + p->id));
         }
      }
      arch->module->add_item(bank);
   }

   return arch;
}
