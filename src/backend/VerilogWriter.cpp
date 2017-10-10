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
 * @file   VerilogWriter.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Methods used to write a Verilog description.
 *
 */
#include "VerilogWriter.hpp"

#include "Functor.hpp"

#include "Architecture.hpp"
#include "Memory.hpp"
#include "Array.hpp"

VerilogWriter::VerilogWriter(unsigned int verbosity, const std::string& output_dir, const FunctorPtr opt_functor) :
   Writer(verbosity, output_dir, opt_functor),
   has_to_generate_top_plm(false)
{

}

VerilogWriter::~VerilogWriter()
{

}

void VerilogWriter::set_has_to_generate_top_plm(bool new_value)
{
   has_to_generate_top_plm = new_value;
}

void VerilogWriter::write_header(const std::string& output_file)
{
   if (!boost::filesystem::exists(output_dir))
      boost::filesystem::create_directories(output_dir);
   os = new std::ofstream(output_dir + "/" + output_file);
   *os << "/**" << std::endl;
   *os << " * Copyright (c) 2014-2017, Columbia University" << std::endl;
   *os << " *" << std::endl;
   *os << " * @author Christian Pilato <pilato.christian@gmail.com>" << std::endl;
   *os << " */" << std::endl;
}

void VerilogWriter::close_file()
{
   if (os)
   {
      *os << std::endl;
      os->close();
      delete os;
   }
}

void VerilogWriter::generate_top_plm()
{
   *os << "module " + opt_functor->accelerator_name + "_PLM (clk";
   for(const auto& arch : opt_functor->wrappers)
   {
      if(arch.first != arch.second->id)
         std::runtime_error("mismatched");
      for(const auto &buff : arch.second->buffers)
      {
         for(const auto &proc : buff->processes)
         {
            for(const auto &int_it : proc->interfaces)
            {
               for(const auto &p : int_it->ports)
               {
                  *os << ", " << p->id;
               }
            }
         }
      }
   }
   *os << ");" << std::endl;
   *os << "  input clk;" << std::endl;
   for(const auto &arch : opt_functor->wrappers)
   {
      for(const auto &buff : arch.second->buffers)
      {
         for(const auto &proc : buff->processes)
         {
            for(const auto &int_it : proc->interfaces)
            {
               *os << "  //buffer: " << buff->name << " - process: " << proc->name  << " - interface: ";
               if (int_it->type == Interface::R)
                  *os << "R";
               else if (int_it->type == Interface::W)
                  *os << "W";
               *os << std::endl;
               for(const auto &p : int_it->ports)
               {
                  if (p->dir == Port::IN)
                     *os << "  input ";
                  else if (p->dir == Port::OUT)
                     *os << "  output ";
                  else
                     throw "VerilogWriter::generate_wrapper: Malformed port description";

                  unsigned int p_size = boost::lexical_cast<unsigned int>(p->size);
                  if (p_size > 1)
                     *os << "[" << p_size-1 << ":0] ";
                  *os << p->id << ";" << std::endl;
               }
            }
         }
      }
      *os << std::endl;
   }

   for(const auto &it : opt_functor->wrappers)
   {
      const MemoryWrapperPtr arch = it.second;
      *os << "  " << arch->id << " " << arch->id << "_0" << " (.CLK(clk)";
      for(const auto &buff : arch->buffers)
      {
         for(const auto &proc : buff->processes)
         {
            for(const auto &int_it : proc->interfaces)
            {
               for(const auto &p : int_it->ports)
               {
                  *os << "," << std::endl;
                  *os << "     ." << p->id << "(" << p->id << ")";
               }
            }
         }
      }
      *os << ");" << std::endl << std::endl;
   }

   *os << "endmodule" << std::endl;
}

bool VerilogWriter::generate_wrappers(const std::string& _output_file)
{
   output_file = _output_file;

   write_header(output_file);

   std::cout << "Generating the wrapper Verilog file..." << std::endl;
   std::cout << "Number of interfaces: " << opt_functor->wrappers.size() << std::endl;

   for(const auto& it : opt_functor->wrappers)
   {
      //std::cout << "-Wrapper module: " << ++num << std::endl;
      if (!generate_wrapper(it.second))
         return false;
   }

   *os << std::endl;

   if (has_to_generate_top_plm)
   {
      generate_top_plm();
   }
   close_file();
   PRINT("Wrapper module(s) generated in file \"" << output_dir + "/" + output_file << "\"" << std::endl);
   return true;
}

bool VerilogWriter::generate_top_entity(const std::string &acc_name, const std::string& name, const std::string& output_file, const std::string& acc_interface)
{
   std::map<std::string, std::map<std::string, std::set<std::string> > > buffer_to_interface;

   std::map<std::string, MemoryWrapperPtr> buffer_to_wrapper;
   for(auto& mem : opt_functor->wrappers)
   {
      const std::vector<ArrayPtr>& buffers = mem.second->buffers;
      for(auto& b : buffers)
      {
         if (acc_name != b->accelerator) continue;
         buffer_to_wrapper[b->name] = mem.second;
      }
   }

   bool first = true;
   try
   {
      std::cout << "Parsing the accelerator interface file \"" + acc_interface + "\"" << std::endl;
      YAML::Node config = YAML::LoadFile(acc_interface);

      std::string acc_logic_name = config["name"].as<std::string>();
      YAML::Node interface = config["interface"];

      if (opt_functor->components->list.find(acc_name) == opt_functor->components->list.end())
      {
         ComponentPtr accelerator(new Component(acc_name));
         opt_functor->components->list[acc_name] = accelerator;
      }
      ComponentPtr accelerator = opt_functor->components->list[acc_name];
      accelerator->parse_interface(interface, buffer_to_wrapper);

      write_header(output_file);
      *os << "module " << name << "(";
      for(const auto& p : interface)
      {
         std::string id = p["id"].as<std::string>();
         bool is_memory_port = true;
         for(auto& b : buffer_to_wrapper)
         {
            if (accelerator->darkmem_to_buffer.find(acc_name + "_" + id) != accelerator->darkmem_to_buffer.end())
               continue;
            std::string type;
            if (accelerator->read_interfaces.find(acc_name + "_" + id) != accelerator->read_interfaces.end())
               type += "r";
            if (accelerator->write_interfaces.find(acc_name + "_" + id) != accelerator->write_interfaces.end())
               type += "w";
            if (type.size() == 0)
            {
               is_memory_port = false;
               continue;
            }
            if (id.find(b.first) != 0) continue;
            if (id.find(b.first + "_CE") != std::string::npos)
            {
               std::string token = b.first + "_CE";
               std::string num = id.substr(id.find(token)+token.size(), id.size());
               buffer_to_interface[b.first][type].insert(b.first + "_CE" + num);
            }
         }
         if (!is_memory_port)
         {
            if (!first) *os << ", ";
            first = false;
            *os << id;
         }
      }
      *os << ");" << std::endl;

      for(const auto& p : interface)
      {
         std::string id = p["id"].as<std::string>();
         if (accelerator->read_interfaces.find(acc_name + "_" + id) == accelerator->read_interfaces.end() &&
             accelerator->write_interfaces.find(acc_name + "_" + id) == accelerator->write_interfaces.end() &&
             accelerator->darkmem_to_buffer.find(acc_name + "_" + id) == accelerator->darkmem_to_buffer.end())
         {
            std::string dir = p["dir"].as<std::string>();
            *os << "  " << dir << " ";
            unsigned int size = p["size"].as<unsigned int>();
            if (size>1)
            {
               *os << "[" << size-1 << ":0] ";
            }
            *os << id << ";" << std::endl;
         }
      }
      *os << std::endl;

      std::set<std::string> list_memory_ports;
      *os << "  //signals to memories" << std::endl;
      for(const auto& p : interface)
      {
         std::string id = p["id"].as<std::string>();
         if (accelerator->read_interfaces.find(acc_name + "_" + id) != accelerator->read_interfaces.end() ||
             accelerator->write_interfaces.find(acc_name + "_" + id) != accelerator->write_interfaces.end() ||
             accelerator->darkmem_to_buffer.find(acc_name + "_" + id) != accelerator->darkmem_to_buffer.end())
         {
            *os << "  wire ";
            unsigned int size = p["size"].as<unsigned int>();
            if (size>1)
            {
               *os << "[" << size-1 << ":0] ";
            }
            *os << acc_name << "_" << id << ";" << std::endl;
            list_memory_ports.insert(acc_name + "_" + id);
         }
      }
      *os << std::endl;

      *os << "  //other signals" << std::endl;
      std::map<MemoryWrapperPtr, unsigned int> memory_id;
      unsigned int mem_num = 0;
      for(auto& it : opt_functor->wrappers)
      {
         const MemoryWrapperPtr mem = it.second;
         unsigned int mem_id = mem_num++;
         memory_id[mem] = mem_id;
         for (auto & b_it : mem->buffers)
         {
            if (acc_name != b_it->accelerator) continue;
            for(auto& inf_it : b_it->interfaces)
            {
               for(auto& p : inf_it->ports)
               {
                  if (list_memory_ports.find(p->id) == list_memory_ports.end() and p->id.find("_WEM") != std::string::npos)
                  {
                     *os << "  wire [" << p->size-1 << ":0] " << p->id << ";" << std::endl;
                     *os << "  assign " << p->id << " = " << "{" + boost::lexical_cast<std::string>(p->size) + "{1'b1}};" << std::endl;
                     list_memory_ports.insert(p->id);
                  }
               }
            }
         }
      }
      *os << std::endl;

      *os << "  //accelerator" << std::endl;
      *os << "  " << acc_logic_name << " acc_0 (";
      bool is_first = true;
      for(const auto& p : interface)
      {
         std::string id = p["id"].as<std::string>();
         if (!is_first)
         {
            *os << "," << std::endl;
            *os << "          ";
         }
         is_first = false;
         if (list_memory_ports.find(acc_name + "_" + id) != list_memory_ports.end())
         {
            *os << "." << id << "(" << acc_name << "_" << id << ")";
         }
         else
         {
            *os << "." << id << "(" << id << ")";
         }
      }
      *os << ");" << std::endl << std::endl;

      if (accelerator->clock_name.size() == 0)
         throw "Missing name for the clock";
      for(auto& it : opt_functor->wrappers)
      {
         const MemoryWrapperPtr mem = it.second;
         unsigned int mem_id = memory_id[mem];
         std::string id = mem->id;
         *os << "  //wrapper for buffers (";
         bool is_f = true;
         for (auto & b_it : mem->buffers)
         {
            std::string buffer_id = b_it->name;
            if (!is_f) *os << ", ";
            is_f = false;
            *os << buffer_id;
         }
         *os << ")" << std::endl;
         if (accelerator->clock_name.size() == 0)
            throw "Missing name for the clock";
         *os << "  " << id << " mem_" << mem_id << "(.CLK(" << accelerator->clock_name << ")";
         for (auto & b_it : mem->buffers)
         {
            for(auto& inf_it : b_it->interfaces)
            {
               std::vector<PortPtr> ports = inf_it->ports;
               for(unsigned int p = 0; p < ports.size(); p++)
               {
                  *os << "," << std::endl;
                  *os << "          ." << ports[p]->id << "(";
                  if (b_it->accelerator == acc_name)
                     *os << ports[p]->id;
                  else
                  {
                     if (ports[p]->dir == Port::IN)
                     {
                        *os << "{" + boost::lexical_cast<std::string>(ports[p]->size) + "{1'b0}}";
                     }
                  }
                  *os << ")";
               }
            }
         }
         *os << ");" << std::endl << std::endl;
      }

      *os << "endmodule" << std::endl;
      close_file();
      PRINT("Top module generated in file \"" << output_dir + "/" + output_file << "\"" << std::endl);
   }
   catch(...)
   {
      std::cout << "ERROR!" << std::endl;
      return false;
   }
   return true;

#if 0
   xmlpp::Document* out_doc = new xmlpp::Document();
   xmlpp::Node* out_root_node = out_doc->create_root_node("soc_design");
   xmlpp::Node* mw_node = out_root_node->add_child("memory_wrappers");
   for(auto& it : opt_functor->wrappers)
   {
      const MemoryWrapperPtr mem = it.second;
      unsigned int mem_id = memory_id[mem];
      xmlpp::Element* el_node = mw_node->add_child("memory_wrapper");
      el_node->set_attribute("id", "mem_" + boost::lexical_cast<std::string>(mem_id));
      for(const auto& b : mem->banks)
      {
         xmlpp::Element* b_node = el_node->add_child("bank");
         b_node->set_attribute("id", b->id);
         b_node->set_attribute("type", b->type);
         b_node->set_attribute("width", boost::lexical_cast<std::string>(b->width));
         b_node->set_attribute("height", boost::lexical_cast<std::string>(b->height));
         b_node->set_attribute("area", boost::lexical_cast<std::string>(b->area));
      }
   }
   out_doc->write_to_file_formatted(output_dir + "/" + acc_name + "_final.xml");
#endif

   return true;
}

bool VerilogWriter::generate_wrapper(const MemoryWrapperPtr arch)
{
   *os << std::endl;
   *os << "module " << arch->id << "(CLK";
   for(const auto &buff : arch->buffers)
   {
      for(const auto &proc : buff->processes)
      {
         for(const auto &int_it : proc->interfaces)
         {
            for(const auto &p : int_it->ports)
            {
               *os << ", " << p->id;
            }
         }
      }
   }
   *os << ");" << std::endl;
   *os << "  input CLK;" << std::endl;
   for(const auto &buff : arch->buffers)
   {
      for(const auto &proc : buff->processes)
      {
         for(const auto &int_it : proc->interfaces)
         {
            *os << "  //buffer: " << buff->name << " - process: " << proc->name  << " - interface: ";
            if (int_it->type == Interface::R)
               *os << "R";
            else if (int_it->type == Interface::W)
               *os << "W";
            *os << std::endl;
            for(const auto &p : int_it->ports)
            {
               if (p->dir == Port::IN)
                  *os << "  input ";
               else if (p->dir == Port::OUT)
                  *os << "  output ";
               else
                  throw "VerilogWriter::generate_wrapper: Malformed port description";

               unsigned int p_size = boost::lexical_cast<unsigned int>(p->size);
               if (p_size > 1)
                  *os << "[" << p_size-1 << ":0] ";
               *os << p->id << ";" << std::endl;
            }
         }
      }
   }
   *os << std::endl;

   if (arch->banks.size() > 1)
   {
      MemoryPtr bank = arch->banks[0];
      *os << "  //signals from banks" << std::endl;
      *os << "  wire [" << bank->width-1 << ":0] Q0_out [0:" << arch->banks.size()-1 << "];" << std::endl;
      *os << "  wire [" << bank->width-1 << ":0] Q1_out [0:" << arch->banks.size()-1 << "];" << std::endl;
      *os << std::endl;
   }

   *os << "  //signals to banks" << std::endl;
   for(const auto &b : arch->banks)
   {
      for(const auto &i : b->interfaces)
      {
         for(const auto &p : i->ports)
         {
            *os << "  wire ";
            unsigned int p_size = boost::lexical_cast<unsigned int>(p->size);
            if (p_size > 1)
               *os << "[" << p_size-1 << ":0] ";
            *os << b->id << "_" << p->id << ";" << std::endl;
         }
      }
   }

   if (arch->wires.size())
   {
      *os << std::endl;
      *os << "  //additional wires" << std::endl;
      bool has_buffered = false;
      for(const auto &w : arch->wires)
      {
         if (!w->binding.size()) continue;
         if (w->is_buffered)
         {
            *os << "  reg ";
            has_buffered = true;
         }
         unsigned int w_size = boost::lexical_cast<unsigned int>(w->size);
         if (w_size > 1)
            *os << "[" << w_size-1 << ":0] ";
         *os << w->id << ";" << std::endl;
      }
      if (has_buffered)
      {
         *os << "  always@(posedge CLK)" << std::endl;
         *os << "  begin" << std::endl;
         for(const auto &w : arch->wires)
         {
            if (!w->binding.size()) continue;
            if (w->is_buffered)
               *os << "    " << w->id << " <= " << w->binding << ";" << std::endl;
         }
         *os << "  end" << std::endl;
      }
   }
   *os << std::endl;

   unsigned int N = 0;
   for(const auto &b : arch->banks)
   {
      *os << "  //signals for bank: " << b->id << std::endl;
      for(const auto &int_it : b->interfaces)
      {
         for(const auto &p : int_it->ports)
         {
            if (p->dir == Port::IN and p->binding.size())
            {
               *os << "  assign " << b->id << "_" << p->id << " = ";
               for (unsigned int c = 0; c < p->binding.size(); c++)
               {
                  *os << p->binding[c];
                  if (p->binding[c] == ':' and p->binding[c+1] == ' ')
                  {
                     bool last = true;
                     for (unsigned int d = c+1; d < p->binding.size(); d++)
                     {
                        if (p->binding[d] == ':') last = false;
                     }
                     if (!last)
                        *os << std::endl << "          ";
                  }
               }
               *os << ";" << std::endl;
            }
         }
      }
      if (arch->banks.size() > 1)
      {
         *os << "  //output bank selection" << std::endl;
         *os << "  assign Q0_out[" << N << "] = " << b->id << "_Q0;" << std::endl;
         *os << "  assign Q1_out[" << N << "] = " << b->id << "_Q1;" << std::endl;
      }
      N++;
      *os << std::endl;
   }

   *os << "  //assigns to output ports" << std::endl;
   for(const auto &buff : arch->buffers)
   {
      for(const auto &proc : buff->processes)
      {
         for(const auto &int_it : proc->interfaces)
         {
            for(const auto &p : int_it->ports)
            {
               if (p->dir == Port::OUT and p->binding.size())
                  *os << "  assign " << p->id << " = " << p->binding << ";" << std::endl;
            }
         }
      }
   }
   *os << std::endl;

   *os << "  //definitions of the banks" << std::endl;
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

   //std::cout << "--Number of banks = " << arch->banks.size() << std::endl;
   for(unsigned int i = 0; i < arch->banks.size(); i++)
   {
      MemoryPtr b = arch->banks[i];
      *os << "  " << b->type << " " << b->id << "(.CLK(CLK)";
      for(const auto &i : b->interfaces)
      {
         for(const auto &p : i->ports)
         {
            if (!p->binding.size())
            {
               //std::cout << "WARNING: binding not yet defined for port " << b->id << "." << p->id << std::endl;
               //continue;
            }
            *os << ",\n      ";
            *os << "." << p->id << "(" << b->id + "_" + p->id << ")";
         }
      }
      *os << ");" << std::endl;

      *os << std::endl;
   }
   *os << "endmodule" << std::endl;

   return true;
}

bool VerilogWriter::generate_top_entity(const std::string& top_name, const std::string& output_file, const std::map<std::string, std::string> &acc_interfaces, const std::vector<std::string>& accelerator_list)
{
   try
   {
      std::map<std::string, std::map<std::string, std::set<std::string> > > buffer_to_interface;

      std::map<std::string, MemoryWrapperPtr> buffer_to_wrapper;
      for(auto& mem : opt_functor->wrappers)
      {
         for(auto& b : mem.second->buffers)
         {
            buffer_to_wrapper[b->name] = mem.second;
         }
      }

      std::map<std::string, YAML::Node> acc_nodes;
      std::map<std::string, std::string> rtl_names;
      for(const auto& acc_interface : acc_interfaces)
      {
         YAML::Node config = YAML::LoadFile(acc_interface.second);
         std::string acc_logic_name = config["name"].as<std::string>();
         YAML::Node interface = config["interface"];
         acc_nodes[acc_interface.first] = interface;
         rtl_names[acc_interface.first] = acc_logic_name;
      }

      for(const auto& acc : acc_nodes)
      {
         if (opt_functor->components->list.find(acc.first) == opt_functor->components->list.end())
         {
            ComponentPtr accelerator(new Component(acc.first));
            opt_functor->components->list[acc.first] = accelerator;
         }
         opt_functor->components->list[acc.first]->parse_interface(acc.second, buffer_to_wrapper);
      }

      write_header(output_file);
      *os << "module " << top_name << "(clk, rst, acc_id, dmain_data, dmain_ready, dmain_valid, dmaout_data, dmaout_ready, dmaout_valid, ";
      *os << "rd_length, rd_index, rd_grant, rd_request, wr_length, wr_index, wr_grant, wr_request, conf_done, acc_done";
      std::map<std::string, std::vector<YAML::Node> > parameters;
      for(const auto& acc : acc_nodes)
      {
         const ComponentPtr accelerator = opt_functor->components->list[acc.first];
         for(const auto& p : acc.second)
         {
            std::string id = p["id"].as<std::string>();
            if (accelerator->clock_name == id || accelerator->reset_name == id || accelerator->acc_done_name == id || accelerator->conf_done_name == id)
               continue;
            if (accelerator->dmain_prefix.find(id) != accelerator->dmain_prefix.end() || accelerator->dmaout_prefix.find(id) != accelerator->dmaout_prefix.end())
               continue;
            if (accelerator->rdreq_prefix.find(id) != accelerator->rdreq_prefix.end() || accelerator->wrreq_prefix.find(id) != accelerator->wrreq_prefix.end() )
               continue;
            bool is_memory_port = true;
            for(auto& b : buffer_to_wrapper)
            {
               if (accelerator->darkmem_to_buffer.find(acc.first + "_" + id) != accelerator->darkmem_to_buffer.end())
                  continue;
               std::string type;
               if (accelerator->read_interfaces.find(acc.first + "_" + id) != accelerator->read_interfaces.end())
                  type += "r";
               if (accelerator->write_interfaces.find(acc.first + "_" + id) != accelerator->write_interfaces.end())
                  type += "w";
               if (type.size() == 0)
               {
                  is_memory_port = false;
                  continue;
               }
               if (id.find(b.first) != 0) continue;
               if (id.find(b.first + "_CE") != std::string::npos)
               {
                  std::string token = b.first + "_CE";
                  std::string num = id.substr(id.find(token)+token.size(), id.size());
                  buffer_to_interface[b.first][type].insert(b.first + "_CE" + num);
               }
            }
            if (!is_memory_port)
            {
               *os << ", " << acc.first << "_" << id;
               parameters[acc.first].push_back(p);
            }
         }
      }
      *os << ");" << std::endl;

      *os << "  input clk;" << std::endl;
      *os << "  input rst;" << std::endl;
      *os << "  input [31:0] acc_id;" << std::endl;
      *os << "  input [31:0] dmain_data;" << std::endl;
      *os << "  output dmain_ready;" << std::endl;
      *os << "  input dmain_valid;" << std::endl;
      *os << "  output [31:0] dmaout_data;" << std::endl;
      *os << "  output dmaout_ready;" << std::endl;
      *os << "  input dmaout_valid;" << std::endl;
      *os << "  output [31:0] rd_length;" << std::endl;
      *os << "  output [31:0] rd_index;" << std::endl;
      *os << "  input rd_grant;" << std::endl;
      *os << "  output rd_request;" << std::endl;
      *os << "  output [31:0] wr_length;" << std::endl;
      *os << "  output [31:0] wr_index;" << std::endl;
      *os << "  input wr_grant;" << std::endl;
      *os << "  output wr_request;" << std::endl;
      *os << "  input conf_done;" << std::endl;
      *os << "  output acc_done;" << std::endl;
      for(const auto& p : parameters)
      {
         for(const auto& pid : p.second)
         {
            *os << "  input ";
            if (pid["size"].as<unsigned int>() > 1)
            {
               *os << "[" << pid["size"].as<unsigned int>()-1 << ":0] ";
            }
            *os << p.first << "_" << pid["id"] << ";" << std::endl;
         }
      }
      *os << std::endl;

      *os << "  //signal for PLM units" << std::endl;
      std::set<std::string> list_memory_ports;
      for(const auto& acc : acc_nodes)
      {
         const ComponentPtr accelerator = opt_functor->components->list[acc.first];
         for(const auto& p : acc.second)
         {
            std::string id = p["id"].as<std::string>();
            if (accelerator->read_interfaces.find(acc.first + "_" + id) != accelerator->read_interfaces.end() ||
                accelerator->write_interfaces.find(acc.first + "_" + id) != accelerator->write_interfaces.end() ||
                accelerator->darkmem_to_buffer.find(acc.first + "_" + id) != accelerator->darkmem_to_buffer.end())
            {
               *os << "  wire ";
               unsigned int size = p["size"].as<unsigned int>();
               if (size>1)
               {
                  *os << "[" << size-1 << ":0] ";
               }
               *os << acc.first << "_" << id << ";" << std::endl;
               list_memory_ports.insert(acc.first + "_" + id);
            }
         }
      }
      *os << std::endl;

      *os << "  //other signals" << std::endl;
      std::map<MemoryWrapperPtr, unsigned int> memory_id;
      unsigned int mem_num = 0;
      for(auto& it : opt_functor->wrappers)
      {
         const MemoryWrapperPtr mem = it.second;
         unsigned int mem_id = mem_num++;
         memory_id[mem] = mem_id;
         for (auto & b_it : mem->buffers)
         {
            for(auto& inf_it : b_it->interfaces)
            {
               for(auto& p : inf_it->ports)
               {
                  if (list_memory_ports.find(p->id) == list_memory_ports.end() and p->id.find("_WEM") != std::string::npos)
                  {
                     *os << "  wire [" << p->size-1 << ":0] " << p->id << ";" << std::endl;
                     *os << "  assign " << p->id << " = " << "{" + boost::lexical_cast<std::string>(p->size) + "{1'b1}};" << std::endl;
                     list_memory_ports.insert(p->id);
                  }
               }
            }
         }
      }
      for(unsigned int i = 0; i < accelerator_list.size(); i++)
      {
         std::string acc_name = accelerator_list[i];
         const ComponentPtr accelerator = opt_functor->components->list[acc_name];
         std::string wrreq_prefix = accelerator->get_wrreq_prefix();
         std::string rdreq_prefix = accelerator->get_rdreq_prefix();
         std::string dmain_prefix = accelerator->get_dmain_prefix();
         std::string dmaout_prefix = accelerator->get_dmaout_prefix();
         *os << "  wire [31:0] " << acc_name << "_" << dmain_prefix << "_data;" << std::endl;
         *os << "  wire " << acc_name << "_" << dmain_prefix << "_ready;" << std::endl;
         *os << "  wire " << acc_name << "_" << dmain_prefix << "_valid;" << std::endl;
         *os << "  wire [31:0] " << acc_name << "_" << dmaout_prefix << "_data;" << std::endl;
         *os << "  wire " << acc_name << "_" << dmaout_prefix << "_ready;" << std::endl;
         *os << "  wire " << acc_name << "_" << dmaout_prefix << "_valid;" << std::endl;
         *os << "  wire [31:0] " << acc_name << "_" << rdreq_prefix << "_index;" << std::endl;
         *os << "  wire [31:0] " << acc_name << "_" << rdreq_prefix << "_length;" << std::endl;
         *os << "  wire " << acc_name << "_" << rdreq_prefix << "_request;" << std::endl;
         *os << "  wire " << acc_name << "_" << rdreq_prefix << "_grant;" << std::endl;
         *os << "  wire [31:0] " << acc_name << "_" << wrreq_prefix << "_index;" << std::endl;
         *os << "  wire [31:0] " << acc_name << "_" << wrreq_prefix << "_length;" << std::endl;
         *os << "  wire " << acc_name << "_" << wrreq_prefix << "_request;" << std::endl;
         *os << "  wire " << acc_name << "_" << wrreq_prefix << "_grant;" << std::endl;
         *os << "  wire " << acc_name << "_" << accelerator->acc_done_name << ";" << std::endl;
      }
      *os << std::endl;

      *os << "  //multiplexing" << std::endl;
      std::string dmain_ready = "1'b0";
      std::string dmaout_data = "32'b0", dmaout_valid = "1'b0";
      std::string rd_length = "32'b0", rd_index = "32'b0", rd_request = "1'b0";
      std::string wr_length = "32'b0", wr_index = "32'b0", wr_request = "1'b0";
      std::string acc_done = "1'b0";
      for(unsigned int i = 0; i < accelerator_list.size(); i++)
      {
         std::string acc_name = accelerator_list[i];
         const ComponentPtr accelerator = opt_functor->components->list[acc_name];
         std::string wrreq_prefix = accelerator->get_wrreq_prefix();
         std::string rdreq_prefix = accelerator->get_rdreq_prefix();
         std::string dmain_prefix = accelerator->get_dmain_prefix();
         std::string dmaout_prefix = accelerator->get_dmaout_prefix();
         dmain_ready = "acc_id == " + boost::lexical_cast<std::string>(i) + " ? " + acc_name + "_" + dmain_prefix + "_ready" + " : (" + dmain_ready + ")";
         dmaout_data = "acc_id == " + boost::lexical_cast<std::string>(i) + " ? " + acc_name + "_" + dmaout_prefix + "_data" + " : (" + dmaout_data + ")";
         dmaout_valid = "acc_id == " + boost::lexical_cast<std::string>(i) + " ? " + acc_name + "_" + dmaout_prefix + "_valid" + " : (" + dmaout_valid + ")";
         rd_length = "acc_id == " + boost::lexical_cast<std::string>(i) + " ? " + acc_name + "_" + rdreq_prefix + "_length" + " : (" + rd_length + ")";
         rd_index = "acc_id == " + boost::lexical_cast<std::string>(i) + " ? " + acc_name + "_" + rdreq_prefix + "_index" + " : (" + rd_index + ")";
         rd_request = "acc_id == " + boost::lexical_cast<std::string>(i) + " ? " + acc_name + "_" + rdreq_prefix + "_request" + " : (" + rd_request + ")";
         wr_length = "acc_id == " + boost::lexical_cast<std::string>(i) + " ? " + acc_name + "_" + wrreq_prefix + "_length" + " : (" + wr_length + ")";
         wr_index = "acc_id == " + boost::lexical_cast<std::string>(i) + " ? " + acc_name + "_" + wrreq_prefix + "_index" + " : (" + wr_index + ")";
         wr_request = "acc_id == " + boost::lexical_cast<std::string>(i) + " ? " + acc_name + "_" + wrreq_prefix + "_request" + " : (" + wr_request + ")";
         acc_done = "acc_id == " + boost::lexical_cast<std::string>(i) + " ? " + acc_name + "_" + accelerator->acc_done_name + " : (" + acc_done + ")";
         *os << "  assign " << acc_name + "_" + dmain_prefix + "_data = acc_id == " + boost::lexical_cast<std::string>(i) + " ? dmain_data : 32'b0;" << std::endl;
         *os << "  assign " << acc_name + "_" + dmain_prefix + "_valid = acc_id == " + boost::lexical_cast<std::string>(i) + " ? dmain_valid : 1'b0;" << std::endl;
         *os << "  assign " << acc_name + "_" + dmaout_prefix + "_ready = acc_id == " + boost::lexical_cast<std::string>(i) + " ? dmaout_ready : 1'b0;" << std::endl;
         *os << "  assign " << acc_name + "_" + rdreq_prefix + "_grant = acc_id == " + boost::lexical_cast<std::string>(i) + " ? rd_grant : 1'b0;" << std::endl;
         *os << "  assign " << acc_name + "_" + wrreq_prefix + "_grant = acc_id == " + boost::lexical_cast<std::string>(i) + " ? wr_grant : 1'b0;" << std::endl;
      }
      *os <<  "  assign dmain_ready = " << dmain_ready << ";" << std::endl;
      *os <<  "  assign dmaout_data = " << dmaout_data << ";" << std::endl;
      *os <<  "  assign dmaout_valid = " << dmaout_valid << ";" << std::endl;
      *os <<  "  assign rd_length = " << rd_length << ";" << std::endl;
      *os <<  "  assign rd_index = " << rd_index << ";" << std::endl;
      *os <<  "  assign rd_request = " << rd_request << ";" << std::endl;
      *os <<  "  assign wr_length = " << wr_length << ";" << std::endl;
      *os <<  "  assign wr_index = " << wr_index << ";" << std::endl;
      *os <<  "  assign wr_request = " << wr_request << ";" << std::endl;
      *os <<  "  assign acc_done = " << acc_done << ";" << std::endl;
      *os << std::endl;

      *os << "  //accelerators" << std::endl;
      for(const auto& acc : accelerator_list)
      {
         *os << "  " << rtl_names[acc] << " " << rtl_names[acc] << "_0 (";
         const ComponentPtr accelerator = opt_functor->components->list[acc];
         if (accelerator->clock_name.size() == 0)
            throw "Missing name for the clock";
         bool is_first = true;
         for(const auto& p : acc_nodes[acc])
         {
            std::string id = p["id"].as<std::string>();
            if (!is_first)
            {
               *os << "," << std::endl;
               *os << "          ";
            }
            is_first = false;
            if (list_memory_ports.find(acc + "_" + id) != list_memory_ports.end())
            {
               *os << "." << id << "(" << acc << "_" << id << ")";
            }
            else if (id == accelerator->clock_name)
            {
               *os << "." << id << "(clk)";
            }
            else if (id == accelerator->reset_name)
            {
               *os << "." << id << "(rst)";
            }
            else if (id == accelerator->conf_done_name)
            {
               *os << "." << id << "(conf_done)";
            }
            else
            {
               *os << "." << id << "(" << acc << "_" << id << ")";
            }
         }
         *os << ");" << std::endl << std::endl;
      }

      for(auto& it : opt_functor->wrappers)
      {
         const MemoryWrapperPtr mem = it.second;
         unsigned int mem_id = memory_id[mem];
         std::string id = mem->id;
         *os << "  //wrapper for buffers (";
         bool is_f = true;
         for (auto & b_it : mem->buffers)
         {
            std::string buffer_id = b_it->name;
            if (!is_f) *os << ", ";
            is_f = false;
            *os << buffer_id;
         }
         *os << ")" << std::endl;
         *os << "  " << id << " " << id << "_0 (.CLK(clk)";
         for (auto & b_it : mem->buffers)
         {
            for(auto& inf_it : b_it->interfaces)
            {
               std::vector<PortPtr> ports = inf_it->ports;
               for(unsigned int p = 0; p < ports.size(); p++)
               {
                  *os << "," << std::endl;
                  *os << "          ." << ports[p]->id << "(";
                  *os << ports[p]->id;
                  *os << ")";
               }
            }
         }
         *os << ");" << std::endl << std::endl;
      }

      *os << "endmodule" << std::endl;
      close_file();
      PRINT("Top module generated in file \"" << output_dir + "/" + output_file << "\"" << std::endl);
   }
   catch(...)
   {
      std::cout << "ERROR!" << std::endl;
      return false;
   }
   return true;
}
