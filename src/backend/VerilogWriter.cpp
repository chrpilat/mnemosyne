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

#include "rtl_architecture.hpp"

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
   out = new std::ofstream(output_dir + "/" + output_file);
   *out << "/**" << std::endl;
   *out << " * Copyright (c) 2014-2017, Columbia University" << std::endl;
   *out << " *" << std::endl;
   *out << " * @author Christian Pilato <pilato.christian@gmail.com>" << std::endl;
   *out << " */" << std::endl;
   spacing = 0;
}

void VerilogWriter::close_file()
{
   if (out)
   {
      *out << std::endl;
      out->close();
      delete out;
   }
}

void VerilogWriter::generate_top_plm()
{
   *out << "module " + opt_functor->accelerator_name + "_PLM (clk";
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
                  *out << ", " << p->id;
               }
            }
         }
      }
   }
   *out << ");" << std::endl;
   *out << "  input clk;" << std::endl;
   for(const auto &arch : opt_functor->wrappers)
   {
      for(const auto &buff : arch.second->buffers)
      {
         for(const auto &proc : buff->processes)
         {
            for(const auto &int_it : proc->interfaces)
            {
               *out << "  //buffer: " << buff->name << " - process: " << proc->name  << " - interface: ";
               if (int_it->type == Interface::R)
                  *out << "R";
               else if (int_it->type == Interface::W)
                  *out << "W";
               *out << std::endl;
               for(const auto &p : int_it->ports)
               {
                  if (p->dir == Port::IN)
                     *out << "  input ";
                  else if (p->dir == Port::OUT)
                     *out << "  output ";
                  else
                     throw "VerilogWriter::generate_wrapper: Malformed port description";

                  unsigned int p_size = boost::lexical_cast<unsigned int>(p->size);
                  if (p_size > 1)
                     *out << "[" << p_size-1 << ":0] ";
                  *out << p->id << ";" << std::endl;
               }
            }
         }
      }
      *out << std::endl;
   }

   for(const auto &it : opt_functor->wrappers)
   {
      const MemoryWrapperPtr arch = it.second;
      *out << "  " << arch->id << " " << arch->id << "_0" << " (.CLK(clk)";
      for(const auto &buff : arch->buffers)
      {
         for(const auto &proc : buff->processes)
         {
            for(const auto &int_it : proc->interfaces)
            {
               for(const auto &p : int_it->ports)
               {
                  *out << "," << std::endl;
                  *out << "     ." << p->id << "(" << p->id << ")";
               }
            }
         }
      }
      *out << ");" << std::endl << std::endl;
   }

   *out << "endmodule" << std::endl;
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
      if (!it.second->module)
         throw std::runtime_error("Module not created for wrapper \"" + it.second->id + "\"");
      write(it.second->module);
   }

   *out << std::endl;

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
      *out << "module " << name << "(";
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
            if (!first) *out << ", ";
            first = false;
            *out << id;
         }
      }
      *out << ");" << std::endl;

      for(const auto& p : interface)
      {
         std::string id = p["id"].as<std::string>();
         if (accelerator->read_interfaces.find(acc_name + "_" + id) == accelerator->read_interfaces.end() &&
             accelerator->write_interfaces.find(acc_name + "_" + id) == accelerator->write_interfaces.end() &&
             accelerator->darkmem_to_buffer.find(acc_name + "_" + id) == accelerator->darkmem_to_buffer.end())
         {
            std::string dir = p["dir"].as<std::string>();
            *out << "  " << dir << " ";
            unsigned int size = p["size"].as<unsigned int>();
            if (size>1)
            {
               *out << "[" << size-1 << ":0] ";
            }
            *out << id << ";" << std::endl;
         }
      }
      *out << std::endl;

      std::set<std::string> list_memory_ports;
      *out << "  //signals to memories" << std::endl;
      for(const auto& p : interface)
      {
         std::string id = p["id"].as<std::string>();
         if (accelerator->read_interfaces.find(acc_name + "_" + id) != accelerator->read_interfaces.end() ||
             accelerator->write_interfaces.find(acc_name + "_" + id) != accelerator->write_interfaces.end() ||
             accelerator->darkmem_to_buffer.find(acc_name + "_" + id) != accelerator->darkmem_to_buffer.end())
         {
            *out << "  wire ";
            unsigned int size = p["size"].as<unsigned int>();
            if (size>1)
            {
               *out << "[" << size-1 << ":0] ";
            }
            *out << acc_name << "_" << id << ";" << std::endl;
            list_memory_ports.insert(acc_name + "_" + id);
         }
      }
      *out << std::endl;

      *out << "  //other signals" << std::endl;
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
                     *out << "  wire [" << p->size-1 << ":0] " << p->id << ";" << std::endl;
                     *out << "  assign " << p->id << " = " << "{" + boost::lexical_cast<std::string>(p->size) + "{1'b1}};" << std::endl;
                     list_memory_ports.insert(p->id);
                  }
               }
            }
         }
      }
      *out << std::endl;

      *out << "  //accelerator" << std::endl;
      *out << "  " << acc_logic_name << " acc_0 (";
      bool is_first = true;
      for(const auto& p : interface)
      {
         std::string id = p["id"].as<std::string>();
         if (!is_first)
         {
            *out << "," << std::endl;
            *out << "          ";
         }
         is_first = false;
         if (list_memory_ports.find(acc_name + "_" + id) != list_memory_ports.end())
         {
            *out << "." << id << "(" << acc_name << "_" << id << ")";
         }
         else
         {
            *out << "." << id << "(" << id << ")";
         }
      }
      *out << ");" << std::endl << std::endl;

      if (accelerator->clock_name.size() == 0)
         throw "Missing name for the clock";
      for(auto& it : opt_functor->wrappers)
      {
         const MemoryWrapperPtr mem = it.second;
         unsigned int mem_id = memory_id[mem];
         std::string id = mem->id;
         *out << "  //wrapper for buffers (";
         bool is_f = true;
         for (auto & b_it : mem->buffers)
         {
            std::string buffer_id = b_it->name;
            if (!is_f) *out << ", ";
            is_f = false;
            *out << buffer_id;
         }
         *out << ")" << std::endl;
         if (accelerator->clock_name.size() == 0)
            throw "Missing name for the clock";
         *out << "  " << id << " mem_" << mem_id << "(.CLK(" << accelerator->clock_name << ")";
         for (auto & b_it : mem->buffers)
         {
            for(auto& inf_it : b_it->interfaces)
            {
               std::vector<PortPtr> ports = inf_it->ports;
               for(unsigned int p = 0; p < ports.size(); p++)
               {
                  *out << "," << std::endl;
                  *out << "          ." << ports[p]->id << "(";
                  if (b_it->accelerator == acc_name)
                     *out << ports[p]->id;
                  else
                  {
                     if (ports[p]->dir == Port::IN)
                     {
                        *out << "{" + boost::lexical_cast<std::string>(ports[p]->size) + "{1'b0}}";
                     }
                  }
                  *out << ")";
               }
            }
         }
         *out << ");" << std::endl << std::endl;
      }

      *out << "endmodule" << std::endl;
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
      *out << "module " << top_name << "(clk, rst, acc_id, dmain_data, dmain_ready, dmain_valid, dmaout_data, dmaout_ready, dmaout_valid, ";
      *out << "rd_length, rd_index, rd_grant, rd_request, wr_length, wr_index, wr_grant, wr_request, conf_done, acc_done";
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
               *out << ", " << acc.first << "_" << id;
               parameters[acc.first].push_back(p);
            }
         }
      }
      *out << ");" << std::endl;

      *out << "  input clk;" << std::endl;
      *out << "  input rst;" << std::endl;
      *out << "  input [31:0] acc_id;" << std::endl;
      *out << "  input [31:0] dmain_data;" << std::endl;
      *out << "  output dmain_ready;" << std::endl;
      *out << "  input dmain_valid;" << std::endl;
      *out << "  output [31:0] dmaout_data;" << std::endl;
      *out << "  output dmaout_ready;" << std::endl;
      *out << "  input dmaout_valid;" << std::endl;
      *out << "  output [31:0] rd_length;" << std::endl;
      *out << "  output [31:0] rd_index;" << std::endl;
      *out << "  input rd_grant;" << std::endl;
      *out << "  output rd_request;" << std::endl;
      *out << "  output [31:0] wr_length;" << std::endl;
      *out << "  output [31:0] wr_index;" << std::endl;
      *out << "  input wr_grant;" << std::endl;
      *out << "  output wr_request;" << std::endl;
      *out << "  input conf_done;" << std::endl;
      *out << "  output acc_done;" << std::endl;
      for(const auto& p : parameters)
      {
         for(const auto& pid : p.second)
         {
            *out << "  input ";
            if (pid["size"].as<unsigned int>() > 1)
            {
               *out << "[" << pid["size"].as<unsigned int>()-1 << ":0] ";
            }
            *out << p.first << "_" << pid["id"] << ";" << std::endl;
         }
      }
      *out << std::endl;

      *out << "  //signal for PLM units" << std::endl;
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
               *out << "  wire ";
               unsigned int size = p["size"].as<unsigned int>();
               if (size>1)
               {
                  *out << "[" << size-1 << ":0] ";
               }
               *out << acc.first << "_" << id << ";" << std::endl;
               list_memory_ports.insert(acc.first + "_" + id);
            }
         }
      }
      *out << std::endl;

      *out << "  //other signals" << std::endl;
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
                     *out << "  wire [" << p->size-1 << ":0] " << p->id << ";" << std::endl;
                     *out << "  assign " << p->id << " = " << "{" + boost::lexical_cast<std::string>(p->size) + "{1'b1}};" << std::endl;
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
         *out << "  wire [31:0] " << acc_name << "_" << dmain_prefix << "_data;" << std::endl;
         *out << "  wire " << acc_name << "_" << dmain_prefix << "_ready;" << std::endl;
         *out << "  wire " << acc_name << "_" << dmain_prefix << "_valid;" << std::endl;
         *out << "  wire [31:0] " << acc_name << "_" << dmaout_prefix << "_data;" << std::endl;
         *out << "  wire " << acc_name << "_" << dmaout_prefix << "_ready;" << std::endl;
         *out << "  wire " << acc_name << "_" << dmaout_prefix << "_valid;" << std::endl;
         *out << "  wire [31:0] " << acc_name << "_" << rdreq_prefix << "_index;" << std::endl;
         *out << "  wire [31:0] " << acc_name << "_" << rdreq_prefix << "_length;" << std::endl;
         *out << "  wire " << acc_name << "_" << rdreq_prefix << "_request;" << std::endl;
         *out << "  wire " << acc_name << "_" << rdreq_prefix << "_grant;" << std::endl;
         *out << "  wire [31:0] " << acc_name << "_" << wrreq_prefix << "_index;" << std::endl;
         *out << "  wire [31:0] " << acc_name << "_" << wrreq_prefix << "_length;" << std::endl;
         *out << "  wire " << acc_name << "_" << wrreq_prefix << "_request;" << std::endl;
         *out << "  wire " << acc_name << "_" << wrreq_prefix << "_grant;" << std::endl;
         *out << "  wire " << acc_name << "_" << accelerator->acc_done_name << ";" << std::endl;
      }
      *out << std::endl;

      *out << "  //multiplexing" << std::endl;
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
         *out << "  assign " << acc_name + "_" + dmain_prefix + "_data = acc_id == " + boost::lexical_cast<std::string>(i) + " ? dmain_data : 32'b0;" << std::endl;
         *out << "  assign " << acc_name + "_" + dmain_prefix + "_valid = acc_id == " + boost::lexical_cast<std::string>(i) + " ? dmain_valid : 1'b0;" << std::endl;
         *out << "  assign " << acc_name + "_" + dmaout_prefix + "_ready = acc_id == " + boost::lexical_cast<std::string>(i) + " ? dmaout_ready : 1'b0;" << std::endl;
         *out << "  assign " << acc_name + "_" + rdreq_prefix + "_grant = acc_id == " + boost::lexical_cast<std::string>(i) + " ? rd_grant : 1'b0;" << std::endl;
         *out << "  assign " << acc_name + "_" + wrreq_prefix + "_grant = acc_id == " + boost::lexical_cast<std::string>(i) + " ? wr_grant : 1'b0;" << std::endl;
      }
      *out <<  "  assign dmain_ready = " << dmain_ready << ";" << std::endl;
      *out <<  "  assign dmaout_data = " << dmaout_data << ";" << std::endl;
      *out <<  "  assign dmaout_valid = " << dmaout_valid << ";" << std::endl;
      *out <<  "  assign rd_length = " << rd_length << ";" << std::endl;
      *out <<  "  assign rd_index = " << rd_index << ";" << std::endl;
      *out <<  "  assign rd_request = " << rd_request << ";" << std::endl;
      *out <<  "  assign wr_length = " << wr_length << ";" << std::endl;
      *out <<  "  assign wr_index = " << wr_index << ";" << std::endl;
      *out <<  "  assign wr_request = " << wr_request << ";" << std::endl;
      *out <<  "  assign acc_done = " << acc_done << ";" << std::endl;
      *out << std::endl;

      *out << "  //accelerators" << std::endl;
      for(const auto& acc : accelerator_list)
      {
         *out << "  " << rtl_names[acc] << " " << rtl_names[acc] << "_0 (";
         const ComponentPtr accelerator = opt_functor->components->list[acc];
         if (accelerator->clock_name.size() == 0)
            throw "Missing name for the clock";
         bool is_first = true;
         for(const auto& p : acc_nodes[acc])
         {
            std::string id = p["id"].as<std::string>();
            if (!is_first)
            {
               *out << "," << std::endl;
               *out << "          ";
            }
            is_first = false;
            if (list_memory_ports.find(acc + "_" + id) != list_memory_ports.end())
            {
               *out << "." << id << "(" << acc << "_" << id << ")";
            }
            else if (id == accelerator->clock_name)
            {
               *out << "." << id << "(clk)";
            }
            else if (id == accelerator->reset_name)
            {
               *out << "." << id << "(rst)";
            }
            else if (id == accelerator->conf_done_name)
            {
               *out << "." << id << "(conf_done)";
            }
            else
            {
               *out << "." << id << "(" << acc << "_" << id << ")";
            }
         }
         *out << ");" << std::endl << std::endl;
      }

      for(auto& it : opt_functor->wrappers)
      {
         const MemoryWrapperPtr mem = it.second;
         unsigned int mem_id = memory_id[mem];
         std::string id = mem->id;
         *out << "  //wrapper for buffers (";
         bool is_f = true;
         for (auto & b_it : mem->buffers)
         {
            std::string buffer_id = b_it->name;
            if (!is_f) *out << ", ";
            is_f = false;
            *out << buffer_id;
         }
         *out << ")" << std::endl;
         *out << "  " << id << " " << id << "_0 (.CLK(clk)";
         for (auto & b_it : mem->buffers)
         {
            for(auto& inf_it : b_it->interfaces)
            {
               std::vector<PortPtr> ports = inf_it->ports;
               for(unsigned int p = 0; p < ports.size(); p++)
               {
                  *out << "," << std::endl;
                  *out << "          ." << ports[p]->id << "(";
                  *out << ports[p]->id;
                  *out << ")";
               }
            }
         }
         *out << ");" << std::endl << std::endl;
      }

      *out << "endmodule" << std::endl;
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

void VerilogWriter::write(const RtlNodePtr node)
{
   if (node->get_node_name() == "BinaryOperation")
   {
      const BinaryOperation* obj = GetPointer<BinaryOperation>(node);
      const Operation::op_t op_type = obj->get_op_type();
      const RtlNodePtr op0 = obj->get_op0();
      if (GetPointer<Operation>(op0) && Operation::get_level(GetPointer<Operation>(op0)->get_op_type()) > Operation::get_level(op_type)) *out << "(";
      write(op0);
      if (GetPointer<Operation>(op0) && Operation::get_level(GetPointer<Operation>(op0)->get_op_type()) > Operation::get_level(op_type)) *out << ")";
      *out << " ";
      switch(op_type)
      {
         case Operation::POWER:
            *out << "**";
            break;
         case Operation::MULT:
            *out << "*";
            break;
         case Operation::DIVIDE:
            *out << "/";
            break;
         case Operation::MOD:
            *out << "%";
            break;
         case Operation::PLUS:
            *out << "+";
            break;
         case Operation::MINUS:
            *out << "-";
            break;
         case Operation::SLL:
            *out << "<<";
            break;
         case Operation::SRL:
            *out << ">>";
            break;
         case Operation::SLA:
            *out << "<<<";
            break;
         case Operation::SRA:
            *out << ">>>";
            break;
         case Operation::LESSTHAN:
            *out << "<";
            break;
         case Operation::GREATERTHAN:
            *out << ">";
            break;
         case Operation::LESSEQ:
            *out << "<=";
            break;
         case Operation::GREATEREQ:
            *out << ">=";
            break;
         case Operation::EQ:
            *out << "==";
            break;
         case Operation::NOTEQ:
            *out << "!=";
            break;
         case Operation::EQL:
            *out << "==";
            break;
         case Operation::NOTEQL:
            *out << "!==";
            break;
         case Operation::AND:
            *out << "&";
            break;
         case Operation::XOR:
            *out << "^";
            break;
         case Operation::XNOR:
            *out << "~^";
            break;
         case Operation::OR:
            *out << "|";
            break;
         case Operation::LAND:
            *out << "&&";
            break;
         case Operation::LOR:
            *out << "||";
            break;
         default: throw std::runtime_error("Not supported operation");
      }
      *out << " ";
      const RtlNodePtr op1 = obj->get_op1();
      if (GetPointer<Operation>(op1) && Operation::get_level(GetPointer<Operation>(op1)->get_op_type()) > Operation::get_level(op_type)) *out << "(";
      write(op1);
      if (GetPointer<Operation>(op1) && Operation::get_level(GetPointer<Operation>(op1)->get_op_type()) > Operation::get_level(op_type)) *out << ")";
   }
   else if (node->get_node_name() == "BinaryValue")
   {
      const BinaryValue* value = GetPointer<BinaryValue>(node);
      *out << value->get_num_bits() << "'";
      if (value->get_type() == BinaryValue::BIN) *out << "b";
      else if (value->get_type() == BinaryValue::HEX) *out << "h";
      else if (value->get_type() == BinaryValue::DEC) *out << "d";
      *out << value->get_string_value();
   }
   else if (node->get_node_name() == "Concat")
   {
      *out << "{";
      const Concat* concat = GetPointer<Concat>(node);
      const std::list<RtlNodePtr> items = concat->get_items();
      bool is_first = true;
      for (const auto& it : items)
      {
         if (!is_first) *out << ",";
         is_first = false;
         write(it);
      }
      *out << "}";
   }
   else if (node->get_node_name() == "CondOperation")
   {
      const CondOperation* cond_op = GetPointer<CondOperation>(node);
      write(cond_op->get_cond());
      *out << " ? ";
      write(cond_op->get_true_value());
      *out << " : ";
      write(cond_op->get_false_value());
   }
   else if (node->get_node_name() == "Delay")
   {
      const Delay* obj = GetPointer<Delay>(node);
      *out << "#";
      write(obj->get_value());
   }
   else if (node->get_node_name() == "FloatValue")
   {
      const FloatValue* value = GetPointer<FloatValue>(node);
      *out << value->get_value();
   }
   else if (node->get_node_name() == "IntegerValue")
   {
      const IntegerValue* value = GetPointer<IntegerValue>(node);
      *out << value->get_value();
   }
   else if (node->get_node_name() == "ModuleAlways")
   {
      const ModuleAlways* always = GetPointer<ModuleAlways>(node);
      *out << std::string(spacing, ' ') << "always@(";
      const std::list<RtlNodePtr> senslist = always->get_senslist();
      bool is_first = true;
      for (const auto& s : senslist)
      {
         if (!is_first) *out << ", ";
         is_first = false;
         write(s);
      }
      *out << ")" << std::endl;
      spacing += 2;
      write(always->get_statement());
      spacing -= 2;
   }
   else if (node->get_node_name() == "ModuleAssign")
   {
      const ModuleAssign* assign = GetPointer<ModuleAssign>(node);
      *out << std::string(spacing, ' ') << "assign ";
      write(assign->get_lvalue());
      *out << " = ";
      write(assign->get_rvalue());
      *out << ";" << std::endl;
   }
   else if (node->get_node_name() == "ModuleBlock")
   {
      spacing -= 2;
      const ModuleBlock* block = GetPointer<ModuleBlock>(node);
      const std::list<RtlNodePtr> stmts = block->get_statements();
      *out << std::string(spacing, ' ') << "begin" << std::endl;
      spacing += 2;
      for (const auto& stmt : stmts)
      {
         write(stmt);
      }
      spacing -= 2;
      *out << std::string(spacing, ' ') << "end" << std::endl;
      spacing += 2;
   }
   else if (node->get_node_name() == "ModuleCall")
   {
      const ModuleCall* obj = GetPointer<ModuleCall>(node);
      if (obj->is_syscall()) *out << "$";
      *out << obj->get_name() << "(";
      const std::list<RtlNodePtr> args = obj->get_args();
      bool is_first = true;
      for (const auto& a : args)
      {
         if (!is_first) *out << ", ";
         is_first = false;
         write(a);
      }
      *out << ")";
   }
   else if (node->get_node_name() == "ModuleCaseStatement")
   {
      const ModuleCaseStatement* case_obj = GetPointer<ModuleCaseStatement>(node);
      *out << "case (";
      write(case_obj->get_cond());
      *out << ")" << std::endl;
      spacing += 2;
      const std::list<RtlNodePtr> cases = case_obj->get_case_list();
      for (const auto& c : cases)
      {
         write(c);
      }
      const RtlNodePtr default_case = case_obj->get_default_case();
      if (default_case)
      {
         *out << "default:" << std::endl;
         spacing += 2;
         write(default_case);
         spacing -= 2;
      }
   }
   else if (node->get_node_name() == "ModuleCase")
   {
      const ModuleCase* case_obj = GetPointer<ModuleCase>(node);
      write(case_obj->get_cond());
      *out << ":" << std::endl;
      spacing += 2;
      write(case_obj->get_statement());
      spacing -= 2;
   }
   else if (node->get_node_name() == "ModuleDecl")
   {
      const ModuleDecl* decl = GetPointer<ModuleDecl>(node);
      const std::list<RtlNodePtr> stmts = decl->get_statements();
      for(const auto& stmt : stmts)
      {
         write(stmt);
         if (stmt->get_node_name() == "Parameter") *out << ";" << std::endl;
      }
   }
   else if (node->get_node_name() == "ModuleDef")
   {
      const ModuleDef* module = GetPointer<ModuleDef>(node);
      const std::vector<RtlNodePtr> ports = module->get_ports();
      *out << "module " << module->get_name() << "(";
      for (unsigned int i = 0; i < ports.size(); i++)
      {
         if (i > 0) *out << ", ";
         *out << GetPointer<ModulePort>(ports[i])->get_name();
      }
      *out << ");" << std::endl;
      spacing += 2;
      const std::list<RtlNodePtr> portitems = module->get_portitems();
      for (const auto& item : portitems)
      {
         write(item);
      }
      *out << std::endl;
      const std::vector<RtlNodePtr> declarations = module->get_declarations();
      for (unsigned int i = 0; i < declarations.size(); i++)
      {
         write(declarations[i]);
      }
      *out << std::endl;
      const std::vector<RtlNodePtr> items = module->get_items();
      for (unsigned int i = 0; i < items.size(); i++)
      {
         write(items[i]);
      }
      spacing -= 2;
      *out << "endmodule" << std::endl << std::endl;
   }
   else if (node->get_node_name() == "ModuleForStatement")
   {
      const ModuleForStatement* for_obj = GetPointer<ModuleForStatement>(node);
      *out << "for (";
      write(for_obj->get_pre());
      *out << "; ";
      write(for_obj->get_cond());
      *out << "; ";
      write(for_obj->get_post());
      *out << ")" << std::endl;
      if (node->get_node_name() != "ModuleBlock") *out << "begin" << std::endl;
      spacing += 2;
      write(for_obj->get_statement());
      spacing -= 2;
      if (node->get_node_name() != "ModuleBlock") *out << "end" << std::endl;
   }
   else if (node->get_node_name() == "ModuleFunction")
   {
      const ModuleFunction* obj = GetPointer<ModuleFunction>(node);
      *out << std::string(spacing, ' ') << "function " << std::endl;
      std::tuple<RtlNodePtr, RtlNodePtr> width = obj->get_ret_width();
      if (std::get<0>(width) && std::get<1>(width))
      {
         *out << "[";
         write(std::get<0>(width));
         *out << ":";
         write(std::get<1>(width));
         *out << "] ";
      }
      *out << obj->get_name() << std::endl;
      spacing += 2;
      const std::list<RtlNodePtr> stmts = obj->get_declarations();
      for(const auto& stmt : stmts)
      {
         write(stmt);
         if (stmt->get_node_name() == "Parameter") *out << ";" << std::endl;
      }
      spacing += 2;
      write(obj->get_statement());
      spacing -= 2;
      spacing -= 2;

   }
   else if (node->get_node_name() == "ModuleGenerate")
   {
      const ModuleGenerate* obj = GetPointer<ModuleGenerate>(node);
      *out << std::string(spacing, ' ') << "generate" << std::endl;
      *out << std::string(spacing, ' ') << "begin" << std::endl;
      spacing += 2;
      const std::list<RtlNodePtr> stmts = obj->get_statements();
      for(const auto& stmt : stmts)
      {
         write(stmt);
      }
      spacing -= 2;
      *out << std::string(spacing, ' ') << "end" << std::endl;
   }
   else if (node->get_node_name() == "ModuleIfStatement")
   {
      const ModuleIfStatement* obj = GetPointer<ModuleIfStatement>(node);
      *out << "if (";
      write(obj->get_cond());
      *out << ")" << std::endl;
      spacing += 2;
      write(obj->get_true_statement());
      spacing -= 2;
      if (obj->get_false_statement())
      {
         spacing += 2;
         write(obj->get_false_statement());
         spacing -= 2;
      }
   }
   else if (node->get_node_name() == "ModuleInitial")
   {
      const ModuleInitial* obj = GetPointer<ModuleInitial>(node);
      *out << std::string(spacing, ' ') << "initial" << std::endl;
      spacing += 2;
      write(obj->get_statement());
      spacing -= 2;
   }
   else if (node->get_node_name() == "ModuleInstance")
   {
      const ModuleInstance* inst = GetPointer<ModuleInstance>(node);
      *out << std::string(spacing, ' ') << inst->get_module_name() << " ";
      const ModuleInstance::binding_t paramlist = inst->get_paramlist();
      if (paramlist.size())
      {
         *out << "#(";
         bool first = true;
         for (const auto& vIt : paramlist)
         {
            if (!first) *out << ", ";
            first = false;
            *out << "." + vIt.first + "(";
            write(vIt.second);
            *out << + ")";
         }
         *out << ") ";
      }
      *out << inst->get_name() << "(";
      spacing += 4;
      const std::vector<std::string> portnamelist = inst->get_portname_list();
      const ModuleInstance::binding_t portlist = inst->get_port_binding();
      for (unsigned int i = 0; i < portnamelist.size(); i++)
      {
         if (i > 0)
         {
            *out << "," << std::endl;
            *out << std::string(spacing, ' ');
         }
         *out << "." << portnamelist[i] << "(";
         write(portlist.find(portnamelist[i])->second);
         *out << ")";
      }
      spacing -= 4;
      *out << ");" << std::endl << std::endl;
   }
   else if (node->get_node_name() == "ModuleParam")
   {
      const ModuleParam* obj = GetPointer<ModuleParam>(node);
      if (obj->is_local_param()) *out << "localparam ";
      else *out << "parameter ";
      *out << obj->get_name() << " = ";
      write(obj->get_value());
   }
   else if (node->get_node_name() == "ModulePort")
   {
      const ModulePort* port = GetPointer<ModulePort>(node);
      std::string direction = "input";
      if (port->get_port_direction() == ModulePort::IN)
      {
         direction = "input";
      }
      else if (port->get_port_direction() == ModulePort::OUT)
      {
         direction = "output";
      }
      else if (port->get_port_direction() == ModulePort::INOUT)
      {
         direction = "inout";
      }
      unsigned int size = port->get_port_size();
      std::string size_string;
      if (size > 1)
      {
         size_string = "[" + STR(size-1) + ":0] ";
      }
      *out << std::string(spacing, ' ') << direction << " " << size_string << port->get_name() << ";" << std::endl;
   }
   else if (node->get_node_name() == "ModuleReg")
   {
      const ModuleReg* reg = GetPointer<ModuleReg>(node);
      *out << std::string(spacing, ' ') << "reg  ";
      if (reg->is_signed()) *out << "signed ";
      if (reg->is_parametric() or reg->get_reg_size() > 1)
      {
         std::tuple<RtlNodePtr, RtlNodePtr> width = reg->get_reg_width();
         *out << "[";
         write(std::get<0>(width));
         *out << ":";
         write(std::get<1>(width));
         *out << "]";
      }
      *out << " " << reg->get_name() << ";" << std::endl;
   }
   else if (node->get_node_name() == "ModuleRegArray")
   {
      const ModuleRegArray* reg = GetPointer<ModuleRegArray>(node);
      std::tuple<RtlNodePtr, RtlNodePtr> width = reg->get_reg_width();
      std::tuple<RtlNodePtr, RtlNodePtr> length = reg->get_reg_length();
      *out << std::string(spacing, ' ') << "reg  ";
      if (reg->is_signed()) *out << "signed ";
      *out << "[";
      write(std::get<0>(width));
      *out << ":";
      write(std::get<1>(width));
      *out << "] " << reg->get_name() << " [";
      write(std::get<0>(length));
      *out << ":";
      write(std::get<1>(length));
      *out << "];" << std::endl;
   }
   else if (node->get_node_name() == "ModuleRepeat")
   {
      const ModuleRepeat* obj = GetPointer<ModuleRepeat>(node);
      *out << "{";
      write(obj->get_times());
      *out << "{";
      write(obj->get_value());
      *out << "}}";
   }
   else if (node->get_node_name() == "ModuleSens")
   {
      const ModuleSens* sens = GetPointer<ModuleSens>(node);
      if (sens->get_type() == ModuleSens::ALL)
      {
         *out << "*";
      }
      else
      {
         if (sens->get_type() == ModuleSens::POSEDGE)
         {
            *out << "posedge ";
         }
         else if (sens->get_type() == ModuleSens::NEGEDGE)
         {
            *out << "negedge ";
         }
         write(sens->get_sig());
      }
   }
   else if (node->get_node_name() == "ModuleSingleStatement")
   {
      const ModuleSingleStatement* obj = GetPointer<ModuleSingleStatement>(node);
      write(obj->get_statement());
      *out << ";" << std::endl;
   }
   else if (node->get_node_name() == "ModuleSubstitution")
   {
      const ModuleSubstitution* stmt = GetPointer<ModuleSubstitution>(node);
      *out << std::string(spacing, ' ');
      write(stmt->get_lvalue());
      *out << (stmt->is_blocking() ? " = " : " <= ");
      write(stmt->get_rvalue());
      *out << ";" << std::endl;
   }
   else if (node->get_node_name() == "ModuleWait")
   {
      const ModuleWait* obj = GetPointer<ModuleWait>(node);
      *out << "wait(";
      write(obj->get_cond());
      *out << ");" << std::endl;
   }
   else if (node->get_node_name() == "ModuleWhileStatement")
   {
      const ModuleForStatement* obj = GetPointer<ModuleForStatement>(node);
      *out << "while (";
      write(obj->get_cond());
      *out << ")" << std::endl;
      if (node->get_node_name() != "ModuleBlock") *out << "begin" << std::endl;
      spacing += 2;
      write(obj->get_statement());
      spacing -= 2;
      if (node->get_node_name() != "ModuleBlock") *out << "end" << std::endl;
   }
   else if (node->get_node_name() == "ModuleWire")
   {
      const ModuleWire* wire = GetPointer<ModuleWire>(node);
      *out << std::string(spacing, ' ') << "wire ";
      if (wire->is_signed()) *out << "signed ";
      if (wire->is_parametric() or wire->get_wire_size() > 1)
      {
         std::tuple<RtlNodePtr, RtlNodePtr> width = wire->get_wire_width();
         *out << "[";
         write(std::get<0>(width));
         *out << ":";
         write(std::get<1>(width));
         *out << "] ";
      }
      *out << wire->get_name() << ";" << std::endl;
   }
   else if (node->get_node_name() == "ModuleWireArray")
   {
      const ModuleWireArray* wire = GetPointer<ModuleWireArray>(node);
      std::tuple<RtlNodePtr, RtlNodePtr> width = wire->get_wire_width();
      std::tuple<RtlNodePtr, RtlNodePtr> length = wire->get_wire_length();
      *out << std::string(spacing, ' ') << "wire ";
      if (wire->is_signed()) *out << "signed ";
      *out << "[";
      write(std::get<0>(width));
      *out << ":";
      write(std::get<1>(width));
      *out << "] " << wire->get_name() << " [";
      write(std::get<1>(length));
      *out << ":";
      write(std::get<0>(length));
      *out << "];" << std::endl;
   }
   else if (node->get_node_name() == "Partselect")
   {
      const Partselect* obj = GetPointer<Partselect>(node);
      write(obj->get_var());
      *out << "[";
      write(obj->get_msb());
      *out << ":";
      write(obj->get_lsb());
      *out << "]";
   }
   else if (node->get_node_name() == "Pointer")
   {
      const Pointer* obj = GetPointer<Pointer>(node);
      write(obj->get_var());
      *out << "[";
      write(obj->get_ptr());
      *out << "]";
   }
   else if (node->get_node_name() == "RtlComment")
   {
      const RtlComment* comment = GetPointer<RtlComment>(node);
      *out << std::endl;
      *out << std::string(spacing, ' ') << "//" << comment->get_text() << std::endl;
   }
   else if (node->get_node_name() == "RtlIdentifier")
   {
      const RtlIdentifier* id = GetPointer<RtlIdentifier>(node);
      *out << id->get_name();
   }
   else if (node->get_node_name() == "StringValue")
   {
      const StringValue* value = GetPointer<StringValue>(node);
      *out << value->get_value();
   }
   else if (node->get_node_name() == "UnaryOperation")
   {
      const UnaryOperation* obj = GetPointer<UnaryOperation>(node);
      Operation::op_t op_type = obj->get_op_type();
      switch(op_type)
      {
         case Operation::UPLUS:
            *out << "+";
            break;
         case Operation::UMINUS:
            *out << "-";
            break;
         case Operation::ULNOT:
            *out << "!";
            break;
         case Operation::UNOT:
            *out << "~";
            break;
         case Operation::UAND:
            *out << "&";
            break;
         case Operation::UNAND:
            *out << "~&";
            break;
         case Operation::UOR:
            *out << "|";
            break;
         case Operation::UNOR:
            *out << "~|";
            break;
         case Operation::UXOR:
            *out << "^";
            break;
         case Operation::UXNOR:
            *out << "~^";
            break;
         default: throw std::runtime_error("Not supported operation");
      }
      const RtlNodePtr op = obj->get_op();
      if (GetPointer<Operation>(op) && Operation::get_level(GetPointer<Operation>(op)->get_op_type()) > Operation::get_level(op_type)) *out << "(";
      write(op);
      if (GetPointer<Operation>(op) && Operation::get_level(GetPointer<Operation>(op)->get_op_type()) > Operation::get_level(op_type)) *out << ")";
   }
   else if (node->get_node_name() == "Variable")
   {
      const Variable* obj = GetPointer<Variable>(node);
      *out << std::string(spacing, ' ');
      const Variable::var_t var_type = obj->get_var_type();
      switch(var_type)
      {
         case Variable::INTEGER:
            *out << "integer";
            break;
         case Variable::REAL:
            *out << "real";
            break;
         case Variable::GENVAR:
            *out << "genvar";
            break;
      }
      *out << " " << obj->get_name();
      std::tuple<RtlNodePtr, RtlNodePtr> width = obj->get_var_width();
      if (std::get<0>(width) && std::get<1>(width))
      {
         *out << "[";
         write(std::get<0>(width));
         *out << ":";
         write(std::get<1>(width));
         *out << "]";
      }
      *out << ";" << std::endl;
   }
   else
      throw std::runtime_error("Element \"" + node->get_node_name() + "\" not supported yet in VerilogWriter");
}
