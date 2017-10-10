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
 * @file   IpxactWriter.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Methods for writing IPXACT descriptions of the wrappers.
 *
 */
#include "IpxactWriter.hpp"

#include "Functor.hpp"

#include "Architecture.hpp"
#include "Memory.hpp"

#include "Array.hpp"

IpxactWriter::IpxactWriter(unsigned int _verbosity, const std::string& _output_dir, const FunctorPtr _opt_functor, unsigned int type) :
   ExportWriter(_verbosity, _output_dir, _opt_functor, type)
{

}

IpxactWriter::~IpxactWriter()
{

}

void IpxactWriter::generate(const std::string& output_verilog_file)
{
   std::string output_file = "component.xml";

   xmlpp::Document* doc = new xmlpp::Document();
   xmlpp::Node* root_node = doc->create_root_node("spirit:component");
   static_cast<xmlpp::Element*>(root_node)->set_attribute("xmlns:xilinx", "http://www.xilinx.com");
   static_cast<xmlpp::Element*>(root_node)->set_attribute("xmlns:spirit", "http://www.spiritconsortium.org/XMLSchema/SPIRIT/1685-2009");
   static_cast<xmlpp::Element*>(root_node)->set_attribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");

   root_node->add_child("spirit:vendor")->add_child_text("xilinx.com");
   root_node->add_child("spirit:library")->add_child_text("ip");
   root_node->add_child("spirit:name")->add_child_text(opt_functor->accelerator_name + "_PLM");
   root_node->add_child("spirit:version")->add_child_text("1.0");

   {
      xmlpp::Element* node = root_node->add_child("spirit:busInterfaces");
      //clk
      xmlpp::Element* bnode = node->add_child("spirit:busInterface");
      bnode->add_child("spirit:name")->add_child_text("clk");
      xmlpp::Element* nnode = bnode->add_child("spirit:busType");
      nnode->set_attribute("spirit:vendor", "xilinx.com");
      nnode->set_attribute("spirit:library", "signal");
      nnode->set_attribute("spirit:name", "clock");
      nnode->set_attribute("spirit:version", "1.0");
      nnode = bnode->add_child("spirit:abstractionType");
      nnode->set_attribute("spirit:vendor", "xilinx.com");
      nnode->set_attribute("spirit:library", "signal");
      nnode->set_attribute("spirit:name", "clock_rtl");
      nnode->set_attribute("spirit:version", "1.0");
      nnode = bnode->add_child("spirit:slave");
      nnode = bnode->add_child("spirit:portMaps");
      xmlpp::Element* inode = nnode->add_child("spirit:portMap");
      xmlpp::Element* lnode = inode->add_child("spirit:logicalPort");
      lnode->add_child("spirit:name")->set_child_text("CLK");
      xmlpp::Element* pnode = inode->add_child("spirit:physicalPort");
      pnode->add_child("spirit:name")->set_child_text("clk");
      nnode = bnode->add_child("spirit:parameters");
      inode = nnode->add_child("spirit:parameter");
      inode->add_child("spirit:name")->set_child_text("LAYERED_METADATA");
      xmlpp::Element* vnode = inode->add_child("spirit:value");
      vnode->set_attribute("spirit:resolve", "generated");
      vnode->set_attribute("spirit:id", "BUSIFPARAM_VALUE.CLK.LAYERED_METADATA");
      vnode->set_child_text("xilinx.com:interface:datatypes:1.0 {CLK {datatype {name {attribs {resolve_type immediate dependency {} format string minimum {} maximum {}} value {}} bitwidth {attribs {resolve_type immediate dependency {} format long minimum {} maximum {}} value 1} bitoffset {attribs {resolve_type immediate dependency {} format long minimum {} maximum {}} value 0}}}}");
      //other ports
      for(const auto& arch : opt_functor->wrappers)
      {
         for(const auto &buff : arch.second->buffers)
         {
            for(const auto &proc : buff->processes)
            {
               for(const auto &int_it : proc->interfaces)
               {
                  for(const auto &p : int_it->ports)
                  {
                     xmlpp::Element* bnode = node->add_child("spirit:busInterface");
                     bnode->add_child("spirit:name")->add_child_text(p->id);
                     xmlpp::Element* nnode = bnode->add_child("spirit:busType");
                     nnode->set_attribute("spirit:vendor", "xilinx.com");
                     nnode->set_attribute("spirit:library", "signal");
                     nnode->set_attribute("spirit:name", "data");
                     nnode->set_attribute("spirit:version", "1.0");
                     nnode = bnode->add_child("spirit:abstractionType");
                     nnode->set_attribute("spirit:vendor", "xilinx.com");
                     nnode->set_attribute("spirit:library", "signal");
                     nnode->set_attribute("spirit:name", "data_rtl");
                     nnode->set_attribute("spirit:version", "1.0");
                     if (p->dir == Port::IN)
                        nnode = bnode->add_child("spirit:slave");
                     else if (p->dir == Port::OUT)
                        nnode = bnode->add_child("spirit:master");
                     nnode = bnode->add_child("spirit:portMaps");
                     xmlpp::Element* inode = nnode->add_child("spirit:portMap");
                     xmlpp::Element* lnode = inode->add_child("spirit:logicalPort");
                     lnode->add_child("spirit:name")->set_child_text("DATA");
                     xmlpp::Element* pnode = inode->add_child("spirit:physicalPort");
                     pnode->add_child("spirit:name")->set_child_text(p->id);
                     nnode = bnode->add_child("spirit:parameters");
                     inode = nnode->add_child("spirit:parameter");
                     inode->add_child("spirit:name")->set_child_text("LAYERED_METADATA");
                     xmlpp::Element* vnode = inode->add_child("spirit:value");
                     vnode->set_attribute("spirit:resolve", "generated");
                     vnode->set_attribute("spirit:id", "BUSIFPARAM_VALUE." + boost::to_upper_copy<std::string>(p->id) + ".LAYERED_METADATA");
                     vnode->set_child_text("xilinx.com:interface:datatypes:1.0 {DATA {datatype {name {attribs {resolve_type immediate dependency {} format string minimum {} maximum {}} value {}} bitwidth {attribs {resolve_type immediate dependency {} format long minimum {} maximum {}} value " + boost::lexical_cast<std::string>(p->size) + "} bitoffset {attribs {resolve_type immediate dependency {} format long minimum {} maximum {}} value 0}}}}");
                  }
               }
            }
         }
      }
   }

   {
      xmlpp::Element* node = root_node->add_child("spirit:model");
      xmlpp::Element* vnodes = node->add_child("spirit:views");
      xmlpp::Element* vnode = vnodes->add_child("spirit:view");
      vnode->add_child("spirit:name")->add_child_text("xilinx_verilogsynthesis");
      vnode->add_child("spirit:displayName")->add_child_text("VerilogSynthesis");
      vnode->add_child("spirit:envIdentifier")->add_child_text("verilogSource:vivado.xilinx.com:synthesis");
      vnode->add_child("spirit:language")->add_child_text("verilog");
      vnode->add_child("spirit:modelName")->add_child_text(opt_functor->accelerator_name + "_PLM");
      xmlpp::Element* fsnode = vnode->add_child("spirit:fileSetRef");
      fsnode->add_child("spirit:localName")->add_child_text("xilinx_verilogsynthesis_view_fileset");

      vnode = vnodes->add_child("spirit:view");
      vnode->add_child("spirit:name")->add_child_text("xilinx_xpgui");
      vnode->add_child("spirit:displayName")->add_child_text("UI Layout");
      vnode->add_child("spirit:envIdentifier")->add_child_text(":vivado.xilinx.com:xgui.ui");
      fsnode = vnode->add_child("spirit:fileSetRef");
      fsnode->add_child("spirit:localName")->add_child_text("xilinx_xpgui_view_fileset");

      xmlpp::Element* pnodes = node->add_child("spirit:ports");
      xmlpp::Element* pnode = pnodes->add_child("spirit:port");
      pnode->add_child("spirit:name")->add_child_text("clk");
      xmlpp::Element* wnode = pnode->add_child("spirit:wire");
      wnode->add_child("spirit:direction")->add_child_text("in");
      xmlpp::Element* wtnodes = wnode->add_child("spirit:wireTypeDefs");
      xmlpp::Element* wtnode = wtnodes->add_child("spirit:wireTypeDef");
      wtnode->add_child("spirit:typeName")->add_child_text("STD_LOGIC");
      wtnode->add_child("spirit:viewNameRef")->add_child_text("xilinx_verilogsynthesis");
      //other ports
      for(const auto& arch : opt_functor->wrappers)
      {
         for(const auto &buff : arch.second->buffers)
         {
            for(const auto &proc : buff->processes)
            {
               for(const auto &int_it : proc->interfaces)
               {
                  for(const auto &p : int_it->ports)
                  {
                     xmlpp::Element* pnode = pnodes->add_child("spirit:port");
                     pnode->add_child("spirit:name")->add_child_text(p->id);
                     xmlpp::Element* wnode = pnode->add_child("spirit:wire");
                     if (p->dir == Port::IN)
                        wnode->add_child("spirit:direction")->add_child_text("in");
                     else if (p->dir == Port::OUT)
                        wnode->add_child("spirit:direction")->add_child_text("out");
                     if (p->size > 1)
                     {
                        xmlpp::Element* vecnode = wnode->add_child("spirit:vector");
                        vecnode->add_child("spirit:left")->add_child_text(boost::lexical_cast<std::string>(p->size));
                        vecnode->add_child("spirit:right")->add_child_text("0");
                     }
                     xmlpp::Element* wtnodes = wnode->add_child("spirit:wireTypeDefs");
                     xmlpp::Element* wtnode = wtnodes->add_child("spirit:wireTypeDef");
                     if (p->size > 1)
                        wtnode->add_child("spirit:typeName")->add_child_text("STD_LOGIC_VECTOR");
                     else
                        wtnode->add_child("spirit:typeName")->add_child_text("STD_LOGIC");
                     wtnode->add_child("spirit:viewNameRef")->add_child_text("xilinx_verilogsynthesis");
                  }
               }
            }
         }
      }
   }

   {
      xmlpp::Element* fsnodes = root_node->add_child("spirit:fileSets");
      xmlpp::Element* fsnode = fsnodes->add_child("spirit:fileSet");
      fsnode->add_child("spirit:name")->add_child_text("xilinx_verilogsynthesis_view_fileset");
      xmlpp::Element* fnode = fsnode->add_child("spirit:file");
      fnode->add_child("spirit:name")->add_child_text(output_verilog_file);
      fnode->add_child("spirit:fileType")->add_child_text("verilogSource");
   }

   {
      xmlpp::Element* node = root_node->add_child("spirit:description");
      node->add_child_text("An IP generated by Mnemosyne");
   }

   {
      xmlpp::Element* pnodes = root_node->add_child("spirit:parameters");
      xmlpp::Element* pnode = pnodes->add_child("spirit:parameter");
      pnode->add_child("spirit:name")->add_child_text("Component_Name");
      xmlpp::Element* vnode = pnode->add_child("spirit:value");
      vnode->set_attribute("spirit:resolve", "user");
      vnode->set_attribute("spirit:id", "PARAM_VALUE.Component_Name");
      vnode->set_attribute("spirit:order", "1");
      vnode->set_child_text(opt_functor->accelerator_name + "_PLM_v1_0");
   }

   {
      xmlpp::Element* nodes = root_node->add_child("spirit:vendorExtensions");
      xmlpp::Element* cnode = nodes->add_child("xilinx:coreExtensions");
      xmlpp::Element* snode = cnode->add_child("xilinx:supportedFamilies");
      xmlpp::Element* fnode = snode->add_child("xilinx:family");
      fnode->set_attribute("xilinx:lifeCycle", "Pre-Production");
      fnode->set_child_text("qzynq");
      snode = cnode->add_child("xilinx:taxonomies");
      snode->add_child("xilinx:taxonomy")->add_child_text("/MNEMOSYNE");
      cnode->add_child("xilinx:displayName")->add_child_text(opt_functor->accelerator_name + "_PLM");
   }

   PRINT("IPXACT description generated in file \"" << output_dir + "/" + output_file << "\"" << std::endl);
   doc->write_to_file_formatted(output_dir + "/" + output_file);
}
