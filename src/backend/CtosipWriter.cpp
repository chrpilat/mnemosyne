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
 * @file   CtosipWriter.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Methods for writing CTOS descriptions of the wrappers.
 *
 */
#include "CtosipWriter.hpp"

#include "Functor.hpp"

#include "Architecture.hpp"
#include "Memory.hpp"

#include "Array.hpp"

CtosipWriter::CtosipWriter(unsigned int _verbosity, const std::string& _output_dir, const FunctorPtr _opt_functor, unsigned int type) :
   ExportWriter(_verbosity, _output_dir, _opt_functor, type)
{

}

CtosipWriter::~CtosipWriter()
{

}

void CtosipWriter::generate(const std::string& output_verilog_file)
{
   std::string output_file = "ctosip.xml";

   xmlpp::Document* doc = new xmlpp::Document();
   xmlpp::Node* root_node = doc->create_root_node("ctos_ip_definitions");

   for(const auto& arch : opt_functor->wrappers)
   {
      MemoryWrapperPtr wrapper = arch.second;
      MemoryPtr bank = wrapper->banks[0];
      xmlpp::Element* node = root_node->add_child("RAMDef");
      node->add_child("name")->set_child_text(wrapper->id);
      node->add_child("wrapper_filename")->set_child_text(output_verilog_file);
      if (bank->liberty.size())
         node->add_child("liberty_filename")->set_child_text(bank->liberty);
      unsigned int width = 0;
      unsigned int num_words = 0;
      for(const auto& array : wrapper->buffers)
      {
         if (array->width > width) width = array->width;
         if (array->height > num_words) num_words = array->height;
      }
      node->add_child("num_words")->set_child_text(boost::lexical_cast<std::string>(num_words));
      node->add_child("width")->set_child_text(boost::lexical_cast<std::string>(width));
      node->add_child("min_write_width")->set_child_text(boost::lexical_cast<std::string>(1));
      for(const auto& array : wrapper->buffers)
      {
         xmlpp::Element* intf_node = node->add_child("interface_types");
         for(const auto& intf : array->interfaces)
         {
            switch (intf->type)
            {
               case Interface::R:
                  intf_node->add_child("elem")->set_child_text("sync_read");
                  break;
               case Interface::W:
                  intf_node->add_child("elem")->set_child_text("sync_write");
                  break;
               case Interface::RW:
                  intf_node->add_child("elem")->set_child_text("sync_read_write");
                  break;
            }
         }
      }
      node->add_child("has_reset")->set_child_text("false");
      node->add_child("reset_async")->set_child_text("false");
      node->add_child("reset_high")->set_child_text("true");
      node->add_child("clock_posedge")->set_child_text("true");
   }

   PRINT("CTOSIP description generated in file \"" << output_dir + "/" + output_file << "\"" << std::endl);
   doc->write_to_file_formatted(output_dir + "/" + output_file);
}
