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
 * @file   VerilogWriter.hpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Class for writing Verilog descriptions.
 *
 */
#ifndef _VERILOG_WRITER_HPP_
#define _VERILOG_WRITER_HPP_

#include "Writer.hpp"

#include "utils.hpp"

FORWARD_DECL(MemoryWrapper);
FORWARD_DECL(Functor);

class VerilogWriter : public Writer
{
      std::ofstream* os;

      bool has_to_generate_top_plm;

      bool generate_wrapper(const MemoryWrapperPtr arch);

      void write_header(const std::string &output_file);

      void close_file();

      void generate_top_plm();

   public:

      VerilogWriter(unsigned int verbosity, const std::string& output_dir, const FunctorPtr opt_functor);

      virtual ~VerilogWriter();

      void set_has_to_generate_top_plm(bool new_value);

      bool generate_top_entity(const std::string& acc_name, const std::string& name, const std::string &output_file, const std::string& acc_interface);

      bool generate_wrappers(const std::string& output_file);

      bool generate_top_entity(const std::string& top_name, const std::string& output_file, const std::map<std::string, std::string>& acc_interfaces, const std::vector<std::string>& accelerator_list);

};

typedef boost::shared_ptr<VerilogWriter> VerilogWriterPtr;

#endif
