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
 * @file   ExportWriter.hpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Class used to export IP descriptions.
 *
 */
#ifndef EXPORTWRITER_HPP
#define EXPORTWRITER_HPP

#include "Writer.hpp"

#include "utils.hpp"

FORWARD_DECL(MemoryWrapper);
FORWARD_DECL(ExportWriter);
FORWARD_DECL(Functor);

class ExportWriter : public Writer
{
   public:

      typedef enum
      {
         IPXACT = 0,
         CTOSIP,
         STRATUSIP,
         _LAST_OPT  //placeholder: do not modify or remove!
      } available_backend;

   public:

      ExportWriter(unsigned int verbosity, const std::string& output_dir, const FunctorPtr opt_functor, unsigned int type);

      virtual ~ExportWriter();

      static const char* TypeToString(unsigned int v);

      static const char* TypeToLongDescription(unsigned int v);

      static std::string get_string_list();

      virtual void generate(const std::string& output_verilog_file) = 0;

      std::string get_string() const;

      static ExportWriterPtr create(unsigned int type, unsigned int verbosity, const std::string& output_dir, const FunctorPtr opt_functor);

      static unsigned int elaborate_option(unsigned int export_type, const std::string& option);

      static void export_ip_descriptions(unsigned int export_type, unsigned int verbosity, const std::string& out_export_dir, const std::string& wrapper_outfile, const FunctorPtr opt_functor);

   protected:

      unsigned int type;
};

typedef boost::shared_ptr<ExportWriter> ExportWriterPtr;

#endif // EXPORTWRITER_HPP
