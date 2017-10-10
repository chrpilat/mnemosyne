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
 * @file   ExportWriter.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Methods for exporting wrapper descriptions.
 *
 */
#include "ExportWriter.hpp"

#include "IpxactWriter.hpp"
#include "CtosipWriter.hpp"
#include "StratusipWriter.hpp"

ExportWriter::ExportWriter(unsigned int _verbosity, const std::string& _output_dir, const FunctorPtr _opt_functor, unsigned int _type) :
   Writer(_verbosity, _output_dir, _opt_functor),
   type(_type)
{

}

ExportWriter::~ExportWriter()
{

}

const char* ExportWriter::TypeToString(unsigned int v)
{
    switch (v)
    {
        case IPXACT:    return "IPXACT";
        case CTOSIP:    return "CTOSIP";
        case STRATUSIP: return "STRATUSIP";
        default:        return "[Unknown export type]";
    }
}

const char* ExportWriter::TypeToLongDescription(unsigned int v)
{
    switch (v)
    {
        case IPXACT:    return "Vivado IP-XACT component";
        case CTOSIP:    return "Cadence C-to-Silicon vendor RAM";
        case STRATUSIP: return "Cadence Stratus memory IP";
        default:        return "[Unknown export type]";
    }
}

ExportWriterPtr ExportWriter::create(unsigned int type, unsigned int verbosity, const std::string& output_dir, const FunctorPtr opt_functor)
{
   switch(type)
   {
      case IPXACT:
         return ExportWriterPtr(new IpxactWriter(verbosity, output_dir, opt_functor, type));
      case CTOSIP:
         return ExportWriterPtr(new CtosipWriter(verbosity, output_dir, opt_functor, type));
      case STRATUSIP:
         return ExportWriterPtr(new StratusipWriter(verbosity, output_dir, opt_functor, type));
      default:
         throw std::runtime_error("Unsupported backend");
   }
   return ExportWriterPtr();
}

std::string ExportWriter::get_string_list()
{
   std::string list;
   for(unsigned int i = 0; i < _LAST_OPT; i++)
   {
      if (list.size()) list += "\n";
      list += "  " + std::string(TypeToString(i)) + ": \t" + std::string(TypeToLongDescription(i));
   }
   return list;
}

std::string ExportWriter::get_string() const
{
   return TypeToString(type);
}

unsigned int ExportWriter::elaborate_option(unsigned int export_type, const std::string& option)
{
   for(unsigned int i = 0; i < _LAST_OPT; i++)
   {
      if (option == TypeToString(i))
      {
         export_type |= (1 << i);
         return export_type;
      }
   }
   throw std::runtime_error("Wrong --export option (\"" + option + "\")");
   return export_type;
}

void ExportWriter::export_ip_descriptions(unsigned int export_type, unsigned int verbosity, const std::string& out_export_dir, const std::string& wrapper_outfile, const FunctorPtr opt_functor)
{
   for(unsigned int i = 0; i < _LAST_OPT; i++)
   {
      if (export_type & (1 << i))
      {
         ExportWriterPtr wrap = ExportWriter::create(i, verbosity, out_export_dir, opt_functor);
         PRINT(" == Memory subsystem generation (" << wrap->get_string() << ") == " << std::endl);
         wrap->generate(wrapper_outfile);
         PRINT(" =====" << std::endl << std::endl);
      }
   }
}
