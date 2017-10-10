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
 * @file   StatusipWriter.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Methods for writing STATUS descriptions.
 *
 */
#include "StratusipWriter.hpp"

#include "Functor.hpp"

#include "Architecture.hpp"
#include "Memory.hpp"

#include "Array.hpp"

StratusipWriter::StratusipWriter(unsigned int _verbosity, const std::string& _output_dir, const FunctorPtr _opt_functor, unsigned int type) :
   ExportWriter(_verbosity, _output_dir, _opt_functor, type)
{

}

StratusipWriter::~StratusipWriter()
{

}

void StratusipWriter::generate(const std::string& output_verilog_file)
{
   for(const auto& arch : opt_functor->wrappers)
   {
      MemoryWrapperPtr wrapper = arch.second;

      generate_bdm_file(wrapper, output_verilog_file);

      generate_systemc_header(wrapper);
   }
}

void StratusipWriter::generate_bdm_file(const MemoryWrapperPtr wrapper, const std::string& output_verilog_file)
{
   std::ofstream os(output_dir + "/" + wrapper->id + ".bdm");
   os << "setName " << wrapper->id << std::endl;
   os << "catch {setToolVersion \"16.20-p100\"}" << std::endl;
   os << "setModelStyle 1" << std::endl;
   unsigned int width = 0;
   unsigned int num_words = 0;
   for(const auto& array : wrapper->buffers)
   {
      if (array->width > width) width = array->width;
      if (array->height > num_words) num_words = array->height;
   }
   os << "setWordSize " << width << std::endl;
   os << "setNumWords " << num_words << std::endl;
   os << "setArea " << std::showpoint << (wrapper->banks[0]->area * (double)wrapper->banks.size()) << std::endl;
   os << "setLatency 1" << std::endl;
   //# TODO: these delays are arbitrary, but should be taken from lib
   os << "setDelay 600.0000" << std::endl;
   os << "setSetup 200.0000" << std::endl;
   os << "setInputDataFormat 1" << std::endl;
   os << "setFileSuffix 0" << std::endl;
   os << "setHasReset 0" << std::endl;
   os << "setNoSpecReads 0" << std::endl;
   os << "setSharedAccess 1" << std::endl;
   os << "setMaxSharedPorts 32" << std::endl;
   os << "setIntRegMemIn 0" << std::endl;
   os << "setIntRegMemOut 0" << std::endl;
   os << "setExtRegMemIn 0" << std::endl;
   os << "setExtRegMemOut 0" << std::endl;
   os << "setIntRegsAtMemIn 0" << std::endl;
   os << "setIntRegsAtMemOut 0" << std::endl;
   os << "setExtRegsAtMemIn 0" << std::endl;
   os << "setExtRegsAtMemOut 0" << std::endl;
   os << "setClockMult 0" << std::endl;
   unsigned int num_interfaces = 0;
   for(const auto& b : wrapper->buffers)
   {
      num_interfaces += b->interfaces.size();
   }
   os << "setNumAccessPorts " << num_interfaces << std::endl;
   os << "setNumExtraPorts 0" << std::endl;
   os << "setPipelined 0" << std::endl;
   if (wrapper->banks[0]->v_file.size() == 0)
      throw std::runtime_error("Missing definition of the memory IP wrapper file");
   os << "setVendorModel \"" << wrapper->banks[0]->v_file << "\"" << std::endl;
   os << "setModelWrapper \"" << boost::filesystem::complete(output_verilog_file).c_str() << "\"" << std::endl;
   os << "setExtraPortsInWrapper 0" << std::endl;
   for(const auto& b : wrapper->buffers)
   {
      for(const auto& it : b->interfaces)
      {
         if (it->type == Interface::W)
         {
            os << "setPortData " << it->idx << " setType 1" << std::endl;
            os << "setPortData " << it->idx << " setAddr \"" << it->address->id << "\"" << std::endl;
            os << "setPortData " << it->idx << " setClk \"CLK\"" << std::endl;
            os << "setPortData " << it->idx << " setReqName \"REQ" << it->idx << "\"" << std::endl;
            os << "setPortData " << it->idx << " setDin \"" << it->data_in->id << "\"" << std::endl;
            os << "setPortData " << it->idx << " setHasRW 1" << std::endl;
            os << "setPortData " << it->idx << " setRwName \"" << it->write_enable->id << "\"" << std::endl;
            os << "setPortData " << it->idx << " setRwActive 1" << std::endl;
            os << "setPortData " << it->idx << " setWMWord 1" << std::endl;
            os << "setPortData " << it->idx << " setWMName \"" << it->write_mask->id << "\"" << std::endl;
            os << "setPortData " << it->idx << " setWMActive 1" << std::endl;
         }
         else if (it->type == Interface::R)
         {
            os << "setPortData " << it->idx << " setType 0" << std::endl;
            os << "setPortData " << it->idx << " setAddr \"" << it->address->id << "\"" << std::endl;
            os << "setPortData " << it->idx << " setClk \"CLK\"" << std::endl;
            os << "setPortData " << it->idx << " setReqName \"REQ" << it->idx << "\"" << std::endl;
            os << "setPortData " << it->idx << " setDout \"" << it->data_out->id << "\"" << std::endl;
            os << "setPortData " << it->idx << " setHasOE 0" << std::endl;
            os << "setPortData " << it->idx << " setHasRE 0" << std::endl;
         }
         else
            throw std::runtime_error("Unsupported interface");

         os << "setPortData " << it->idx << " setHasCE 1" << std::endl;
         os << "setPortData " << it->idx << " setCEName \"" << it->enable->id << "\"" << std::endl;
         os << "setPortData " << it->idx << " setCEActive 1" << std::endl;
      }
   }
   os.close();
   PRINT("STRATUS IP description generated in file \"" << output_dir + "/" + wrapper->id + ".bdm\"" << std::endl);
}

void StratusipWriter::generate_systemc_header(const MemoryWrapperPtr wrapper)
{
   std::ofstream os(output_dir + "/" + wrapper->id + ".hpp");
   os << "#ifndef __" << boost::to_upper_copy<std::string>(wrapper->id) << "_HPP__" << std::endl;
   os << "#define __" << boost::to_upper_copy<std::string>(wrapper->id) << "_HPP__" << std::endl;
   os << "#include \"" << wrapper->id << ".h\"" << std::endl;
   os << "template<class T, unsigned S, typename ioConfig=CYN::PIN>" << std::endl;
   os << "class " << wrapper->id << "_t : public sc_module" << std::endl;
   os << "{" << std::endl;
   os << "" << std::endl;
   os << "  HLS_INLINE_MODULE;" << std::endl;
   os << "public:" << std::endl;
   os << "  " << wrapper->id << "_t(const sc_module_name& name = sc_gen_unique_name(\"" << wrapper->id << "\"))" << std::endl;
   os << "  : sc_module(name)" << std::endl;
   os << "  , clk(\"clk\")" << std::endl;
   unsigned int num_interfaces = 0;
   for(const auto& b : wrapper->buffers)
   {
      num_interfaces += b->interfaces.size();
   }
   for (unsigned int i = 0; i < num_interfaces; i++)
      os << "  , port" << i+1 << "(\"port" << i+1 << "\")" << std::endl;
   os << "  {" << std::endl;
   os << "    m_m0.clk_rst(clk);" << std::endl;
   for (unsigned int i = 0; i < num_interfaces; i++)
      os << "    port" << i+1 << "(m_m0.if" << i+1 << ");" << std::endl;
   os << "  }" << std::endl;
   os << "" << std::endl;
   os << "  sc_in<bool> clk;" << std::endl;
   os << "" << std::endl;
   os << "  " << wrapper->id << "::wrapper<ioConfig> m_m0;" << std::endl;
   os << "" << std::endl;
   for (unsigned int i = 0; i < num_interfaces; i++)
   {
      //TODO: there is a bug in Stratus that prevents a port interface to have a single dimension (so use [1] for now)
      os << "  typedef " << wrapper->id << "::port_" << i+1 << "<ioConfig, T[1][S]> Port" << i+1 << "_t;" << std::endl;
   }
   os << "" << std::endl;
   for (unsigned int i = 0; i < num_interfaces; i++)
      os << "  Port" << i+1 << "_t port" << i+1 << ";" << std::endl;
   os << "};" << std::endl;
   os << "#endif" << std::endl;
   os.close();
   PRINT("STRATUS header generated in file \"" << output_dir + "/" + wrapper->id + ".hpp\"" << std::endl);
}
