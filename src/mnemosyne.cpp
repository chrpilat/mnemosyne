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
 * @file   mnemosyne.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  This is the main file of the Mnemosyne tool.
 */
#include "Allocation.hpp"
#include "Functor.hpp"
#include "Memory.hpp"
#include "Array.hpp"
#include "Component.hpp"

#include "VerilogWriter.hpp"
#include "ExportWriter.hpp"

#include "branch_name.hpp"
#include "revision_hash.hpp"

#include "utils.hpp"

/*! This is only used to improve the readability of the code */
#define po boost::program_options

/**
 * Starting point of the tool.
 *
 * @param argc Number of parameters
 * @param argv List of command-line option to be parsed with Boost Program Options
 */
int main(int argc, char** argv)
{
   unsigned int verbosity;
   std::string out_verilog_dir, out_export_dir;
   std::string temp_dir;
   std::string library_ip;
   std::string name;
   std::string input_cgraph;
   std::string acc_config;
   std::string multiacc_config;
   std::string scenario;
   std::string alg_str;
   std::string verilog_name, top_verilog_name;
   std::string top_name;
   Functor::optimization_t algorithm = Functor::AREA_HEURISTIC;
   unsigned int sharing = Allocation::ADDRESS_SPACE | Allocation::INTERFACE | Allocation::COLORING;
   std::vector<std::string> sharing_options;
   unsigned int export_type = 0;
   std::vector<std::string> export_options;
   std::string acc_interface;
   bool generate_top_plm;
   std::string config_file;

   std::string mode;
   std::string interfaces;
   unsigned int width, height;

   std::string export_parameter_opt = "Export mode:\n" + ExportWriter::get_string_list();
   std::string opt_parameter_opt = "Optimization goal:\n" + Functor::get_string_list();

   po::options_description desc("General options");
   desc.add_options()
         ("help",                                                                          "Print help message")
         ("verbosity,v", po::value<unsigned int>(&verbosity)->default_value(0),            "Verbosity level")
         ("config", po::value<std::string>(&config_file),                                  "Configuration file")
         ;

   po::options_description input_desc("Input options");
   input_desc.add_options()
         ("name", po::value<std::string>(&name),                                           "Name of the component")
         ("memlib", po::value<std::string>(&library_ip),                                   "Library of memory IPs (YAML)")
         ("mode", po::value<std::string>(&mode)->default_value("opt"),                     "Execution mode (batch, opt)")
         ("opt-goal", po::value<std::string>(&alg_str)->default_value("AREA_HEURISTIC"),      opt_parameter_opt.c_str())
         ("disable-optimization", po::value<std::vector<std::string> >(&sharing_options),  "Disable specific optimizations (COLORING,ADDRESS_SPACE,INTERFACE)")
         ;

   po::options_description batch_desc("Batch-mode options");
   batch_desc.add_options()
         ("interfaces", po::value<std::string>(&interfaces),                               "List of memory interfaces")
         ("width", po::value<unsigned int>(&width),                                        "Array bitwidth")
         ("height", po::value<unsigned int>(&height),                                      "Array height (in words)")
         ;

   po::options_description opt_desc("Optimization-mode options");
   opt_desc.add_options()
         ("input-cgraph", po::value<std::string>(&input_cgraph),                           "Compatibility graph (YAML)")
         ("acc-config", po::value<std::string>(&acc_config),                               "Accelerator configuration (YAML)")
         ("acc-interface", po::value<std::string>(&acc_interface),                         "Accelerator interface (YAML)")
         ("scenario", po::value<std::string>(&scenario),                                   "Accelerator scenarios (YAML)")
         ("acc-list", po::value<std::string>(&multiacc_config),                            "List of accelerators' configurations (YAML)")
         ;

   po::options_description output_desc("Output options");
   output_desc.add_options()
         ("target-verilog-dir", po::value<std::string>(&out_verilog_dir)->default_value("./output"), "Output directory (Verilog files)")
         ("verilog-name", po::value<std::string>(&verilog_name),                             "Verilog filename (memory subsystem)")
         ("temp-dir", po::value<std::string>(&temp_dir)->default_value("./work"),            "Temporary directory")
         ("export", po::value<std::vector<std::string> >(&export_options),                   export_parameter_opt.c_str())
         ("generate-top-plm", po::value<bool>(&generate_top_plm)->default_value(false),      "Generate top module of memory subsystem")
         ("top-name", po::value<std::string>(&top_name),                                     "Name of top module")
         ("top-verilog-name", po::value<std::string>(&top_verilog_name),                     "Verilog filename (top module)")
         ("target-export-dir", po::value<std::string>(&out_export_dir)->default_value("./output"),  "Output directory (Export files)")
         ;

   po::options_description cmdline_options(" == Command-line parameters");
   cmdline_options.add(desc).add(input_desc).add(batch_desc).add(opt_desc).add(output_desc);

   po::options_description cfgfile_options(" == Config parameters");
   cfgfile_options.add(input_desc).add(batch_desc).add(opt_desc).add(output_desc);

   PRINT(std::endl);
   PRINT("              .:: Mnemosyne ::.              " << std::endl);
   PRINT(" Columbia University, Copyright (c) 2014-2017" << std::endl << std::endl);
   std::string revision = reinterpret_cast<const char*>(revision_hash);
   boost::replace_all(revision, "\n", "");
   std::string branch = reinterpret_cast<const char*>(branch_name);
   boost::replace_all(branch, "\n", "");
   PRINT("Revision: " + revision + "(" + branch + ")" << std::endl << std::endl);

   po::variables_map vm;
   try
   {
      po::store(po::parse_command_line(argc, argv, cmdline_options), vm);
      po::notify(vm);
      if (vm.count("config"))
      {
         std::ifstream file(config_file.c_str());
         po::store(po::parse_config_file(file, cfgfile_options), vm);
         file.close();
      }
      /// print help
      if (vm.count("help") or argc == 1)
      {
         std::cout << cmdline_options << std::endl;
         return 0;
      }
      po::notify(vm);

      if (!name.size())
         throw std::runtime_error("Component name not specified (--name option)");

      if (!boost::filesystem::exists(library_ip))
         throw std::runtime_error("Technology library file does not exist: \"" + library_ip + "\"");

      for (unsigned int i = 0; i < sharing_options.size(); i++)
      {
         if (sharing_options[i] == "ADDRESS_SPACE")
         {
            sharing &= !Allocation::ADDRESS_SPACE;
         }
         else if (sharing_options[i] == "INTERFACE")
         {
            sharing &= !Allocation::INTERFACE;
         }
         else if (sharing_options[i] == "COLORING")
         {
            sharing &= !Allocation::COLORING;
         }
         else
            throw std::runtime_error("Wrong --disable-optimization option (\"" + sharing_options[i] + "\")");
      }

      if (alg_str == "AREA_HEURISTIC")
         algorithm = Functor::AREA_HEURISTIC;
#ifdef EXPERIMENTAL
      else if (alg_str == "POWER_ILP")
         algorithm = Functor::POWER_ILP;
#endif
      else
         throw std::runtime_error("Wrong --opt-goal option (\"" + alg_str + "\")");

      if (mode == "opt")
      {
         if (!vm.count("acc-list") and !(vm.count("name") and vm.count("input-cgraph") and vm.count("acc-config")))
            throw std::runtime_error("Missing information for memory optimization. Specify --acc-xml or --name and --input-cgraph and --acc-config options.");

         if (multiacc_config.size())  // multi-accelerator configuration
         {
            if (!boost::filesystem::exists(multiacc_config))
               throw std::runtime_error("Accelerators' configuration file does not exist: \"" + multiacc_config + "\"");
         }
         else  // single-accelerator configuration
         {
            if (!boost::filesystem::exists(input_cgraph))
               throw "Compatibility graph file does not exist: \"" + input_cgraph + "\"";
            if (!boost::filesystem::exists(acc_config))
               throw "Accelerator's configuration file does not exist: \"" + acc_config + "\"";
            if (scenario.size() and !boost::filesystem::exists(scenario))
               throw std::runtime_error("Accelerator's scenario file does not exist: \"" + scenario + "\"");
         }

         if (!top_name.size())
            top_name = "top_" + name + "_rtl";
      }
      else if (mode == "batch")
      {
         if (!(vm.count("interfaces") and vm.count("width") and vm.count("height")))
            throw std::runtime_error("missing configuration for batch execution");
      }
      else
         throw std::runtime_error("Wrong --mode option (\"" + mode + "\")");

      if (!verilog_name.size())
         verilog_name = "mem_" + name + "_rtl.v";
      if (!top_verilog_name.size())
         top_verilog_name = "top_" + name + "_rtl.v";

      for (unsigned int i = 0; i < export_options.size(); i++)
      {
         export_type = ExportWriter::elaborate_option(export_type, export_options[i]);
      }

   }
   catch(std::exception& e)
   {
      std::cout << "Configuration Error: " << e.what() << std::endl << std::endl;
      return -1;
   }
   catch(...)
   {
      std::cout << "Unknown configuration error!" << std::endl << std::endl;
      return -1;
   }

   DEBUG(DBG_VERBOSE, verbosity, " == Mnemosyne Configuration  == " << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << "Design name: " << name << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << "Input:" << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << std::string(2, '-') + "Technology library: " << library_ip << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << std::string(2, '-') + "Optimization goal: " << alg_str << std::endl);

   if (mode == "opt")
   {
      if (scenario.size())
         DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << std::string(2, '-') + "Scenario file: " << scenario << std::endl);
      if (vm.count("acc-xml"))
      {
         DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << std::string(2, '-') + "Multi-Accelerator configuration file: " << multiacc_config << std::endl);
      }
      else
      {
         DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << std::string(2, '-') + "Compatibility graph: " << input_cgraph << std::endl);
         DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << std::string(2, '-') + "Accelerator configuration file: " << acc_config << std::endl);
         if (vm.count("acc-interface"))
         {
            DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << std::string(2, '-') + "Accelerator interface: " << acc_interface << std::endl);
         }
      }
   }
   else if (mode == "batch")
   {
      DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << std::string(2, '-') + "Width: " << width << std::endl);
      DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << std::string(2, '-') + "Height: " << height << std::endl);
      DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << std::string(2, '-') + "Interfaces: " << interfaces << std::endl);
   }

   DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << "Output:" << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << std::string(2, '-') + "Output directory (Verilog)  : " << out_verilog_dir << std::endl);
   if (export_type)
      DEBUG(DBG_VERBOSE, verbosity, std::setw(35) << std::left << std::string(2, '-') + "Output directory (Export)   : " << out_export_dir << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, " =====" << std::endl << std::endl);

   /// Starting the real execution...
   try
   {
#ifdef NDEBUG
      if (verbosity > DBG_MINIMUM)
         verbosity = DBG_MINIMUM;
#endif

      PRINT(" == Configuration == " << std::endl);
      ///create the output directories
      if (!boost::filesystem::exists(out_verilog_dir))
         boost::filesystem::create_directories(out_verilog_dir);
      if (export_type and !boost::filesystem::exists(out_export_dir))
         boost::filesystem::create_directories(out_export_dir);

      ///create the memory library
      MemoryLibraryPtr mem_library(new MemoryLibrary(verbosity));
      if (!mem_library->parse_config(library_ip))
         throw std::runtime_error("Error during library parsing");
      mem_library->printLibrary();
      PRINT("Memory library = " << mem_library->db.size() << " IPs" << std::endl);
      DEBUG(DBG_VERBOSE, verbosity, std::endl);

      ///create the set of array to optimize
      ComponentListPtr components(new ComponentList(verbosity));
      if (mode == "opt")
      {
         if (multiacc_config.size())
         {
            if (!components->parse_config(name, multiacc_config))
               throw std::runtime_error("Error during multi-acc configuration file parsing");
         }
         else
         {
            if (!components->parse_config(name, acc_config, input_cgraph, scenario))
               throw std::runtime_error("Error during configuration file parsing");
         }
      }
      else if (mode == "batch")
      {
         components->create_array(name, width, height, interfaces);
      }
      PRINT("Number of arrays = " << components->id_to_buffer.size() << std::endl);

      ///create the optimization functor
      FunctorPtr opt_functor = Functor::create(algorithm, verbosity, temp_dir + "/" + name, mem_library, components);

      ///create the optimization engine
      AllocationPtr alloc = AllocationPtr(new Allocation(verbosity, temp_dir + "/" + name, name, mem_library, components));
      PRINT(" =====" << std::endl << std::endl);

      /// memory optimization
      PRINT(" == Memory subsystem optimization == " << std::endl);
      if (mode == "batch")
      {
         alloc->set_compose_id(false);
      }
      alloc->perform_allocation(opt_functor, sharing, opt_functor->wrappers);
      PRINT(" =====" << std::endl << std::endl);

      /// wrapper generation (Verilog)
      PRINT(" == Memory subsystem generation (Verilog) == " << std::endl);
      VerilogWriterPtr f(new VerilogWriter(verbosity, out_verilog_dir, opt_functor));
      f->set_has_to_generate_top_plm(generate_top_plm);
      f->generate_wrappers(verilog_name);
      PRINT(" =====" << std::endl << std::endl);

      /// export wrapper IP
      ExportWriter::export_ip_descriptions(export_type, verbosity, out_export_dir, verilog_name, opt_functor);

      /// top module generation (Verilog)
      if (multiacc_config.size())
      {
         try
         {
            DEBUG(DBG_VERBOSE, verbosity, "\n** Parsing the accelerators' configuration file \"" + multiacc_config + "\"..." << std::endl);
            YAML::Node config = YAML::LoadFile(multiacc_config);
            YAML::Node accelerators = config["accelerator"];
            bool all_interfaces = true;
            std::map<std::string, std::string> acc_interfaces;
            std::vector<std::string> accelerator_list;
            for(const auto& a_it : accelerators)
            {
               std::string acc_name = a_it["name"].as<std::string>();
               accelerator_list.push_back(acc_name);
               if (a_it["acc_interface"])
               {
                  std::string acc_interface = a_it["acc_interface"].as<std::string>();
                  acc_interfaces[acc_name] = acc_interface;
               }
               else
                  all_interfaces = false;
            }
            if (all_interfaces)
            {
               PRINT(" == Top module generation (Verilog) -- " << top_name << " == " << std::endl);
               VerilogWriterPtr gen(new VerilogWriter(verbosity, out_verilog_dir, opt_functor));
               if (!gen->generate_top_entity(top_name, top_verilog_name, acc_interfaces, accelerator_list))
                  throw std::runtime_error("Error during top module generation");
               PRINT(" =====" << std::endl << std::endl);
            }
         }
         catch(...)
         {
            std::cout << "ERROR!" << std::endl;
            return -1;
         }
      }
      else
      {
         if (acc_interface.size())
         {
            ///top generation
            PRINT(" == Top module generation (Verilog) == " << std::endl);
            VerilogWriterPtr gen(new VerilogWriter(verbosity, out_verilog_dir, opt_functor));
            if (!gen->generate_top_entity(name, top_name, top_verilog_name, acc_interface))
               throw std::runtime_error("Error during top module generation");
            PRINT(" =====" << std::endl << std::endl);
         }
      }
   }
   catch(std::exception& e)
   {
      std::cout << "Execution Error: " << e.what() << std::endl << std::endl;
      PRINT("Send an email with the error description to: Christian Pilato <christian.pilato@gmail.com>" << std::endl << std::endl);
      return -1;
   }
   catch(const std::string& str)
   {
      std::cout << "Execution Error: " << str << std::endl << std::endl;
      PRINT("Send an email with the error description to: Christian Pilato <christian.pilato@gmail.com>" << std::endl << std::endl);
      return -1;
   }
   catch(...)
   {
      std::cout << "Unknown error!" << std::endl;
      PRINT("Send an email with the error description to: Christian Pilato <christian.pilato@gmail.com>" << std::endl << std::endl);
      return -1;
   }

   return 0;
}
