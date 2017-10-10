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
 * * @file   Functor.hpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Class to represent an optimization functor.
 *
 */
#ifndef _FUNCTOR_HPP_
#define _FUNCTOR_HPP_

#include "utils.hpp"

#include "Component.hpp"
FORWARD_DECL(Functor);
FORWARD_DECL(MemoryWrapper);
FORWARD_DECL(MemoryLibrary);
FORWARD_DECL(Partition);
FORWARD_DECL(ComponentList);

/**
 * @brief The Functor class is used to specify an optimization functor
 */
class Functor
{
   public:

      ///available optimization criteria
      typedef enum
      {
         AREA_HEURISTIC,
         _LAST_OPT  //placeholder: do not modify or remove!
      } optimization_t;

      ///list of generated PLM controllers
      std::map<std::string, MemoryWrapperPtr> wrappers;

      ///top component name;
      std::string accelerator_name;

      ///library of available memory IPs
      const MemoryLibraryPtr mem_library;

      ///class representing the components to be optimized
      const ComponentListPtr components;

   public:

      /**
       * @brief Constructor
       * @param verbosity is the verbosity level of the class
       * @param out_dir is the path of the output directory
       * @param mem_library is the library of available memory IPs
       * @param components is the class representing the components to be optimized
       */
      Functor(unsigned int verbosity, const std::string& out_dir, const MemoryLibraryPtr mem_library, const ComponentListPtr components);

      /**
       * @brief Destructor
       */
      virtual ~Functor();

      /**
       * @brief Return the name of the optimization goal
       */
      static const char* TypeToString(unsigned int v);

      /**
       * @brief Return the description of the optimization goal
       */
      static const char* TypeToLongDescription(unsigned int v);

      /**
       * @brief Return the list of options in a text-based form
       * @retun the list of available optimizations
       */
      static std::string get_string_list();

      /**
       * @brief Factory method to create a Functor class based on the optimization method
       * @param opt is the optimization goal (optimization_t enum)
       * @param verbosity is the verbosity level of the class
       * @param out_dir is the path of the output directory
       * @param mem_library is the library of available memory IPs
       * @param components is the class representing the components to be optimized
       * @return
       */
      static FunctorPtr create(optimization_t opt, unsigned int verbosity, const std::string& out_dir,
                               const MemoryLibraryPtr mem_library, const ComponentListPtr components);

      /**
       * @brief Determine the cost of the given clique
       * @param clique is the set of nodes composing the clique
       * @return a high-level microarchitecture of the generated clique
       */
      virtual PartitionPtr compute_clique_cost(const std::set<ComponentList::node_t>& clique) = 0;

   protected:

      ///verbosity level of the class
      unsigned int verbosity;
      ///path of the output directory
      const std::string out_dir;
};
///refcount definition
typedef boost::shared_ptr<Functor> FunctorPtr;

#endif
