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
 * @file   AreaHeuristic.hpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Class for memory optimization with area minimization
 *
 */
#ifndef _AREA_HEURISTIC_HPP_
#define _AREA_HEURISTIC_HPP_

#include "Functor.hpp"

/**
 * @brief The AreaHeuristic class is used for area minimization
 */
class AreaHeuristic : public Functor
{

   public:

      /**
       * @brief Constructor
       * @param verbosity is the verbosity level of the class
       * @param out_dir is the path of the output directory
       * @param mem_library is the library of available memory IPs
       * @param components is the class representing the components to be optimized
       */
      AreaHeuristic(unsigned int verbosity, const std::string& out_dir, const MemoryLibraryPtr mem_library, const ComponentListPtr components);

      /**
       * @brief Destructor
       */
      virtual ~AreaHeuristic();

      /**
       * @brief Determine the cost of the given clique
       * @param clique is the set of nodes composing the clique
       * @return a high-level microarchitecture of the generated clique
       */
      PartitionPtr compute_clique_cost(const std::set<ComponentList::node_t>& clique);

};
///refcount definition
typedef boost::shared_ptr<AreaHeuristic> AreaHeuristicPtr;

#endif
