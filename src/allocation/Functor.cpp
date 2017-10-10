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
 * * @file   Functor.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Implementation of the methods for the optimization functor.
 *
 */
#include "Functor.hpp"

#include "AreaHeuristic.hpp"

#include "Architecture.hpp"
#include "Memory.hpp"

Functor::Functor(unsigned int _verbosity, const std::string& _out_dir, const MemoryLibraryPtr _mem_library, const ComponentListPtr _components) :
   verbosity(_verbosity),
   mem_library(_mem_library),
   components(_components),
   out_dir(_out_dir)
{

}

Functor::~Functor()
{

}

const char* Functor::TypeToString(unsigned int v)
{
    switch (v)
    {
        case AREA_HEURISTIC:  return "AREA_HEURISTIC";
        default:              return "[Unknown optimization goal]";
    }
}

const char* Functor::TypeToLongDescription(unsigned int v)
{
    switch (v)
    {
        case AREA_HEURISTIC:  return "Area Minimization";
        default:              return "[Unknown export type]";
    }
}

FunctorPtr Functor::create(Functor::optimization_t opt_goal, unsigned int verbosity, const std::string &out_dir, const MemoryLibraryPtr mem_library, const ComponentListPtr components)
{
   switch (opt_goal)
   {
      case AREA_HEURISTIC:
         return FunctorPtr(new AreaHeuristic(verbosity, out_dir, mem_library, components));
      default:
         throw std::runtime_error("Unsupported optimization goal");
   }
   return FunctorPtr();
}

std::string Functor::get_string_list()
{
   std::string list;
   for(unsigned int i = 0; i < _LAST_OPT; i++)
   {
      if (list.size()) list += "\n";
      list += "  " + std::string(TypeToString(i)) + ": \t" + std::string(TypeToLongDescription(i));
   }
   return list;
}
