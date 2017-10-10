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
 * @file   Scenario.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Methods used to describe a scenario in the accelerator.
 *
 */
#include "Scenario.hpp"

Scenario::Scenario() :
   frequency(0)
{

}

void Scenario::print(std::ostream &os) const
{
   os << "Name = " << std::setw(15) << std::left << name;
   os << " -- Frequency = " << std::setw(10) << std::right << frequency;
   os << std::endl;
   os << std::setw(20) << std::left << "Array";
   os << std::setw(10) << std::right << "Height";
   os << std::setw(10) << std::right << "Width";
   os << std::endl;
   os << std::string(40, '-') << std::endl;
   for(const auto &b_it : buffers)
   {
      os << std::setw(20) << std::left << (b_it.first);
      os << std::setw(10) << std::right << std::get<0>(b_it.second);
      os << std::setw(10) << std::right << std::get<1>(b_it.second);
      os << std::endl;
   }
   os << std::string(80, '+') << std::endl;
}

bool Scenario::parse_config(const YAML::Node& node)
{
   name = node["id"].as<std::string>();
   frequency = node["frequency"].as<double>();
   YAML::Node buffer_list = node["buffer"];
   for(const auto &b_it : buffer_list)
   {
      std::string name = b_it["name"].as<std::string>();
      unsigned int height = b_it["height"].as<unsigned int>();
      unsigned int width = b_it["width"].as<unsigned int>();

      ScenarioArray buffer(height, width);
      buffers[name] = buffer;
   }
   return true;
}

