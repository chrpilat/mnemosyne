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
 * @file   utils.hpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Wrapper header for all utility files.
 *
 */
#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include "debug.hpp"
#include "time_utils.hpp"
#include "shared_pointers.hpp"

/// BOOST related includes
#include <boost/algorithm/string.hpp>
#include "boost/filesystem.hpp"
#include "boost/foreach.hpp"
#include <boost/graph/graphml.hpp>
#include "boost/lexical_cast.hpp"
#include "boost/program_options.hpp"
#include <boost/tokenizer.hpp>

/// XMLPP includes
#include "libxml++/libxml++.h"

/// YAML includes
#include "yaml-cpp/yaml.h"

#include <map>
#include <vector>
#include <tuple>

/// System includes
#include <iostream>
#include <iostream>
#include <fstream>
#include <exception>

#endif
