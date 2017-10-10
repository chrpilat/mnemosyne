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
 * @file   Partition.hpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Class to describe a set of arrays to be stored in the memory
 *
 */
#ifndef _PARTITION_HPP_
#define _PARTITION_HPP_

#include "utils.hpp"

FORWARD_DECL(Array);
FORWARD_DECL(Memory);
FORWARD_DECL(Tag);

struct Partition
{
   ///map between buffer and its offset
   std::map<ArrayPtr, unsigned int> buffer_offset;
   ///map <buffer, <parallel_block, bank_id> >
   std::map<ArrayPtr, std::map<unsigned int, std::vector<unsigned int> > > buffer_configuration;
   MemoryPtr memory;
   unsigned int num_banks;
   std::map<ArrayPtr, TagPtr> buffer_tag;

   double cost;
};
typedef boost::shared_ptr<Partition> PartitionPtr;

#endif
