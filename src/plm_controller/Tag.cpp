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
 * @file   Tag.cpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Methods used to describe an array tag.
 *
 */
#include "Tag.hpp"

#include "Partition.hpp"
#include "Architecture.hpp"
#include "PBlock.hpp"
#include "Memory.hpp"

#include "Array.hpp"

Tag::Tag(const PartitionPtr partition, const ArrayPtr buff, unsigned int verbosity)
{
   buff_size = ceil((double)log(buff->height) / (double)log(2));
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::left << "  Height " << " = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(7) << std::right << buff->height << " --> bits = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(5) << std::right << buff_size);
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::left << " (logical address)");
   DEBUG(DBG_VERBOSE, verbosity, std::endl);

   const MemoryPtr bank = partition->memory;
   bank_size = ceil((double)log(bank->height) / (double)log(2));
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::left << "  Bank Size " << " = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(7) << std::right << bank->height << " --> bits = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(5) << std::right << bank_size);
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::left << " (physical address)" << std::endl);

   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::left << "  Offset " << " = ");
   unsigned int offset_size = ceil((double)log(bank->height) / (double)log(2));
   DEBUG(DBG_VERBOSE, verbosity, std::setw(7) << std::right << partition->buffer_offset[buff] << " --> bits = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(5) << std::right << offset_size << std::endl);

   p_blocks = buff->p_blocks;
   ptag_size = 0;
   if (buff->data_partitioning)
      ptag_size = ceil((double)log(buff->p_blocks / buff->split) / (double)log(2));
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::left << "  Split  " << " = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(7) << std::right << buff->split << std::endl);
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::left << "  P Blocks  " << " = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(7) << std::right << p_blocks << " --> bits = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(5) << std::right << ptag_size << " (ptag_size)" << std::endl);

   unsigned int block_size = ceil((double)log(buff->pblocks[0]->height) / (double)log(2));
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::left << "  P Block Size" << " = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(7) << std::right << buff->pblocks[0]->height << " --> bits = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(5) << std::right << block_size << " (block_size)" << std::endl);

   mwtag_size = ceil((double)log(buff->w_block_size / buff->split) / (double)log(2));
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::left << "  WB Size  " << " = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(7) << std::right << buff->w_block_size << " --> bits = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(5) << std::right << mwtag_size << " (mwtag_size)" << std::endl);

   mtag_size = ceil((double)log(buff->merge) / (double)log(2));
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::left << "  Merge  " << " = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(7) << std::right << buff->merge << " --> bits = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(5) << std::right << mtag_size << " (mtag_size)" << std::endl);

   unsigned int s_blocks = partition->buffer_configuration[buff][0].size();
   stag_size = ceil((double)log(s_blocks) / (double)log(2));
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::left << "  S Blocks  " << " = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(7) << std::right << s_blocks << " --> bits = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(5) << std::right << stag_size << " (stag_size)" << std::endl);

   padding = 0;
   if (bank_size > buff_size)
   {
      padding = bank_size - buff_size - mtag_size;
      if (buff->data_partitioning)
         padding -= ptag_size;
   }
   DEBUG(DBG_VERBOSE, verbosity, std::setw(15) << std::left << "  Padding  " << " = ");
   DEBUG(DBG_VERBOSE, verbosity, std::setw(7) << std::right << padding << std::endl);

   ///sanity checks
   if (padding)
   {
      if (buff->data_partitioning)
         assert(bank_size == (buff_size + ptag_size + mtag_size + mwtag_size) + padding);
      else
         assert(bank_size == (buff_size + mtag_size) + padding);
   }
}

