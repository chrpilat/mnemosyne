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
 * @file   Component.hpp
 * @author Christian Pilato <pilato.christian@gmail.com>
 *
 * @brief  Class to describe a component with memory access
 *
 */
#ifndef _COMPONENT_HPP_
#define _COMPONENT_HPP_

#include "utils.hpp"
FORWARD_DECL(Array);
FORWARD_DECL(ArrayList);
FORWARD_DECL(ComponentList);
FORWARD_DECL(MemoryWrapper);
#include "UGraph.hpp"

/**
 * @brief Component Declaration
 */
struct Component
{
   //! Identifier of the component.
   const std::string name;

   std::string clock_name;
   std::string reset_name;
   std::string conf_done_name;
   std::string acc_done_name;
   std::set<std::string> dmain_prefix;
   std::set<std::string> dmaout_prefix;
   std::set<std::string> rdreq_prefix;
   std::set<std::string> wrreq_prefix;
   std::set<std::string> read_interfaces;
   std::set<std::string> write_interfaces;
   std::map<std::string, std::string> darkmem_to_buffer;
   std::map<std::string, std::set<std::string> > buffer_to_darkmem;

   /**
    * @brief Constructor
    */
   Component(const std::string& name);

   /**
    * @brief Print method
    * @param os is the output stream
    */
   void print(std::ostream& os) const;

   /**
    * @brief Overloaded operator to support print
    * @param os is the output stream
    * @param b is the component to be printed
    * @return is the returned stream
    */
   friend std::ostream& operator<<(std::ostream& os, const Component& b)
   {
      b.print(os);
      return os;
   }

   void parse_interface(const YAML::Node& interface, const std::map<std::string, MemoryWrapperPtr> &buffer_to_wrapper);

   std::string get_rdreq_prefix() const;
   std::string get_wrreq_prefix() const;
   std::string get_dmain_prefix() const;
   std::string get_dmaout_prefix() const;

};
///refcount definition
typedef boost::shared_ptr<Component> ComponentPtr;

struct ComponentList
{
    ///verbosity level of the class
    unsigned int verbosity;

    ///name of the top component
    std::string top_name;

    ///archive of components
    std::map<std::string, ComponentPtr> list;

    ///list of buffers to be stored
    ArrayListPtr buffers;

    typedef std::tuple<UGraphPtr, UNode> node_t;
    std::map<std::string, ArrayPtr> id_to_buffer;
    std::map<std::string, node_t> id_to_node;
    std::map<UGraphPtr, std::map<UNode, ArrayPtr> > node_to_buffer;
    std::map<UGraphPtr, std::string> graph_to_acc_name;
    std::vector<node_t> node_list;
    std::map<node_t, std::set<node_t> > comp_list;

    /**
    * @brief Component
    * @param verbosity is the verbosity level of the class
    */
    ComponentList(unsigned int verbosity);

    /**
     * @brief Create single array
     * @param name is the id of the array
     * @param width is the bitwidth
     * @param height is the number of words
     * @param interfaces is the list of interfaces
     */
    void create_array(const std::string& name, const unsigned int width, unsigned int height, const std::string& interfaces);

    /**
     * @brief Parse the multi-component definitions
     * @param name is the name of the top component
     * @param multiacc_config is the path to the file to be parsed
     */
    bool parse_config(const std::string& name, const std::string& multiacc_config);

    /**
     * @brief Parse a single component definition
     * @param name is the name of the component
     * @param acc_config is the path to the configuration file to be parsed
     * @param input_cgraph is the path to the compatibility graph file to be parsed
     */
    bool parse_config(const std::string& name, const std::string& acc_config, const std::string& input_cgraph, const std::string& scenario_config);

    /**
     * @brief Prepare buffer data structures for the given accelerator
     * @param name is the name of the accelerator
     */
    void bufferLoad(const std::string& name);

    /**
     * @brief Parse the file describing the compatibilities
     * @param name is the name of the current component to be analyzed
     * @param input_cgraph is the file describing the compatibilities
     */
    void parse_accelerator_config(const std::string& name, const std::string& input_cgraph);

    /**
     * @brief Get a string-based representation of the given clique
     * @param clique is the set of nodes composing the clique
     * @return the string representing the clique
     */
    std::string get_clique_string(const std::set<node_t>& clique);

};
///refcount definition
typedef boost::shared_ptr<ComponentList> ComponentListPtr;

#endif
