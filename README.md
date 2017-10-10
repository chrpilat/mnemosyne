# Mnemosyne

Mnemosyne is a prototype CAD tool for memory optimization developed by the System-Level Design Group at Columbia University.

Welcome to Mnemosyne!

This is a tool for memory optimization at the hardware level. With Mnemosyne, designers
can create and evaluate multi-bank memories for heterogeneous architectures, including
but not limited to private local memories, specialized caches, and memory intellectual
property (IP) blocks.

If you use the code, we would appreciate a citation to:

> System-Level Optimization of Accelerator Local Memory for Heterogeneous Systems-on-Chip.
> Christian Pilato, Paolo Mantovani, Giuseppe Di Guglielmo, and Luca P. Carloni.
> IEEE Transactions on CAD of Integrated Circuits and Systems, 36:3, pages 435-448 (March 2017).

For any questions/concerns, please email [Christian Pilato](christian.pilato@usi.ch).

## Requirements: ##

### Mnemosyne dependencies ####

To build Mnemosyne, you will need to satisfy the following dependencies:
1. libxml2-devel
2. glib2-devel
3. libsigc++20-devel
4. glibmm24-devel
5. libxml++-devel
6. glpk-utils
7. glpk-devel

### System configuration ####

Mnemosyne requires that `/bin/sh` points to `/bin/bash` instead of `/bin/dash`, which
is a common configuration for Ubuntu systems. This problem can be solved as follows:

  ```
  sudo rm -rf /bin/sh
  sudo ln -s /bin/bash /bin/sh
  ```

## Installation ##

### Setting up the source code ###

1. Clone Mnemosyne.

  ```
  git clone https://github.com/chrpilat/mnemosyne
  ```

2. Install [Xilinx Vivado Design Suite](https://www.xilinx.com/support/download.html) (optional)

3. Download and uncompress the [Synopsys SAED 32nm SRAM library](https://www.synopsys.com/community/university-program/teaching-resources.html) (optional)

### Building Mnemosyne ###

Type the following commands to build and install the sources:

  ```
  mkdir obj && cd obj
  cmake -DCMAKE_INSTALL_PREFIX=<install_prefix> ..
  make && make install
  ```

## Running Mnemosyne ##

Mnemosyne can be run in two ways: batch and optimization.

In the batch mode, only one data structure / memory IP can be created. Mnemosyne
will simply create a multi-port memory block based on the given parameters (e.g., width,
height, number of read/write ports).
In the optimization mode, multiple data structures can be concurrently optimized, searching
for sharing opportunities. Hence, it is also necessary to specify the compatibility graph
as input.

Mnemosyne uses the `mode` command-line parameter to distinguish between 
these two modes (`batch` and `opt`, respectively).

Additional information can be found in the [Wiki](https://github.com/chrpilat/mnemosyne/wiki) pages.

-----------------------

### Contact ###

Christian Pilato (USI Lugano): christian.pilato@usi.ch

Luca P. Carloni (Columbia University): luca@cs.columbia.edu
