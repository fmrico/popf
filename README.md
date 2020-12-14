# popf

[![GitHub Action
Status](https://github.com/fmrico/popf/workflows/POPF/badge.svg)](https://github.com/fmrico/)

The POPF planner from KCL planning group with some modifications to make it work with "modern" compilers...
=== POPF1.1 ===

This archive contains the source release of POPF, as described in the 2010 ICAPS paper, plus a few bug fixes.  The directory 'src/popf' contains the sources of the planner itself, and the directory 'src/VALfiles' contains PDDL parsing and action instantiation code, taken from VAL (the PDDL validator).  Contact details for the authors are available from the Strathclyde Planning Group Website:

http://planning.cis.strath.ac.uk/

In the first instance, for bug reports or technical discussions, contact Andrew Coles (firstname.lastname@cis.strath.ac.uk).

== Compiling POPF ==

A precompiled, statically linked binary of POPF for Linux x86 is available to download.  For those wishing to compile POPF themselves, carry on reading, otherwise, skip to the 'Running POPF' section.

Build prerequisites:
- cmake ( http://www.cmake.org/ )
- the CBC mixed integer programming solver ( https://projects.coin-or.org/Cbc/ )
- perl, bison and flex to build the parser

These are packaged with most linux distributions - on Ubuntu/Debian, the following should suffice:

sudo apt-get install cmake coinor-libcbc-dev coinor-libclp-dev coinor-libcoinutils-dev bison flex

Once installed, open a terminal and type the following:

cd <directory containing this file>
cd <build>
./buildscript
make

Assuming all goes well, the file 'build/popf/popf-clp' is then a the binary of popf.

== Running POPF ==

To run POPF (using the new heuristic described in the ICAPS paper), type:

popf-clp domain.pddl problem.pddl

To disable the new heuristic, add the -c option, i.e. type:

popf-clp -c domain.pddl problem.pddl

To read a solution plan in (to then lift a partial order from and print the resulting plan) run:

popf-clp -r domain.pddl problem.pddl plan.soln

If you are using POPF in a paper, to print the appropriate BibTeX reference run:

popf-clp -citation

For full details of the other command-line options, run popf without any arguments.

== Known Issues ==

There used to be some (known) bugs affecting the enhanced heuristic.  Those should be fixed now; but, if it crashes or exhibits other strange behaviour, try running with the -c option, as described above.  If you have a particularly small domain and problem file pair for which passing -c to POPF fixes the bug, then email Andrew Coles - such files are useful for debugging.

Otherwise, if -c doesn't fix the problem, and POPF crashes, then you may want to compile with debugging information.  Change build/buildscript, and follow the instructions given in the comments (instead of passing RelWithDebInfo to cmake, pass Debug).  Then, run the newly built popf binary, and send the output it produces along with your domain and problem file to Andrew Coles (firstname.lastname@cis.strath.ac.uk).

== Licence ==

POPF is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

POPF is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

For details of the license, see the file LICENCE in this directory.

