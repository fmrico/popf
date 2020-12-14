/************************************************************************
 * Copyright 2010, Strathclyde Planning Group,
 * Department of Computer and Information Sciences,
 * University of Strathclyde, Glasgow, UK
 * http://planning.cis.strath.ac.uk/
 *
 * Andrew Coles, Amanda Coles - Code for POPF
 * Maria Fox, Richard Howey and Derek Long - Code from VAL
 * Stephen Cresswell - PDDL Parser
 *
 * This file is part of the planner POPF.
 *
 * POPF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * POPF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with POPF.  If not, see <http://www.gnu.org/licenses/>.
 *
 ************************************************************************/

#ifndef __GLOBALS
#define __GLOBALS

#ifndef NDEBUG
#define ENABLE_DEBUGGING_HOOKS 1
#endif

#include <instantiation.h>

using Inst::Literal;

namespace Planner
{

struct LiteralLT {

    bool operator()(const Literal* const & a, const Literal* const & b) const {
        return (a->getID() < b->getID());
    }

};

typedef map<int, bool> StepAndEpsilon;

typedef set<Literal*, LiteralLT> LiteralSet;

/** Class containing global variables */
class Globals
{

public:

    /**
     *  Global verbosity flag.  Is a bit-mask, where each bit corresponds to whether
     *  debugging output should be provided for a certain part of the code.
     *  
     *  - 1: provide basic information about how search is progressing
     *  - 2: when expanding a state, print the plan that reached that state
     *  - 16: provide (lots of) information about RPG construction and action grounding
     *  - 4096: provide information about the STP constraints used within the incremental Bellman-Ford implementation
     *  - 65536: print out a list of all ground action names, fact names, and variable names
     *  - 131072: print out information about the action pruning performed in the preprocessor
     *  - 1048576: provide information about the ordering constraints added to the partial order when applying an action
     */
    static const int & globalVerbosity;
    
    static int writeableVerbosity;
    
    /**
     *  Debugging flag.  If set to true (pass the <code>-D</code> flag at the command line), the plan is scheduled using
     *  three techniques, at every state: the LP, the incremental Bellman-Ford, and Floyd-Walshall.  Additionally, the
     *  latter is ran each time an edge is added to the incremental Bellman-Ford algorithm to check the incremental updates
     *  are correct.
     *
     *  @see LPScheduler
     */
    static bool paranoidScheduling;
    
    /**
    *  Debugging flag.  If set to true (pass the <code>-P</code> flag at the command line), the plan is scheduled using
    *  both the LP and incremental Bellman-Ford, without the two being integrated.  The profile data produced by
    *  gprof can then be used to ascertain the comparative performance of the two approaches:
    *  - Time taken for the STP is <code>LPScheduler::prime()</code> + <code>ParentData::spawnChildData()</code>
    *  - Time taken for the LP is <code>LPScheduler::LPScheduler()</code> - <code>ParentData::spawnChildData()</code>
    *    (as the latter is called from within the LP scheduler constructor, but needs to be discounted as in profiling
    *     mode the two are not integrated.)
    *
    *  @see LPScheduler
    */
    static bool profileScheduling;
    
    
    #ifdef ENABLE_DEBUGGING_HOOKS

    /**
     *  A vector of which actions definitely must be kept, i.e. not pruned in preprocessing.  Is present for
     *  debugging purposes.  To populate the vector:
     *  - use the <code>-H</code> command line flag
     *  - provide a plan filename after the domain and problem filenames
     *  Then, the <code>instantiatedOp</code>s used in the plan will have their entries in this vector
     *  set to true.
     */
    static vector<bool> actionHasToBeKept;
    
    /**
     *  An exemplar plan for the current problem, to be read in for debugging purposes.
     *  @see actionHasToBeKept
     */
    static const char * planFilename;
    
    /**
     *  Record that the actions in the exemplar plan <code>planFilename</code> must not be pruned
     *  in the preprocessor.
     *  @see actionHasToBeKept
     */
    static void markThatActionsInPlanHaveToBeKept();
    
    /**
     *  Note that the action with the specified ID has been pruned, due to the given reason.  Will
     *  lead to an assertion failure if the action must not be pruned.
     *
     *  @param i         The action index that has been eliminated
     *  @param synopsis  A short reason for why the action was eliminated.  This is printed if the pruning is known to be in error.
     *
     *  @see actionHasToBeKept
     */
    static void eliminatedAction(const int & i, const char * synopsis);
    #endif
};

};

#endif
