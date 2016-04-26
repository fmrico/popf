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

#include "globals.h"

#ifdef ENABLE_DEBUGGING_HOOKS

#include <FlexLexer.h>

#include <fstream>
using std::ifstream;

#include <ptree.h>
#include <instantiation.h>
using namespace Inst;

namespace VAL
{
    extern yyFlexLexer* yfl;
};

extern int yyparse();
extern int yydebug;

using namespace VAL;

#endif

namespace Planner
{

int Globals::writeableVerbosity = 0;

const int & Globals::globalVerbosity = writeableVerbosity;

bool Globals::paranoidScheduling = false;
bool Globals::profileScheduling = false;

#ifdef ENABLE_DEBUGGING_HOOKS

vector<bool> Globals::actionHasToBeKept;
const char * Globals::planFilename = 0;

void Globals::markThatActionsInPlanHaveToBeKept()
{    
    actionHasToBeKept.resize(instantiatedOp::howMany(), false);
 
    if (!planFilename) return;
    
    ifstream * const current_in_stream = new ifstream(planFilename);
    if (!current_in_stream->good()) {
        cout << "Exiting: could not open plan file " << planFilename << "\n";
        exit(1);
    }

    VAL::yfl = new yyFlexLexer(current_in_stream, &cout);
    yyparse();

    VAL::plan * const the_plan = dynamic_cast<VAL::plan*>(top_thing);

    delete VAL::yfl;
    delete current_in_stream;



    if (!the_plan) {
        cout << "Exiting: failed to load plan " << planFilename << "\n";
        exit(1);
    };

    if (!theTC->typecheckPlan(the_plan)) {
        cout << "Exiting: error when type-checking plan " << planFilename << "\n";
        exit(1);
    }

    pc_list<plan_step*>::const_iterator planItr = the_plan->begin();
    const pc_list<plan_step*>::const_iterator planEnd = the_plan->end();

    for (int idebug = 0, i = 0; planItr != planEnd; ++planItr, ++i, ++idebug) {
        plan_step* const currStep = *planItr;

        instantiatedOp * const currOp = instantiatedOp::findInstOp(currStep->op_sym, currStep->params->begin(), currStep->params->end());
        if (!currOp) {
            cout << "Exiting: step " << idebug << " in the input plan uses an action that the instantiation code did not create.\n";
            cout << "Use VALfiles/testing/checkplanstepsexist to find out why this is the case\n";
            exit(1);
        }
        const int ID = currOp->getID();
        
        actionHasToBeKept[ID] = true;
        
        cout << "Marking that " << *currOp << " (" << ID << ") must not be eliminated by the preprocessor\n";
    }

}

void Globals::eliminatedAction(const int & i, const char * synopsis)
{
    if (!planFilename) return;
    
    if (actionHasToBeKept[i]) {
        cout << "Error in preprocessor.  Pruned operator " << i << ", but should not have done.  Reason given was:\n";
        cout << synopsis << std::endl;
        exit(1);
    } else {
        cout << "Eliminated " << i << ": " << synopsis << std::endl;
    }
}

#endif

}
