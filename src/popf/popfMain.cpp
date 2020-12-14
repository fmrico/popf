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

#include <unistd.h> //  This works  DRJ
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include "ptree.h"
#include <assert.h>
#include <FlexLexer.h>
#include "instantiation.h"
#include "SimpleEval.h"
#include "DebugWriteController.h"
#include "typecheck.h"
#include "TIM.h"
#include "FuncAnalysis.h"

//#include "graphconstruct.h"
#include "RPGBuilder.h"
#include "FFSolver.h"
#include "globals.h"
#include "totalordertransformer.h"
#include "partialordertransformer.h"
#include "lpscheduler.h"

#include <sys/times.h>
#include <sys/time.h>
#include <sys/unistd.h>

#include <sstream>

using std::ifstream;
using std::cerr;
using std::endl;
using std::ostringstream;

using namespace TIM;
using namespace Inst;
using namespace VAL;
using namespace Planner;

namespace VAL
{
bool ContinueAnyway;
bool ErrorReport;
bool InvariantWarnings;
bool LaTeX;
bool makespanDefault;
};

void usage(char * argv[])
{
    cout << "POPF, as ICAPS 2010, plus bug fixes\n";
    cout << "By releasing this code we imply no warranty as to its reliability\n";
    cout << "and its use is entirely at your own risk.\n\n";
    cout << "Usage: " << argv[0] << " [OPTIONS] domainfile problemfile [planfile, if -r specified]\n\n";
    cout << "Options are: \n\n";
    cout << "\t" << "-citation" << "\t" << "Display citation to relevant conference paper (ICAPS 2010);\n";
    cout << "\t" << "-b" << "\t\t" << "Disable best-first search - if EHC fails, abort;\n";
    cout << "\t" << "-E" << "\t\t" << "Skip EHC: go straight to best-first search;\n";
    cout << "\t" << "-e" << "\t\t" << "Use standard EHC instead of steepest descent;\n";
    cout << "\t" << "-h" << "\t\t" << "Disable helpful-action pruning;\n";
    cout << "\t" << "-k" << "\t\t" << "Disable compression-safe action detection;\n";
    cout << "\t" << "-c" << "\t\t" << "Disable the tie-breaking in RPG that favour actions that slot into the partial order earlier;\n";
    cout << "\t" << "-S" << "\t\t" << "Sort initial layer facts in RPG by availability order (only use if using -c);\n";
    cout << "\t" << "-m" << "\t\t" << "Disable the tie-breaking in search that favours plans with shorter makespans;\n";
    cout << "\t" << "-F" << "\t\t" << "Full FF helpful actions (rather than just those in the RP applicable in the current state);\n";
    cout << "\t" << "-d" << "\t\t" << "Branch ordering so actions violating fewer other actions' preconditions come first;\n";
    cout << "\t" << "-A" << "\t\t" << "Try to use better actions in the heuristic, using h-add costs;\n";
    cout << "\t" << "-r" << "\t\t" << "Read in a plan instead of planning;\n";
    cout << "\t" << "-I" << "\t\t" << "Disable the use of an STP in cases where it is sufficient;\n";
    cout << "\t" << "-T" << "\t\t" << "Rather than building a partial order, build a total-order;\n";
    cout << "\t" << "-v<n>" << "\t\t" << "Verbose to degree n (n defaults to 1 if not specified).\n";
    cout << "\t" << "-L<n>" << "\t\t" << "LP verbose to degree n (n defaults to 1 if not specified).\n";
};

list<FFEvent> * readPlan(char* filename);

string threeDP(double d)
{
    ostringstream toReturn;

    d *= 1000;

    int asInt = d;

    d -= asInt;
    if (d >= 0.5) {
        asInt += 1;
    }

    int fractionalPart = asInt % 1000;

    toReturn << asInt / 1000 << ".";

    if (fractionalPart < 100) {
        toReturn << "0";
    }
    if (fractionalPart < 10) {
        toReturn << "0";
    }

    toReturn << asInt % 1000;

    return toReturn.str();
}

int main(int argc, char * argv[])
{

    FAverbose = false;

    int argcount = 1;

    FF::steepestDescent = false;
    FF::incrementalExpansion = false;
    FF::invariantRPG = false;
    FF::timeWAStar = false;
    LPScheduler::hybridBFLP = true;

    bool benchmark = false;
    bool readInAPlan = false;
    bool totalOrder = false;
    bool postHocTotalOrder = false;
    bool debugPreprocessing = false;
    bool postHocScheduleToMetric = false;

    while (argcount < argc && argv[argcount][0] == '-') {

        string remainder(&(argv[argcount][1]));
        if (remainder == "citation") {

            cout << "Citation for the paper describing POPF:\n\n";
            cout << "@CONFERENCE{colespopficaps,\n";
            cout << "\tauthor = \"A. J. Coles and A. I. Coles and M. Fox and D. Long\",\n";
            cout << "\ttitle = \"Forward-Chaining Partial-Order Planning\",\n";
            cout << "\tbooktitle = \"Twentieth International Conference on Automated Planning and Scheduling (ICAPS 10)\",\n";
            cout << "\tyear = \"2010\",\n";
            cout << "\tpublisher = \"AAAI Press\",\n";
            cout << "\tmonth = \"May\"\n";
            cout << "}\n\n";

            cout << "--------------------------------------------------------------------------------\n\n";

        } else {

            switch (argv[argcount][1]) {
            case '2': {
                RPGBuilder::noSelfOverlaps = true;
                break;
            }
            case 'D': {
                Globals::paranoidScheduling = true;
                break;
            }
            case 'A': {
                RPGHeuristic::hAddCostPropagation = true;
                break;
            }
            case 'd': {
                FF::nonDeletorsFirst = true;
                break;
            }
            case 'g': {
                RPGHeuristic::setGuidance(&(argv[argcount][2]));
                break;
            }
            case 'G': {
                FF::biasG = true;
                break;
            }
            case '8': {
                FF::biasD = true;
                break;
            }
            case 'S': {
                RPGBuilder::sortedExpansion = true;
                break;
            }
            case 'F': {
                RPGBuilder::fullFFHelpfulActions = true;
                break;
            }
            case 'b': {
                FF::bestFirstSearch = false;
                break;
            }
            case 'B': {
                benchmark = true;
                break;
            }
            case 'e': {
                FF::steepestDescent = true;
                break;
            }
            case 'E': {
                FF::skipEHC = true;
                break;
            }
            case 'k': {
                RPGBuilder::doSkipAnalysis = false;
                break;
            }
            case 'm': {
                FF::makespanTieBreak = false;
                break;
            }
            case 'c': {
                RPGBuilder::modifiedRPG = false;
                break;
            }
            #ifdef ENABLE_DEBUGGING_HOOKS
            case 'H': {
                debugPreprocessing = true;
                break;
            }
            #endif
            case 'h': {
                FF::helpfulActions = false;
                break;
            }
            case 'i': {
                FF::firstImprover = true;
                break;
            }
            case 'O': {
                FF::startsBeforeEnds = false;
                break;
            }
            case 'o': {
                LPScheduler::optimiseOrdering = false;
                break;
            }
            case 'p': {
                FF::pruneMemoised = false;
                break;
            }
            case 'R': {
                FF::invariantRPG = true;
                break;
            }
            case 'T': {
                totalOrder = true;
                RPGBuilder::modifiedRPG = false;
                if (argv[argcount][2] == 'T') {
                    postHocTotalOrder = true;
                }
                break;
            }
            case 'Q': {
                postHocScheduleToMetric = true;
                break;
            }
            case 'v': {
                if (argv[argcount][2] != 0) {
                    Globals::writeableVerbosity = atoi(&(argv[argcount][2]));
                } else {
                    cout << "No verbosity level specified with -v, defaulting to 1\n";
                    Globals::writeableVerbosity = 1;
                }
                break;
            }
            case 'L': {
                if (argv[argcount][2] != 0) {
                    LPScheduler::lpDebug = atoi(&(argv[argcount][2]));
                } else {
                    LPScheduler::lpDebug = 255;
                }
                break;
            }
            case 'W': {
                FF::doubleU = atof(&(argv[argcount][2]));
                break;
            }
            case 'I': {
                LPScheduler::hybridBFLP = false;
                break;
            }
            case 'r': {
                readInAPlan = true;
                break;
            }
            case 's': {
                FF::planMustSucceed = true;
                break;
            }
            case 'z': {
                FF::zealousEHC = false;
                break;
            }
            default:
                cout << "Unrecognised command-line switch '" << argv[argcount][1] << "'\n";
                usage(argv);
                exit(0);

            }

        }
        ++argcount;
    }

    if (argcount + ((readInAPlan || debugPreprocessing) ? 3 : 2) > argc) {
        usage(argv);
        exit(0);
    }

    performTIMAnalysis(&argv[argcount]);

    cout << std::setprecision(3) << std::fixed;

    if (totalOrder) {
        MinimalState::setTransformer(new TotalOrderTransformer());
    } else {
        MinimalState::setTransformer(new PartialOrderTransformer());
    }

    #ifdef ENABLE_DEBUGGING_HOOKS
    if (debugPreprocessing) {
        Globals::planFilename = argv[argc - 1];
    }
    #endif

    RPGBuilder::initialise();

    bool reachesGoals;

    pair<list<FFEvent>*, TemporalConstraints*> planAndConstraints;

    list<FFEvent> * & spSoln = planAndConstraints.first;
    if (readInAPlan) {
        spSoln = readPlan(argv[argc - 1]);
        reachesGoals = true;
#ifndef NDEBUG
        spSoln = FF::doBenchmark(reachesGoals, spSoln, false);
#endif
    } else {
        planAndConstraints = FF::search(reachesGoals);
    }

    if (spSoln) {

        for (int pass = 0; pass < 2; ++pass) {
            if (pass) {
                if (!postHocScheduleToMetric) break;

                if (!spSoln->empty()) {
                    if (totalOrder && !postHocTotalOrder) {
                        MinimalState::setTransformer(new PartialOrderTransformer());
                    }
                    assert(planAndConstraints.second);
                    spSoln = FF::reprocessPlan(spSoln, planAndConstraints.second);
                    planAndConstraints.second = 0;
                }
                cout << ";;;; Post-hoc optimised solution\n";
            } else {
                cout << ";;;; Solution Found\n";
            }
            tms refReturn;
            times(&refReturn);

            double secs = ((double)refReturn.tms_utime + (double)refReturn.tms_stime) / ((double) sysconf(_SC_CLK_TCK));

            int twodp = (int)(secs * 100.0);
            int wholesecs = twodp / 100;
            int centisecs = twodp % 100;

            cout << "; Time " << wholesecs << ".";
            if (centisecs < 10) cout << "0";
            cout << centisecs << "\n";
            list<FFEvent>::iterator planItr = spSoln->begin();
            const list<FFEvent>::iterator planEnd = spSoln->end();
            const int planSize = spSoln->size();
            vector<double> endTS(planSize);
            vector<FFEvent*> planVector(planSize);
            map<double, list<int> > sorted;
            for (int i = 0; planItr != planEnd; ++planItr, ++i) {
                if (planItr->time_spec == VAL::E_AT_START) {
                    sorted[planItr->lpTimestamp].push_back(i);
                    planVector[i] = &(*planItr);
                } else if (planItr->time_spec == VAL::E_AT_END) {
                    endTS[i] = planItr->lpTimestamp;
                }
            }
            map<double, list<int> >::iterator sortedItr = sorted.begin();
            const map<double, list<int> >::iterator sortedEnd = sorted.end();

            for (; sortedItr != sortedEnd; ++sortedItr) {
                list<int>::iterator iItr = sortedItr->second.begin();
                const list<int>::iterator iEnd = sortedItr->second.end();

                for (; iItr != iEnd; ++iItr) {
                    FFEvent * const planItr = planVector[*iItr];
                    if (planItr->lpTimestamp < 0.0000001) {
                        cout << "0.000";
                    } else {
                        cout << threeDP(planItr->lpTimestamp);
                    }
                    cout << ": " << *(planItr->action) << " ";
                    if (planItr->pairWithStep >= 0) {
                        const double dur = endTS[planItr->pairWithStep] - planItr->lpTimestamp;
                        cout << " [" << threeDP(dur) << "]\n";
                    } else if (RPGBuilder::getRPGDEs(planItr->action->getID()).empty()) {
                        cout << " [" << threeDP(RPGBuilder::getNonTemporalDurationToPrint()[planItr->action->getID()]) << "]\n";
                    } else {
                        assert(false);
                    }
                }
            }
        }

        if (benchmark) {
            FF::doBenchmark(reachesGoals, spSoln);
        }

        return 0;
    } else {
        cout << ";; Problem unsolvable!\n";
        tms refReturn;
        times(&refReturn);
        double secs = ((double)refReturn.tms_utime + (double)refReturn.tms_stime) / ((double) sysconf(_SC_CLK_TCK));

        int twodp = (int)(secs * 100.0);
        int wholesecs = twodp / 100;
        int centisecs = twodp % 100;

        cout << "; Time " << wholesecs << ".";
        if (centisecs < 10) cout << "0";
        cout << centisecs << "\n";
        return 1;
    }


}

extern int yyparse();
extern int yydebug;

void split(const int & insAt, list<FFEvent>::iterator insStart, const list<FFEvent>::iterator & insItr, const list<FFEvent>::iterator & insEnd)
{

    {
        for (; insStart != insItr; ++insStart) {
            int & currPWS = insStart->pairWithStep;
            if (currPWS != -1) {
                if (currPWS >= insAt) {
                    ++currPWS;
                }
            }
        }
    }
    {
        list<FFEvent>::iterator insPost = insItr;
        for (; insPost != insEnd; ++insPost) {
            int & currPWS = insPost->pairWithStep;
            if (currPWS != -1) {
                if (insPost->time_spec == VAL::E_AT_START) {
                    ++currPWS;
                } else if (insPost->time_spec == VAL::E_AT_END) {
                    if (currPWS >= insAt) {
                        ++currPWS;
                    }
                }
            }
        }
    }
}

namespace VAL
{
extern yyFlexLexer* yfl;
};

list<FFEvent> * readPlan(char* filename)
{
    static const bool debug = true;

    ifstream * const current_in_stream = new ifstream(filename);
    if (!current_in_stream->good()) {
        cout << "Exiting: could not open plan file " << filename << "\n";
        exit(1);
    }

    VAL::yfl = new yyFlexLexer(current_in_stream, &cout);
    yyparse();

    VAL::plan * const the_plan = dynamic_cast<VAL::plan*>(top_thing);

    delete VAL::yfl;
    delete current_in_stream;



    if (!the_plan) {
        cout << "Exiting: failed to load plan " << filename << "\n";
        exit(1);
    };

    if (!theTC->typecheckPlan(the_plan)) {
        cout << "Exiting: error when type-checking plan " << filename << "\n";
        exit(1);
    }

    list<FFEvent> * const toReturn = new list<FFEvent>();

    pc_list<plan_step*>::const_iterator planItr = the_plan->begin();
    const pc_list<plan_step*>::const_iterator planEnd = the_plan->end();

    for (int idebug = 0, i = 0; planItr != planEnd; ++planItr, ++i, ++idebug) {
        plan_step* const currStep = *planItr;

        instantiatedOp * const currOp = instantiatedOp::findInstOp(currStep->op_sym, currStep->params->begin(), currStep->params->end());
        if (!currOp) {
            const instantiatedOp * const debugOp = instantiatedOp::getInstOp(currStep->op_sym, currStep->params->begin(), currStep->params->end());
            cout << "Exiting: step " << idebug << " in the input plan uses the action " << *(debugOp) << ", which the instantiation code in the planner does not recognise.\n";
            exit(1);
        }
        const int ID = currOp->getID();

        if (RPGBuilder::getRPGDEs(ID).empty()) {// non-durative action
            FFEvent toInsert(currOp, 0.001, 0.001);
            const double ts = currStep->start_time;
            if (debug) cout << "; input " << ts << ": " << *currOp << " (id=" << ID << "), non-temporal";
            toInsert.lpTimestamp = ts;
            toInsert.lpMinTimestamp = ts;
            int insAt = 0;
            list<FFEvent>::iterator insItr = toReturn->begin();
            const list<FFEvent>::iterator insEnd = toReturn->end();
            for (; insItr != insEnd; ++insItr, ++insAt) {
                if (ts < insItr->lpTimestamp) {
                    split(insAt, toReturn->begin(), insItr, insEnd);
                    toReturn->insert(insItr, toInsert);
                    break;
                }
            }
            if (insItr == insEnd) {
                toReturn->push_back(toInsert);
            }
            if (debug) cout << " putting at step " << insAt << "\n";
        } else {
            int startIdx = -1;
            list<FFEvent>::iterator startIsAt = toReturn->end();
            const double actDur = currStep->duration;
            for (int pass = 0; pass < 2; ++pass) {
                if (pass) assert(startIdx >= 0);
                const double ts = (pass ? currStep->start_time + actDur : currStep->start_time);
                if (debug) {
                    cout << "; input " << ts << ": " << *currOp;
                    if (pass) {
                        cout << " end";
                    } else {
                        cout << " start";
                    }
                    cout << " (id=" << ID << ")";
                }
                FFEvent toInsert = (pass ? FFEvent(currOp, startIdx, actDur, actDur) : FFEvent(currOp, actDur, actDur));
                toInsert.lpTimestamp = ts;
                toInsert.lpMinTimestamp = ts;

                list<FFEvent>::iterator insItr = toReturn->begin();
                const list<FFEvent>::iterator insEnd = toReturn->end();
                int insAt = 0;
                for (; insItr != insEnd; ++insItr, ++insAt) {
                    if (ts < insItr->lpTimestamp) {
                        split(insAt, toReturn->begin(), insItr, insEnd);
                        const list<FFEvent>::iterator dest = toReturn->insert(insItr, toInsert);
                        if (pass) {
                            startIsAt->pairWithStep = insAt;
                            if (debug) cout << " putting at step " << insAt << ", pairing with " << startIdx << "\n";
                        } else {
                            startIsAt = dest;
                            startIdx = insAt;
                            if (debug) cout << " putting at step " << insAt << "\n";
                        }
                        break;
                    }
                }
                if (insItr == insEnd) {
                    toReturn->push_back(toInsert);
                    if (pass) {
                        startIsAt->pairWithStep = insAt;
                        if (debug) cout << " putting at step " << insAt << ", pairing with " << startIdx << "\n";
                    } else {
                        startIsAt = toReturn->end();
                        --startIsAt;
                        startIdx = insAt;
                        if (debug) cout << " putting at step " << insAt << "\n";
                    }
                }

            }
        }
    }

    const vector<RPGBuilder::FakeTILAction*> & tils = RPGBuilder::getTILVec();
    const int tilCount = tils.size();

    for (int t = 0; t < tilCount; ++t) {
        FFEvent toInsert(t);
        const double tilTS = tils[t]->duration;
        toInsert.lpMaxTimestamp = tilTS;
        toInsert.lpMinTimestamp = tilTS;
        toInsert.lpTimestamp = tilTS;

        if (debug) {
            cout << "TIL " << toInsert.divisionID << " goes at " << tilTS << endl;
        }

        list<FFEvent>::iterator insItr = toReturn->begin();
        const list<FFEvent>::iterator insEnd = toReturn->end();
        for (int insAt = 0; insItr != insEnd; ++insItr, ++insAt) {
            if (tilTS < insItr->lpTimestamp) {
                split(insAt, toReturn->begin(), insItr, insEnd);
                toReturn->insert(insItr, toInsert);
                break;
            }
        }
        if (insItr == insEnd) {
            toReturn->push_back(toInsert);
        }
    }

    if (debug) {
        list<FFEvent>::iterator insItr = toReturn->begin();
        const list<FFEvent>::iterator insEnd = toReturn->end();

        for (int i = 0; insItr != insEnd; ++insItr, ++i) {
            cout << i << ": ";
            if (insItr->action) {
                cout << *(insItr->action);
                if (insItr->time_spec == VAL::E_AT_START) {
                    cout << " start\n";
                } else {
                    cout << " end\n";
                }
            } else {
                cout << "TIL " << insItr->divisionID << endl;
            }
        }
    }

    return toReturn;
};
