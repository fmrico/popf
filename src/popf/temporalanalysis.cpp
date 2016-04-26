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

//
// C++ Implementation: temporalanalysis
//
// Description:
//
//
// Author: Amanda Coles, Andrew Coles, Maria Fox, Derek Long <firstname.lastname@cis.strath.ac.uk>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "temporalanalysis.h"

using std::endl;

namespace Planner
{

vector<vector<pair<double, double> > > TemporalAnalysis::actionTSBounds;
LiteralSet TemporalAnalysis::initialState;

void TemporalAnalysis::dummyAnalysis()
{

    const int actCount = RPGBuilder::getFixedDEs().size();
    actionTSBounds = vector<vector<pair<double, double> > >(actCount, vector<pair<double, double> >(2, pair<double, double>(0.0, DBL_MAX)));

};

map<int, list<pair<double, double> > > TemporalAnalysis::windows;

void TemporalAnalysis::processTILDeadlines()
{

    static const bool debug = (Globals::globalVerbosity & 32);
    if (debug) cout << "Performing TIL deadline analysis\n";

    static const double EPSILON = 0.001;

    const int actCount = RPGBuilder::getFixedDEs().size();

    actionTSBounds = vector<vector<pair<double, double> > >(actCount, vector<pair<double, double> >(2, pair<double, double>(0.0, DBL_MAX)));

    static const list<pair<double, double> > emptyList;

    map<int, double> lastAppearance;

    map<int, bool> windowable;

    {

        vector<double> initialFluents;

        RPGBuilder::getInitialState(initialState, initialFluents);

        const vector<RPGBuilder::FakeTILAction*> & allTILs = RPGBuilder::getTILVec();

        if (debug) {
            cout << "Number of TIL happenings: " << allTILs.size() << endl;            
        }
        
        map<int, double> literalAppears;
        map<int, double> literalDisappearsForGood;

        const int tilCount = allTILs.size();
        
        for (int tilID = 0; tilID < tilCount; ++tilID) {
            RPGBuilder::FakeTILAction * const tilItr = allTILs[tilID];
            
            if (debug) cout << "\tTIL " << tilID << " at " << tilItr->duration << "\n";

            {

                list<Literal*> & effs = tilItr->delEffects;

                list<Literal*>::iterator effItr = effs.begin();
                const list<Literal*>::iterator effEnd = effs.end();

                for (; effItr != effEnd; ++effItr) {
                    const int litID = (*effItr)->getID();
                    const pair<map<int, bool>::iterator, bool> wItr = windowable.insert(make_pair(litID, true));
                    if (wItr.second) {
                        const list<pair<int, VAL::time_spec> > & etaList = RPGBuilder::getEffectsToActions(litID);
                        list<pair<int, VAL::time_spec> >::const_iterator etItr = etaList.begin();
                        const list<pair<int, VAL::time_spec> >::const_iterator etEnd = etaList.end();

                        for (; etItr != etEnd; ++etItr) {
                            if (etItr->second != VAL::E_AT) {
                                if (debug) {
                                    if (wItr.first->second) {
                                        cout << *(*effItr) << " does not form windows, is added by:\n";
                                    }
                                }
                                wItr.first->second = false;
                                if (!debug) break;
                                if (debug) {
                                    if (etItr->second == VAL::E_AT_START) {
                                        cout << "\t" << *(RPGBuilder::getInstantiatedOp(etItr->first)) << " start\n";
                                    } else if (etItr->second == VAL::E_AT_END) {
                                        cout << "\t" << *(RPGBuilder::getInstantiatedOp(etItr->first)) << " end\n";
                                    }
                                }
                            }
                        }

                        if (debug && wItr.first->second) {
                            cout << *(*effItr) << " forms windows - only added by TILs\n";
                        }

                    }
                    if (wItr.first->second) {
                        literalDisappearsForGood.insert(pair<int, double>(litID, tilItr->duration));
                        list<pair<double, double> > & dest = windows[litID];

                        if (debug) {
                            cout << "Looking at window behaviour for " << *(*effItr);
                        }

                        const pair<map<int, double>::iterator, bool> laItr = lastAppearance.insert(make_pair(litID, DBL_MAX));
                        double & startOfWindow = laItr.first->second;

                        if (laItr.second) {
                            if (dest.empty()) {
                                if (initialState.find(*effItr) != initialState.end()) {
                                    startOfWindow = 0.0;
                                    if (debug) cout << " - true in the initial state";
                                } else {
                                    if (debug) cout << " - not true in the initial state";
                                }
                            } else {
                                if (debug) cout << " - not reappeared since last delete";
                            }
                        } else {
                            if (debug) cout << " - last appeared at " << startOfWindow;
                        }



                        if (startOfWindow != DBL_MAX) {
                            if (debug) {
                                cout << " - making a window [" << startOfWindow << "," << tilItr->duration << "]";
                            }
                            dest.push_back(make_pair(startOfWindow, tilItr->duration));
                        }

                        lastAppearance.erase(laItr.first);

                        if (debug) cout << " - done\n";
                    } else {

                    }
                }

            }

            {

                list<Literal*> & effs = tilItr->addEffects;

                list<Literal*>::iterator effItr = effs.begin();
                const list<Literal*>::iterator effEnd = effs.end();

                for (; effItr != effEnd; ++effItr) {

                    const int litID = (*effItr)->getID();
                    const pair<map<int, bool>::iterator, bool> wItr = windowable.insert(make_pair(litID, true));
                    if (wItr.second) {
                        const list<pair<int, VAL::time_spec> > & etaList = RPGBuilder::getEffectsToActions(litID);
                        list<pair<int, VAL::time_spec> >::const_iterator etItr = etaList.begin();
                        const list<pair<int, VAL::time_spec> >::const_iterator etEnd = etaList.end();

                        for (; etItr != etEnd; ++etItr) {
                            if (etItr->second != VAL::E_AT) {
                                if (debug) {
                                    if (wItr.first->second) {
                                        cout << *(*effItr) << " does not form windows, is added by:\n";
                                    }
                                }
                                wItr.first->second = false;
                                if (!debug) break;
                                if (debug) {
                                    if (etItr->second == VAL::E_AT_START) {
                                        cout << "\t" << *(RPGBuilder::getInstantiatedOp(etItr->first)) << " start\n";
                                    } else if (etItr->second == VAL::E_AT_END) {
                                        cout << "\t" << *(RPGBuilder::getInstantiatedOp(etItr->first)) << " end\n";
                                    }
                                }
                            }
                        }

                        if (debug && wItr.first->second) {
                            cout << *(*effItr) << " forms windows - only added by TILs\n";
                        }

                    }
                    if (wItr.first->second) {
                        literalDisappearsForGood.erase(litID);

                        const pair<map<int, double>::iterator, bool> laItr = lastAppearance.insert(make_pair(litID, 0.0));

                        if (laItr.second) {
                            double visibleFrom = tilItr->duration;
                            if (windows.find(litID) == windows.end()) { // appeared, but no time-windows defined yet
                                if (initialState.find(*effItr) != initialState.end()) visibleFrom = 0.0;
                            }
                            laItr.first->second = visibleFrom;
                        }

                        if (initialState.find(*effItr) == initialState.end() && RPGBuilder::getEffectsToActions((*effItr)->getID()).empty()) {
                            literalAppears.insert(pair<int, double>((*effItr)->getID(), tilItr->duration));
                        }
                    } else {
                        if (debug) {
                            cout << *(*effItr) << " does not form windows, can be added by:\n";
                            const list<pair<int, VAL::time_spec> > & etaList = RPGBuilder::getEffectsToActions(litID);
                            list<pair<int, VAL::time_spec> >::const_iterator etItr = etaList.begin();
                            const list<pair<int, VAL::time_spec> >::const_iterator etEnd = etaList.end();

                            for (; etItr != etEnd; ++etItr) {
                                if (etItr->second == VAL::E_AT_START) {
                                    cout << "\t" << *(RPGBuilder::getInstantiatedOp(etItr->first)) << " start\n";
                                } else if (etItr->second == VAL::E_AT_END) {
                                    cout << "\t" << *(RPGBuilder::getInstantiatedOp(etItr->first)) << " end\n";
                                } else if (etItr->second == VAL::E_AT) {
                                    cout << "\tTIL " << etItr->first << "\n";
                                }
                            }
                        }
                    }
                }
            }

        }

        {
            map<int, double>::iterator restrictItr = literalAppears.begin();
            const map<int, double>::iterator restrictEnd = literalAppears.end();

            for (; restrictItr != restrictEnd; ++restrictItr) {

                const double epsilonOff = restrictItr->second + EPSILON;

                list<pair<int, VAL::time_spec> > & affected = RPGBuilder::getPresToActions()[restrictItr->first];
                list<pair<int, VAL::time_spec> >::iterator affItr = affected.begin();
                const list<pair<int, VAL::time_spec> >::iterator affEnd = affected.end();

                for (; affItr != affEnd; ++affItr) {
                    if (affItr->second == VAL::E_AT_START) {
                        double & currMin = actionTSBounds[affItr->first][0].first;
                        if (epsilonOff > currMin) currMin = epsilonOff;

                        const double endOff = epsilonOff + RPGBuilder::getOpMinDuration(affItr->first, 0);
                        double & endMin = actionTSBounds[affItr->first][1].first;
                        if (endOff > endMin) endMin = endOff;
                    } else {
                        double & currMin = actionTSBounds[affItr->first][1].first;
                        if (epsilonOff > currMin) currMin = epsilonOff;

                        const double startOff = epsilonOff - RPGBuilder::getOpMaxDuration(affItr->first, 0);
                        double & startMin = actionTSBounds[affItr->first][0].first;
                        if (startOff > startMin) startMin = startOff;
                    }
                }
            }
        }

        {
            map<int, double>::iterator restrictItr = literalDisappearsForGood.begin();
            const map<int, double>::iterator restrictEnd = literalDisappearsForGood.end();

            for (; restrictItr != restrictEnd; ++restrictItr) {

                if (debug) cout << *(RPGBuilder::getLiteral(restrictItr->first)) << " disappears at " << restrictItr->second << "\n";

                const double epsilonOff = restrictItr->second - EPSILON;

                list<pair<int, VAL::time_spec> > & affected = RPGBuilder::getRawPresToActions()[restrictItr->first];

                if (debug) cout << "Bounding " << affected.size() << " start/end points\n";
                list<pair<int, VAL::time_spec> >::iterator affItr = affected.begin();
                const list<pair<int, VAL::time_spec> >::iterator affEnd = affected.end();

                for (; affItr != affEnd; ++affItr) {
                    if (affItr->second == VAL::E_AT_START) {
                        double & currMax = actionTSBounds[affItr->first][0].second;
                        if (epsilonOff < currMax) currMax = epsilonOff;

                        const double endOff = epsilonOff + RPGBuilder::getOpMaxDuration(affItr->first, 0);
                        double & endMax = actionTSBounds[affItr->first][1].second;
                        if (endOff < endMax) endMax = endOff;
                    } else { // invariant or at end
                        double & currMax = actionTSBounds[affItr->first][1].second;
                        if (epsilonOff < currMax) currMax = epsilonOff;

                        const double startOff = epsilonOff - RPGBuilder::getOpMinDuration(affItr->first, 0);
                        double & startMax = actionTSBounds[affItr->first][0].second;
                        if (startOff < startMax) startMax = startOff;
                    }
                }
            }
        }

        {
            map<int, double>::const_iterator laItr = lastAppearance.begin();
            const map<int, double>::const_iterator laEnd = lastAppearance.end();

            for (; laItr != laEnd; ++laItr) {
                list<pair<double, double> > & dest = windows[laItr->first];
                if (!dest.empty()) {
                    dest.push_back(make_pair(laItr->second, DBL_MAX));
                }
            }
        }
        {

            map<int, list<pair<double, double> > >::const_iterator winItr = windows.begin();
            const map<int, list<pair<double, double> > >::const_iterator winEnd = windows.end();

            for (; winItr != winEnd; ++winItr) {
                const list<pair<double, double> > & currList = winItr->second;
                if (currList.empty()) continue;

                if (debug) {
                    cout << "Windows on " << *(RPGBuilder::getLiteral(winItr->first)) << ":";
                    list<pair<double, double> >::const_iterator cwItr = currList.begin();
                    const list<pair<double, double> >::const_iterator cwEnd = currList.end();

                    for (; cwItr != cwEnd; ++cwItr) {
                        cout << " [" << cwItr->first << ",";
                        if (cwItr->second == DBL_MAX) {
                            cout << "end]";
                        } else {
                            cout << cwItr->second << "]";
                        }
                    }
                    cout << "\n";
                }
            }
        }
    }
    /*
    for (int i = 0; i < actCount; ++i) {
        if (!RPGBuilder::rogueActions[i]) {
            double & startMax = actionTSBounds[i][0].second;
            // if (startMax < DBL_MAX) cout << "Start of " << *(RPGBuilder::getInstantiatedOp(i)) << " is applicable no later than " << startMax << "\n";
        }
    }*/

};

void TemporalAnalysis::findGoalDeadlines(list<Literal*> & goals, list<double> & dest)
{

    static const double EPSILON = 0.001;

    list<Literal*>::iterator gItr = goals.begin();
    const list<Literal*>::iterator gEnd = goals.end();

    for (; gItr != gEnd; ++gItr) {

        double goalDeadline = 0.0;

        if (initialState.find(*gItr) == initialState.end()) {
            list<pair<int, VAL::time_spec> > & eta = RPGBuilder::getEffectsToActions((*gItr)->getID());

            list<pair<int, VAL::time_spec> >::iterator etaItr = eta.begin();
            const list<pair<int, VAL::time_spec> >::iterator etaEnd = eta.end();

            for (; etaItr != etaEnd; ++etaItr) {

                const double & thisDeadline = (etaItr->second == VAL::E_AT_START ? actionTSBounds[etaItr->first][0].second : actionTSBounds[etaItr->first][1].second);

                if (thisDeadline > goalDeadline) {
                    goalDeadline = thisDeadline;
                    if (goalDeadline == DBL_MAX) break;
                    goalDeadline += EPSILON;
                }
            }
        } else {
            goalDeadline = DBL_MAX;
        }
        if (goalDeadline != DBL_MAX) {
//          cout << "Deadline on goal " << *(*gItr) << ": " << goalDeadline << "\n";
        } else {
//          cout << "No deadline on goal " << *(*gItr) << "\n";
        }
        dest.push_back(goalDeadline);
    }
};

bool TemporalAnalysis::actionIsNeverApplicable(const int & a)
{
    if ((actionTSBounds[a][0].first > actionTSBounds[a][0].second) || (actionTSBounds[a][1].first > actionTSBounds[a][1].second)) return true;
    if ((actionTSBounds[a][0].first + RPGBuilder::getOpMinDuration(a, -1)) > actionTSBounds[a][1].second) return true;
    if ((actionTSBounds[a][1].second - RPGBuilder::getOpMaxDuration(a, -1)) > actionTSBounds[a][0].second) return true;

    return false;
}

}
