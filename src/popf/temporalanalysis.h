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
// C++ Interface: temporalanalysis
//
// Description:
//
//
// Author: Amanda Coles, Andrew Coles, Maria Fox, Derek Long <firstname.lastname@cis.strath.ac.uk>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef PLANNERTEMPORALANALYSIS_H
#define PLANNERTEMPORALANALYSIS_H

#include "RPGBuilder.h"

namespace Planner
{

class TemporalAnalysis
{

private:
    static map<int, list<pair<double, double> > > windows;
    static vector<vector<pair<double, double> > > actionTSBounds;
    static LiteralSet initialState;

public:

    static void dummyAnalysis();
    static void processTILDeadlines();
    static void findGoalDeadlines(list<Literal*> &, list<double> &);
    static vector<vector<pair<double, double> > > & getActionTSBounds() {
        return actionTSBounds;
    }
    static const list<pair<double, double> > * factIsVisibleInWindows(const Literal* const l) {
        map<int, list<pair<double, double> > >::const_iterator wItr = windows.find(l->getID());
        if (wItr == windows.end()) return 0;
        return &(wItr->second);
    }
    static void suggestNewStartLowerBound(const int & a, const double & d) {
        if (actionTSBounds[a][0].first < d) {
            actionTSBounds[a][0].first = d;
        }
    }

    static void suggestNewEndLowerBound(const int & a, const double & d) {
        if (actionTSBounds[a][1].first < d) {
            actionTSBounds[a][1].first = d;
        }
    }

    static bool actionIsNeverApplicable(const int & a);
    static bool okayToStart(const int & a, const double & ts) {
        return (ts <= actionTSBounds[a][0].second);
    }

    static bool okayToEnd(const int & a, const double & ts) {
        return (ts <= actionTSBounds[a][1].second);
    }


};

}

#endif
