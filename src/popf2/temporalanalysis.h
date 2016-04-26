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
    static vector<bool> startEndSkip;
    
    static map<int, list<pair<double, double> > > windows;
    static vector<vector<pair<double, double> > > actionTSBounds;
    static LiteralSet initialState;
            
    static void recogniseHoldThroughoutDeleteAtEndIdiom(LiteralSet & factsIdentified);
    
    #ifdef POPF3ANALYSIS
    static bool endNumericEffectsAreCompressionSafe(const list<int> & effects,
                                                    vector<bool> & allInteractionWithVarIsCompressionSafe);
    static void markCompressionSafetyConditions(const int & actID, const list<int> & effects,
                                                vector<set<int> > & actionsDependingOnVarBeingCompressionSafe);
    static void markAffectedVariablesAsNotCompressionSafe(const list<int> & effects,
                                                          vector<bool> & allInteractionWithVarIsCompressionSafe,
                                                          set<int> & newlyUnsafe);
    #endif
public:

           
    static void dummyDeadlineAnalysis();
    static void processTILDeadlines();
    static void findGoalDeadlines(list<Literal*> &, list<double> &);
    static void findActionTimestampLowerBounds();
    static void findCompressionSafeActions();
    
    
    
    static vector<vector<pair<double, double> > > & getActionTSBounds() {
        return actionTSBounds;
    }
    
    static const list<pair<double, double> > * factIsVisibleInWindows(const Literal* const l) {
        map<int, list<pair<double, double> > >::const_iterator wItr = windows.find(l->getStateID());
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

    static bool canSkipToEnd(const int & i) {
        return startEndSkip[i];
    };
    

};

}

#endif
