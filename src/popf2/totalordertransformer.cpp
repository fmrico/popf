/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "totalordertransformer.h"
#include "temporalconstraints.h"
#include "RPGBuilder.h"

using Inst::Literal;

namespace Planner
{

TemporalConstraints * TotalOrderTransformer::cloneTemporalConstraints(const TemporalConstraints * const other, const int extendBy)
{
    return t.cloneTemporalConstraints(other, extendBy);
}

TemporalConstraints * TotalOrderTransformer::emptyTemporalConstraints()
{
    return t.emptyTemporalConstraints();
}

int TotalOrderTransformer::stepThatMustPrecedeUnfinishedActions(const TemporalConstraints * const cons) const
{
    return cons->getMostRecentStep();
}

double TotalOrderTransformer::latestTimePointOfActionsStartedHere(const int & i) const
{
    static const int tilCount = RPGBuilder::getTILVec().size();
    
    if (tilCount >= i) return DBL_MAX;
    return (RPGBuilder::getTILVec()[i]->duration);
}


MinimalState * TotalOrderTransformer::applyAction(MinimalState & theStateHidden, const ActionSegment & a, const bool & inPlace,
        const double & minDur, const double & maxDur)
{
    const int previousMostRecent = theStateHidden.temporalConstraints->getMostRecentStep();

    MinimalState * toReturn = t.applyAction(theStateHidden, a, inPlace, minDur, maxDur);

    if (previousMostRecent != -1) { // if this isn't the first step in the plan
        const int newMostRecent = toReturn->temporalConstraints->getMostRecentStep();
        toReturn->temporalConstraints->addOrdering(previousMostRecent, newMostRecent, true); // then impose the total ordering constraint
        if (Globals::globalVerbosity & 4096) {
            cout << "TO constraint: " << previousMostRecent << " comes before " << newMostRecent << std::endl;
        }
    } else {
        if (Globals::globalVerbosity & 4096) {
            const int newMostRecent = toReturn->temporalConstraints->getMostRecentStep();
            cout << "No TO constraint for step " << newMostRecent << std::endl;
        }
    }

    return toReturn;
};

#ifdef POPF3ANALYSIS
void TotalOrderTransformer::updateWhenEndOfActionIs(MinimalState & s, const int & actID, const int & stepID, const double & newTS)
{
    t.updateWhenEndOfActionIs(s,actID,stepID,newTS);
}
#endif


};
