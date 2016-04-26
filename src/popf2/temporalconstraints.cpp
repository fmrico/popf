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

#include "temporalconstraints.h"
#include "RPGBuilder.h"

namespace Planner
{

#ifndef NDEBUG
pair<map<int, int>::iterator, bool> INVARIANTINSERT(map<int, int> & dest, const pair<int, int> & toInsert, const int & bound)
{
    assert(toInsert.first >= 0);
    assert(toInsert.second >= 0);

    assert(toInsert.first < bound);
    assert(toInsert.second < bound);

    return dest.insert(toInsert);
}

void INVARIANTERASE(map<int, int> & dest, const int & toErase, const int & bound)
{
    assert(toErase >= 0);

    assert(toErase < bound);

    dest.erase(toErase);
}
#endif

TemporalConstraints::TemporalConstraints()
        : stepsComeBeforeThisOne(0), mostRecentStep(-1)
        #ifndef TOTALORDERSTATES
        ,lastStepToTouchPNE(RPGBuilder::getPNECount(), FluentInteraction())
        #endif
{
}

TemporalConstraints::TemporalConstraints(const TemporalConstraints & other, const int extendBy)
        : mostRecentStep(other.mostRecentStep)
        #ifndef TOTALORDERSTATES
        , lastStepToTouchPNE(other.lastStepToTouchPNE)
        #endif
{
    const int loopLim = other.stepsComeBeforeThisOne.size();
    stepsComeBeforeThisOne = vector<map<int, bool>* >(loopLim + extendBy, (map<int, bool>*)0);
    for (int i = 0; i < loopLim; ++i) {
        const map<int, bool>* const toCopy = other.stepsComeBeforeThisOne[i];
        if (toCopy) {
            stepsComeBeforeThisOne[i] = new map<int, bool>(*toCopy);
        }
    }
}

TemporalConstraints::~TemporalConstraints()
{

    const int loopLim = stepsComeBeforeThisOne.size();

    for (int i = 0; i < loopLim; ++i) {
        delete stepsComeBeforeThisOne[i];
    }
}


void TemporalConstraints::addOrdering(const unsigned int & b, const unsigned int & a, const bool & ep)
{

    assert(a < stepsComeBeforeThisOne.size());
    assert(a >= 0);
    assert(b < stepsComeBeforeThisOne.size());
    assert(b >= 0);
    assert(a != b);
    map<int, bool> *& thingsBeforeA = stepsComeBeforeThisOne[a];

    if (!thingsBeforeA) {
        thingsBeforeA = new map<int, bool>();
    }

    const pair<map<int, bool>::iterator, bool> orderItr = thingsBeforeA->insert(make_pair(b, ep));

    if (!orderItr.second && ep) {
        orderItr.first->second = true;
    }
    
    /*if (orderItr.first->second) {
        cout << b << " epsilon before " << a << std::endl;
    } else {
        cout << b << " 0 before " << a << std::endl;
    }*/
    
}

void TemporalConstraints::extend(const int & extendBy)
{
    const int loopLim = stepsComeBeforeThisOne.size();
    stepsComeBeforeThisOne.resize(loopLim + extendBy);
    for (int i = 0; i < extendBy; ++i) {
        stepsComeBeforeThisOne[i + loopLim] = (map<int, bool>*)0;
    }
}

};
