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


#ifndef PARTIALORDERTRANSFORMER_H
#define PARTIALORDERTRANSFORMER_H

#include "minimalstate.h"

namespace Planner
{

class TemporalConstraints;

class PartialOrderTransformer : public StateTransformer
{

public:
    PartialOrderTransformer() {
    }

    virtual ~PartialOrderTransformer() {
    }

    virtual TemporalConstraints * cloneTemporalConstraints(const TemporalConstraints * const, const int extendBy = 0);
    virtual TemporalConstraints * emptyTemporalConstraints();
    virtual MinimalState & applyAction(MinimalState & s, const ActionSegment & a, const bool & inPlace, const double & minDur, const double & maxDur);

};

}

#endif              // PARTIALORDERTRANSFORMER_H
