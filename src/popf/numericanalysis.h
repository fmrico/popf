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

#ifndef NUMERICANALYSIS_H
#define NUMERICANALYSIS_H

#include <vector>
using std::vector;

namespace Planner {
    
class NumericAnalysis
{
    public:
        /**
        *  Enum denoting the identified dominance constraint on a given task variable:
        *   - <code>E_NODOMINANCE</code> denotes that no dominance constraint has been identified
        *   - <code>E_METRICTRACKING</code> denotes that the task variable never appears as a precondition of
        *     an action, or within one of the goals.  It may, however, appear in the metric.  The key consequence,
        *     though, is that, metric considerations aside, no value of the variable is better or worse than any other.
        *   - <code>E_SMALLERISBETTER</code> denotes that smaller values of the variable are preferable to larger values
        *     (e.g. all preconditions are of the form <code>(<= v c)</code>, and <code>v</code> can only be increased).
        *   - <code>E_BIGGERISBETTER</code> denotes that smaller values of the variable are preferable to larger values
        *     (e.g. all preconditions are of the form <code>(>= v c)</code>, and <code>v</code> can only be decreased). 
        */
        enum dominance_constraint { E_NODOMINANCE, E_METRICTRACKING, E_SMALLERISBETTER, E_BIGGERISBETTER};
        
    protected:
        /**
         *  Dominance constraints on each task variable, defined in the order in which they
         *  occur when iterating from <code>Inst::instantiatedOp::pnesBegin()</code> to
         *  <code>Inst::instantiatedOp::pnesEnd()</code>.
         */
        static vector<dominance_constraint> dominanceConstraints;
        
        /**
         *  For each task variable, this defines whether the effects on that variable are
         *  order-independent.  For now, this is determined by one of two simple rules: all the
         *  effects must be either:
         *  - increases or decreases by fixed, constant values.
         *  - increases or decreases by a continuous linear effect/duration dependent effect
         *    of an action for which the duration is bounded by fixed, constant values.
         */
        static vector<bool> allEffectsAreOrderIndependent;
    public:                
        
        /**
         *  Use static analysis to identify the dominance constraints on the
         *  task numeric variables.  Each variable is given one of the values from
         *  <code>NumericAnalysis::dominance_constraint</code>.  To obtain the results of this analysis,
         *  use <code>getDominanceConstraints()</code>.
         */
        static void findDominanceConstraintsAndMetricTrackingVariables();
        
        /**
         *  Use static analysis to identify the variables for which all effects are
         *  order-independent, i.e. those for which the affecting actions can be 
         *  interleaved arbitrarily without affecting the final change on the 
         *  variable.  To obtain the results of this analysis, use
         *  <code>getDataOnWhichVariablesHaveOrderIndependentEffects()</code>.
         */
        static void findWhichVariablesHaveOrderIndependentEffects();
        
        /**
         *  Return the results of the analysis to determine which variables have only order-independent
         *  effects, as performed by <code>findWhichVariablesHaveOrderIndependentEffects()</code>.
         *
         *  @return A const reference to <code>NumericAnalysis::allEffectsAreOrderIndependent</code>,
         *          where an entry of <code>true</code> denotes that the corresponding variable's
         *          effects are order-independent.
         */        
        static const vector<bool> & getDataOnWhichVariablesHaveOrderIndependentEffects() {
            return allEffectsAreOrderIndependent;
        }
        
        
        /**
         *  Return the results of the analysis to determine dominance constraints on variables, as
         *  performed by <code>findDominanceConstraintsAndMetricTrackingVariables()</code>
         *
         *  @return A const reference to <code>NumericAnalysis::dominanceConstraints</code>, the vector of dominance
         *          constraints on each task variable.
         *
         */        
        static const vector<dominance_constraint> & getDominanceConstraints() {
            return dominanceConstraints;
        }
        
};

};

#endif // NUMERICANALYSIS_H
