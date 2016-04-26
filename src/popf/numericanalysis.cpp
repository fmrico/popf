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

#include "numericanalysis.h"
#include "RPGBuilder.h"

using namespace VAL;

namespace Planner {

vector<NumericAnalysis::dominance_constraint> NumericAnalysis::dominanceConstraints;
vector<bool> NumericAnalysis::allEffectsAreOrderIndependent;

void NumericAnalysis::findDominanceConstraintsAndMetricTrackingVariables()
{
    
    const int pneCount =  RPGBuilder::getPNECount();
    
    const vector<RPGBuilder::RPGNumericPrecondition> & rpgNumericPreconditions = RPGBuilder::getNumericPreTable();
    const vector<RPGBuilder::ArtificialVariable> & rpgArtificialVariables = RPGBuilder::getArtificialVariableTable();
    
    dominanceConstraints.resize(pneCount, E_NODOMINANCE);
    
    for (int i = 0; i < pneCount; ++i) {
        
        const int negativeI = i + pneCount;
        PNE* const currPNE = RPGBuilder::getPNE(i);
        
        { // case one - never appears as a precondition, presume it's a tracking quantity
            bool neverInPrecondition = true;
            const int rnpCount = rpgNumericPreconditions.size();
            for (int rnp = 0; rnp < rnpCount; ++rnp) {
                const RPGBuilder::RPGNumericPrecondition & currRNP = rpgNumericPreconditions[rnp];
                if (currRNP.LHSVariable == i || currRNP.LHSVariable == negativeI
                    ||  currRNP.RHSVariable == i || currRNP.RHSVariable == negativeI) {
                    neverInPrecondition = false;
                break;
                }
            }
            if (neverInPrecondition) {
                const int avCount = rpgArtificialVariables.size();
                for (int av = 0; av < avCount; ++av) {
                    const RPGBuilder::ArtificialVariable & currAV = rpgArtificialVariables[av];
                    const int avfSize = currAV.size;
                    for (int f = 0; f < avfSize; ++f) {
                        const int currF = currAV.fluents[f];
                        if (currF == i || currF == negativeI) {
                            neverInPrecondition = false;
                            break;
                        }
                    }
                }
            }
            if (neverInPrecondition) {
                // cout << "Have a metric tracking fluent: " << *currPNE << "\n";
                dominanceConstraints[i] = E_METRICTRACKING;
            }
        }
        
        if (dominanceConstraints[i] == E_NODOMINANCE) {
            if (EFT(currPNE->getHead())->onlyGoingDown()) {
                //  cout << "One-way fluent - " << *currPNE << " - implement the dominance checking for it\n";
                // implement something here
            }
        }
    }
}

bool durationsAreConstantBounded(const list<pair<int, VAL::time_spec> > & actions)
{
       
    list<pair<int, VAL::time_spec> >::const_iterator actItr = actions.begin();
    const list<pair<int, VAL::time_spec> >::const_iterator actEnd = actions.end();
    
    int de;
    for (; actItr != actEnd; ++actItr) {
        const vector<RPGBuilder::RPGDuration*> & duration = RPGBuilder::getRPGDEs(actItr->first);
        assert(!duration.empty()); // would mean a non-temporal action has a duration-dependent effect
        
        for (de = 0; de < 3; ++de) {
            const list<RPGBuilder::DurationExpr *> & currList = (*(duration[0]))[de];
            {
                list<RPGBuilder::DurationExpr*>::const_iterator exprItr = currList.begin();
                const list<RPGBuilder::DurationExpr*>::const_iterator exprEnd = currList.end();
                for (; exprItr != exprEnd; ++exprItr) {
                    if (!(*exprItr)->variables.empty()) { // if the duration constraint depends on other variables
                        return false;
                    }
                }
            }
        }        
    }
    
    return true;
} 

void NumericAnalysis::findWhichVariablesHaveOrderIndependentEffects()
{
    const int pneCount =  RPGBuilder::getPNECount();
    
    allEffectsAreOrderIndependent.resize(pneCount, true);
    
    const vector<RPGBuilder::RPGNumericEffect> & numericEffects = RPGBuilder::getNumericEff();
    
    const int effCount = numericEffects.size();
    
    for (int i = 0; i < effCount; ++i) {
        
        if (numericEffects[i].isAssignment) {  // assignment effects cannot be ordered arbitrarily
            allEffectsAreOrderIndependent[numericEffects[i].fluentIndex] = false;
        } else if (numericEffects[i].size > 1) {
            allEffectsAreOrderIndependent[numericEffects[i].fluentIndex] = false;
        } else if (numericEffects[i].size == 1) {
            // the only order-independent non-constant effects are those that depend on the duration
            // of an action, where that duration does not depend on variables
            
            if (   numericEffects[i].variables[0] != -3 // -3 denotes the variable ?duration
                && numericEffects[i].variables[0] != -19) { // -19 denotes -?duration
                
                allEffectsAreOrderIndependent[numericEffects[i].fluentIndex] = false;
                continue;   
            }
            
            if (durationsAreConstantBounded(RPGBuilder::getRpgNumericEffectsToActions()[i])) {
                allEffectsAreOrderIndependent[numericEffects[i].fluentIndex] = false;
            }
        }        
    }
    
    const vector<RPGBuilder::LinearEffects*> ctsEffects = RPGBuilder::getLinearDiscretisation();
    const int opCount = ctsEffects.size();
    
    for (int op = 0; op < opCount; ++op) {
        if (!(ctsEffects[op])) continue;
        
        list<pair<int, VAL::time_spec> > dummyList;
        dummyList.push_back(make_pair(op, VAL::E_AT_START));
        
        if (!durationsAreConstantBounded(dummyList)) { // if the duration of the operator depends on other variables
            const int vCount = ctsEffects[op]->vars.size();
            for (int v = 0; v < vCount; ++v) {
                allEffectsAreOrderIndependent[ctsEffects[op]->vars[v]] = false; // then it could have a non-order-independent effect on the variables it changes
            }
        }
    }
    
}



};

