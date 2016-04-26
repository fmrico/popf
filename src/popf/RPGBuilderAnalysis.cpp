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

#include "RPGBuilder.h"
#include "globals.h"
#include "numericanalysis.h"
#include "temporalanalysis.h"
#include "FFSolver.h"

#include "typecheck.h"
#include "TIM.h"
#include "FuncAnalysis.h"

using namespace TIM;
using namespace Inst;
using namespace VAL;

#include <sstream>
using std::ostringstream;

namespace Planner
{



void RPGBuilder::findUninterestingnessCriteria()
{
    const int opCount = instantiatedOps.size();

    for (int i = 0; i < opCount; ++i) {
        if (!rogueActions[i]) {
            bool allUninteresting = true;
            int criterion = -1;
            {
                list<Literal*> & effs = actionsToStartEffects[i];
                list<Literal*>::iterator effItr = effs.begin();
                const list<Literal*>::iterator effEnd = effs.end();

                for (; effItr != effEnd; ++effItr) {
                    if (negativeEffectsToActions[(*effItr)->getID()].empty()) {
                        criterion = (*effItr)->getID();
                    } else {
                        allUninteresting = false;
                        break;
                    }
                }
            }
            if (allUninteresting) {
                list<Literal*> & effs = actionsToEndEffects[i];
                list<Literal*>::iterator effItr = effs.begin();
                const list<Literal*>::iterator effEnd = effs.end();

                for (; effItr != effEnd; ++effItr) {
                    if (negativeEffectsToActions[(*effItr)->getID()].empty()) {
                        criterion = (*effItr)->getID();
                    } else {
                        allUninteresting = false;
                        break;
                    }
                }
            }

            if (allUninteresting) { // checked props, now onto numerics
                //if (criterion == -1) {
                //  cout << "Action " << i << " - " << *(instantiatedOps[i]) << " is propositionally uninteresting once applied: no propositional effects (" << actionsToStartEffects[i].size() << ", " << actionsToEndEffects[i].size() << ")\n";
                //} else {
                //  cout << "Action " << i << " - " << *(instantiatedOps[i]) << " is propositionally uninteresting once applied: nothing deletes " << *(literals[criterion]) << "\n";
                //}
                {
                    list<int> & numEffs = actionsToRPGNumericStartEffects[i];

                    list<int>::iterator neItr = numEffs.begin();
                    const list<int>::iterator neEnd = numEffs.end();

                    for (; neItr != neEnd; ++neItr) {
                        RPGNumericEffect & currRNE = rpgNumericEffects[*neItr];
                        if (NumericAnalysis::getDominanceConstraints()[currRNE.fluentIndex] == NumericAnalysis::E_METRICTRACKING) {

                        } else {
                            allUninteresting = false;
                            break;
                        }
                    }
                }

                LinearEffects * const currLE = linearDiscretisation[i];

                if (currLE) {
                    const int varCount = currLE->vars.size();
                    for (int vc = 0; vc < varCount; ++vc) {
                        if (NumericAnalysis::getDominanceConstraints()[currLE->vars[vc]] == NumericAnalysis::E_METRICTRACKING) {

                        } else {
                            allUninteresting = false;
                            break;
                        }
                    }
                }

                if (allUninteresting) {
                    list<int> & numEffs = actionsToRPGNumericEndEffects[i];

                    list<int>::iterator neItr = numEffs.begin();
                    const list<int>::iterator neEnd = numEffs.end();

                    for (; neItr != neEnd; ++neItr) {
                        RPGNumericEffect & currRNE = rpgNumericEffects[*neItr];
                        if (NumericAnalysis::getDominanceConstraints()[currRNE.fluentIndex] == NumericAnalysis::E_METRICTRACKING) {

                        } else {
                            allUninteresting = false;
                            break;
                        }
                    }
                }
                if (allUninteresting) {
                    // cout << "Action " << i << " - " << *(instantiatedOps[i]) << " is uninteresting";
                    if (criterion != -1) {
                        // cout << " once we have fact " << *(literals[criterion]) << "\n";
                        uninterestingnessCriteria[i] = criterion;
                    } else {
                        // cout << " full stop - there's no reason ever to apply it\n";
                        uninterestingnessCriteria[i] = -1;
                    }
                }
            }
        }
    }
};

void RPGBuilder::checkConditionalNumericEffectsAreOnlyOnMetricTrackingVariables()
{
    const int opCount = actionsToConditionalEffects.size();

    for (int op = 0; op < opCount; ++op) {
        if (rogueActions[op]) continue;

        list<ConditionalEffect>::const_iterator eff = actionsToConditionalEffects[op].begin();
        const list<ConditionalEffect>::const_iterator effEnd = actionsToConditionalEffects[op].end();

        for (; eff != effEnd; ++eff) {
            list<pair<int, VAL::time_spec> >::const_iterator numEff = eff->getNumericEffects().begin();
            const list<pair<int, VAL::time_spec> >::const_iterator numEffEnd = eff->getNumericEffects().end();

            for (; numEff != numEffEnd; ++numEff) {
                const RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[numEff->first];
                if (NumericAnalysis::getDominanceConstraints()[currEff.fluentIndex] != NumericAnalysis::E_METRICTRACKING) {
                    postmortem_noADL();
                }
            }
        }
    }
}


void copyLiteralListToSet(list<Literal*> & theList, LiteralSet & theSet)
{

    list<Literal*>::iterator tlItr = theList.begin();
    const list<Literal*>::iterator tlEnd = theList.end();

    for (; tlItr != tlEnd; ++tlItr) theSet.insert(*tlItr);

};

//class OneShotKShotFormula : public KShotFormula {
//
//private:
//list<int> watchedLiterals;
//public:
//OneShotKShotFormula(list<int> & toWatch) : KShotFormula(), watchedLiterals(toWatch) {};
//  virtual int getLimit(MinimalState & s);
//};

int RPGBuilder::OneShotKShotFormula::getLimit(const MinimalState & s) const
{

    list<int>::const_iterator wlItr = watchedLiterals.begin();
    const list<int>::const_iterator wlEnd = watchedLiterals.end();

    for (; wlItr != wlEnd; ++wlItr) {
        if (s.first.find(*wlItr) == s.first.end()) return 0;
    }

    return 1;

};

int RPGBuilder::OneShotKShotFormula::getOptimisticLimit(const MinimalState &) const
{

    return 1;

};


int RPGBuilder::KShotKShotFormula::getLimit(const MinimalState & s) const
{

    int minShots = INT_MAX;
    
    list<ShotCalculator>::const_iterator sfItr = formulae.begin();
    const list<ShotCalculator>::const_iterator sfEnd = formulae.end();
    
    for (; sfItr != sfEnd; ++sfItr) {
        
        const int thisShots = (int)((s.secondMax[sfItr->variable] - sfItr->greaterThan) / sfItr->decreaseBy);
        if (thisShots < minShots) minShots = thisShots;
        
    }
    
    return (minShots > 0 ? minShots : 0);

}

int RPGBuilder::KShotKShotFormula::getOptimisticLimit(const MinimalState & s) const
{

    return getLimit(s);

};


bool constOnlyCalculate(const list<RPGBuilder::Operand> & formula, vector<double> & fluents, double & cValue)
{

    list<double> RHS;

    list<RPGBuilder::Operand>::const_iterator fItr = formula.begin();
    const list<RPGBuilder::Operand>::const_iterator fEnd = formula.end();

    for (; fItr != fEnd; ++fItr) {
        const RPGBuilder::Operand & currOperand = *fItr;
        const RPGBuilder::math_op currMathOp = currOperand.numericOp;
        switch (currMathOp) {
        case RPGBuilder::NE_ADD: {
            const double oldFront = RHS.front(); RHS.pop_front();
            RHS.front() += oldFront;
        }
        break;
        case RPGBuilder::NE_SUBTRACT: {
            const double oldFront = RHS.front(); RHS.pop_front();
            RHS.front() -= oldFront;
        }
        break;
        case RPGBuilder::NE_MULTIPLY: {
            const double oldFront = RHS.front(); RHS.pop_front();
            RHS.front() *= oldFront;
        }
        break;
        case RPGBuilder::NE_DIVIDE: {
            const double oldFront = RHS.front(); RHS.pop_front();
            RHS.front() /= oldFront;
        }
        break;
        case RPGBuilder::NE_CONSTANT:
            RHS.push_front(currOperand.constantValue);
            break;
        case RPGBuilder::NE_FLUENT:
            return false;
            break;
        default:
            // this should never happen
            assert(false);
        }
    }

    cValue = RHS.front();
    return true;

};

void RPGBuilder::findSelfMutexes()
{

    const int opCount = instantiatedOps.size();
    selfMutexes = vector<bool>(opCount, false);

    // For each literal, the actions in the associated set (each of which has a start pre and
    // a start delete of that literal) are self-mutex iff the only actions adding this fact
    // are in that set
    map<int, set<int> > potentiallySelfMutexActionsDueToFact;
    
    for (int i = 0; i < opCount; ++i) {

        if (rogueActions[i]) continue;
        
        LiteralSet startPreconditions;
        LiteralSet startDeletes;
        LiteralSet startAdds;
        
        copyLiteralListToSet(actionsToStartPreconditions[i], startPreconditions);
        copyLiteralListToSet(actionsToStartNegativeEffects[i], startDeletes);
        copyLiteralListToSet(actionsToStartEffects[i], startAdds);
        
        
        // facts deleted at the start which aren't immediately re-added
        LiteralSet overallStartDeletes;        
        std::set_difference(startDeletes.begin(), startDeletes.end(),
                            startAdds.begin(), startAdds.end(),
                            std::insert_iterator<LiteralSet>(overallStartDeletes, overallStartDeletes.begin()));
        
        // facts needed at the start that are then immediately deleted
        LiteralSet isect;        
        std::set_intersection(startPreconditions.begin(), startPreconditions.end(),
                              overallStartDeletes.begin(), overallStartDeletes.begin(),
                              std::insert_iterator<LiteralSet>(isect, isect.begin()));
        
        // bail out if there are no possibly self-mutex-inducing facts
        if (isect.empty()) {
            continue;
        }
        
        LiteralSet endAdds;
        copyLiteralListToSet(actionsToEndEffects[i], endAdds);
                             
        // facts that are in isect, and also added at the end
        LiteralSet candidates;        
        std::set_intersection(endAdds.begin(), endAdds.end(),
                              isect.begin(), isect.end(),
                              std::insert_iterator<LiteralSet>(candidates, candidates.begin()));
        
        LiteralSet::const_iterator fItr = isect.begin();
        const LiteralSet::const_iterator fEnd = isect.end();
        
        for (; fItr != fEnd; ++fItr) {
            // note that we might have a self-mutex inducing fact for this action
            potentiallySelfMutexActionsDueToFact[(*fItr)->getID()].insert(i);
        }

    }
    
    map<int, set<int> >::const_iterator  mutexGroup = potentiallySelfMutexActionsDueToFact.begin();
    const map<int, set<int> >::const_iterator  mutexGroupEnd = potentiallySelfMutexActionsDueToFact.end();
    
    for (; mutexGroup != mutexGroupEnd; ++mutexGroup) {
        set<int> adders;
        
        {
            set<int>::iterator insItr = adders.begin();
            
            const list<pair<int, VAL::time_spec> > & addedBy = effectsToActions[mutexGroup->first];
            
            list<pair<int, VAL::time_spec> >::const_iterator addItr = addedBy.begin();
            const list<pair<int, VAL::time_spec> >::const_iterator addEnd = addedBy.end();
            
            for (; addItr != addEnd; ++addItr) {
                if (addItr->second != VAL::E_AT_END) {
                    break;
                }
                insItr = adders.insert(insItr, addItr->first);
            }
            if (addItr != addEnd) continue;
        }
        
        // now we check that the set of actions that add the fact are a subset or equal to those in the set
        // if any aren't accounted for, we can't guarantee a self mutex
        
        set<int> notAccountedFor;        
        std::set_difference(adders.begin(), adders.end(), mutexGroup->second.begin(), mutexGroup->second.end(),
                            std::insert_iterator<set<int> >(notAccountedFor, notAccountedFor.begin()));
                            
        if (notAccountedFor.empty()) {
            set<int>::const_iterator gItr = mutexGroup->second.begin();
            const set<int>::const_iterator gEnd = mutexGroup->second.end();
            
            for (; gItr != gEnd; ++gItr) {
                selfMutexes[*gItr] = true;                
            }
        }
    }

};

void RPGBuilder::kshotInferForAction(const int & i, MinimalState & refState, LiteralSet & maybeOneShotLiteral, vector<double> & initialFluents, const int & fluentCount)
{


    int oldKShotLimit = INT_MAX;

    if (kShotFormulae[i]) oldKShotLimit = kShotFormulae[i]->getLimit(refState);

    if (oldKShotLimit > 1) {



        { // first, let's see if there's a simple propositional reason for it being oneshot
            // look for a delete effect not added by anything else, and required as a precond.

            LiteralSet allPreconditions;
            LiteralSet allDeletes;

            copyLiteralListToSet(actionsToStartPreconditions[i], allPreconditions);
            copyLiteralListToSet(actionsToInvariants[i], allPreconditions);
            copyLiteralListToSet(actionsToEndPreconditions[i], allPreconditions);

            copyLiteralListToSet(actionsToStartNegativeEffects[i], allDeletes);
            copyLiteralListToSet(actionsToEndNegativeEffects[i], allDeletes);

            set<int> theIntersection;

            {
                LiteralSet::iterator apItr = allPreconditions.begin();
                const LiteralSet::iterator apEnd = allPreconditions.end();

                LiteralSet::iterator adItr = allDeletes.begin();
                const LiteralSet::iterator adEnd = allDeletes.end();

                while (apItr != apEnd && adItr != adEnd) {
                    const int idOne = (*apItr)->getID();
                    const int idTwo = (*adItr)->getID();
                    if (idOne < idTwo) {
                        ++apItr;
                    } else if (idTwo < idOne) {
                        ++adItr;
                    } else {
                        theIntersection.insert(idOne);
                        ++apItr;
                        ++adItr;
                    }
                };
            }

            list<int> toWatch;

            {
                set<int>::iterator adItr = theIntersection.begin();
                const set<int>::iterator adEnd = theIntersection.end();

                for (; adItr != adEnd; ++adItr) {
                    const int currLitID = (*adItr);
                    if (effectsToActions[currLitID].empty()) {
                        toWatch.push_back(currLitID);
                    } else if (oneShotLiterals[currLitID]) {
                        toWatch.push_back(currLitID);
                    } else {
//                      cout << "\t" << *(getLiteral(*adItr)) << " is added by actions, and isn't one-shot\n";
                    }
                }

            }

            if (!toWatch.empty()) {
//              cout << "Have shown that " << *(instantiatedOps[i]) << " is one-shot\n";
                kShotFormulae[i] = new OneShotKShotFormula(toWatch);
            }

        }

        if (!kShotFormulae[i]) {
            // next case
            // - precondition says v > c where c is constant, probably 0 in any reasonable encoding, but may as well generalise
            // - v is strictly decreasing (ask TIM)
            // - the action decreases v by some amount d, where d is constant or strictly increasing (again, ask TIM)
            // - and thus, k = a formula (v - c) / d
            // this is weaker than the previous case, as the previous determines a bound of 1 or 0 depending on logical information
            // rather than k based on numeric information

            //actionsToRPGNumericStartPreconditions = vector<list<int> >(opCount);
            //actionsToRPGNumericInvariants = vector<list<int> >(opCount);
            //actionsToRPGNumericEndPreconditions = vector<list<int> >(opCount);


            list<int> candidatePreconditions;

            {
                for (int pass = 0; pass < 3; ++pass) {
                    list<int> & currNP = (pass == 0 ? actionsToRPGNumericStartPreconditions[i] : (pass == 1 ? actionsToRPGNumericInvariants[i] : actionsToRPGNumericEndPreconditions[i]));
                    list<int>::iterator npItr = currNP.begin();
                    const list<int>::iterator npEnd = currNP.end();

                    for (; npItr != npEnd; ++npItr) {
                        RPGNumericPrecondition & currPrec = rpgNumericPreconditions[*npItr];

                        if (currPrec.RHSVariable != -1) break;

                        const int LHSVariable = currPrec.LHSVariable;
                        if (LHSVariable == -1) break;
                        if (LHSVariable >= fluentCount) break;

                        candidatePreconditions.push_back(*npItr);

                    }


                }


            }

            list<ShotCalculator> shotFormulae; // more than one, take min over

            {

                list<int>::iterator cpItr = candidatePreconditions.begin();
                const list<int>::iterator cpEnd = candidatePreconditions.end();
                
                for (; cpItr != cpEnd; ++cpItr) {
                    
                    RPGNumericPrecondition & currPrec = rpgNumericPreconditions[*cpItr];
                    
                    const double RHSConstant = currPrec.RHSConstant;
                    const int LHSVariable = currPrec.LHSVariable;
                    
                    bool foundDecreaser = false;
                    double decreaseBy = 0.0;
                    
                    for (int pass = 0; pass < 2; ++pass) {
                        list<NumericEffect> & currNE = (pass == 0 ? actionsToStartNumericEffects[i] : actionsToEndNumericEffects[i]);
                        list<NumericEffect>::iterator neItr = currNE.begin();
                        const list<NumericEffect>::iterator neEnd = currNE.end();
                        
                        for (; neItr != neEnd; ++neItr) {
                            
                            if (neItr->fluentIndex == LHSVariable) {
                                
                                double decVal;
                                
                                if (constOnlyCalculate(neItr->formula, initialFluents, decVal)) {
                                    if (neItr->op == VAL::E_DECREASE){
                                        if (decVal > 0) {
                                            
                                            if (!foundDecreaser) {
                                                decreaseBy = decVal;
                                                foundDecreaser = true;
                                            } else {
                                                decreaseBy += decVal; // must have a start then end dec effect
                                            }
                                        }
                                    } else if (neItr->op == VAL::E_INCREASE){
                                        if (decVal < 0) {
                                            
                                            if (!foundDecreaser) {
                                                decreaseBy = -decVal;
                                                foundDecreaser = true;
                                            } else {
                                                decreaseBy -= decVal; // must have a start then end dec effect
                                            }
                                        }
                                    }
                                    
                                }
                                
                            }
                            
                        }
                        
                    }
                    
                    if (foundDecreaser && decreaseBy > 0.000000001) {
                        
                        PNE* const currPNE = getPNE(LHSVariable);
                        
                        if (EFT(currPNE->getHead())->onlyGoingDown()) {
                            /*cout << "Have shown that " << *(instantiatedOps[i]) << " is k-shot\n where ";
                            if (RHSConstant) {
                                cout << "k = ceil(" << *(currPNE) << " - " << (RHSConstant - decreaseBy) << ")";
                            } else {
                                cout << "k = " << *(currPNE);
                            }
                            if (decreaseBy != 1.0) {
                                cout << " / " << decreaseBy << "\n";
                            } else {
                                cout << "\n";
                            }*/
                            shotFormulae.push_back(ShotCalculator(LHSVariable, (RHSConstant - decreaseBy), decreaseBy));
                            
                        }
                        
                    }
                                        
                }

            }

            if (!shotFormulae.empty()) {
                kShotFormulae[i] = new KShotKShotFormula(shotFormulae);
            }

        }

        // otherwise, if we can't prove anything useful, it's unlimited

        if (!kShotFormulae[i]) {
            kShotFormulae[i] = new UnlimitedKShotFormula();
        } else {
//          cout << "Have an analysis now - says limit is " << kShotFormulae[i]->getLimit(refState) << "\n";
        }

        if (kShotFormulae[i]->getLimit(refState) <= 1) {
            LiteralSet startAdds;
            LiteralSet endDeletes;

            copyLiteralListToSet(actionsToStartEffects[i], startAdds);
            copyLiteralListToSet(actionsToEndNegativeEffects[i], endDeletes);

            set<int> theIntersection;

            {
                LiteralSet::iterator apItr = startAdds.begin();
                const LiteralSet::iterator apEnd = startAdds.end();

                LiteralSet::iterator adItr = endDeletes.begin();
                const LiteralSet::iterator adEnd = endDeletes.end();

                while (apItr != apEnd && adItr != adEnd) {
                    const int idOne = (*apItr)->getID();
                    const int idTwo = (*adItr)->getID();
                    if (idOne < idTwo) {
                        ++apItr;
                    } else if (idTwo < idOne) {
                        ++adItr;
                    } else {
                        theIntersection.insert(idOne);
                        ++apItr;
                        ++adItr;
                    }
                };
            }

            if (theIntersection.empty()) {
//              cout << "Intersection of start adds and end deletes is empty\n";
            }

            set<int>::iterator tiItr = theIntersection.begin();
            const set<int>::iterator tiEnd = theIntersection.end();

            for (; tiItr != tiEnd; ++tiItr) {
                if (effectsToActions[*tiItr].size() == 1 && refState.first.find(*tiItr) == refState.first.end()) { // only added by one thing, i.e. this action
                    oneShotLiterals[*tiItr] = true;
                    maybeOneShotLiteral.insert(getLiteral(*tiItr));
                    list<pair<int, VAL::time_spec> > & listToUse = processedPreconditionsToActions[*tiItr];
                    list<pair<int, VAL::time_spec> >::iterator affectedItr = listToUse.begin();
                    const list<pair<int, VAL::time_spec> >::iterator affectedEnd = listToUse.end();

                    for (; affectedItr != affectedEnd; ++affectedItr) {
                        if (affectedItr->second == VAL::E_AT_END) {
                            actionsToEndOneShots[affectedItr->first].insert(getLiteral(*tiItr));
//                          cout << "Has " << *(getLiteral(*tiItr)) << " as a one-shot wrapped literal on which an end is dependent\n";
                        }
                    }
                } else {
//                  cout << *(getLiteral(*tiItr)) << " isn't one-shot wrapped in the action";
//                  cout << "Is added by " << effectsToActions[*tiItr].size() << " actions\n";
//                  if (refState.first.find(*tiItr) != refState.first.end()) cout << "Is in initial state\n";
                }
            }

            {
                list<Literal*> & currEffectsList = actionsToStartEffects[i];

                list<Literal*>::iterator effItr = currEffectsList.begin();
                const list<Literal*>::iterator effEnd = currEffectsList.end();


                for (; effItr != effEnd; ++effItr) {
//                  cout << *(*effItr) << " is maybe one shot\n";
                    maybeOneShotLiteral.insert(*effItr);
                }

            }


            {
                list<Literal*> & currEffectsList = actionsToEndEffects[i];

                list<Literal*>::iterator effItr = currEffectsList.begin();
                const list<Literal*>::iterator effEnd = currEffectsList.end();

                for (; effItr != effEnd; ++effItr) {
//                  cout << *(*effItr) << " is maybe one shot\n";
                    maybeOneShotLiteral.insert(*effItr);
                }


            }

        } else {
//          cout << " is > 1 shot, so can't do extra one-shot-literal-esque inference\n";
        }

    }



}

void RPGBuilder::oneShotInferForTILs()
{

    const int tilCount = timedInitialLiteralsVector.size();

    map<int, set<int> > & tilAdds = tilsThatAddFact;
    map<int, set<int> > & tilDels = tilsThatDeleteFact;

    for (int t = 0; t < tilCount; ++t) {
        FakeTILAction * const currTIL = timedInitialLiteralsVector[t];

        {
            list<Literal*> & effs = currTIL->delEffects;

            list<Literal*>::iterator effItr = effs.begin();
            const list<Literal*>::iterator effEnd = effs.end();

            for (; effItr != effEnd; ++effItr) {
                tilDels[(*effItr)->getID()].insert(t);
            }
        }

        {
            list<Literal*> & effs = currTIL->addEffects;

            list<Literal*>::iterator effItr = effs.begin();
            const list<Literal*>::iterator effEnd = effs.end();

            for (; effItr != effEnd; ++effItr) {
                tilAdds[(*effItr)->getID()].insert(t);
            }
        }
    }


    map<int, set<int> >::iterator delsItr = tilDels.begin();
    const map<int, set<int> >::iterator delsEnd = tilDels.end();

    for (; delsItr != delsEnd; ++delsItr) {
        if (tilAdds.find(delsItr->first) == tilAdds.end() && effectsToActions[delsItr->first].empty()) {
            // if nothing adds this fact, and this TIL deletes it, the it's simple deadline fact
            // and anything that needs it must happen before now; i.e. if it's an end precondition
            // we can shove that back to being an over all too, hence allowing start--end skipping

            list<pair<int, VAL::time_spec> > & affects = preconditionsToActions[delsItr->first];

            set<int> asOverAll;

            { // first pass - collect actions for which it's an invariant
                list<pair<int, VAL::time_spec> >::iterator affItr = affects.begin();
                const list<pair<int, VAL::time_spec> >::iterator affEnd = affects.end();

                for (; affItr != affEnd; ++affItr) {
                    if (affItr->second == VAL::E_OVER_ALL) asOverAll.insert(affItr->first);
                }
            }

            {   // second pass - for actions having it as an end precondition, but not as an invariant
                // add it as an invariant, too
                list<pair<int, VAL::time_spec> >::iterator affItr = affects.begin();
                const list<pair<int, VAL::time_spec> >::iterator affEnd = affects.end();

                for (; affItr != affEnd; ++affItr) {
                    if (affItr->second == VAL::E_AT_END
                            && asOverAll.find(affItr->first) == asOverAll.end()) {

                        affects.insert(affItr, pair<int, VAL::time_spec>(affItr->first, VAL::E_OVER_ALL));

                    }

                }
            }


        }
    }

}

void RPGBuilder::doSomeUsefulMetricRPGInference()
{

    const int opCount = instantiatedOps.size();
    const int litCount = literals.size();

    kShotFormulae = vector<KShotFormula*>(opCount, (KShotFormula*) 0);
    actionsToEndOneShots = vector<LiteralSet>(opCount);

    oneShotLiterals = vector<bool>(litCount, false);

    LiteralSet initialState;
    vector<double> initialFluents;

    getInitialState(initialState, initialFluents);

    const int fluentCount = initialFluents.size();

    MinimalState refState;
    refState.insertFacts(initialState.begin(), initialState.end(), StepAndBeforeOrAfter());

    refState.secondMin = initialFluents;
    refState.secondMax = initialFluents;

    LiteralSet maybeOneShotLiteral;



    for (int i = 0; i < opCount; ++i) {
        if (!rogueActions[i]) {
//          cout << "Considering whether " << *(getInstantiatedOp(i)) << " is one shot\n";
            kshotInferForAction(i, refState, maybeOneShotLiteral, initialFluents, fluentCount);
        }
    };

    set<int> revisit;

    while (!maybeOneShotLiteral.empty()) {

        revisit.clear();

        LiteralSet::iterator litItr = maybeOneShotLiteral.begin();
        const LiteralSet::iterator litEnd = maybeOneShotLiteral.end();

        for (; litItr != litEnd; ++litItr) {

            const int lID = (*litItr)->getID();

            list<pair<int, VAL::time_spec> > & eta = effectsToActions[lID];

            if (eta.size() == 1 && refState.first.find(lID) == refState.first.end()) { //

                const int actID = eta.front().first;
                if (kShotFormulae[actID]->getLimit(refState) <= 1) {
                    oneShotLiterals[lID] = true;
                    //cout << "Have shown that literal " << *(*litItr) << " is one shot: only added by " << *(getInstantiatedOp(actID)) << "\n";
                    list<pair<int, VAL::time_spec> >::iterator depItr = preconditionsToActions[lID].begin();
                    const list<pair<int, VAL::time_spec> >::iterator depEnd = preconditionsToActions[lID].end();

                    for (; depItr != depEnd; ++depItr) {
                        revisit.insert(depItr->first);
                    }

                    if (eta.front().second == VAL::E_AT_START) {
                        list<Literal*> & ene = actionsToEndNegativeEffects[actID];
                        list<Literal*>::iterator eneItr = ene.begin();
                        const list<Literal*>::iterator eneEnd = ene.end();

                        for (; eneItr != eneEnd; ++eneItr) {
                            if (*eneItr == *litItr) {
                                //cout << "Literal is one-shot wrapped\n";
                                actionsToEndOneShots[actID].insert(*eneItr);
                                break;
                            }
                        }

                    }

                } else {
//                  cout << *(*litItr) << " isn't one-shot: the single achieving action, " << *(getInstantiatedOp(actID)) << ", can be applied " << kShotFormulae[actID]->getLimit(refState) << " times\n";
                }
            } else {
//              if (eta.size() > 1) cout << *(*litItr) << " isn't one shot: is added by " << eta.size() << " actions\n";
            }

        }

        maybeOneShotLiteral.clear();

        set<int>::iterator riItr = revisit.begin();
        const set<int>::iterator riEnd = revisit.end();

        for (; riItr != riEnd; ++riItr) {
            if (!rogueActions[*riItr]) {
//              cout << "Revisiting " << *(getInstantiatedOp(*riItr)) << " due to one-shot literal\n";
                kshotInferForAction(*riItr, refState, maybeOneShotLiteral, initialFluents, fluentCount);
            }
        }
    }

};


bool RPGBuilder::isInteresting(const int & act, const map<int, PropositionAnnotation> & facts, const map<int, set<int> > & started)
{
    const map<int, int>::iterator intrItr = uninterestingnessCriteria.find(act);
    if (intrItr == uninterestingnessCriteria.end()) return true;
    const int criterion = intrItr->second;
    if (criterion == -1) return false;
    if (facts.find(criterion) != facts.end()) return false;
    if (started.find(act) != started.end()) return false;

    return true;
};


void removeFirst(list<pair<int, VAL::time_spec> > & from, const pair<int, VAL::time_spec> & toRemove)
{

    list<pair<int, VAL::time_spec> >::iterator fItr = from.begin();
    const list<pair<int, VAL::time_spec> >::iterator fEnd = from.end();

    for (; fItr != fEnd; ++fItr) {
        if (*fItr == toRemove) {
            from.erase(fItr);
            return;
        }
    }

};

bool RPGBuilder::considerAndFilter(LiteralSet & initialState, LiteralSet & revisit, const int & operatorID)
{


    bool localDebug = (Globals::globalVerbosity & 131072);

    if (localDebug) cout << "Considering pruning " << *(getInstantiatedOp(operatorID)) << "\n";

    bool revisitUpdated = false;

    bool eliminate = false;

    for (int pass = 0; pass < 3; ++pass) {

        list<Literal*> * currList = 0;

        switch (pass) {
        case 0: {
            currList = &actionsToStartPreconditions[operatorID];
            if (localDebug) cout << "Start Pres:\n";
            break;
        }
        case 1: {
            currList = &actionsToInvariants[operatorID];
            if (localDebug) cout << "Invs:\n";
            break;
        }
        case 2: {
            currList = &actionsToEndPreconditions[operatorID];
            if (localDebug) cout << "End Pres:\n";
            break;
        }
        };
        list<Literal*>::iterator llItr = currList->begin();
        const list<Literal*>::iterator llEnd = currList->end();

        for (; llItr != llEnd; ++llItr) {
            pair<bool, bool> & currStatic = isStatic(*llItr);
            if (currStatic.first) {
                if (!currStatic.second) {
                    #ifdef ENABLE_DEBUGGING_HOOKS
                    {
                        ostringstream s;
                        s << "Has a precondition " << *(*llItr) << " that is static and false";                        
                        Globals::eliminatedAction(operatorID, s.str().c_str());
                    }
                    #endif
                                    
                    if (localDebug) cout << *(*llItr) << " is static and initially false\n";
                    eliminate = true;
                    break;

                }
            } else {
                if (initialState.find(*llItr) == initialState.end()) {
                    if (effectsToActions[(*llItr)->getID()].empty()) {
                        #ifdef ENABLE_DEBUGGING_HOOKS
                        {
                            ostringstream s;
                            s << "Has a precondition " << *(*llItr) << " that is false in the initial state, and never added by an action\n";
                            Globals::eliminatedAction(operatorID, s.str().c_str());
                        }
                        #endif

                        if (localDebug) cout << "Nothing adds " << *(*llItr) << " and it isn't in the initial state\n";
                        eliminate = true;
                        break;
                    }
                }
            }
            if (localDebug) cout << "\t" << *(*llItr) << " is okay\n";
        }
        if (eliminate) break;
    }

    if (eliminate) {
        
        if (localDebug || (Globals::globalVerbosity & 16)) cout << "Pruning action " << *(getInstantiatedOp(operatorID)) << "\n";
        set<int> noLongerGet;
        {
            list<Literal*> & currEffectsList = actionsToStartEffects[operatorID];

            list<Literal*>::iterator effItr = currEffectsList.begin();
            const list<Literal*>::iterator effEnd = currEffectsList.end();


            for (; effItr != effEnd; ++effItr) {
                const int effID = (*effItr)->getID();
                removeFirst(effectsToActions[effID], pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
                if (effectsToActions[effID].empty()) {
                    noLongerGet.insert(effID);
                }
            }
            currEffectsList.clear();

        }

        {
            list<Literal*> & currEffectsList = actionsToStartNegativeEffects[operatorID];

            list<Literal*>::iterator effItr = currEffectsList.begin();
            const list<Literal*>::iterator effEnd = currEffectsList.end();


            for (; effItr != effEnd; ++effItr) {
                const int effID = (*effItr)->getID();

                removeFirst(negativeEffectsToActions[effID], pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
            }
            currEffectsList.clear();

        }

        {
            list<Literal*> & currEffectsList = actionsToEndEffects[operatorID];

            list<Literal*>::iterator effItr = currEffectsList.begin();
            const list<Literal*>::iterator effEnd = currEffectsList.end();

            for (; effItr != effEnd; ++effItr) {
                const int effID = (*effItr)->getID();
                removeFirst(effectsToActions[effID], pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
                if (effectsToActions[effID].empty()) {
                    noLongerGet.insert(effID);
                }

            }
            currEffectsList.clear();

        }

        {
            list<Literal*> & currEffectsList = actionsToEndNegativeEffects[operatorID];

            list<Literal*>::iterator effItr = currEffectsList.begin();
            const list<Literal*>::iterator effEnd = currEffectsList.end();

            for (; effItr != effEnd; ++effItr) {
                const int effID = (*effItr)->getID();
                removeFirst(negativeEffectsToActions[effID], pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
            }
            currEffectsList.clear();

        }

        {

            list<Literal*> & currPreconditionsList = actionsToStartPreconditions[operatorID];

            list<Literal*>::iterator precItr = currPreconditionsList.begin();
            const list<Literal*>::iterator precEnd = currPreconditionsList.end();

            for (; precItr != precEnd; ++precItr) {
                const int precID = (*precItr)->getID();
                removeFirst(preconditionsToActions[precID], pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));

            }
            currPreconditionsList.clear();
        }


        {
            list<Literal*> & currPreconditionsList = actionsToInvariants[operatorID];

            list<Literal*>::iterator precItr = currPreconditionsList.begin();
            const list<Literal*>::iterator precEnd = currPreconditionsList.end();

            for (; precItr != precEnd; ++precItr) {
                const int precID = (*precItr)->getID();
                removeFirst(preconditionsToActions[precID], pair<int, VAL::time_spec>(operatorID, VAL::E_OVER_ALL));

            }
            currPreconditionsList.clear();

        }


        {
            list<Literal*> & currPreconditionsList = actionsToEndPreconditions[operatorID];

            list<Literal*>::iterator precItr = currPreconditionsList.begin();
            const list<Literal*>::iterator precEnd = currPreconditionsList.end();

            for (; precItr != precEnd; ++precItr) {
                const int precID = (*precItr)->getID();
                removeFirst(preconditionsToActions[precID], pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
                removeFirst(processedPreconditionsToActions[precID], pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
            }

            if (currPreconditionsList.empty()) {
                removeFirst(preconditionlessActions, pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
            }
            currPreconditionsList.clear();
        }

        {
            list<Literal*> & newStartPrecs = actionsToProcessedStartPreconditions[operatorID];
            if (newStartPrecs.empty()) {
                removeFirst(preconditionlessActions, pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
            }

            list<Literal*>::iterator precItr = newStartPrecs.begin();
            const list<Literal*>::iterator precEnd = newStartPrecs.end();

            for (; precItr != precEnd; ++precItr) {
                const int precID = (*precItr)->getID();
                removeFirst(processedPreconditionsToActions[precID], pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
            }

            newStartPrecs.clear();

        }

        set<int>::iterator nlgItr = noLongerGet.begin();
        const set<int>::iterator nlgEnd = noLongerGet.end();

        for (; nlgItr != nlgEnd; ++nlgItr) {
            revisit.insert(literals[*nlgItr]);

        }

        #ifdef ENABLE_DEBUGGING_HOOKS
        {
            Globals::eliminatedAction(operatorID, "Has one or more unreachable preconditions");
        }
        #endif
                                                                        
        realRogueActions[operatorID] = true;

    } else {
        if (localDebug) cout << "Keeping it\n";
    }

    return revisitUpdated;
};



void RPGBuilder::postFilterUnreachableActions()
{

    const int opCount = instantiatedOps.size();

    LiteralSet initialState;
    vector<double> initialFluents;

    getInitialState(initialState, initialFluents);

    cout << "Post filtering unreachable actions: ";
    cout.flush();

    const int percentageAt = (opCount / 10);

    LiteralSet revisit;
    for (int operatorID = 0; operatorID < opCount; ++operatorID) {
        if (percentageAt) {
            if (!((operatorID + 1) % percentageAt)) {
                cout << " [" << ((operatorID + 1) / percentageAt) << "0%]";
                cout.flush();
            }
        }
        if (!rogueActions[operatorID]) {
            considerAndFilter(initialState, revisit, operatorID);
        }
    }

    cout << "\n";

    set<int> opRevisit;
    set<int> dpRevisit;

    {
        LiteralSet::iterator rvItr = revisit.begin();
        const LiteralSet::iterator rvEnd = revisit.end();

        for (; rvItr != rvEnd; ++rvItr) {
            list<pair<int, VAL::time_spec> >::iterator depItr = processedPreconditionsToActions[(*rvItr)->getID()].begin();
            const list<pair<int, VAL::time_spec> >::iterator depEnd = processedPreconditionsToActions[(*rvItr)->getID()].end();

            for (; depItr != depEnd; ++depItr) {
                opRevisit.insert(depItr->first);
            }
        }

    }


    set<int>::iterator rvItr = opRevisit.begin();
    set<int>::iterator rvEnd = opRevisit.end();
    while (rvItr != rvEnd) {
        const int operatorID = *rvItr;
        if (!rogueActions[operatorID]) {
            if (considerAndFilter(initialState, revisit, operatorID)) {
                opRevisit.erase(rvItr);

                {
                    LiteralSet::iterator rvItr = revisit.begin();
                    const LiteralSet::iterator rvEnd = revisit.end();

                    for (; rvItr != rvEnd; ++rvItr) {
                        list<pair<int, VAL::time_spec> >::iterator depItr = processedPreconditionsToActions[(*rvItr)->getID()].begin();
                        const list<pair<int, VAL::time_spec> >::iterator depEnd = processedPreconditionsToActions[(*rvItr)->getID()].end();

                        for (; depItr != depEnd; ++depItr) {
                            opRevisit.insert(depItr->first);
                        }
                    }

                }

                revisit.clear();

                rvItr = opRevisit.begin();
            } else {
                set<int>::iterator rvPrev = rvItr;
                ++rvItr;
                opRevisit.erase(rvPrev);
            }
        } else {
            set<int>::iterator rvPrev = rvItr;
            ++rvItr;
            opRevisit.erase(rvPrev);
        }

    }


};

pair<bool, bool> & RPGBuilder::isStatic(Literal * l)
{
    return staticLiterals[l->getID()];
}


void RPGBuilder::findStaticLiterals()
{

    LiteralSet initialState;
    vector<double> initialFluents;

    getInitialState(initialState, initialFluents);

    LiteralSet tilAdds;
    LiteralSet tilDels;

    list<RPGBuilder::FakeTILAction> & tilActs = RPGBuilder::getTILs();

    list<RPGBuilder::FakeTILAction>::iterator tilItr = tilActs.begin();
    const list<RPGBuilder::FakeTILAction>::iterator tilEnd = tilActs.end();

    for (; tilItr != tilEnd; ++tilItr) {
        tilAdds.insert(tilItr->addEffects.begin(), tilItr->addEffects.end());
        tilDels.insert(tilItr->delEffects.begin(), tilItr->delEffects.end());
    }


    const int litCount = literals.size();

    staticLiterals = vector<pair<bool, bool> >(litCount);

    for (int i = 0; i < litCount; ++i) {
        if (effectsToActions[i].empty() && negativeEffectsToActions[i].empty()
                && tilAdds.find(literals[i]) == tilAdds.end() && tilDels.find(literals[i]) == tilDels.end()) {
            staticLiterals[i] = pair<bool, bool>(true, initialState.find(literals[i]) != initialState.end());
            /*cout << *(literals[i]) << " is static";
            if (!staticLiterals[i].second) {
            cout << ", initially false\n";
            } else {
            cout << ", initially true\n";
            }*/
            preconditionsToActions[i].clear();
            processedPreconditionsToActions[i].clear();
            negativePreconditionsToActions[i].clear();
            processedNegativePreconditionsToActions[i].clear();
        } else {
            staticLiterals[i] = pair<bool, bool>(false, false);
        }
    }

}

void RPGBuilder::pruneStaticPreconditions(list<Literal*> & toPrune, int & toDec)
{
    list<Literal*>::iterator tpItr = toPrune.begin();
    const list<Literal*>::iterator tpEnd = toPrune.end();

    while (tpItr != tpEnd) {
        if (isStatic(*tpItr).first) {
            list<Literal*>::iterator tpPrev = tpItr;
            ++tpItr;
            toPrune.erase(tpPrev);
            --toDec;
        } else {
            ++tpItr;
        }
    }
}

void RPGBuilder::pruneStaticPreconditions()
{


    const int opCount = instantiatedOps.size();
    for (int i = 0; i < opCount; ++i) {
        if (!rogueActions[i]) {
            {
                int & toDec = initialUnsatisfiedStartPreconditions[i];
                pruneStaticPreconditions(actionsToStartPreconditions[i], toDec);
            }
            {
                int dummy = 0;
                pruneStaticPreconditions(actionsToStartNegativePreconditions[i], dummy);
            }
            {
                int & toDec = initialUnsatisfiedProcessedStartPreconditions[i];
                const int before = toDec;
                pruneStaticPreconditions(actionsToProcessedStartPreconditions[i], toDec);
                if (!toDec && before) {
                    preconditionlessActions.push_back(pair<int, VAL::time_spec>(i, VAL::E_AT_START));
                }
            }
            {
                int dummy = 0;
                pruneStaticPreconditions(actionsToProcessedStartNegativePreconditions[i], dummy);
            }

            {
                int & toDec = initialUnsatisfiedInvariants[i];
                pruneStaticPreconditions(actionsToInvariants[i], toDec);
            }
            {
                int dummy = 0;
                pruneStaticPreconditions(actionsToNegativeInvariants[i], dummy);
            }

            {
                int & toDec = initialUnsatisfiedEndPreconditions[i];
                const int before = toDec;
                pruneStaticPreconditions(actionsToEndPreconditions[i], toDec);
                if (!toDec && before) {
                    preconditionlessActions.push_back(pair<int, VAL::time_spec>(i, VAL::E_AT_END));
                }
            }
            {
                int dummy = 0;
                pruneStaticPreconditions(actionsToEndNegativePreconditions[i], dummy);
            }
        }
    }



};


void RPGBuilder::findActionTimestampLowerBounds()
{

    if (!globalHeuristic) {
        globalHeuristic = RPGBuilder::generateRPGHeuristic();
    }

    RPGHeuristic* const currRPG = globalHeuristic;

    LiteralSet initialState;
    vector<double> initialFluents;

    getInitialState(initialState, initialFluents);


    MinimalState refState;
    refState.insertFacts(initialState.begin(), initialState.end(), StepAndBeforeOrAfter());

    refState.secondMin = initialFluents;
    refState.secondMax = initialFluents;


//  refState.derivePredicates();

    currRPG->doFullExpansion(refState);

    const int actCount = actionsToStartPreconditions.size();

    for (int a = 0; a < actCount; ++a) {

        if (!rogueActions[a]) {

            bool isRogue = false;

            double earliestStart = RPGHeuristic::getEarliestForStarts()[a];
            double earliestEnd = RPGHeuristic::getEarliestForEnds()[a];

            if (earliestStart != DBL_MAX && earliestEnd != DBL_MAX) {
                //cout << "Initial RPG bounds " << *(getInstantiatedOp(a)) << " no earlier than " << earliestStart << " ; " << earliestEnd << "\n";
                const double maxActDur = getOpMaxDuration(a, -1);
                if (earliestStart < (earliestEnd - maxActDur) - 0.0005) {
                    earliestStart = earliestEnd - maxActDur;
//                  cout << "Maximum act duration = " << maxActDur << ", so pulling start to " << earliestStart << "\n";
                }

                TemporalAnalysis::suggestNewStartLowerBound(a, earliestStart);
                TemporalAnalysis::suggestNewEndLowerBound(a, earliestEnd);

                isRogue = TemporalAnalysis::actionIsNeverApplicable(a);

                if (isRogue) {
                    cout << "Pruning " << *(getInstantiatedOp(a)) << " - temporal contradiction\n";
                }
            } else {
                cout << "Pruning " << *(getInstantiatedOp(a)) << " - never appeared in initial RPG\n";
                isRogue = true;
            }

            if (isRogue) {

                pruneIrrelevant(a);

            }

        }

    }

}


struct NextRegress {

    int fact;
    int fluent;
    double ts;
    set<int> forGoal;


    NextRegress(const int & a, const int & b, const double & c) : fact(a), fluent(b), ts(c) {};
    NextRegress(const int & a, const int & b, const double & c, const set<int> & d) : fact(a), fluent(b), ts(c), forGoal(d) {};

    bool operator <(const NextRegress & o) const {
        if (ts > o.ts) return true;
        if (ts < o.ts) return true;

        if (fact < o.fact) return true;
        if (fact < o.fact) return false;

        if (fluent < o.fluent) return true;
        if (fluent > o.fluent) return false;

        return false;
    }

};


class RPGBuilder::CommonRegressionData
{

private:

    const int fluentCount;

    vector<map<int, double> > literalSeenForGoalAtTime;
    vector<map<int, double> > fluentSeenForGoalAtTime;
    vector<map<int, double> > opAppliedForGoalAtTime;
    vector<map<int, double> > dpAppliedForGoalAtTime;
    vector<map<int, set<VAL::time_spec> > > fluentAffectedBy;

    set<NextRegress> expansionQueue;

    void requestVisitHandler(const int & literal, const int & fluent, const double & time, set<int> forGoal) {

        if (time < 0.0) return;

        map<int, double> & bestSeen = (literal != -1 ? literalSeenForGoalAtTime[literal] : fluentSeenForGoalAtTime[fluent]);

        set<int>::iterator whittle = forGoal.begin();

        while (whittle != forGoal.end()) {
            double & oldTime = bestSeen.insert(make_pair(*whittle, -1.0)).first->second;
            if (oldTime < time) {
                oldTime = time;
                ++whittle;
            } else {
                set<int>::iterator toDel = whittle++;
                forGoal.erase(toDel);
            }
        }

        if (forGoal.empty()) return;

        NextRegress newNR(literal, fluent, time, forGoal);
        pair<set<NextRegress>::iterator, bool> ins = expansionQueue.insert(newNR);
        if (!ins.second) {
            const_cast<set<int>* >(&(ins.first->forGoal))->insert(forGoal.begin(), forGoal.end());
        }
    }

public:

    CommonRegressionData(const int & lc, const int  & fc, const int & ac, const int & dc)
            : fluentCount(fc), literalSeenForGoalAtTime(lc), fluentSeenForGoalAtTime(fc), opAppliedForGoalAtTime(ac), dpAppliedForGoalAtTime(dc), fluentAffectedBy(fc) {



        {
            vector<list<pair<int, VAL::time_spec> > >::const_iterator numEffItr = rpgNumericEffectsToActions.begin();
            const vector<list<pair<int, VAL::time_spec> > >::const_iterator numEffEnd = rpgNumericEffectsToActions.end();

            for (int i = 0; numEffItr != numEffEnd; ++numEffItr, ++i) {

                const int affFluent = rpgNumericEffects[i].fluentIndex;

                bool proceed = false;

                if (NumericAnalysis::getDominanceConstraints()[affFluent] == NumericAnalysis::E_NODOMINANCE) {

                    proceed = true;

                } else if (NumericAnalysis::getDominanceConstraints()[affFluent] == NumericAnalysis::E_BIGGERISBETTER) {

                    RPGNumericEffect & currEff = rpgNumericEffects[i];

                    proceed = (currEff.isAssignment || (currEff.constant > 0) || (currEff.size > 0));

                } else if (NumericAnalysis::getDominanceConstraints()[affFluent] == NumericAnalysis::E_SMALLERISBETTER) {

                    RPGNumericEffect & currEff = rpgNumericEffects[i];

                    proceed = (currEff.isAssignment || (currEff.constant < 0) || (currEff.size > 0));

                }

                if (proceed) {
                    map<int, set<VAL::time_spec> > & destSet = fluentAffectedBy[affFluent];

                    const list<pair<int, VAL::time_spec> > & achievers = *numEffItr;
                    list<pair<int, VAL::time_spec> >::const_iterator accItr = achievers.begin();
                    const list<pair<int, VAL::time_spec> >::const_iterator accEnd = achievers.end();
                    for (; accItr != accEnd; ++accItr) {
                        destSet[accItr->first].insert(accItr->second);
                    }
                } else {
                    //cout << "Effect " << i << " is not interesting in its own right\n";
                }
            }
        }
        {
            for (int i = 0; i < ac; ++i) {
                LinearEffects* const currLE = linearDiscretisation[i];
                if (!rogueActions[i] && currLE) {
                    const int looplim = currLE->vars.size();
                    for (int s = 0; s < looplim; ++s) {
                        const int currVar = currLE->vars[s];
                        fluentAffectedBy[currVar][i].insert(VAL::E_AT_START);
                    }
                }
            }
        }

    };

    void requestLiteralVisit(const int & literal, const double & time, const set<int> & forGoal) {
        requestVisitHandler(literal, -1, time, forGoal);
    };

    void requestLiteralVisit(const int & literal, const double & time, const int & forGoal) {
        set<int> tmp; tmp.insert(forGoal);
        requestLiteralVisit(literal, time, tmp);
    };

    void requestFluentVisit(const int & fluent, const double & time, const set<int> & forGoal) {
        requestVisitHandler(-1, fluent, time, forGoal);
    };

    void requestFluentVisit(const int & fluent, const double & time, const int & forGoal) {
        set<int> tmp; tmp.insert(forGoal);
        requestFluentVisit(fluent, time, tmp);
    };

    typedef set<NextRegress>::iterator iterator;

    iterator begin() {
        return expansionQueue.begin();
    };
    const iterator end() {
        return expansionQueue.end();
    };
    void erase(const iterator & i) {
        expansionQueue.erase(i);
    };

    bool empty() const {
        return expansionQueue.empty();
    };

    map<int, set<VAL::time_spec> > & relevantToFluent(const int & currVar) {
        return fluentAffectedBy[currVar];
    }

    void regressThroughAction(const int & actID, const VAL::time_spec & ts, const NextRegress & inAidOf) {

        static const bool debug = false;

        if (ts == VAL::E_AT) return;

        if (ts == VAL::E_AT_START) {
            if (TemporalAnalysis::getActionTSBounds()[actID][0].first > inAidOf.ts) return;
        } else {
            if (TemporalAnalysis::getActionTSBounds()[actID][1].first > inAidOf.ts) return;
        }

        set<int> forGoal(inAidOf.forGoal);

        {
            double atTime = inAidOf.ts;
            if (ts == VAL::E_AT_END && atTime != DBL_MAX) atTime -= getOpMinDuration(actID, -1);

            pair<double, double> & tsBounds = TemporalAnalysis::getActionTSBounds()[actID][0];

            if (atTime < tsBounds.first) return;
            if (atTime > tsBounds.second) atTime = tsBounds.second;

            set<int>::iterator fgItr = forGoal.begin();
            const set<int>::iterator fgEnd = forGoal.end();

            while (fgItr != fgEnd) {
                double & insAt = opAppliedForGoalAtTime[actID].insert(make_pair(*fgItr, -DBL_MAX)).first->second;
                if (insAt >= atTime) {
                    set<int>::iterator fgDel = fgItr++;
                    forGoal.erase(fgDel);
                } else {
                    insAt = atTime;
                    ++fgItr;
                }
            }
        }

        if (forGoal.empty()) return;

        static const double EPSILON = 0.001;
        for (int pass = 0; pass < 3; ++pass) {
            list<Literal*> * currList = 0;
            list<int> * currNumList = 0;
            double atTime = inAidOf.ts;
            switch (pass) {
            case 0: {
                currList = &actionsToProcessedStartPreconditions[actID];
                currNumList = &actionsToRPGNumericStartPreconditions[actID];
                if (ts == VAL::E_AT_START) {
                    if (atTime != DBL_MAX) atTime -= EPSILON;
                } else {
                    if (atTime != DBL_MAX) {
                        if (debug) cout << "Duration of " << *(getInstantiatedOp(actID)) << " = " << getOpMinDuration(actID, -1) << "\n";
                        atTime -= getOpMinDuration(actID, -1);
                    }
                }
                if (debug) {
                    cout << "Adding pres for start of " << *(getInstantiatedOp(actID)) << " at time ";
                    if (atTime == DBL_MAX) {
                        cout << "infinity\n";
                    } else {
                        cout << atTime << "\n";
                    }
                }
                break;
            }
            case 1: {
                currList = 0;
                currNumList = &actionsToRPGNumericInvariants[actID];
                if (ts == VAL::E_AT_START) {
                    if (atTime != DBL_MAX) atTime += getOpMaxDuration(actID, -1);
                } else {
                    if (atTime != DBL_MAX) atTime -= EPSILON;
                }
                break;
            }
            case 2: {
                currList = &actionsToEndPreconditions[actID];
                currNumList = &actionsToRPGNumericEndPreconditions[actID];
                if (ts == VAL::E_AT_START) {
                    if (atTime != DBL_MAX) atTime += getOpMaxDuration(actID, -1);
                } else {
                    if (atTime != DBL_MAX) atTime -= EPSILON;
                }
                if (debug) {
                    cout << "Adding pres for end of " << *(getInstantiatedOp(actID)) << " at time ";
                    if (atTime == DBL_MAX) {
                        cout << "infinity\n";
                    } else {
                        cout << atTime << "\n";
                    }
                }
                break;
            }
            };
            if (currList) {
                list<Literal*>::iterator preItr = currList->begin();
                const list<Literal*>::iterator preEnd = currList->end();

                for (; preItr != preEnd; ++preItr) {
                    const int litID = (*preItr)->getID();
                    requestLiteralVisit(litID, atTime, forGoal);
                }
            }
            list<int>::iterator numPreItr = currNumList->begin();
            const list<int>::iterator numPreEnd = currNumList->end();

            for (; numPreItr != numPreEnd; ++numPreItr) {
                int fID = rpgNumericPreconditions[*numPreItr].LHSVariable;
                if (fID < 0) {

                } else if (fID < fluentCount) {
                    requestFluentVisit(fID, atTime, forGoal);
                } else if (fID < (2 * fluentCount)) {
                    fID -= fluentCount;
                    assert(fID < fluentCount);
                    requestFluentVisit(fID, atTime, forGoal);
                } else {
                    ArtificialVariable & currAV = getArtificialVariable(fID);

                    const int size = currAV.size;
                    for (int i = 0; i < size; ++i) {
                        int afID = currAV.fluents[i];

                        if (afID >= fluentCount) afID -= fluentCount;
                        requestFluentVisit(afID, atTime, forGoal);
                    }
                }
            }
        }
    }

    /*  void regressThroughDerivationRule(const int & ruleID, const NextRegress & inAidOf) {

        set<int> forGoal(inAidOf.forGoal);

        {
        const double atTime = inAidOf.ts;

        if (atTime < 0.0) return;

        set<int>::iterator fgItr = forGoal.begin();
        const set<int>::iterator fgEnd = forGoal.end();

        while (fgItr != fgEnd) {
        double & insAt = dpAppliedForGoalAtTime[ruleID].insert(make_pair(*fgItr,-DBL_MAX)).first->second;
        if (insAt >= atTime) {
        set<int>::iterator fgDel = fgItr++;
        forGoal.erase(fgDel);
    } else {
        insAt = atTime;
        ++fgItr;
    }
    }
    }

        if (forGoal.empty()) return;

        {
        list<int> & currList = DerivedPredicatesEngine::getPreLiterals(ruleID);

        list<int>::iterator preItr = currList.begin();
        const list<int>::iterator preEnd = currList.end();

        for (; preItr != preEnd; ++preItr) {
        const int litID = (*preItr);
        requestLiteralVisit(litID, inAidOf.ts, forGoal);
    }
    }
        {
        list<int> * currList = &(DerivedPredicatesEngine::getPreNums(ruleID));
        list<int>::iterator preItr = currList->begin();
        const list<int>::iterator preEnd = currList->end();

        for (; preItr != preEnd; ++preItr) {
        int fID = rpgNumericPreconditions[*preItr].LHSVariable;
        if (fID < 0) {

    } else if (fID < fluentCount) {
        assert(fID < fluentCount);
        requestFluentVisit(fID, inAidOf.ts, forGoal);
    } else if (fID < (2 * fluentCount)) {
        fID -= fluentCount;
        assert(fID < fluentCount);
        requestFluentVisit(fID, inAidOf.ts, inAidOf.forGoal);
    } else {
        ArtificialVariable & currAV = getArtificialVariable(fID);

        const int size = currAV.size;
        for (int i = 0; i < size; ++i) {
        int afID = currAV.fluents[i];

        if (afID >= fluentCount) afID -= fluentCount;

        assert(afID < fluentCount);
        requestFluentVisit(afID, inAidOf.ts, forGoal);
    }
    }
    }
    }
    }
    */
    bool opNeverApplied(const int & i) const {
        return (opAppliedForGoalAtTime[i].empty());
    };

    double latestUsefulPoint(const int & op) {
        double toReturn = 0.0;
        map<int, double>::iterator fgItr = opAppliedForGoalAtTime[op].begin();
        const map<int, double>::iterator fgEnd = opAppliedForGoalAtTime[op].end();

        for (; fgItr != fgEnd; ++fgItr) {
            const double & currT = fgItr->second;
            if (currT == DBL_MAX) return DBL_MAX;
            if (currT > toReturn) toReturn = currT;
        }

        return toReturn;
    }
};

void RPGBuilder::pruneIrrelevant(const int & operatorID)
{

    #ifdef ENABLE_DEBUGGING_HOOKS
    Globals::eliminatedAction(operatorID, "No reason known - has been passed to pruneIrrelevant");
    #endif
    
    realRogueActions[operatorID] = true;

    {
        list<Literal*> & currEffectsList = actionsToStartEffects[operatorID];

        list<Literal*>::iterator effItr = currEffectsList.begin();
        const list<Literal*>::iterator effEnd = currEffectsList.end();


        for (; effItr != effEnd; ++effItr) {
            const int effID = (*effItr)->getID();
            effectsToActions[effID].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
        }
        currEffectsList.clear();

    }

    {
        list<Literal*> & currEffectsList = actionsToStartNegativeEffects[operatorID];

        list<Literal*>::iterator effItr = currEffectsList.begin();
        const list<Literal*>::iterator effEnd = currEffectsList.end();


        for (; effItr != effEnd; ++effItr) {
            const int effID = (*effItr)->getID();
            negativeEffectsToActions[effID].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
        }
        currEffectsList.clear();

    }

    {
        list<Literal*> & currEffectsList = actionsToEndEffects[operatorID];

        list<Literal*>::iterator effItr = currEffectsList.begin();
        const list<Literal*>::iterator effEnd = currEffectsList.end();

        for (; effItr != effEnd; ++effItr) {
            const int effID = (*effItr)->getID();
            effectsToActions[effID].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
        }
        currEffectsList.clear();

    }

    {
        list<Literal*> & currEffectsList = actionsToEndNegativeEffects[operatorID];

        list<Literal*>::iterator effItr = currEffectsList.begin();
        const list<Literal*>::iterator effEnd = currEffectsList.end();

        for (; effItr != effEnd; ++effItr) {
            const int effID = (*effItr)->getID();
            negativeEffectsToActions[effID].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
        }
        currEffectsList.clear();

    }

    {

        list<Literal*> & currPreconditionsList = actionsToStartPreconditions[operatorID];

        list<Literal*>::iterator precItr = currPreconditionsList.begin();
        const list<Literal*>::iterator precEnd = currPreconditionsList.end();

        for (; precItr != precEnd; ++precItr) {
            const int precID = (*precItr)->getID();
            preconditionsToActions[precID].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));

        }
        currPreconditionsList.clear();
    }


    {
        list<Literal*> & currPreconditionsList = actionsToInvariants[operatorID];

        list<Literal*>::iterator precItr = currPreconditionsList.begin();
        const list<Literal*>::iterator precEnd = currPreconditionsList.end();

        for (; precItr != precEnd; ++precItr) {
            const int precID = (*precItr)->getID();
            preconditionsToActions[precID].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_OVER_ALL));

        }
        currPreconditionsList.clear();

    }


    {
        list<Literal*> & currPreconditionsList = actionsToEndPreconditions[operatorID];

        list<Literal*>::iterator precItr = currPreconditionsList.begin();
        const list<Literal*>::iterator precEnd = currPreconditionsList.end();

        for (; precItr != precEnd; ++precItr) {
            const int precID = (*precItr)->getID();
            preconditionsToActions[precID].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
            processedPreconditionsToActions[precID].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
        }

        if (currPreconditionsList.empty()) {
            preconditionlessActions.remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
            onlyNumericPreconditionActions.remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
        }
        currPreconditionsList.clear();
    }

    {
        list<Literal*> & newStartPrecs = actionsToProcessedStartPreconditions[operatorID];
        if (newStartPrecs.empty()) {
            preconditionlessActions.remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
            onlyNumericPreconditionActions.remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
        }

        list<Literal*>::iterator precItr = newStartPrecs.begin();
        const list<Literal*>::iterator precEnd = newStartPrecs.end();

        for (; precItr != precEnd; ++precItr) {
            const int precID = (*precItr)->getID();
            processedPreconditionsToActions[precID].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
        }

        newStartPrecs.clear();

    }

    {
        list<int> & currPreconditionsList = actionsToRPGNumericStartPreconditions[operatorID];

        list<int>::iterator precItr = currPreconditionsList.begin();
        const list<int>::iterator precEnd = currPreconditionsList.end();
        for (; precItr != precEnd; ++precItr) {
            const int precID = *precItr;
            rpgNumericPreconditionsToActions[precID].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
        }
        currPreconditionsList.clear();
    }

    {
        list<int> & currPreconditionsList = actionsToRPGNumericInvariants[operatorID];

        list<int>::iterator precItr = currPreconditionsList.begin();
        const list<int>::iterator precEnd = currPreconditionsList.end();
        for (; precItr != precEnd; ++precItr) {
            const int precID = *precItr;
            rpgNumericPreconditionsToActions[precID].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_OVER_ALL));
        }
        currPreconditionsList.clear();
    }

    {
        list<int> & currPreconditionsList = actionsToProcessedStartRPGNumericPreconditions[operatorID];

        list<int>::iterator precItr = currPreconditionsList.begin();
        const list<int>::iterator precEnd = currPreconditionsList.end();
        for (; precItr != precEnd; ++precItr) {
            const int precID = *precItr;
            processedRPGNumericPreconditionsToActions[precID].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
        }

        currPreconditionsList.clear();
    }


    {
        list<int> & currPreconditionsList = actionsToRPGNumericEndPreconditions[operatorID];

        list<int>::iterator precItr = currPreconditionsList.begin();
        const list<int>::iterator precEnd = currPreconditionsList.end();
        for (; precItr != precEnd; ++precItr) {
            const int precID = *precItr;
            rpgNumericPreconditionsToActions[precID].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
            processedRPGNumericPreconditionsToActions[precID].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
        }

        currPreconditionsList.clear();
    }

    {
        list<int> & currEffList = getStartEffNumerics()[operatorID];

        list<int>::iterator effItr = currEffList.begin();
        const list<int>::iterator effEnd = currEffList.end();
        for (; effItr != effEnd; ++effItr) {
            rpgNumericEffectsToActions[*effItr].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
        }

        currEffList.clear();
    }

    {
        list<int> & currEffList = getEndEffNumerics()[operatorID];

        list<int>::iterator effItr = currEffList.begin();
        const list<int>::iterator effEnd = currEffList.end();
        for (; effItr != effEnd; ++effItr) {
            rpgNumericEffectsToActions[*effItr].remove(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
        }

        currEffList.clear();
    }

    delete linearDiscretisation[operatorID];
    linearDiscretisation[operatorID] = 0;


}

void RPGBuilder::postFilterIrrelevantActions()
{

    const int opCount = instantiatedOps.size();
    const int dpCount = 0; // DerivedPredicatesEngine::getRuleCount();
    const int litCount = literals.size();
    const int fluentCount = pnes.size();

    CommonRegressionData data(litCount, fluentCount, opCount, dpCount);

//  interestingActionUntilWeHaveLiteral = vector<set<int> >(opCount);
//  interestingActionUntilWeHaveNumeric = vector<set<int> >(opCount);


    {
        list<Literal*>::iterator afItr = literalGoals.begin();
        const list<Literal*>::iterator afEnd = literalGoals.end();

        list<double>::iterator gdItr = literalGoalDeadlines.begin();

        for (int i = 0; afItr != afEnd; ++afItr, ++gdItr, ++i) {
            data.requestLiteralVisit((*afItr)->getID(), *gdItr, i);
        }
    }

    {
        list<pair<int, int> >::iterator afItr = numericRPGGoals.begin();
        const list<pair<int, int> >::iterator afEnd = numericRPGGoals.end();

        for (int i = -1; afItr != afEnd; ++afItr, --i) {
            for (int pass = 0 ; pass < 2; ++pass) {
                const int local = (pass ? afItr->second : afItr->first);
                if (local != -1) {
                    int fID = rpgNumericPreconditions[local].LHSVariable;
                    if (fID < 0) {

                    } else if (fID < fluentCount) {
                        data.requestFluentVisit(fID, DBL_MAX, i);
                    } else if (fID < (2 * fluentCount)) {
                        fID -= fluentCount;
                        assert(fID < fluentCount);
                        data.requestFluentVisit(fID, DBL_MAX, i);
                    } else {
                        ArtificialVariable & currAV = getArtificialVariable(fID);
                        const int size = currAV.size;
                        for (int j = 0; j < size; ++j) {
                            int afID = currAV.fluents[j];

                            if (afID >= fluentCount) afID -= fluentCount;

                            assert(afID < fluentCount);
                            data.requestFluentVisit(afID, DBL_MAX, i);
                        }
                    }
                }
            }
        }
    }

    while (!data.empty()) {

        const CommonRegressionData::iterator dItr = data.begin();

        NextRegress currPair = *dItr;

        data.erase(dItr);


        if (currPair.fact != -1) {
            const int currLit = currPair.fact;

            if (Globals::globalVerbosity & 16) {
                cout << "Finding achievers for " << *(getLiteral(currLit)) << " at time ";
                if (currPair.ts == DBL_MAX) {
                    cout << "infinity\n";
                } else {
                    cout << currPair.ts << "\n";
                }
            }

            list<pair<int, VAL::time_spec> > & achievers = effectsToActions[currLit];
            //set<int> & ruleAchievers = DerivedPredicatesEngine::rulesWillDerive(currLit);

            list<pair<int, VAL::time_spec> >::iterator accItr = achievers.begin();
            const list<pair<int, VAL::time_spec> >::iterator accEnd = achievers.end();

            for (; accItr != accEnd; ++accItr) {
                data.regressThroughAction(accItr->first, accItr->second, currPair);
            }
            /*
                        set<int>::iterator raItr = ruleAchievers.begin();
                        const set<int>::iterator raEnd = ruleAchievers.end();

                        for (; raItr != raEnd; ++raItr) {
                        data.regressThroughDerivationRule(*raItr, currPair);
                    }
            */
        } else if (currPair.fluent != -1) {
            const int currVar = currPair.fluent;


            map<int, set<VAL::time_spec> >::iterator faItr = data.relevantToFluent(currVar).begin();
            const map<int, set<VAL::time_spec> >::iterator faEnd = data.relevantToFluent(currVar).end();

            for (; faItr != faEnd; ++faItr) {
                set<VAL::time_spec>::iterator tsItr = faItr->second.begin();
                const set<VAL::time_spec>::iterator tsEnd = faItr->second.end();

                for (; tsItr != tsEnd; ++tsItr) {
                    data.regressThroughAction(faItr->first, *tsItr, currPair);
                }

            }

        } else {
            cout << "For some reason, the goal of achieving precisely nothing has worked its way onto the regression filter stack\n";
            assert(false);
        }
    }

    for (int i = 0; i < opCount; ++i) {
        if (!rogueActions[i] && data.opNeverApplied(i)) {
            
            #ifdef ENABLE_DEBUGGING_HOOKS
            {                
                Globals::eliminatedAction(i, "Can never be usefully applied on any path to the goal");
            }
            #endif

            
            if (Globals::globalVerbosity & 16) cout << *(instantiatedOps[i]) << " is irrelevant\n";
            pruneIrrelevant(i);

        }
    }

    for (int i = 0; i < opCount; ++i) {
        if (!rogueActions[i]) {
            const double startBound = data.latestUsefulPoint(i);
            const double endBound = startBound + getOpMaxDuration(i, -1);

            double & oldStart = TemporalAnalysis::getActionTSBounds()[i][0].second;
            double & oldEnd = TemporalAnalysis::getActionTSBounds()[i][1].second;

            if (oldStart > startBound) {
//              cout << *getInstantiatedOp(i) << " is interesting no later than " << startBound << "\n";
                oldStart = startBound;
            }
            if (oldEnd > endBound) {
                oldEnd = endBound;
            }
        }
    }

    /*for (int i = 0; i < opCount; ++i) {
    if (!rogueActions[i]) {
    interestingActionUntilWeHaveLiteral[i] = opApplied[i];
    {
    set<int>::iterator ggItr = interestingActionUntilWeHaveLiteral[i].begin();
    const set<int>::iterator ggEnd = interestingActionUntilWeHaveLiteral[i].end();
    bool killAfter = false;
    while (ggItr != ggEnd) {
    if (*ggItr < 0) {
    interestingActionUntilWeHaveNumeric[i].insert(-1 - *ggItr);
    set<int>::iterator ggPrev = ggItr;
    ++ggItr;
    interestingActionUntilWeHaveLiteral[i].erase(ggPrev);
    // sort this out later
    killAfter = true;
    } else {
    assert(*ggItr < (int) goalLiteralsSafe.size());
    if (!goalLiteralsSafe[*ggItr]) {
    killAfter = true;
    }
    ++ggItr;
    }
    }
    if (killAfter) {
    interestingActionUntilWeHaveLiteral[i].clear();
    interestingActionUntilWeHaveNumeric[i].clear();
    } else if (false) {
    cout << *(instantiatedOps[i]) << " is uninteresting once goals";
    ggItr = interestingActionUntilWeHaveLiteral[i].begin();
    for (; ggItr != ggEnd; ++ggItr) cout << " " << *ggItr;
    cout << " are achieved\n";

    }
    }

    {
    set<int>::iterator ggItr = interestingActionUntilWeHaveNumeric[i].begin();
    const set<int>::iterator ggEnd = interestingActionUntilWeHaveNumeric[i].end();
    bool killAfter = false;
    while (ggItr != ggEnd) {
    if (!goalNumericsSafe[*ggItr]) {
    killAfter = true;
    break;
    }
    ++ggItr;

    }
    if (killAfter) {
    interestingActionUntilWeHaveLiteral[i].clear();
    interestingActionUntilWeHaveNumeric[i].clear();
    }
    }
    }
    }*/

};

bool firstIsSubsumedBySecond(const list<Literal*> & endPre, const list<Literal*> & inv)
{
    list<Literal*>::const_iterator pItr = endPre.begin();
    const list<Literal*>::const_iterator pEnd = endPre.end();
    
    for (; pItr != pEnd; ++pItr) {
        list<Literal*>::const_iterator iItr = inv.begin();
        const list<Literal*>::const_iterator iEnd = inv.end();
        
        for (; iItr != iEnd; ++iItr) {
            if (*iItr == *pItr) break;
        }
        if (iItr == iEnd) return false;
    }
    
    return true;
}


bool noOverlap(const list<Literal*> & effs, const vector<list<pair<int, VAL::time_spec> > > & affects)
{
    list<Literal*>::const_iterator effItr = effs.begin();
    const list<Literal*>::const_iterator effEnd = effs.end();

    for (; effItr != effEnd; ++effItr) {
        if (!affects[(*effItr)->getID()].empty()) return false;
    }

    return true;
}

void RPGBuilder::findCompressionSafeActions()
{

    const int actCount = instantiatedOps.size();
    const int pneCount = pnes.size();

    startEndSkip = vector<bool>(actCount, false);

    if (!doSkipAnalysis) return;

    int compressionSafeActionCount = 0;
    int nonRogueCount = 0;

    for (int i = 0; i < actCount; ++i) {

        if (!rogueActions[i]) {
            if (rpgDurationExpressions[i].empty()) continue;

            ++nonRogueCount;

            startEndSkip[i] =     actionsToRPGNumericEndEffects[i].empty() && actionsToRPGNumericInvariants[i].empty()
                                  && actionsToRPGNumericEndPreconditions[i].empty() && !linearDiscretisation[i]
                                  && firstIsSubsumedBySecond(actionsToEndPreconditions[i], actionsToInvariants[i])
                                  && firstIsSubsumedBySecond(actionsToEndNegativePreconditions[i], actionsToNegativeInvariants[i])
                                  && noOverlap(actionsToEndNegativeEffects[i], preconditionsToActions)
                                  && noOverlap(actionsToEndEffects[i], negativePreconditionsToActions);


            if (startEndSkip[i]) {
                bool ddes = false;
                if (fixedDurationExpressions[i].empty()) {
                    // check for duration-dependent preconditions/effects

                    {
                        list<int> & currList = actionsToRPGNumericStartPreconditions[i];
                        list<int>::iterator clItr = currList.begin();
                        const list<int>::iterator clEnd = currList.end();

                        for (; clItr != clEnd; ++clItr) {

                            RPGNumericPrecondition & currPre = rpgNumericPreconditions[*clItr];
                            if (currPre.LHSVariable < -1) {
                                ddes = true;
                                break;
                            } else if (currPre.LHSVariable >= 2 * pneCount) {
                                ArtificialVariable & currAV = getArtificialVariable(currPre.LHSVariable);
                                for (int j = 0; j < currAV.size; ++j) {
                                    if (currAV.fluents[j] < -1) {
                                        ddes = true;
                                        break;
                                    }
                                }
                                if (ddes) break;
                            }
                        }

                    }

                    if (!ddes) {
                        list<int> & currList = actionsToRPGNumericStartEffects[i];
                        list<int>::iterator clItr = currList.begin();
                        const list<int>::iterator clEnd = currList.end();

                        for (; clItr != clEnd; ++clItr) {

                            RPGNumericEffect & currEff = rpgNumericEffects[*clItr];

                            for (int j = 0; j < currEff.size; ++j) {
                                if (currEff.variables[j] < -1) {
                                    ddes = true;
                                    break;
                                }
                            }
                            if (ddes) break;
                        }

                    }
                }

                startEndSkip[i] = !ddes;
                if (startEndSkip[i]) {
                    ++compressionSafeActionCount;
                    if (Globals::globalVerbosity & 16) {
                        cout << *(getInstantiatedOp(i)) << " is a candidate for start-end skipping\n";
                    }
                }
            }

        }

    }

    if (nonRogueCount) {
        if (compressionSafeActionCount == nonRogueCount) {
            cout << "All the ground actions in this problem are compression-safe\n";
        } else if (compressionSafeActionCount == 0) {
            cout << "None of the ground temporal actions in this problem have been recognised as compression-safe\n";
        } else {
            const int percentage = (int)((double) compressionSafeActionCount / (double) nonRogueCount * 100.0);
            cout << percentage << "% of the ground temporal actions in this problem are compression-safe\n";
        }
    }
}

void RPGBuilder::separateOptimisationTILs()
{
    list<FakeTILAction>::iterator tilItr = timedInitialLiterals.begin();
    const list<FakeTILAction>::iterator tilEnd = timedInitialLiterals.end();

    for (int ti = 0; tilItr != tilEnd; ++ti) {

        FakeTILAction metricParts(tilItr->duration, LiteralSet(), LiteralSet());
        
        bool metricOnly = true;
        bool anyMetric = false;
        {
            list<Literal*> & effList = tilItr->addEffects;

            list<Literal*>::iterator elItr = effList.begin();
            const list<Literal*>::iterator elEnd = effList.end();

            while (elItr != elEnd) {
                if (   !processedPreconditionsToActions[(*elItr)->getID()].empty()
                    || !processedNegativePreconditionsToActions[(*elItr)->getID()].empty()) {
                    metricOnly = false;
                    ++elItr;
                } else {
                    effectsToActions[(*elItr)->getID()].remove(make_pair(ti, VAL::E_AT));
                    metricParts.addEffects.push_back(*elItr);
                    const list<Literal*>::iterator elDel = elItr++;
                    effList.erase(elDel);
                    anyMetric = true;
                }
            }
        }
        {
            list<Literal*> & effList = tilItr->delEffects;

            list<Literal*>::iterator elItr = effList.begin();
            const list<Literal*>::iterator elEnd = effList.end();

            while (elItr != elEnd) {
                if (   !processedPreconditionsToActions[(*elItr)->getID()].empty()
                    || !processedNegativePreconditionsToActions[(*elItr)->getID()].empty()) {
                    
                    metricOnly = false;
                    ++elItr;
                } else {
                    negativeEffectsToActions[(*elItr)->getID()].remove(make_pair(ti, VAL::E_AT));
                    metricParts.delEffects.push_back(*elItr);
                    const list<Literal*>::iterator elDel = elItr++;
                    effList.erase(elDel);
                    anyMetric = true;
                }
            }
        }
        
        if (metricOnly) {
            optimisationTimedInitialLiterals.push_back(*tilItr);
            const list<FakeTILAction>::iterator tilDel = tilItr++;
            timedInitialLiterals.erase(tilDel);
        } else {
            if (anyMetric) {
                optimisationTimedInitialLiterals.push_back(metricParts);
            }
            ++tilItr;
        }        
    }

    {
        const int tilCount = timedInitialLiterals.size();
        timedInitialLiteralsVector.resize(tilCount);
        
        list<FakeTILAction>::iterator tilcItr = timedInitialLiterals.begin();
        const list<FakeTILAction>::iterator tilcEnd = timedInitialLiterals.end();
        
        for (int i = 0; tilcItr != tilcEnd; ++tilcItr, ++i) {
            timedInitialLiteralsVector[i] = &(*tilcItr);
        }
        
        FFEvent::tilLimit = ActionSegment::tilLimit = tilCount - 1;
    }
    
    {
        const int tilCount = optimisationTimedInitialLiterals.size();
        optimisationTimedInitialLiteralsVector.resize(tilCount);
        
        list<FakeTILAction>::iterator tilcItr = optimisationTimedInitialLiterals.begin();
        const list<FakeTILAction>::iterator tilcEnd = optimisationTimedInitialLiterals.end();
        
        for (int i = 0; tilcItr != tilcEnd; ++tilcItr, ++i) {
            optimisationTimedInitialLiteralsVector[i] = &(*tilcItr);
        }
                
    }
    
    {
        const int tilCount = timedInitialLiteralsVector.size();
        const int optTilCount = optimisationTimedInitialLiteralsVector.size();
        
        allTimedInitialLiteralsVector.resize(tilCount + optTilCount);
        
        int ti = 0;
        int oti = 0;
        int cti = 0;
        while (ti < tilCount && oti < optTilCount) {
            if (timedInitialLiteralsVector[ti]->duration < optimisationTimedInitialLiteralsVector[oti]->duration) {
                allTimedInitialLiteralsVector[cti] = timedInitialLiteralsVector[ti];
                ++ti;
            } else {
                allTimedInitialLiteralsVector[cti] = optimisationTimedInitialLiteralsVector[oti];
                ++oti;
            }
            ++cti;
        }
        for (; ti < tilCount; ++ti, ++cti) {
            allTimedInitialLiteralsVector[cti] = timedInitialLiteralsVector[ti];
        }
        for (; oti < optTilCount; ++oti, ++cti) {
            allTimedInitialLiteralsVector[cti] = optimisationTimedInitialLiteralsVector[oti];
        }
        
    }    
}

};
