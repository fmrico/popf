#include "numericanalysis.h"
#include "RPGBuilder.h"

#include "colours.h"

using std::cerr;
using std::endl;

using namespace VAL;

namespace Planner {

vector<NumericAnalysis::dominance_constraint> NumericAnalysis::dominanceConstraints;
vector<NumericAnalysis::order_independence> NumericAnalysis::allEffectsAreOrderIndependent;

#ifdef POPF3ANALYSIS

bool NumericAnalysis::doGoalLimitAnalysis = true;

NumericAnalysis::dominance_constraint NumericAnalysis::preconditionDominanceInOneDirection(const int & varID)
{
    static const bool localDebug = false;
    
    const vector<RPGBuilder::RPGNumericPrecondition> & rpgNumericPreconditions = RPGBuilder::getNumericPreTable();
    const vector<RPGBuilder::ArtificialVariable> & rpgArtificialVariables = RPGBuilder::getArtificialVariableTable();
    
    /* Index 0: true if bigger is better; Index 1: true if smaller is better */
    vector<bool> possibleAnswers(2,true);
    
    const int pneCount =  RPGBuilder::getPNECount();
    const int rnpCount = rpgNumericPreconditions.size();
    const int negativeI = varID + pneCount;
    
    for (int rnp = 0; rnp < rnpCount; ++rnp) {
        const RPGBuilder::RPGNumericPrecondition & currRNP = rpgNumericPreconditions[rnp];
        if (currRNP.LHSVariable == varID) {
            if (localDebug) {
                cout << "Found a precondition on " << *(RPGBuilder::getPNE(varID)) << ": " << currRNP << endl;
            }
            if (possibleAnswers[1]) {
                if (!requiringNotFullOnlySupportsReplenishment(rnp)) {
                    // we have a precondition v > or >= c, which doesn't just support resource replenishment, so smaller isn't preferable
                    possibleAnswers[1] = false;
                    if (!possibleAnswers[0]) return E_NODOMINANCE;
                }
            }
        } else if (currRNP.LHSVariable == negativeI) {
            if (localDebug) {
                cout << "Found a precondition on " << *(RPGBuilder::getPNE(varID)) << ": " << currRNP << endl;
            }

            if (possibleAnswers[0]) {
                if (!requiringNotFullOnlySupportsReplenishment(rnp)) {
                // we have a precondition v < or <= c,  which doesn't just support resource replenishment, so bigger isn't preferable
                    possibleAnswers[0] = false;
                    if (!possibleAnswers[1]) return E_NODOMINANCE;
                }
            }
        } else if (currRNP.LHSVariable >= 2 * pneCount) {
            const RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(currRNP.LHSVariable);
            if (currAV.size == 1) {
                if (currAV.fluents[0] == varID) {
                    if (localDebug) {
                        cout << "Found a precondition on " << *(RPGBuilder::getPNE(varID)) << ": " << currRNP << endl;
                    }
                    
                    if (possibleAnswers[1]) {
                        if (!requiringNotFullOnlySupportsReplenishment(rnp)) {
                            // we have a precondition v > or >= c, which doesn't just support resource replenishment, so smaller isn't preferable
                            possibleAnswers[1] = false;
                            if (!possibleAnswers[0]) return E_NODOMINANCE;
                        }
                    }                    
                } else if (currAV.fluents[0] == negativeI) {
                    if (localDebug) {
                        cout << "Found a precondition on " << *(RPGBuilder::getPNE(varID)) << ": " << currRNP << endl;
                    }
                    
                    if (possibleAnswers[0]) {
                        if (!requiringNotFullOnlySupportsReplenishment(rnp)) {
                            // we have a precondition v < or <= c,  which doesn't just support resource replenishment, so bigger isn't preferable
                            possibleAnswers[0] = false;
                            if (!possibleAnswers[1]) return E_NODOMINANCE;
                        }
                    }                                        
                }
            }
        }
    }
    
    
    // next, check artificial variables - these are only ever used in preconditions of the form
    // av {>, >=} constant
    
    const int avCount = rpgArtificialVariables.size();
    for (int av = 0; av < avCount; ++av) {
        const RPGBuilder::ArtificialVariable & currAV = rpgArtificialVariables[av];
        const int avfSize = currAV.size;
        
        if (avfSize == 1) {
            if (currAV.fluents[0] == varID || currAV.fluents[0] == negativeI) {
                // is an AV on just the variable we're looking at
                // caught these cases in the for loop higher up
                continue;
            }            
        }
        
        for (int f = 0; f < avfSize; ++f) {
            const int currF = currAV.fluents[f];
            if (currF == varID) {
                if (localDebug) {
                    cout << "Found an AV on " << *(RPGBuilder::getPNE(varID)) << ": " << currAV << endl;
                }
                
                // we have a precondition v > or >= c, so smaller isn't preferable
                possibleAnswers[1] = false;
                if (!possibleAnswers[0]) return E_NODOMINANCE;
            } else if (currF == negativeI) {
                if (localDebug) {
                    cout << "Found an AV on " << *(RPGBuilder::getPNE(varID)) << ": " << currAV << endl;
                }
                                
                // we have a precondition v < or <= c, so bigger isn't preferable
                possibleAnswers[0] = false;
                if (!possibleAnswers[1]) return E_NODOMINANCE;
            }
        }
    }                    
    
    // if we get this far, then there is dominance in the preconditions
    
    if (possibleAnswers[0] && possibleAnswers[1]) {
        cout << "Fatal internal error: is good to have bigger and smaller values of " << *(RPGBuilder::getPNE(varID)) << endl;
        assert(!(possibleAnswers[0] && possibleAnswers[1]));
    }
    
    if (possibleAnswers[0]) {
        return E_BIGGERISBETTER;
    }
    
    if (possibleAnswers[1]) {
        return E_SMALLERISBETTER;
    }
    
    return E_NODOMINANCE;
}

bool NumericAnalysis::requiringNotFullOnlySupportsReplenishment(const int & preID)
{

    static LiteralSet literalGoals;    
    static bool lgDefined = false;
    
    if (!lgDefined) {
        lgDefined = true;
        literalGoals.insert(RPGBuilder::getLiteralGoals().begin(), RPGBuilder::getLiteralGoals().end());
    }
    
    const RPGBuilder::RPGNumericPrecondition & currRNP = RPGBuilder::getNumericPreTable()[preID];
    const list<pair<int, VAL::time_spec> > & actionsWithThatPre = RPGBuilder::getRpgNumericPreconditionsToActions()[preID];
    
    {
        // first, if this precondion is a goal, then it's definitely worth getting in its own right
        const list<pair<int, int> > & numericGoals = RPGBuilder::getNumericRPGGoals();
        
        list<pair<int, int> >::const_iterator ngItr = numericGoals.begin();
        const list<pair<int, int> >::const_iterator ngEnd = numericGoals.end();
        
        for (; ngItr != ngEnd; ++ngItr) {
            
            if (ngItr->first == preID) {
                return false;
            }
            
            if (ngItr->second == preID) {
                return false;
            }
        }
    }
    
    if (actionsWithThatPre.empty()) {
        
        // if no actions require the precondition, bail out here
        return true;
    }
    
    // otherwise, see if we can reduce the precondition to a simple v >=c or v <= c
    
    int varID = currRNP.LHSVariable;
    bool geq = true; // if false: means it's a <=
    double constant = currRNP.RHSConstant;
    
    if (varID >= 2 * RPGBuilder::getPNECount()) {
        const RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(varID);
        if (currAV.size != 1) {
            // is a compound numeric precondition, so no obvious resource analogy
            return false;
        }
        varID = currAV.fluents[0];
        constant -= currAV.constant;
    }
    
    if (varID >= RPGBuilder::getPNECount()) {
        varID -= RPGBuilder::getPNECount();
        if (constant != 0.0) {
            constant = -constant;
        }
        geq = false;
    }
    
    if (varID < 0) {
        // is a condition on something other than a simple task variable
        return false;
    }
    
    // if we get this far, we know it's v <= c or v >= c
    // if the effects are all (assign v c), then we can return true
    
    list<pair<int, VAL::time_spec> >::const_iterator actItr = actionsWithThatPre.begin();
    const list<pair<int, VAL::time_spec> >::const_iterator actEnd = actionsWithThatPre.end();
    
    for (; actItr != actEnd; ++actItr) {
        // first, see if anything needs the propositional effects of the action with this precondition,
        // in which case meeting the precondition could be a good idea
        
        for (int pass = 0; pass < 2; ++pass) {
            const list<Literal*> & effList = (pass ? RPGBuilder::getEndPropositionAdds()[actItr->first]
                                                   : RPGBuilder::getStartPropositionAdds()[actItr->first] );
            
            list<Literal*>::const_iterator eItr = effList.begin();
            const list<Literal*>::const_iterator eEnd = effList.end();
            
            for (int fID; eItr != eEnd; ++eItr) {
                fID = (*eItr)->getStateID();
                if (!RPGBuilder::getPresToActions()[fID].empty()) {
                    // an action needs this fact
                    return false;
                }
                if (literalGoals.find(*eItr) != literalGoals.end()) {
                    // fact is a goal
                    return false;
                }
            }
        }
        
        // so far have shown that this action has no useful propositional effects
        // now check that all numeric effects are equally dull, either on
        // irrelevant variables, or replenishing this resource to the value 'constant'
        
        for (int pass = 0; pass < 2; ++pass) {
            const list<int> & effs = (pass ? RPGBuilder::getEndEffNumerics()[actItr->first] : RPGBuilder::getStartEffNumerics()[actItr->first]);
            
            list<int>::const_iterator effItr = effs.begin();
            const list<int>::const_iterator effEnd = effs.end();
            
            for (; effItr != effEnd; ++effItr) {
                const RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[*effItr];
                if (dominanceConstraints[currEff.fluentIndex] == E_IRRELEVANT) {
                    continue;
                }
                
                if (currEff.fluentIndex != varID) {
                    // found effect on another numeric variable, give up
                    return false;
                }
                
                if (currEff.size) {
                    // complex effect, not simple constant
                    return false;
                }
                                                
                                                                
                if (!currEff.isAssignment) {
                    if (geq) {
                        if (currEff.constant < 0) {
                            return false;
                        }
                        // willing to tolerate 'v >= c  means we can increase c'
                    } else {
                        if (currEff.constant > 0) {
                            return false;
                        }
                        // willing to tolerate 'v <= c  means we can decrease c'
                    }
                    
                } else {                    
                    if (geq) {
                        if (currEff.constant - constant < -0.000001 ) {
                            // have a 'gap' situation: v >= c allows (assign v d), where d < c
                            return false;
                        }
                        // will happily also ignore the case where v >= c
                                                                        
                    } else {                    
                        if (currEff.constant - constant > 0.000001 ) {
                            // have a 'gap' situation: v <= c allows (assign v d), where d > c
                            return false;
                        }
                        // will happily also ignore the case where v >= c
                    }
                }
            }
        }                    
    }
    
    return true;
        
}


void NumericAnalysis::updateForDurationDependence(const int & v, const double & w,
                                                  const int & durVar, const bool & pass,
                                                  vector<dominance_constraint> & workingDominanceConstraints)
{
    switch (workingDominanceConstraints[v]) {
        case E_NODOMINANCE:
        {
            workingDominanceConstraints[durVar] = E_NODOMINANCE;
            break;
        }
        case E_SMALLERISBETTER:
        {
            if (w < 0.0) {
                // if we are decreasing a variable that is better smaller, then we
                // want as long a duration as possible
                
                if (pass ? workingDominanceConstraints[durVar] == E_BIGGERISBETTER
                         : workingDominanceConstraints[durVar] == E_SMALLERISBETTER) {
                    workingDominanceConstraints[durVar] = E_NODOMINANCE;
                }
            } else if (w > 0.0) {
                // if we are increasing a variable that is better smaller, then we
                // want as short a duration as possible
                                                
                if (pass ? workingDominanceConstraints[durVar] == E_SMALLERISBETTER
                         : workingDominanceConstraints[durVar] == E_BIGGERISBETTER) {
                    workingDominanceConstraints[durVar] = E_NODOMINANCE;
                }
            }
            break;
        }
        case E_BIGGERISBETTER:
        {
            if (w < 0.0) {
                // if we are decreasing a variable that is better bigger, then we
                // want as short a duration as possible
                
                if (pass ? workingDominanceConstraints[durVar] == E_SMALLERISBETTER
                         : workingDominanceConstraints[durVar] == E_BIGGERISBETTER) {
                    workingDominanceConstraints[durVar] = E_NODOMINANCE;
                }
            } else if (w > 0.0) {
                // if we are inccreasing a variable that is better bigger, then we
                // want as long a duration as possible
                                                
                if (pass ? workingDominanceConstraints[durVar] == E_BIGGERISBETTER
                         : workingDominanceConstraints[durVar] == E_SMALLERISBETTER) {
                    workingDominanceConstraints[durVar] = E_NODOMINANCE;
                }
            }
            break;
        }
        case E_IRRELEVANT:
        case E_METRICTRACKING:
        {
            break;            
        }
    }
}

vector<map<int, list<int> > > NumericAnalysis::goalHasIndependentCostOnLimit;
bool NumericAnalysis::thereAreIndependentGoalCosts = false;

void NumericAnalysis::considerCostsOfLiteralGoalsReachedByGoalOnlyOperators()
{        
    
    static const unsigned int localDebug = 7;
    
    const list<Literal*> & litGoals = RPGBuilder::getLiteralGoals();
    
    goalHasIndependentCostOnLimit.resize(litGoals.size());
 
    const int nldSize = goalNumericUsageLimits.size();
    
    if (!nldSize) {
        if (localDebug) {
            cout << COLOUR_light_blue << "No analytic limits found, not considering limit effects of goal-only operators" << COLOUR_default << endl;
        }
        return;
    }
    
    map<int, list<int> > variableIsInLimit;
    bool variableIsInLimitDefined = false;
    
    list<Literal*>::const_iterator gItr = litGoals.begin();
    const list<Literal*>::const_iterator gEnd = litGoals.end();
    
    for (int gID = 0; gItr != gEnd; ++gItr, ++gID) {
        
        // first, exclude static goals, or goals which are preconditions
        
        if (RPGBuilder::isStatic(*gItr).first
            || !RPGBuilder::getRawPresToActions()[(*gItr)->getStateID()].empty()) {
            if (localDebug & 1) {
                cout << COLOUR_light_blue << "For limits: literal goal index " << gID << ", fact " << *(*gItr) << ", is static" << COLOUR_default << endl;
            }
            
            continue;
        }
        
        const list<pair<int, VAL::time_spec> > & eta = RPGBuilder::getEffectsToActions((*gItr)->getStateID());
        
        set<int> achievedBy;
        
        if (localDebug & 4) {
            cout << COLOUR_light_red << "Looking for achievers for goal index " << gID << ", fact " << *(*gItr) << " with fID " << (*gItr)->getStateID() <<  COLOUR_default << endl;
        }
        
        list<pair<int, VAL::time_spec> >::const_iterator eItr = eta.begin();
        const list<pair<int, VAL::time_spec> >::const_iterator eEnd = eta.end();
        
        for (; eItr != eEnd; ++eItr) {
            if (eItr->second == VAL::E_AT) {
                break;
            }
            if (localDebug & 4) {
                cout << " " << *(RPGBuilder::getInstantiatedOp(eItr->first));
            }
            achievedBy.insert(eItr->first);
            
        }
        
        if (localDebug & 4) {
            cout << endl;
        }
        
        if (eItr != eEnd) {
            
            // was reached by a TIL, skip it
            if (localDebug & 1) {
                cout << COLOUR_light_blue << "For limits: literal goal index " << gID << ", fact " << *(*gItr) << ", could be reached by a TIL" << COLOUR_default << endl;
            }
            continue;
        }
        
        // now we see whether the achieving actions' effects are all boring
        
        bool safe = true;
        bool anyNum = false;
        
        {
            set<int>::const_iterator aItr = achievedBy.begin();
            const set<int>::const_iterator aEnd = achievedBy.end();
            
            for (; safe && aItr != aEnd; ++aItr) {
                
                
                // check that it only achieves this goal
                            
                for (int pass = 0; safe && pass < 2; ++pass) {
                    const list<Literal*> & adds = (pass ? RPGBuilder::getEndPropositionAdds()[*aItr] : RPGBuilder::getStartPropositionAdds()[*aItr]);
                    
                    list<Literal*>::const_iterator aeItr = adds.begin();
                    const list<Literal*>::const_iterator aeEnd = adds.end();
                    
                    for (; aeItr != aeEnd; ++aeItr) {
                        if (*aeItr != *gItr) {
                            if (localDebug & 1) {
                                cout << COLOUR_light_blue << "For limits: literal goal index " << gID << ", fact " << *(*gItr) << ", could be achieved by operator ";
                                cout << *(RPGBuilder::getInstantiatedOp(*aItr))  << ", which has other interesting effects (including one on " << *(*aeItr) << " )" << COLOUR_default << endl;
                            }
                            safe = false;
                            break;
                        }
                    }
                }
                
                if (!safe) {
                    break;
                }
                
                
                for (int pass = 0; safe && pass < 2; ++pass) {
                    const list<int> & numEffs = (pass ? RPGBuilder::getEndEffNumerics()[*aItr] : RPGBuilder::getStartEffNumerics()[*aItr]);
                    
                    list<int>::const_iterator neItr = numEffs.begin();
                    const list<int>::const_iterator neEnd = numEffs.end();
                    
                    for (; safe && neItr != neEnd; ++neItr) {
                        const RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[*neItr];
                        
                        if (dominanceConstraints[currEff.fluentIndex] == E_IRRELEVANT) {
                            continue;
                        }

                        if (currEff.size || currEff.isAssignment) {
                            if (localDebug & 1) {
                                cout << COLOUR_light_blue << "For limits: literal goal index " << gID << ", fact " << *(*gItr) << ", could be achieved by operator ";
                                cout << *(RPGBuilder::getInstantiatedOp(*aItr))  << ", which has a non-trivial numeric effect " << currEff << COLOUR_default << endl;
                            }
                            
                            safe = false;
                            break;
                        }
                        
                        if (currEff.constant == 0.0) {
                            continue;
                        }
                        
                        switch (dominanceConstraints[currEff.fluentIndex]) {
                            case E_IRRELEVANT:
                            {
                                cerr << "Internal error: this should never, ever happen, as it means the dominance constraints have changed mid-function\n";
                                exit(1);
                            }
                            case E_METRICTRACKING:
                            {
                                if (Globals::optimiseSolutionQuality) {
                                    anyNum = true;
                                }
                                break;
                            }
                            case E_SMALLERISBETTER: {
                                if (currEff.constant < 0.0) {
                                    // beneficial effect: makes something smaller
                                    safe = false;
                                    if (localDebug & 1) {
                                        cout << COLOUR_light_blue << "For limits: literal goal index " << gID << ", fact " << *(*gItr) << ", could be achieved by operator ";
                                        cout << *(RPGBuilder::getInstantiatedOp(*aItr))  << ", which has a beneficial decrease effect " << currEff << COLOUR_default << endl;
                                    }
                                    
                                }
                                anyNum = true;
                                break;
                            }
                            case E_BIGGERISBETTER: {
                                if (currEff.constant > 0.0) {
                                    // beneficial effect: makes something smaller
                                    safe = false;
                                    if (localDebug & 1) {
                                        cout << COLOUR_light_blue << "For limits: literal goal index " << gID << ", fact " << *(*gItr) << ", could be achieved by operator ";
                                        cout << *(RPGBuilder::getInstantiatedOp(*aItr))  << ", which has a beneficial increase effect " << currEff << COLOUR_default << endl;
                                    }
                                    
                                    
                                }
                                anyNum = true;
                                break;
                            }
                            default: {
                                if (localDebug & 1) {
                                    cout << COLOUR_light_blue << "For limits: literal goal index " << gID << ", fact " << *(*gItr) << ", could be achieved by operator ";
                                    cout << *(RPGBuilder::getInstantiatedOp(*aItr))  << ", which has a no-known-dominance effect " << currEff << COLOUR_default << endl;
                                }
                                safe = false;
                            }
                        }
                                            
                    }
                }
            }
        }
                
        if (safe && anyNum) {
            // if we get this far, the goal is achieved only by actions that 
            //  i) only achieve one fact - the goal
            // ii) have numeric effects that are only ever a bad idea, so we'd never want to apply them in their
            //     own right for that
            // iii) some sort of numeric effectage that might have an effect on a cost limit

            if (!variableIsInLimitDefined) {

                variableIsInLimitDefined = true;
                
                for (int nl = 0; nl < nldSize; ++nl) {
                    
                    const NumericAnalysis::NumericLimitDescriptor & nld = goalNumericUsageLimits[nl];
                    
                    map<int,double>::const_iterator vItr = nld.var.begin();
                    const map<int,double>::const_iterator vEnd = nld.var.end();
                    
                    for (; vItr != vEnd; ++vItr) {
                        variableIsInLimit[vItr->first].push_back(nl);
                    }
                    
                }
            }
    
            
            
            set<int>::const_iterator aItr = achievedBy.begin();
            const set<int>::const_iterator aEnd = achievedBy.end();
            
            for (; aItr != aEnd; ++aItr) {
                
                for (int pass = 0; safe && pass < 2; ++pass) {
                    const list<int> & numEffs = (pass ? RPGBuilder::getEndEffNumerics()[*aItr] : RPGBuilder::getStartEffNumerics()[*aItr]);
                    
                    list<int>::const_iterator neItr = numEffs.begin();
                    const list<int>::const_iterator neEnd = numEffs.end();
                    
                    for (; safe && neItr != neEnd; ++neItr) {
                        const RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[*neItr];
                        
                        assert(!currEff.size && !currEff.isAssignment);

                        if (dominanceConstraints[currEff.fluentIndex] == E_IRRELEVANT) {
                            continue;
                        }                            
                        
                        if (currEff.constant == 0.0) {
                            continue;
                        }
                        
                        const map<int,list<int> >::const_iterator lsItr = variableIsInLimit.find(currEff.fluentIndex);
                        
                        if (lsItr != variableIsInLimit.end()) {
                            
                            list<int>::const_iterator lItr = lsItr->second.begin();
                            const list<int>::const_iterator lEnd = lsItr->second.end();
                            
                            for (; lItr != lEnd; ++lItr) {                                
                                goalHasIndependentCostOnLimit[gID][*lItr].push_back(*aItr);
                            }
                            
                        }
                    }
                }
            }
            
            if ((localDebug & 2) && !goalHasIndependentCostOnLimit[gID].empty()) {
                
                cout << COLOUR_yellow << "To achieve goal index " << gID << ", fact " << *(*gItr) << ", there is a definite effect on one or more limits:\n" << COLOUR_default;
                
                map<int, list<int> >::const_iterator leItr = goalHasIndependentCostOnLimit[gID].begin();
                const map<int, list<int> >::const_iterator leEnd = goalHasIndependentCostOnLimit[gID].end();
                
                for (; leItr != leEnd; ++leItr) {
                    cout << "Limit " << leItr->first;
                    
                    if (goalNumericUsageLimits[leItr->first].optimisationLimit) {
                        cout << " (the metric)";
                    }
                    cout << ", if using one of:";
                    list<int>::const_iterator oItr = leItr->second.begin();
                    const list<int>::const_iterator oEnd = leItr->second.end();
                    
                    for (; oItr != oEnd; ++oItr) {
                        cout << " " << *(RPGBuilder::getInstantiatedOp(*oItr));
                    }
                    cout << endl;
                }
                
            }
        }
    }
    
    {
        const int gc = goalHasIndependentCostOnLimit.size();
        
        for (int gID = 0; gID < gc; ++gID) {
            if (!goalHasIndependentCostOnLimit[gID].empty()) {
                thereAreIndependentGoalCosts = true;
                return;
            }
        }
    }
    
}

void NumericAnalysis::considerConflictingDominanceThroughEffects(vector<dominance_constraint> & workingDominanceConstraints)
{
    
    const int pneCount =  RPGBuilder::getPNECount();
    
    const vector<RPGBuilder::RPGNumericEffect> & rpgNumericEffects = RPGBuilder::getNumericEff();
    
    const int effCount = rpgNumericEffects.size();
    
    int v;
    double w;
    
    map<int, map<int,bool> > actionToDurationDependentEffects;
    
    for (int effID = 0; effID < effCount; ++effID) {
        
        const RPGBuilder::RPGNumericEffect & currEff = rpgNumericEffects[effID];
        
        int effectIsDurative = 0;
        switch (workingDominanceConstraints[currEff.fluentIndex]) {
            case E_NODOMINANCE:
            {
                for (int vi = 0; vi < currEff.size; ++vi) {
                    v = currEff.variables[vi];
                    if (v < 0) {
                        if (v == -3) {
                            effectIsDurative = 1;
                        } else if (v == -19) {
                            effectIsDurative = -1;
                        } else {
                            cerr << "Internal error: effects can only depend on task variables, constant, or ?duration\n";
                            exit(1);
                        }
                        continue;
                    }
                    if (v >= pneCount) {
                        workingDominanceConstraints[v-pneCount] = E_NODOMINANCE;
                    } else {
                        workingDominanceConstraints[v] = E_NODOMINANCE;
                    }
                }
                break;
            }
            case E_BIGGERISBETTER:
            {
                for (int vi = 0; vi < currEff.size; ++vi) {
                    v = currEff.variables[vi];
                    if (v < 0) {
                        if (v == -3) {
                            effectIsDurative = 1;
                        } else if (v == -19) {
                            effectIsDurative = -1;
                        } else {
                            cerr << "Internal error: effects can only depend on task variables, constant, or ?duration\n";
                            exit(1);
                        }
                        continue;
                    }
                    if (v >= pneCount) {
                        if (currEff.weights[vi] > 0.0) {
                            if (workingDominanceConstraints[v-pneCount] == E_BIGGERISBETTER) {
                                workingDominanceConstraints[v-pneCount] = E_NODOMINANCE;
                            }
                        } else if (currEff.weights[vi] < 0.0) {
                            if (workingDominanceConstraints[v-pneCount] == E_SMALLERISBETTER) {
                                workingDominanceConstraints[v-pneCount] = E_NODOMINANCE;
                            }
                        }
                    } else {
                        if (currEff.weights[vi] > 0.0) {
                            if (workingDominanceConstraints[v] == E_SMALLERISBETTER) {
                                workingDominanceConstraints[v] = E_NODOMINANCE;
                            }
                        } else if (currEff.weights[vi] < 0.0) {
                            if (workingDominanceConstraints[v] == E_BIGGERISBETTER) {
                                workingDominanceConstraints[v] = E_NODOMINANCE;
                            }
                        }
                    }
                }
                break;
            }
            case E_SMALLERISBETTER:
            {
                for (int vi = 0; vi < currEff.size; ++vi) {
                    v = currEff.variables[vi];
                    if (v < 0) {
                        if (v == -3) {
                            effectIsDurative = 1;
                        } else if (v == -19) {
                            effectIsDurative = -1;
                        } else {
                            cerr << "Internal error: effects can only depend on task variables, constant, or ?duration\n";
                            exit(1);
                        }                        
                        continue;
                    }
                    if (v >= pneCount) {
                        if (currEff.weights[vi] > 0.0) {
                            if (workingDominanceConstraints[v-pneCount] == E_SMALLERISBETTER) {
                                workingDominanceConstraints[v-pneCount] = E_NODOMINANCE;
                            }
                        } else if (currEff.weights[vi] < 0.0) {
                            if (workingDominanceConstraints[v-pneCount] == E_BIGGERISBETTER) {
                              workingDominanceConstraints[v-pneCount] = E_NODOMINANCE;
                            }
                        }   
                    } else {
                        if (currEff.weights[vi] > 0.0) {
                            if (workingDominanceConstraints[v] == E_BIGGERISBETTER) {
                                workingDominanceConstraints[v] = E_NODOMINANCE;
                            }
                        } else if (currEff.weights[vi] < 0.0) {
                            if (workingDominanceConstraints[v] == E_SMALLERISBETTER) {
                                workingDominanceConstraints[v] = E_NODOMINANCE;
                            }
                        }
                    }
                }
                break;
            }
            case E_IRRELEVANT:
            case E_METRICTRACKING:
            {
                break;
            }
        }
        
        if (effectIsDurative != 0) {
            const list<pair<int, VAL::time_spec> > & eta = RPGBuilder::getEffectsToActions(effID);
            list<pair<int, VAL::time_spec> >::const_iterator etaItr = eta.begin();
            const list<pair<int, VAL::time_spec> >::const_iterator etaEnd = eta.end();
            
            for (; etaItr != etaEnd; ++etaItr) {
                actionToDurationDependentEffects[etaItr->first].insert(make_pair(effID, effectIsDurative == 1 ? true : false));
            }
        }
    }
    
    const vector<RPGBuilder::LinearEffects*> & ctsEffs = RPGBuilder::getLinearDiscretisation();    
    const int actCount = ctsEffs.size();
    
    map<int,map<int,bool> >::const_iterator ddEffs = actionToDurationDependentEffects.begin();
    
    const RPGBuilder::LinearEffects* leffs;
    
    for (int actID = 0; actID < actCount; ++actID) {
        if (RPGBuilder::rogueActions[actID]) continue;
        
        leffs = ctsEffs[actID];
        
        while (ddEffs != actionToDurationDependentEffects.end() && ddEffs->first < actID) {
            ++ddEffs;
        }
        
        if (ddEffs == actionToDurationDependentEffects.end() && !leffs) {
            // action has no duration-dependent effects or continuous effects
            continue;
        }
        
        const vector<RPGBuilder::RPGDuration*> DEs = RPGBuilder::getRPGDEs(actID);
        assert(!DEs.empty());
        
        set<int> makesDurBigger;
        set<int> makesDurSmaller;
        
        for (int pass = 0; pass < 3; ++pass) {
            const list<RPGBuilder::DurationExpr*> & exprs = (*(DEs[0]))[pass];
            
            list<RPGBuilder::DurationExpr*>::const_iterator dItr = exprs.begin();
            const list<RPGBuilder::DurationExpr*>::const_iterator dEnd = exprs.end();
            
            for (; dItr != dEnd; ++dItr) {
                
                const int vCount = (*dItr)->variables.size();
                
                for (int vID = 0; vID < vCount; ++vID) {
                    #ifdef STOCHASTICDURATIONS
                    v = (*dItr)->variables[vID].first;
                    if (v < 0) {
                        continue;
                    }
                    #else
                    v = (*dItr)->variables[vID];
                    #endif
                    w = (*dItr)->weights[vID];
                    
                    if (v >= pneCount) {
                        if (w > 0.0) {
                            makesDurSmaller.insert(v - pneCount);
                        } else if (w < 0.0) {
                            makesDurBigger.insert(v - pneCount);
                        }
                    } else {
                        if (w > 0.0) {
                            makesDurBigger.insert(v);
                        } else if (w < 0.0) {
                            makesDurSmaller.insert(v);
                        }
                    }
                }
            }
        }
        
        const int ctsVCount = (leffs ? (*leffs).vars.size() : 0);
        
        for (int pass = 0; pass < 2; ++pass) {
            const set<int> & loopover = (pass ? makesDurSmaller : makesDurBigger);
            set<int>::const_iterator vItr = loopover.begin();
            const set<int>::const_iterator vEnd = loopover.end();
            
            for (; vItr != vEnd; ++vItr) {
                for (int cvID = 0; cvID < ctsVCount; ++cvID) {
                    v = (*leffs).vars[cvID];
                    w = (*leffs).effects[0][cvID].constant;
                    updateForDurationDependence(v,w,*vItr,(pass == 1),workingDominanceConstraints);
                }
                
                if (ddEffs != actionToDurationDependentEffects.end()) {
                    map<int,bool>::const_iterator effID = ddEffs->second.begin();
                    const map<int,bool>::const_iterator effIDEnd = ddEffs->second.end();
                    
                    for (; effID != effIDEnd; ++effID) {
                        v = rpgNumericEffects[effID->first].fluentIndex;
                        w = (effID->second ? 1.0 : -1.0);
                        updateForDurationDependence(v,w,*vItr,(pass == 1),workingDominanceConstraints);
                    }
                }
            }
        }
    }
}

vector<NumericAnalysis::NumericLimitDescriptor> NumericAnalysis::goalNumericUsageLimits;

NumericAnalysis::NumericLimitDescriptor::NumericLimitDescriptor(const int & v, const double & w, const VAL::comparison_op & cOp, const double & lim)
    : op(cOp), limit(lim), optimisationLimit(false)
{
        
    const int pneCount = RPGBuilder::getPNECount();
        
    assert(v >= 0);
    assert(v < 2 * pneCount);
    
    if (v < pneCount) {
        var.insert(make_pair(v,w));
    } else {
        var.insert(make_pair(v - pneCount,-w));
    }
    
}
            
NumericAnalysis::NumericLimitDescriptor::NumericLimitDescriptor(const vector<int> & v, const vector<double> & w, const int & size, const VAL::comparison_op & cOp, const double & lim)
    : op(cOp), limit(lim), optimisationLimit(false)
{
        
        
    const int pneCount = RPGBuilder::getPNECount();
    
    for (int s = 0; s < size; ++s) {
        assert(v[s] >= 0);
        assert(v[s] < 2 * pneCount);
        
        if (v[s] < pneCount) {
            var.insert(make_pair(v[s],w[s]));
        } else {
            var.insert(make_pair(v[s] - pneCount,-w[s]));
        }
    }
    
}
            
NumericAnalysis::NumericLimitDescriptor::NumericLimitDescriptor(const vector<int> & v, const vector<double> & w, const int & size)
    : op(VAL::E_GREATEQ), limit(-DBL_MAX), optimisationLimit(true)
{
                    
    const int pneCount = RPGBuilder::getPNECount();
        
    for (int s = 0; s < size; ++s) {
        assert(v[s] >= 0);
        assert(v[s] < 2 * pneCount);
        if (v[s] < pneCount) {
            var.insert(make_pair(v[s],w[s]));
        } else {
            var.insert(make_pair(v[s] - pneCount,-w[s]));
        }
    }
        
}

void seeIfInducesALimit(const NumericAnalysis::NumericLimitDescriptor & hypothesisedLimit, map<NumericAnalysis::NumericLimitDescriptor,bool> & upperBounds)
{

    const map<NumericAnalysis::NumericLimitDescriptor,bool>::iterator bItr = upperBounds.find(hypothesisedLimit);

    if (bItr != upperBounds.end()) {
        if (bItr->second) {
            // if effects on this variable are one way, check if we should update the limit
            
            NumericAnalysis::NumericLimitDescriptor * const editableFirst = const_cast<NumericAnalysis::NumericLimitDescriptor*>(&(bItr->first));
            
            if (hypothesisedLimit.op == VAL::E_GREATEQ && editableFirst->op == VAL::E_GREATEQ) {
                if (hypothesisedLimit.limit > editableFirst->limit) {
                    editableFirst->limit = hypothesisedLimit.limit;
                }
            } else if (hypothesisedLimit.op == VAL::E_GREATER && editableFirst->op == VAL::E_GREATEQ) {
                if (hypothesisedLimit.limit >= editableFirst->limit) {
                    editableFirst->limit = hypothesisedLimit.limit;
                    editableFirst->op = VAL::E_LESS;
                }
            } else if (hypothesisedLimit.op == VAL::E_GREATEQ && editableFirst->op == VAL::E_GREATER) {
                if (hypothesisedLimit.limit > editableFirst->limit) {
                    editableFirst->limit = hypothesisedLimit.limit;
                    editableFirst->op = VAL::E_GREATEQ;
                }
            } else {
                if (hypothesisedLimit.limit > editableFirst->limit) {
                    editableFirst->limit = hypothesisedLimit.limit;
                }
            }
        }
                // if we've just updated a previous constraint, we don't need to go on to one-way checking and then deduce whether to keep the limit based on that
        return;
    }
                
    bool allDecreasers = true;
    
    const vector<RPGBuilder::RPGNumericEffect> & rpgNumericEffects = RPGBuilder::getNumericEff();
    
    const int effCount = rpgNumericEffects.size();
                
    for (int effID = 0; effID < effCount; ++effID) {
        const RPGBuilder::RPGNumericEffect & currEff = rpgNumericEffects[effID];
        
        const map<int,double>::const_iterator weight = hypothesisedLimit.var.find(currEff.fluentIndex);
        if (weight == hypothesisedLimit.var.end()) {
            // ignore effects on other variables
            continue;
        }
        
        if (currEff.isAssignment || currEff.size) {
            // all bets are off if it's an assignment effect, or an increase/decrease
            // that depends on another variable
            allDecreasers = false;
            break;
        }
        
        if (currEff.constant * weight->second > 0.0) {
            allDecreasers = false;                   
            break;
        }            
    }
    
    upperBounds.insert(make_pair(hypothesisedLimit, allDecreasers));

}

void NumericAnalysis::findGoalNumericUsageLimits()
{
    if (!doGoalLimitAnalysis) return;
    
    const int pneCount =  RPGBuilder::getPNECount();
    
    const list<pair<int, int> > & numericGoals = RPGBuilder::getNumericRPGGoals();
    
    map<NumericLimitDescriptor,bool> upperBounds;
    
    if (Globals::optimiseSolutionQuality) {
        RPGBuilder::Metric * theMetric = RPGBuilder::getMetric();
        
        if (theMetric) {
            const int size = theMetric->variables.size();
            
            vector<double> weights(size);
            vector<int> vars(size);
                      
            bool metricOnlyUsesTaskVariables = true;
            
            list<int>::const_iterator mvItr = theMetric->variables.begin();
            list<double>::const_iterator mwItr = theMetric->weights.begin();
            const list<int>::const_iterator mvEnd = theMetric->variables.end();
            
            for (int s = 0; mvItr != mvEnd; ++mvItr, ++mwItr, ++s) {
                if (*mvItr < 0) {
                    metricOnlyUsesTaskVariables = false;
                    break;
                }
                if (theMetric->minimise) {
                    weights[s] = -(*mwItr);
                } else {
                    weights[s] = (*mwItr);
                }
                vars[s] = *mvItr;
            }
            
            if (metricOnlyUsesTaskVariables) {
                NumericLimitDescriptor hypothesisedLimit(vars, weights, size);
            
                seeIfInducesALimit(hypothesisedLimit, upperBounds);
            }
            
        }
    }
    
    
    list<pair<int, int> >::const_iterator ngItr = numericGoals.begin();
    const list<pair<int, int> >::const_iterator ngEnd = numericGoals.end();
    
    for (; ngItr != ngEnd; ++ngItr) {
        for (int pass = 0; pass < 2; ++pass) {
            const int preID = (pass ? ngItr->second : ngItr->first);            
            if (preID == -1) continue;
            
            const RPGBuilder::RPGNumericPrecondition & currPre = RPGBuilder::getNumericPreTable()[preID];
            
            NumericLimitDescriptor hypothesisedLimit;
            
            if (currPre.LHSVariable < 2 * pneCount) {
                hypothesisedLimit = NumericLimitDescriptor(currPre.LHSVariable, 1.0, currPre.op, currPre.RHSConstant);
            } else {
                const RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(currPre.LHSVariable);
                hypothesisedLimit = NumericLimitDescriptor(currAV.fluents, currAV.weights, currAV.size, currPre.op, currPre.RHSConstant - currAV.constant);
            }
                                    
            seeIfInducesALimit(hypothesisedLimit, upperBounds);
        }
    }
    
    {        
        map<NumericLimitDescriptor,bool>::const_iterator limItr = upperBounds.begin();
        const map<NumericLimitDescriptor,bool>::const_iterator limEnd = upperBounds.end();
        
        for (; limItr != limEnd; ++limItr) {
            if (!limItr->second) {
                // limit not defined - not all effects were decreasers/increasers
                continue;
            }

            // if we get this far, we now have a limit
            cout << "Recognised a monotonic-change-induced limit on ";
            
            {
                map<int,double>::const_iterator vItr = limItr->first.var.begin();
                const map<int,double>::const_iterator vEnd = limItr->first.var.end();
                
                for (bool second=false; vItr != vEnd; ++vItr, second=true) {
                    if (second) {
                        cout << " + ";
                    }
                    if (vItr->second != 1.0) {
                        cout << vItr->second << "*";
                    }
                    if (vItr->first < 0) {
                        cout << "<special>";
                    } else {
                        cout << "var" << vItr->first;
                        cout.flush();
                        cout << *(RPGBuilder::getPNE(vItr->first));
                    }
                }
            }
            cout << std::endl;
            switch (limItr->first.op) {
                case VAL::E_GREATEQ:
                {
                    cout << "- Must be >= ";
                    break;
                }
                case VAL::E_GREATER:                
                {
                    cout << "- Must be > ";
                    break;
                }                                
                default:                    
                {
                    cout << "Internal error - must only have > or >= limits here\n";
                    exit(1);
                }
            }
            if (limItr->first.limit == -DBL_MAX) {
                assert(limItr->first.optimisationLimit);
                cout << " the metric";
            } else {
                cout << limItr->first.limit;
                if (limItr->first.optimisationLimit) {
                    cout << " and/or the metric";
                }
            }
            cout << std::endl;
            goalNumericUsageLimits.push_back(limItr->first);
        }
    }
    
    considerCostsOfLiteralGoalsReachedByGoalOnlyOperators();
}


#endif

vector<double> NumericAnalysis::maximumPositiveGradientOnVariable;
vector<double> NumericAnalysis::maximumNegativeGradientOnVariable;

vector<bool> NumericAnalysis::variableOnlyIncreasesByGradients;
vector<bool> NumericAnalysis::variableOnlyDecreasesByGradients;


void NumericAnalysis::findMaximumGradients()
{

    const int varCount = RPGBuilder::getPNECount();
    maximumNegativeGradientOnVariable.resize(varCount,0.0);
    maximumPositiveGradientOnVariable.resize(varCount,0.0);
    variableOnlyDecreasesByGradients.resize(varCount,true);
    variableOnlyIncreasesByGradients.resize(varCount,true);
    
    
    const vector<RPGBuilder::LinearEffects*> & ctsEffs = RPGBuilder::getLinearDiscretisation();
    
    const int actCount = ctsEffs.size();
    int vCount;
    bool selfOverlaps;
    double w;
    int v;
    
    for (int act = 0; act < actCount; ++act) {
        if (RPGBuilder::rogueActions[act]) {
            continue;
        }
        if (!ctsEffs[act]) {
            continue;
        }
        selfOverlaps = !RPGBuilder::isSelfMutex(act);
        
        vCount = ctsEffs[act]->vars.size();
        
        for (int i = 0; i < vCount; ++i) {
            w = ctsEffs[act]->effects[0][i].constant;
            v = ctsEffs[act]->vars[i];
            if (w > 0.0) {
                if (selfOverlaps) {
                    maximumPositiveGradientOnVariable[v] = DBL_MAX;
                } else {
                    if (maximumPositiveGradientOnVariable[v] != DBL_MAX) {
                        maximumPositiveGradientOnVariable[v] += w;
                    }
                }
            } else if (w < 0.0) {
                if (selfOverlaps) {
                    maximumNegativeGradientOnVariable[v] = -DBL_MAX;
                } else {
                    if (maximumNegativeGradientOnVariable[v] != -DBL_MAX) {
                        maximumNegativeGradientOnVariable[v] += w;
                    }
                }
            }
        }
    }
    
    const vector<RPGBuilder::RPGNumericEffect> & numericEffs = RPGBuilder::getNumericEff();
    const vector<list<pair<int, VAL::time_spec> > > & eta = RPGBuilder::getRpgNumericEffectsToActions();

    const int effCount = numericEffs.size();
    
    for (int eff = 0; eff < effCount; ++eff) {
        if (eta[eff].empty()) {
            continue;
        }
        if (numericEffs[eff].isAssignment || numericEffs[eff].size) {
            // simplication: assume that assignment could be increase or decrease
            // and effects that are non-constant could have positive or negative outcome
            variableOnlyDecreasesByGradients[numericEffs[eff].fluentIndex] = false;
            variableOnlyIncreasesByGradients[numericEffs[eff].fluentIndex] = false;
        } else {
            if (numericEffs[eff].constant > 0) {
                variableOnlyIncreasesByGradients[numericEffs[eff].fluentIndex] = false;
            } else if (numericEffs[eff].constant < 0) {
                variableOnlyDecreasesByGradients[numericEffs[eff].fluentIndex] = false;
            }
        }
    }
}


void NumericAnalysis::findDominanceConstraintsAndMetricTrackingVariables()
{
    const int pneCount =  RPGBuilder::getPNECount();
    
    const vector<RPGBuilder::RPGNumericPrecondition> & rpgNumericPreconditions = RPGBuilder::getNumericPreTable();
    const vector<RPGBuilder::ArtificialVariable> & rpgArtificialVariables = RPGBuilder::getArtificialVariableTable();
    
    dominanceConstraints.resize(pneCount, E_NODOMINANCE);
    
    list<int> knownToBeUseful;
    
    for (int i = 0; i < pneCount; ++i) {
        
        const int negativeI = i + pneCount;
        //PNE* const currPNE = RPGBuilder::getPNE(i);
        
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
                dominanceConstraints[i] = E_IRRELEVANT;
            }
        }
    }
    
    // case two - also must not appear as an input to an effect on a non-tracking variable
    
    for (int i = 0; i < pneCount; ++i) {
        if (dominanceConstraints[i] != E_IRRELEVANT) {
            knownToBeUseful.push_back(i);
        }
    }
    
    vector<bool> inputToAnEffect(pneCount, false);
    
    while (!knownToBeUseful.empty()) {
        list<int> newKnownToBeUseful;
        
        vector<RPGBuilder::RPGNumericEffect> & effs = RPGBuilder::getNumericEff();
        
        const int effCount = effs.size();
        int currVar;
        for (int effID = 0; effID < effCount; ++effID) {
            if (dominanceConstraints[effs[effID].fluentIndex] == E_IRRELEVANT) {
                // ignore effects on irrelevant variables
                continue;
            }
            
            for (int i = 0; i < effs[effID].size; ++i) {
                currVar = effs[effID].variables[i];
                if (currVar < 0) {
                    // ignore terms such as ?duration
                    continue;
                }
                if (currVar > pneCount) {
                    currVar -= pneCount;
                }
                
                if (dominanceConstraints[currVar] == E_IRRELEVANT) {
                    // this variable is an input to an effect on a known-useful variable
                    // it isn't irrelvant any more...
                    dominanceConstraints[currVar] = E_NODOMINANCE;
                    inputToAnEffect[currVar] = true;
                    // ...and next time round, make sure the inputs to any effects on this variable
                    // are also not marked as irrelevant
                    newKnownToBeUseful.push_back(currVar);
                }
            }
        }
        
        knownToBeUseful.swap(newKnownToBeUseful);
    }
    
    #ifdef POPF3ANALYSIS
    
    if (Globals::optimiseSolutionQuality) {
        RPGBuilder::Metric * theMetric = RPGBuilder::getMetric();
        
        if (theMetric) {
            list<int>::const_iterator mvItr = theMetric->variables.begin();
            const list<int>::const_iterator mvEnd = theMetric->variables.end();
            
            for (; mvItr != mvEnd; ++mvItr) {
                if (*mvItr >= 0 && dominanceConstraints[*mvItr] == E_IRRELEVANT) {
                    dominanceConstraints[*mvItr] = E_METRICTRACKING;
                }
            }
        }
    }

    bool anyCandidates = false;
    vector<dominance_constraint> workingDCs(dominanceConstraints);
    
    for (int i = 0; i < pneCount; ++i) {
        
        if (workingDCs[i] == E_NODOMINANCE && !inputToAnEffect[i]) {
            if ((workingDCs[i] = preconditionDominanceInOneDirection(i)) != E_NODOMINANCE) {
                anyCandidates = true;
            }
        }
    }
    
    if (!anyCandidates) return;
    
    considerConflictingDominanceThroughEffects(workingDCs);
    for (int i = 0; i < pneCount; ++i) {
        
        if (workingDCs[i] == E_BIGGERISBETTER) {
            dominanceConstraints[i] = E_BIGGERISBETTER;
            cout << "Have identified that bigger values of " << *(RPGBuilder::getPNE(i)) << " are preferable\n";
        } else if (workingDCs[i] == E_SMALLERISBETTER) {
            dominanceConstraints[i] = E_SMALLERISBETTER;
            cout << "Have identified that smaller values of " << *(RPGBuilder::getPNE(i)) << " are preferable\n";
        }
    }

    #endif
}

bool durationsAreConstantBounded(const list<pair<int, VAL::time_spec> > & actions)
{
       
    list<pair<int, VAL::time_spec> >::const_iterator actItr = actions.begin();
    const list<pair<int, VAL::time_spec> >::const_iterator actEnd = actions.end();
    
    int de;
    for (; actItr != actEnd; ++actItr) {
        const vector<RPGBuilder::RPGDuration*> & duration = RPGBuilder::getRPGDEs(actItr->first);
                
        if (duration.empty()) {
            cerr << "Fatal internal error: have a duration-dependent effect on " << *(RPGBuilder::getInstantiatedOp(actItr->first)) << ", but no durations\n";
            assert(!duration.empty()); // would mean a non-temporal action has a duration-dependent effect
        }
        
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
    
    allEffectsAreOrderIndependent.resize(pneCount, E_ORDER_INDEPENDENT);
    
    const vector<RPGBuilder::RPGNumericEffect> & numericEffects = RPGBuilder::getNumericEff();
    
    const int effCount = numericEffects.size();
    
    for (int i = 0; i < effCount; ++i) {
        
        if (numericEffects[i].isAssignment) {  // assignment effects cannot be ordered arbitrarily
            allEffectsAreOrderIndependent[numericEffects[i].fluentIndex] = E_ORDER_DEPENDENT;
        } else if (numericEffects[i].size > 1) {
            allEffectsAreOrderIndependent[numericEffects[i].fluentIndex] = E_ORDER_DEPENDENT;
        } else if (numericEffects[i].size == 1) {
            // the only order-independent non-constant effects are those that depend on the duration
            // of an action, where that duration does not depend on variables
            
            if (   numericEffects[i].variables[0] != -3 // -3 denotes the variable ?duration
                && numericEffects[i].variables[0] != -19) { // -19 denotes -?duration
                
                allEffectsAreOrderIndependent[numericEffects[i].fluentIndex] = E_ORDER_DEPENDENT;
                continue;   
            }
            
            if (durationsAreConstantBounded(RPGBuilder::getRpgNumericEffectsToActions()[i])) {
                allEffectsAreOrderIndependent[numericEffects[i].fluentIndex] = E_ORDER_DEPENDENT;
            }
        }        
    }
    
    const vector<RPGBuilder::LinearEffects*> ctsEffects = RPGBuilder::getLinearDiscretisation();
    const int opCount = ctsEffects.size();
    
    for (int op = 0; op < opCount; ++op) {
        if (!(ctsEffects[op])) continue;
        
        list<pair<int, VAL::time_spec> > dummyList;
        dummyList.push_back(make_pair(op, VAL::E_AT_START));
        
        const bool constantDur = durationsAreConstantBounded(dummyList);
        
        const int vCount = ctsEffects[op]->vars.size();
        for (int v = 0; v < vCount; ++v) {
            order_independence & currStatus = allEffectsAreOrderIndependent[ctsEffects[op]->vars[v]];
            if (!constantDur) { // if the duration is non constant
                currStatus = E_ORDER_DEPENDENT; // then the effect could be order-dependent
            } else if (currStatus == E_ORDER_INDEPENDENT) { // otherwise, if it's currently order-independent
                currStatus = E_ORDER_INDEPENDENT_AT_END; // then the CTS effect is order-independent, but only if no actions are executing
            }
        }
        
    }
    
}


#ifdef POPF3ANALYSIS

vector<bool> NumericAnalysis::onlyAtStartConditionsOnVariable;

void NumericAnalysis::findWhichVariablesAreOnlyInAtStarts()
{
    const int pneCount =  RPGBuilder::getPNECount();
    
    onlyAtStartConditionsOnVariable.resize(pneCount, true);
    
    const vector<RPGBuilder::RPGNumericPrecondition> & numPres = RPGBuilder::getNumericPreTable();
    
    vector<list<pair<int, VAL::time_spec> > > & presToActions = RPGBuilder::getRawNumericPresToActions();
    
    assert(numPres.size() == presToActions.size());
    
    const int npCount = numPres.size();
    
    for (int np = 0; np < npCount; ++np) {
        list<pair<int, VAL::time_spec> >::const_iterator depItr = presToActions[np].begin();
        const list<pair<int, VAL::time_spec> >::const_iterator depEnd = presToActions[np].end();
        
        for (; depItr != depEnd; ++depItr) {
            if (depItr->second != VAL::E_AT_START) {
                break;
            }
        }
        
        if (depItr != depEnd) { // then it was found in an invariant
            
            if (numPres[np].LHSVariable < pneCount) {
                onlyAtStartConditionsOnVariable[numPres[np].LHSVariable] = false;
            } else if (numPres[np].LHSVariable < 2 * pneCount) {
                onlyAtStartConditionsOnVariable[numPres[np].LHSVariable - pneCount] = false;
            } else {
                const RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(numPres[np].LHSVariable);
                for (int s = 0; s < currAV.size; ++s) {
                    if (currAV.fluents[s] < 0) {
                        continue;
                    } else if (currAV.fluents[s] < pneCount) {
                        onlyAtStartConditionsOnVariable[currAV.fluents[s]] = false;
                    } else {
                        onlyAtStartConditionsOnVariable[currAV.fluents[s] - pneCount] = false;
                    }
                }
            }
            
        }
    }
}  

bool NumericAnalysis::metricIsMonotonicallyWorsening = false;

void NumericAnalysis::findWhetherTheMetricIsMonotonicallyWorsening()
{
    const int lCount = goalNumericUsageLimits.size();
    
    for (int l = 0; l < lCount; ++l) {
        const NumericLimitDescriptor & currLim = goalNumericUsageLimits[l];
        
        if (currLim.optimisationLimit) {
            metricIsMonotonicallyWorsening = true;
        }
    }
    
}

#endif

};

