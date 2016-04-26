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
#include "temporalanalysis.h"

#include "typecheck.h"
#include "TIM.h"
#include "FuncAnalysis.h"
#include "temporalconstraints.h"

using namespace TIM;
using namespace Inst;
using namespace VAL;

using std::cerr;
using std::endl;

namespace Planner
{


struct RPGRegress {

    map<int, double> propositionalGoals;
    map<int, double> numericGoals;
    map<int, double> actionStarts;
    map<int, pair<int, double> > actionEnds;

};

void rpprintState(MinimalState & e)
{

    e.printState(cout);

};




class RPGHeuristic::Private
{

public:
    Private(const bool & b,
            vector<list<Literal*> > * atse,
            vector<list<Literal*> > * atee,
            vector<list<pair<int, VAL::time_spec> > > * eta,
            vector<list<Literal*> > * atsne,
            vector<list<Literal*> > * atene,
            vector<list<pair<int, VAL::time_spec> > > * neta,
            vector<list<pair<int, VAL::time_spec> > > * pta,
            vector<list<Literal*> > * atsp,
            vector<list<Literal*> > * ati,
            vector<list<Literal*> > * atep,
            vector<list<RPGBuilder::NumericEffect> > * atnuse,
            vector<list<RPGBuilder::NumericEffect> > * atnuee,
            vector<list<int> > * atrnuse,
            vector<list<int> > * atrnuee,
            vector<list<int> > * atnusp,
            vector<list<int> > * atnui,
            vector<list<int> > * atnuep,
            vector<list<int> > * atpnuep,
            vector<int> * iusp,
            vector<int> * iuip,
            vector<int> * iuep,
            vector<double> * ail,
            vector<double> * ailr,
            vector<pair<int, VAL::time_spec> > * ab,
            vector<pair<int, VAL::time_spec> > * abr,
            vector<double> * nail,
            vector<double> * nailr,
            vector<ActionFluentModification*> * nab,
            vector<ActionFluentModification*> * nabr,
            vector<int> * iunsp,
            vector<int> * iuni,
            vector<int> * iunep,
            vector<RPGBuilder::RPGNumericPrecondition> * rnp,
            vector<RPGBuilder::RPGNumericEffect> * rne,
            vector<list<pair<int, VAL::time_spec> > > * ppta,
            vector<list<pair<int, VAL::time_spec> > > * nppta,
            vector<list<Literal*> > * atpsp,
            vector<int> * iupsp,
            vector<int> * iupsnp,
            list<pair<int, VAL::time_spec> > * pla,
            list<pair<int, VAL::time_spec> > * onpa)
            :   actionsToStartEffects(atse),
            actionsToEndEffects(atee),
            effectsToActions(eta),
            actionsToStartNegativeEffects(atsne),
            actionsToEndNegativeEffects(atene),
            negativeEffectsToActions(neta),
            preconditionsToActions(pta),
            actionsToProcessedStartPreconditions(atpsp),
            actionsToStartPreconditions(atsp),
            actionsToInvariants(ati),
            actionsToEndPreconditions(atep),
            actionsToNumericStartEffects(atnuse),
            actionsToNumericEndEffects(atnuee),
            actionsToRPGNumericStartEffects(atrnuse),
            actionsToRPGNumericEndEffects(atrnuee),
            actionsToNumericStartPreconditions(atnusp),
            actionsToNumericInvariants(atnui),
            actionsToNumericEndPreconditions(atnuep),
            actionsToProcessedStartNumericPreconditions(atpnuep),
            initialUnsatisfiedStartPreconditions(iusp),
            initialUnsatisfiedInvariants(iuip),
            initialUnsatisfiedEndPreconditions(iuep),
            achievedInLayer(ail),
            achievedInLayerReset(ailr),
            achievedBy(ab),
            achievedByReset(abr),
            numericAchievedInLayer(nail),
            numericAchievedInLayerReset(nailr),
            numericAchievedBy(nab),
            numericAchievedByReset(nabr),
            initialUnsatisfiedNumericStartPreconditions(iunsp),
            initialUnsatisfiedNumericInvariants(iuni),
            initialUnsatisfiedNumericEndPreconditions(iunep),
            rpgNumericPreconditions(rnp),
            rpgNumericEffects(rne),
            processedPreconditionsToActions(ppta),
            processedNumericPreconditionsToActions(nppta),
            initialUnsatisfiedProcessedStartPreconditions(iupsp),
            initialUnsatisfiedProcessedStartNumericPreconditions(iupsnp),
            preconditionlessActions(pla),
            onlyNumericPreconditionActions(onpa),
            deleteArrays(b), expandFully(false), doneIntegration(false), evaluateDebug(false) {
        earliestPropositionPOTimes = vector<double>(achievedInLayer->size());
        earliestNumericPOTimes = vector<double>(RPGBuilder::getPNECount());
        hAddCostOfFact.resize(achievedInLayer->size());
//            earliestNumericPrePOTimes = vector<double>(numericAchievedInLayer->size());

    }


    static const double EPSILON;

    struct FactLayerEntry {

        pair<set<int>, set<int> > * endOfJustApplied;
        list<int> first;
        list<int> second;
        list<int> ees;
        list<pair<int, int> > TILs;
        list<int> negativeTILs;

        FactLayerEntry() : endOfJustApplied(0) {};
    };

    struct EndPrecRescale {
        int ID;
        int var;
        double offset;
        double totalchange;
        double duration;

        EndPrecRescale(const int & v, const double & off, const double & tc, const double & dur) : var(v), offset(off), totalchange(tc), duration(dur) {};

        EndPrecRescale() {};

        EndPrecRescale(const EndPrecRescale & e, const double & remaining) : var(e.var), offset(e.offset), totalchange(e.totalchange *(remaining / duration)), duration(remaining) {};

        bool operator <(const EndPrecRescale & r) const;

        bool isSatisfied(const vector<double> & tab) const {
            return (tab[var] >= totalchange + offset);
        }

    };

    class BuildingPayload
    {

    public:

        const MinimalState & startState;

        vector<int> startPreconditionCounts;
        vector<int> endPreconditionCounts;
        vector<int> numericStartPreconditionCounts;
        vector<int> numericEndPreconditionCounts;

        map<double, FactLayerEntry, EpsilonComp > factLayers;
        map<double, vector<double>, EpsilonComp > fluentLayers;
        map<double, map<int, list<ActionFluentModification> >, EpsilonComp> fluentModifications;
        map<double, list<int>, EpsilonComp > endActionsAtTime;

        vector<double> startActionSchedule;
        vector<double> endActionSchedule;
        vector<double> openEndActionSchedule;

        const map<int, set<int> > & startedActions;
        int unsatisfiedGoals;
        int unappearedEnds;
        map<int, set<int> > insistUponEnds;
        map<int, int> forbiddenStart;
        map<int, int> forbiddenEnd;
        const int vCount;
        const int avCount;

        const vector<double> & minTimestamps;

        list<ActionSegment> & helpfulActions;

        map<int, double> earliestStartOf;

        map<int, pair<double, double> > propositionMustBeDeletedAddedAfter;

        list<pair<set<int>, set<int> > > gc;

        BuildingPayload(const MinimalState & theState,
                        vector<int> & spc, vector<int> & epc, vector<int> & nspc, vector<int> & nepc,
                        const int & easSize, const int goalCount,
                        const vector<double> & mtsIn, list<ActionSegment> & haIn)
                : startState(theState),
                startPreconditionCounts(spc), endPreconditionCounts(epc),
                numericStartPreconditionCounts(nspc), numericEndPreconditionCounts(nepc),
                startActionSchedule(easSize, -1.0), endActionSchedule(easSize, -1.0), openEndActionSchedule(easSize, -1.0),
                startedActions(theState.startedActions),
                unsatisfiedGoals(goalCount), unappearedEnds(0),
                vCount(theState.secondMin.size()), avCount(RPGBuilder::getAVCount()),
                minTimestamps(mtsIn), helpfulActions(haIn)

        {
        }

    };

#ifdef MDIDEBUG

#define MAKEENDMDI(dest,iv,ot,a) MaxDependentInfo dest(iv,ot,a)
#define MAKESTARTMDI(dest,a) MaxDependentInfo dest(a)


    class MaxDependentInfo
    {

    private:

        double offsetToEarlier;
        bool haveCalculated;
        double value;

        VAL::time_spec ts;
        int currAct;


        static BuildingPayload * referTo;

    public:

        static bool debug;

        static void updatePayload(BuildingPayload* const p) {
            referTo = p;
        }



        MaxDependentInfo(const double & initialValue, const double & offsetTE, const int & actID)
                : offsetToEarlier(offsetTE), haveCalculated(false), value(initialValue),
                ts(VAL::E_AT_END), currAct(actID) {


            if (debug) {
                cout << "Made MDI for " << *(RPGBuilder::getInstantiatedOp(currAct)) << " end\n";
            }

        }


        MaxDependentInfo(const int & actID)
                : offsetToEarlier(0.001), haveCalculated(false), value(0.0),
                ts(VAL::E_AT_START), currAct(actID) {

#ifdef MDIDEBUG
            if (debug) {
                cout << "Made MDI for " << *(RPGBuilder::getInstantiatedOp(currAct)) << " start\n";
            }
#endif
        }


        const double & get() {
            if (haveCalculated) return value;
            if (debug) {
                cout << "MDI for " << *currAct;
                if (ts == VAL::E_AT_START) {
                    cout << " start";
                } else {
                    cout << " end";
                }

                cout << " = max[";
            }
            /*
                            for (int pass = 0; pass < 3; ++pass) {
                                double offset;
                                const list<Literal*> * loopOn;
                                const list<int> * numLoopOn;

                                switch(pass) {
                                    case 0:
                                    {
                                        offset = offsetToEarlier;
                                        loopOn = &(RPGBuilder::getStartPropositionalPreconditions()[currAct]);
                                        numLoopOn = &(RPGBuilder::getStartPreNumerics()[currAct]);
                                        break;
                                    }

                                    case 1:
                                    {
                                        offset = offsetToEarlier - 0.001;
                                        loopOn = &(RPGBuilder::getInvariantPropositionalPreconditions()[currAct]);
                                        numLoopOn = &(RPGBuilder::getInvariantNumerics()[currAct]);
                                        break;
                                    }

                                    case 2:
                                    {
                                        if (ts == VAL::E_AT_START) {
                                            loopOn = (list<Literal*>*)0;
                                            numLoopOn = (list<int>*)0;
                                        } else {
                                            offset = 0.001;
                                            loopOn = &(RPGBuilder::getEndPropositionalPreconditions()[currAct]);
                                            numLoopOn = &(RPGBuilder::getEndPreNumerics()[currAct]);
                                        }
                                        break;
                                    }

                                }

                                if (loopOn) {

                                    list<Literal*>::const_iterator preItr = loopOn->begin();
                                    const list<Literal*>::const_iterator preEnd = loopOn->end();

                                    for (; preItr != preEnd; ++preItr) {
                                        const int ID = (*preItr)->getID();
                                        const double poTS = RPGHeuristic::Private::earliestPropositionPOTimes[ID] + offset;
                                        if (debug) {
                                            if (pass == 0) {
                                                cout << " " << *(*preItr) << "s=" << poTS;
                                            } else if (pass == 1) {
                                                cout << " " << *(*preItr) << "i=" << poTS;
                                            } else {
                                                cout << " " << *(*preItr) << "e=" << poTS;
                                            }
                                        }
                                        if (poTS > value) value = poTS;
                                    }
                                }

                                if (numLoopOn) {
                                    list<int>::const_iterator preItr = numLoopOn->begin();
                                    const list<int>::const_iterator preEnd = numLoopOn->end();

                                    for (; preItr != preEnd; ++preItr) {
                                        RPGBuilder::RPGNumericPrecondition & currPre = RPGBuilder::getNumericPreTable()[*preItr];
                                        const double poTS = RPGHeuristic::Private::earliestPointForNumericPrecondition(currPre) + offset;
                                        if (debug) {
                                            if (pass == 0) {
                                                cout << " num" << *preItr << "sn=" << poTS;
                                            } else if (pass == 1) {
                                                cout << " num" << *preItr << "in=" << poTS;
                                            } else {
                                                cout << " num" << *preItr << "en=" << poTS;
                                            }
                                        }

                                        if (poTS > value) value = poTS;
                                    }
                                }

                            }

                            const int passCount = (ts == VAL::E_AT_START ? 1 : 2);
                            for (int pass = 0; pass < passCount; ++pass) {
                                const double offset = (pass ? 0.001 : offsetToEarlier);
                                const list<Literal*> * const addLoop = (pass ? &(RPGBuilder::getEndPropositionAdds()[currAct]) : &(RPGBuilder::getStartPropositionAdds()[currAct]));
                                const list<Literal*> * const delLoop = (pass ? &(RPGBuilder::getEndPropositionDeletes()[currAct]) : &(RPGBuilder::getStartPropositionDeletes()[currAct]));
                                const list<int> * const numLoop = (pass ? &(RPGBuilder::getEndEffNumerics()[currAct]) : &(RPGBuilder::getStartEffNumerics()[currAct]));

                                {
                                    list<Literal*>::const_iterator preItr = addLoop->begin();
                                    const list<Literal*>::const_iterator preEnd = addLoop->end();

                                    for (; preItr != preEnd; ++preItr) {
                                        const int ID = (*preItr)->getID();
                                        const map<int, pair<double,double> >::iterator poRestrictItr = referTo->propositionMustBeDeletedAddedAfter.find(ID);
                                        if (poRestrictItr != referTo->propositionMustBeDeletedAddedAfter.end()) {
                                            const double poTS = poRestrictItr->second.second + offset;
                                            #ifdef MDIDEBUG
                                            if (debug) {
                                                if (pass == 0) {
                                                    cout << " " << *(*preItr) << "s+=" << poTS;
                                                } else {
                                                    cout << " " << *(*preItr) << "e-=" << poTS;
                                                }
                                            }
                                            #endif
                                            if (poTS > value) value = poTS;
                                        }
                                        #ifdef MDIDEBUG
                                        else {
                                            if (debug) {
                                                if (pass == 0) {
                                                    cout << " " << *(*preItr) << "s+=I";
                                                } else {
                                                    cout << " " << *(*preItr) << "e+=I";
                                                }
                                            }
                                        }
                                        #endif
                                    }
                                }

                                {
                                    list<Literal*>::const_iterator preItr = delLoop->begin();
                                    const list<Literal*>::const_iterator preEnd = delLoop->end();

                                    for (; preItr != preEnd; ++preItr) {
                                        const int ID = (*preItr)->getID();
                                        const map<int, pair<double,double> >::iterator poRestrictItr = referTo->propositionMustBeDeletedAddedAfter.find(ID);
                                        if (poRestrictItr != referTo->propositionMustBeDeletedAddedAfter.end()) {
                                            const double poTS = poRestrictItr->second.first + offset;
                                            #ifdef MDIDEBUG
                                            if (debug) {
                                                if (pass == 0) {
                                                    cout << " " << *(*preItr) << "s-=" << poTS;
                                                } else {
                                                    cout << " " << *(*preItr) << "e-=" << poTS;
                                                }
                                            }
                                            #endif
                                            if (poTS > value) value = poTS;
                                        }
                                        #ifdef MDIDEBUG
                                        else {
                                            if (debug) {
                                                if (pass == 0) {
                                                    cout << " " << *(*preItr) << "s-=I";
                                                } else {
                                                    cout << " " << *(*preItr) << "e-=I";
                                                }
                                            }
                                        }
                                        #endif

                                    }
                                }

                                {
                                    list<int>::const_iterator preItr = numLoop->begin();
                                    const list<int>::const_iterator preEnd = numLoop->end();

                                    for (; preItr != preEnd; ++preItr) {
                                        const RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[*preItr];
                                        const double poTS = RPGHeuristic::Private::earliestPointForNumericEffect(currEff) + offset;
                                        #ifdef MDIDEBUG
                                        if (debug) {
                                            if (pass == 0) {
                                                cout << " " << *preItr << "sne=" << poTS;
                                            } else {
                                                cout << " " << *preItr << "ene=" << poTS;
                                            }
                                        }
                                        #endif
                                        if (poTS > value) value = poTS;

                                    }
                                }

                            }

                            if (RPGBuilder::getRPGDEs(currAct).empty()) {
                                const double poTS = earliestPointForDuration(*(RPGBuilder::getRPGDEs(currAct)[0])) + (ts == VAL::E_AT_START ? 0.001 : offsetToEarlier);

                                #ifdef MDIDEBUG
                                if (debug) {
                                    cout << " dur=" << poTS;
                                }
                                #endif
                                if (poTS > value) value = poTS;
                            }
            */

            if (debug) {
                cout << "] = " << value << "\n";
            }

            haveCalculated = true;


            return value;
        }

    };

#else

    typedef int MaxDependentInfo;

#define MAKEENDMDI(dest,iv,ot,a) MaxDependentInfo dest = a
#define MAKESTARTMDI(dest,a) MaxDependentInfo dest = a
#endif

    set<int> goals;
    set<int>::iterator gsEnd;
    set<int> goalFluents;
    set<int>::iterator gfEnd;



    vector<list<Literal*> > * const actionsToStartEffects;
    vector<list<Literal*> > * const actionsToEndEffects;
    vector<list<pair<int, VAL::time_spec> > > * const effectsToActions;

    vector<list<Literal*> > * const actionsToStartNegativeEffects;
    vector<list<Literal*> > * const actionsToEndNegativeEffects;
    vector<list<pair<int, VAL::time_spec> > > * const negativeEffectsToActions;


    vector<list<pair<int, VAL::time_spec> > > * const preconditionsToActions;

    vector<list<Literal*> > * const actionsToProcessedStartPreconditions;
    vector<list<Literal*> > * const actionsToStartPreconditions;
    vector<list<Literal*> > * const actionsToInvariants;
    vector<list<Literal*> > * const actionsToEndPreconditions;

    vector<list<RPGBuilder::NumericEffect> > * const actionsToNumericStartEffects;
    vector<list<RPGBuilder::NumericEffect> > * const actionsToNumericEndEffects;

    vector<list<int> > integratedCTSEffectVar;
    vector<list<double> > integratedCTSEffectChange;

    vector<list<int> > * const actionsToRPGNumericStartEffects;
    vector<list<int> > * const actionsToRPGNumericEndEffects;

    vector<list<int> > * const actionsToNumericStartPreconditions;
    vector<list<int> > * const actionsToNumericInvariants;
    vector<list<int> > * const actionsToNumericEndPreconditions;
    vector<list<int> > * const actionsToProcessedStartNumericPreconditions;

    vector<int> * const initialUnsatisfiedStartPreconditions;
    vector<int> * const initialUnsatisfiedInvariants;
    vector<int> * const initialUnsatisfiedEndPreconditions;

    vector<double> hAddCostOfFact;
    vector<double> * const achievedInLayer;
    vector<double> * const achievedInLayerReset;
    vector<pair<int, VAL::time_spec> > * const achievedBy;
    vector<pair<int, VAL::time_spec> > * const achievedByReset;

    vector<double> * const numericAchievedInLayer;
    vector<double> * const numericAchievedInLayerReset;
    vector<ActionFluentModification*> * const numericAchievedBy;
    vector<ActionFluentModification*> * const numericAchievedByReset;

    vector<int> * const initialUnsatisfiedNumericStartPreconditions;
    vector<int> * const initialUnsatisfiedNumericInvariants;
    vector<int> * const initialUnsatisfiedNumericEndPreconditions;

    vector<RPGBuilder::RPGNumericPrecondition> * const rpgNumericPreconditions;
    vector<RPGBuilder::RPGNumericEffect> * const rpgNumericEffects;


    vector<list<pair<int, VAL::time_spec> > > * const processedPreconditionsToActions;
    vector<list<pair<int, VAL::time_spec> > > * const processedNumericPreconditionsToActions;

    vector<int> * const initialUnsatisfiedProcessedStartPreconditions;
    vector<int> * const initialUnsatisfiedProcessedStartNumericPreconditions;

    list<pair<int, VAL::time_spec> > * const preconditionlessActions;
    list<pair<int, VAL::time_spec> > * const onlyNumericPreconditionActions;
    list<pair<int, VAL::time_spec> > noLongerForbidden;


    static vector<double> earliestStartAllowed;
    static vector<double> earliestEndAllowed;
    static vector<double> latestStartAllowed;
    static vector<double> latestEndAllowed;
    static vector<double> deadlineAtTime;
    static vector<double> earliestDeadlineRelevancyStart;
    static vector<double> earliestDeadlineRelevancyEnd;

    static vector<list<int> > tilEffects;
    static vector<list<int> > tilNegativeEffects;
    static vector<list<int> > tilTemporaryNegativeEffects;
    static vector<double> tilTimes;
    static bool tilInitialised;
    static int tilCount;

    static vector<double> earliestPropositionPOTimes;
    static vector<double> earliestNumericPOTimes;
//    static vector<double> earliestNumericPrePOTimes;

    static vector<vector<set<int> > > actionsAffectedByFluent;

    const bool deleteArrays;
    bool expandFully;
    bool doneIntegration;
    bool evaluateDebug;


    double calculateActCost(BuildingPayload * const payload, const int & currAct, const VAL::time_spec & currTS);
    
    void setDebugFlag(const bool & b) {
        evaluateDebug = b;
    }

    void buildEmptyActionFluentLookupTable() {

        const int varCount = RPGBuilder::getPNECount();
        actionsAffectedByFluent = vector<vector<set<int> > >(varCount, vector<set<int> >(2));


    }

    void populateActionFluentLookupTable() {

        static bool populated = false;

        if (populated) return;

        populated = true;

        const int actCount = actionsToNumericStartPreconditions->size();
        const int varCount = RPGBuilder::getPNECount();

        const vector<RPGBuilder::RPGNumericPrecondition> & preTable = RPGBuilder::getNumericPreTable();
        const vector<RPGBuilder::RPGNumericEffect> & effTable = RPGBuilder::getNumericEff();

        for (int pass = 0; pass < 2; ++pass) {
            const vector<list<int> > * const nowOn = (pass ? actionsToNumericEndPreconditions : actionsToProcessedStartNumericPreconditions);

            for (int a = 0; a < actCount; ++a) {
                if (RPGBuilder::rogueActions[a]) continue;
                list<int>::const_iterator pItr = ((*nowOn)[a]).begin();
                const list<int>::const_iterator pEnd = ((*nowOn)[a]).end();

                for (; pItr != pEnd; ++pItr) {
                    const RPGBuilder::RPGNumericPrecondition & currPre = preTable[*pItr];
                    for (int side = 0; side < 2; ++side) {
                        int var = (side ? currPre.RHSVariable : currPre.LHSVariable);
                        if (var == -1) continue;

                        if (var < 2 * varCount) {
                            if (var >= varCount) var -= varCount;
                            assert(var < varCount);
                            assert(var >= 0);
                            assert(!RPGBuilder::rogueActions[a]);
                            actionsAffectedByFluent[var][pass].insert(a);
                        } else {
                            const RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(var);

                            for (int i = 0; i < currAV.size; ++i) {
                                int lvar = currAV.fluents[i];
                                if (lvar >= varCount) lvar -= varCount;
                                assert(lvar < varCount);
                                assert(lvar >= 0);
                                assert(!RPGBuilder::rogueActions[a]);
                                actionsAffectedByFluent[lvar][pass].insert(a);
                            }
                        }
                    }
                }
            }
        }

        for (int pass = 0; pass < 2; ++pass) {
            const vector<list<int> > * const nowOn = (pass ? actionsToRPGNumericEndEffects : actionsToRPGNumericStartEffects);

            for (int a = 0; a < actCount; ++a) {
                if (RPGBuilder::rogueActions[a]) continue;

                list<int>::const_iterator eItr = ((*nowOn)[a]).begin();
                const list<int>::const_iterator eEnd = ((*nowOn)[a]).end();

                for (; eItr != eEnd; ++eItr) {
                    const RPGBuilder::RPGNumericEffect & currEff = effTable[*eItr];
                    assert(!RPGBuilder::rogueActions[a]);
                    actionsAffectedByFluent[currEff.fluentIndex][pass].insert(a);

                    for (int i = 0; i < currEff.size; ++i) {
                        int var = currEff.variables[i];
                        if (var < 0) continue;
                        if (var >= varCount) var -= varCount;
                        actionsAffectedByFluent[var][pass].insert(a);
                    }
                }
            }
        }

        for (int a = 0; a < actCount; ++a) {
            if (RPGBuilder::rogueActions[a]) continue;
            const vector<RPGBuilder::RPGDuration*> & currDEs = RPGBuilder::getRPGDEs(a);
            if (currDEs.empty()) continue;

            for (int pass = 0; pass < 3; ++pass) {
                const list<RPGBuilder::DurationExpr*> & currList = (*(currDEs[0]))[pass];

                list<RPGBuilder::DurationExpr*>::const_iterator clItr = currList.begin();
                const list<RPGBuilder::DurationExpr*>::const_iterator clEnd = currList.end();

                for (; clItr != clEnd; ++clItr) {
                    const int loopLim = (*clItr)->variables.size();
                    for (int i = 0; i < loopLim; ++i) {
                        assert(!RPGBuilder::rogueActions[a]);
                        actionsAffectedByFluent[(*clItr)->variables[i]][0].insert(a);
                    }
                }
            }
        }



        vector<RPGBuilder::LinearEffects*> & LD = RPGBuilder::getLinearDiscretisation();

        if (!LD.empty()) {

            for (int a = 0; a < actCount; ++a) {
                if (RPGBuilder::rogueActions[a]) continue;
                const RPGBuilder::LinearEffects * const currLD = LD[a];

                if (!currLD) continue;
                const int effCount = currLD->vars.size();

                for (int eff = 0; eff < effCount; ++eff) {
                    assert(!RPGBuilder::rogueActions[a]);
                    actionsAffectedByFluent[currLD->vars[eff]][0].insert(a);
                }
            }

        }
    }

    void integrateContinuousEffects() {
        if (doneIntegration) return;

        vector<RPGBuilder::LinearEffects*> & LD = RPGBuilder::getLinearDiscretisation();

        const int loopLim = LD.size();

        integratedCTSEffectVar = vector<list<int> >(loopLim);
        integratedCTSEffectChange = vector<list<double> >(loopLim);


        for (int lda = 0; lda < loopLim; ++lda) {
            if (RPGBuilder::rogueActions[lda]) continue;
            RPGBuilder::LinearEffects * const currLD = LD[lda];
            if (currLD) {
                const double dur = RPGBuilder::getOpMaxDuration(lda, 0);
                const int effCount = currLD->vars.size();
                for (int eff = 0; eff < effCount; ++eff) {
                    integratedCTSEffectVar[lda].push_back(currLD->vars[eff]);
                    integratedCTSEffectChange[lda].push_back(currLD->effects[0][eff].constant * dur);
                }


            }
        }

        doneIntegration = true;
    }

    BuildingPayload * spawnNewPayload(const MinimalState & theState, const vector<double> & minTimestamps, list<ActionSegment> & haIn) {

        static const int easSize = initialUnsatisfiedEndPreconditions->size();

        BuildingPayload * const toReturn =
            new BuildingPayload(theState,
                                *(initialUnsatisfiedProcessedStartPreconditions),
                                *(initialUnsatisfiedEndPreconditions),
                                *(initialUnsatisfiedProcessedStartNumericPreconditions),
                                *(initialUnsatisfiedNumericEndPreconditions),
                                easSize, goals.size() + goalFluents.size(),
                                minTimestamps, haIn);

        {
            vector<double> maxFluentTable(toReturn->vCount * 2 + toReturn->avCount);
            toReturn->fluentLayers.insert(pair<double, vector<double> >(0.0, maxFluentTable));

        }
        vector<double> * maxFluentTable = &(toReturn->fluentLayers.begin()->second);

        {
            for (int i = 0; i < toReturn->vCount; ++i) {
                if (evaluateDebug) cout << "Fluent " << i << " " << *(RPGBuilder::getPNE(i)) << " bounds: [";
                {
                    const double ov = theState.secondMax[i];
                    (*maxFluentTable)[i] = ov;

                }
                {
                    const double ov = theState.secondMin[i];
                    if (ov != 0.0) {
                        (*maxFluentTable)[i + toReturn->vCount] = 0.0 - ov;
                    } else {
                        (*maxFluentTable)[i + toReturn->vCount] = 0.0;
                    }
                    if (evaluateDebug) cout << ov << ",";
                }
                if (evaluateDebug) {
                    cout << (*maxFluentTable)[i] << "]\n";
                }
            }
        }
        {
            const int startLim = toReturn->vCount * 2;
            const int endLim = startLim + RPGBuilder::getAVCount();
            for (int i = startLim; i < endLim; ++i) {
                (*maxFluentTable)[i] = RPGBuilder::getArtificialVariable(i).evaluate(*maxFluentTable);
                if (evaluateDebug) cout << "AV " << i << " = " << (*maxFluentTable)[i] << "\n";
            }
        }




        {
            map<int, set<int> >::const_iterator saItr = theState.startedActions.begin();
            const map<int, set<int> >::const_iterator saEnd = theState.startedActions.end();

            for (; saItr != saEnd; ++saItr) {
                double earliest = DBL_MAX;

                set<int>::const_iterator instanceItr = saItr->second.begin();
                const set<int>::const_iterator instanceEnd = saItr->second.end();

                for (; instanceItr != instanceEnd; ++instanceItr) {
                    const double newTS = minTimestamps[*instanceItr];
                    if (newTS < earliest) earliest = newTS;
                }

                earliest += RPGBuilder::getOpMinDuration(saItr->first, 0);

                toReturn->earliestStartOf.insert(make_pair(saItr->first, earliest));
            }
        }

#ifdef MDIDEBUG
        MaxDependentInfo::updatePayload(toReturn);
#endif
        return toReturn;
    }

    void giveUsTheEffectsOfExecutingActions(BuildingPayload * const payload) {

        vector<double> * maxFluentTable = 0;
        map<int, set<int> >::const_iterator saItr = payload->startState.startedActions.begin();
        const map<int, set<int> >::const_iterator saEnd = payload->startState.startedActions.end();

        for (; saItr != saEnd; ++saItr) {
            const int multiplier = saItr->second.size();

            list<int> & varList = integratedCTSEffectVar[saItr->first];
            list<double> & changeList = integratedCTSEffectChange[saItr->first];

            list<int>::iterator vlItr = varList.begin();
            const list<int>::iterator vlEnd = varList.end();

            list<double>::iterator clItr = changeList.begin();
            const list<double>::iterator clEnd = changeList.end();

            for (; vlItr != vlEnd; ++vlItr, ++clItr) {
                const double change = *clItr * multiplier;
                if (change > 0.0) {
                    if (!maxFluentTable) {
                        maxFluentTable = &(payload->fluentLayers.begin()->second);
                    }
                    (*maxFluentTable)[*vlItr] += change;
                } else if (change < 0.0) {
                    if (!maxFluentTable) {
                        maxFluentTable = &(payload->fluentLayers.begin()->second);
                    }
                    (*maxFluentTable)[payload->vCount + *vlItr] += (0.0 - change);
                }
            }

            //if (evaluateDebug || eeDebug) cout << "Clearing extra ends attached to " << *(RPGBuilder::getInstantiatedOp(saItr->first)) << "\n";
        }


    }

    void resetAchievedBy() {
        *achievedBy = (*achievedByReset);
        *achievedInLayer = (*achievedInLayerReset);

        *numericAchievedBy = (*numericAchievedByReset);
        *numericAchievedInLayer = (*numericAchievedInLayerReset);

    }

    static double earliestPointForNumericPrecondition(const RPGBuilder::RPGNumericPrecondition & p) {
        static const int varCount = RPGBuilder::getPNECount();
        double TS = 0.0;

        for (int pass = 0; pass < 2; ++pass) {
            int var = (pass ? p.RHSVariable : p.LHSVariable);
            if (var == -1) continue;
            if (var >= 2 * varCount) {
                const RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(var);

                for (int i = 0; i < currAV.size; ++i) {
                    int varTwo = currAV.fluents[i];
                    if (varTwo >= varCount) varTwo -= varCount;
                    if (earliestNumericPOTimes[varTwo] > TS) TS = earliestNumericPOTimes[varTwo];
                }
            } else {
                if (var >= varCount) var -= varCount;
                if (earliestNumericPOTimes[var] > TS) TS = earliestNumericPOTimes[var];
            }
        }

        return TS;
    }

    static double earliestPointForNumericEffect(const RPGBuilder::RPGNumericEffect & p) {

        static const int varCount = RPGBuilder::getPNECount();

        double TS = 0.0;

        if (earliestNumericPOTimes[p.fluentIndex] > TS) TS = earliestNumericPOTimes[p.fluentIndex];

        for (int s = 0; s < p.size; ++s) {
            int var = p.variables[s];
            if (var < 0) continue;
            if (var >= varCount) var -= varCount;
            if (earliestNumericPOTimes[var] > TS) TS = earliestNumericPOTimes[var];
        }

        return TS;
    }


    static double earliestPointForDuration(const RPGBuilder::RPGDuration & currDE) {
        double TS = 0.0;

        for (int pass = 0; pass < 3; ++pass) {
            const list<RPGBuilder::DurationExpr*> & currList = currDE[pass];
            list<RPGBuilder::DurationExpr*>::const_iterator lItr = currList.begin();
            const list<RPGBuilder::DurationExpr*>::const_iterator lEnd = currList.end();

            for (; lItr != lEnd; ++lItr) {
                const int vSize = ((*lItr)->variables.size());
                for (int i = 0; i < vSize; ++i) {
                    const int var = (*lItr)->variables[i];
                    if (earliestNumericPOTimes[var] > TS) TS = earliestNumericPOTimes[var];
                }
            }
        }
        return TS;
    }

    void recordFactLayerZero(BuildingPayload * const payload) {
        {
            map<int, PropositionAnnotation>::const_iterator stateItr = payload->startState.first.begin();
            const map<int, PropositionAnnotation>::const_iterator stateEnd = payload->startState.first.end();

            for (; stateItr != stateEnd; ++stateItr) {
                const int factID = stateItr->first;
                (*achievedInLayer)[factID] = 0.0;
                hAddCostOfFact[factID] = 0.0;
                pair<double, double> * toUpdate = 0;
                const StepAndBeforeOrAfter & from = stateItr->second.availableFrom;
                if (from.beforeOrAfter == StepAndBeforeOrAfter::BEFORE) { // special case: initial state is anything achieved /before/ step 0, rather than at step 0
                    earliestPropositionPOTimes[factID] = 0.0;
                    //if (!stateItr->second.deletableFrom.empty()) {
                    //    toUpdate = (&payload->propositionMustBeDeletedAddedAfter.insert(make_pair(factID,make_pair(0.0,0.0))).first->second);
                    //}
                    //if (!expandFully) cout << "Initial Fact " << *(RPGBuilder::getLiteral(factID)) << " is true in the initial state\n";
                } else {
                    const unsigned int dependsOnActionAtStep = from.stepID;
                    const double actTS = payload->minTimestamps[dependsOnActionAtStep];
                    earliestPropositionPOTimes[factID] = actTS;
                    //if (!expandFully) cout << "Initial Fact " << *(RPGBuilder::getLiteral(factID)) << " appears in PO no earlier than action " << dependsOnActionAtStep << ", i.e. t=" << earliestPropositionPOTimes[factID] << "\n";

                    toUpdate = &(payload->propositionMustBeDeletedAddedAfter.insert(make_pair(factID, make_pair(0.0, 0.0))).first->second);

                    if (toUpdate->first < actTS) toUpdate->first = actTS;
                    if (toUpdate->second < actTS) toUpdate->second = actTS;

                    if (RPGBuilder::modifiedRPG && actTS >= 0.001) {
                        if (evaluateDebug) cout << "RPG modified: delaying " << *(RPGBuilder::getLiteral(factID)) << " to layer " << (actTS - 0.001) << "\n";
                        (*achievedInLayer)[factID] = actTS - 0.001;
                    }

                }

                if (toUpdate) {
                    map<StepAndBeforeOrAfter, bool>::const_iterator invItr = stateItr->second.deletableFrom.begin();
                    const map<StepAndBeforeOrAfter, bool>::const_iterator invEnd = stateItr->second.deletableFrom.end();

                    for (; invItr != invEnd; ++invItr) {
                        const double actTS = payload->minTimestamps[invItr->first.stepID] - (invItr->first.beforeOrAfter == StepAndBeforeOrAfter::BEFORE ? 0.001 : 0.0);
                        if (actTS > 0.0) {
                            if (toUpdate->first < actTS) toUpdate->first = actTS;
                        }
                    }
                }
            }

            //if (!expandFully) MaxDependentInfo::debug = true;
        }

        {
            map<int, PropositionAnnotation>::const_iterator stateItr = payload->startState.retired.begin();
            const map<int, PropositionAnnotation>::const_iterator stateEnd = payload->startState.retired.end();

            for (; stateItr != stateEnd; ++stateItr) {
                const int factID = stateItr->first;
                const StepAndBeforeOrAfter & from = stateItr->second.negativeAvailableFrom;
                const unsigned int dependsOnActionAtStep = from.stepID;
                const double actTS = payload->minTimestamps[dependsOnActionAtStep];
                pair<double, double> & toUpdate = payload->propositionMustBeDeletedAddedAfter.insert(make_pair(factID, make_pair(0.0, 0.0))).first->second;

                if (toUpdate.first < actTS) toUpdate.first = actTS;
                if (toUpdate.second < actTS) toUpdate.second = actTS;

                map<StepAndBeforeOrAfter, bool>::const_iterator invItr = stateItr->second.deletableFrom.begin();
                const map<StepAndBeforeOrAfter, bool>::const_iterator invEnd = stateItr->second.deletableFrom.end();

                for (; invItr != invEnd; ++invItr) {
                    const double actTS = payload->minTimestamps[invItr->first.stepID] - (invItr->first.beforeOrAfter == StepAndBeforeOrAfter::BEFORE ? 0.001 : 0.0);
                    if (actTS > 0.0) {
                        if (toUpdate.first < actTS) toUpdate.first = actTS;
                    }
                }
            }

        }

        {
            const vector<FluentInteraction> & lastStepToTouchPNE = payload->startState.temporalConstraints->lastStepToTouchPNE;

            const int loopLim = lastStepToTouchPNE.size();

            for (int i = 0; i < loopLim; ++i) {
                const int stepID = lastStepToTouchPNE[i].lastInstantaneousEffect;
                const double actTS = (stepID == -1 ? -0.001 : payload->minTimestamps[stepID]);
                earliestNumericPOTimes[i] = actTS;
            }
        }

        {
            const int loopLim = rpgNumericPreconditions->size();

            if (loopLim) {
                vector<double> * const maxFluentTable = &(payload->fluentLayers.begin()->second);
                for (int i = 0; i < loopLim; ++i) {
                    if (ignoreNumbers || (*rpgNumericPreconditions)[i].isSatisfied(*maxFluentTable)) {
                        const double poTS = earliestPointForNumericPrecondition((*rpgNumericPreconditions)[i]);
                        //earliestNumericPrePOTimes[i] = poTS;
                        (*numericAchievedBy)[i] = 0;
                        (*numericAchievedInLayer)[i] = (RPGBuilder::modifiedRPG && poTS >= 0.001 ? (poTS - 0.001) : 0.0);
                        if (evaluateDebug && RPGBuilder::modifiedRPG && (*numericAchievedInLayer)[i] > 0.0) {
                            cout << "RPG modified: delaying numeric fact " << i << " to layer " << (*numericAchievedInLayer)[i] << "\n";
                        }
                    }
                }
            }
        }

    }

    void addRequirementToHaveSeenTheEndOfAllCurrentlyExecutingActions(BuildingPayload * const payload) {
        map<int, set<int> >::const_iterator saItr = payload->startedActions.begin();
        const map<int, set<int> >::const_iterator saEnd = payload->startedActions.end();

        for (; saItr != saEnd; ++saItr) {
            if (!RPGBuilder::canSkipToEnd(saItr->first)) {
                payload->insistUponEnds.insert(*saItr);
                if (evaluateDebug) cout << "Insisting on the end of " << saItr->first << " - is not skippable\n";
            } else {
                if (evaluateDebug) cout << "End of " << *(RPGBuilder::getInstantiatedOp(saItr->first)) << " is a skippable\n";
            }

        }

        payload->unappearedEnds = payload->insistUponEnds.size();
    }

    void seeWhatGoalsAreTrueToStartWith(BuildingPayload * const payload) {

        {
            set<int>::iterator gsItr = goals.begin();

            for (; gsItr != gsEnd; ++gsItr) {
                const map<int, PropositionAnnotation>::const_iterator gfItr = payload->startState.first.find(*gsItr);
                if (gfItr != payload->startState.first.end()) {
                    if (evaluateDebug) {
                        cout << "\t" << *gsItr << " true in initial state\n";
                    }
                    --(payload->unsatisfiedGoals);
                }
            }
        }

        {

            set<int>::iterator gfItr = goalFluents.begin();

            if (gfItr == gfEnd) return;

            vector<double> * const maxFluentTable = &(payload->fluentLayers.begin()->second);

            for (; gfItr != gfEnd; ++gfItr) {
                if (ignoreNumbers || (*rpgNumericPreconditions)[*gfItr].isSatisfied(*maxFluentTable)) {
                    if (evaluateDebug) {
                        cout << "\t" << (*rpgNumericPreconditions)[*gfItr] << " true in initial state\n";
                    }

                    --(payload->unsatisfiedGoals);
                } else if (evaluateDebug) {
                    cout << "\t" << (*rpgNumericPreconditions)[*gfItr] << " false in initial state\n";
                }
            }

        }


    }


    void addTemporalConstraintsFromActiveActions(BuildingPayload * const payload) {
        if (!RPGBuilder::modifiedRPG) return;

        map<int, pair<double, double> >::iterator constrItr = payload->propositionMustBeDeletedAddedAfter.begin();
        const map<int, pair<double, double> >::iterator constrEnd = payload->propositionMustBeDeletedAddedAfter.end();

        for (; constrItr != constrEnd; ++constrItr) {

            const int litID = constrItr->first;

            for (int pass = 0; pass < 2; ++pass) {
                const list<pair<int, VAL::time_spec> > & effList = (pass ? (*negativeEffectsToActions)[litID] : (*effectsToActions)[litID]);
                const double layer = (pass ? constrItr->second.second : constrItr->second.first) - 0.001;

                if (layer <= 0.001) continue;

                assert(layer >= 0.000);
                FactLayerEntry & layerEntry = payload->factLayers[layer];

                if (!layerEntry.endOfJustApplied) {
                    payload->gc.push_back(make_pair(set<int>(), set<int>()));
                    layerEntry.endOfJustApplied = &(payload->gc.back());
                }

                pair<set<int>, set<int> > * const dest = layerEntry.endOfJustApplied;

                list<pair<int, VAL::time_spec> >::const_iterator effItr = effList.begin();
                const list<pair<int, VAL::time_spec> >::const_iterator effEnd = effList.end();

                for (; effItr != effEnd; ++effItr) {
                    if (!RPGBuilder::rogueActions[effItr->first]) {
                        if (effItr->second == VAL::E_AT_START) {
                            if (dest->first.insert(effItr->first).second) {
                                ++(payload->forbiddenStart.insert(pair<int, int>(effItr->first, 0)).first->second);
                            }
                        } else {
                            if (dest->second.insert(effItr->first).second) {
                                ++(payload->forbiddenEnd.insert(pair<int, int>(effItr->first, 0)).first->second);
                            }
                        }
                    }
                }

                if (layerEntry.endOfJustApplied->first.empty() && layerEntry.endOfJustApplied->second.empty()) {
                    layerEntry.endOfJustApplied = 0;
                    payload->gc.pop_back();
                }
            }

        }

        const int varCount = earliestNumericPOTimes.size();

        for (int v = 0; v < varCount; ++v) {
            const double layer = earliestNumericPOTimes[v] - 0.001;
            if (layer <= 0.001) continue;
            assert(layer >= 0.000);
            FactLayerEntry & layerEntry = payload->factLayers[layer];

            if (!layerEntry.endOfJustApplied) {
                payload->gc.push_back(make_pair(set<int>(), set<int>()));
                layerEntry.endOfJustApplied = &(payload->gc.back());
            }

            pair<set<int>, set<int> > * const dest = layerEntry.endOfJustApplied;

            for (int pass = 0; pass < 2; ++pass) {
                set<int> & destSet = (pass ? dest->second : dest->first);
                const set<int> & loopSet = actionsAffectedByFluent[v][pass];

                set<int>::const_iterator aItr = loopSet.begin();
                const set<int>::const_iterator aEnd = loopSet.end();

                for (; aItr != aEnd; ++aItr) {
                    if (!RPGBuilder::rogueActions[*aItr]) {
                        if (destSet.insert(*aItr).second) {
                            if (pass) {
                                ++(payload->forbiddenEnd.insert(pair<int, int>(*aItr, 0)).first->second);
                            } else {
                                ++(payload->forbiddenStart.insert(pair<int, int>(*aItr, 0)).first->second);
                            }
                        }
                    }
                }
            }


            if (layerEntry.endOfJustApplied->first.empty() && layerEntry.endOfJustApplied->second.empty()) {
                layerEntry.endOfJustApplied = 0;
                payload->gc.pop_back();
            }

        }

        //static vector<double> earliestNumericPOTimes;
    }


    bool addTemporalConstraintsFromActiveActions(BuildingPayload * const payload, map<double, list<pair<int, int> > > * const justApplied, const double & stateTS, const int & nextTIL, const double & tilFrom) {

        if (!justApplied) {
            if (evaluateDebug) {
                cout << "Not just applied a start, so no definitely active invariants\n";
            }

            return true;
        }

        double TILoffset = (nextTIL < tilCount ? tilTimes[nextTIL] - tilFrom : 0.0);

        list<pair<set<int>, set<int> > > setsToForbid;


        map<double, list<pair<int, int> > >::iterator jaItr = justApplied->begin();
        const map<double, list<pair<int, int> > >::iterator jaEnd = justApplied->end();
        for (; jaItr != jaEnd; ++jaItr) {
            const double insLayer = jaItr->first;
            if (insLayer > 0.0) {
                setsToForbid.push_back(pair<set<int>, set<int> >());
                set<int> & affectedStarts = setsToForbid.back().first;
                set<int> & affectedEnds = setsToForbid.back().second;

                list<pair<int, int> >::iterator liItr = jaItr->second.begin();
                const list<pair<int, int> >::iterator liEnd = jaItr->second.end();

                for (; liItr != liEnd; ++liItr) {
                    const int currActID = liItr->first;
                    {

                        list<Literal*> & invs = (*actionsToInvariants)[currActID];
                        list<Literal*>::iterator invItr = invs.begin();
                        const list<Literal*>::iterator invEnd = invs.end();

                        for (; invItr != invEnd; ++invItr) {
                            const int litID = (*invItr)->getID();
                            list<pair<int, VAL::time_spec> > & affected = (*negativeEffectsToActions)[litID];
                            list<pair<int, VAL::time_spec> >::iterator affItr = affected.begin();
                            const list<pair<int, VAL::time_spec> >::iterator affEnd = affected.end();
                            for (; affItr != affEnd; ++affItr) {
                                if (affItr->second == VAL::E_AT_START) {
                                    if (evaluateDebug) {
                                        cout << "Delaying start of " << affItr->first << " to " << insLayer << "\n";
                                    }
                                    affectedStarts.insert(affItr->first);
                                } else {
                                    affectedEnds.insert(affItr->first);
                                    if (evaluateDebug) {
                                        cout << "Delaying end of " << affItr->first << " to " << insLayer << "\n";
                                    }

                                }
                            }
                            if (nextTIL < tilCount) {

                                for (int i = nextTIL; i < tilCount; ++i) {
                                    const double earliestTILtime = tilTimes[i] - TILoffset;

                                    if (earliestTILtime < insLayer) {
                                        list<int>::iterator effItr = tilTemporaryNegativeEffects[i].begin();
                                        const list<int>::iterator effEnd = tilTemporaryNegativeEffects[i].end();

                                        for (; effItr != effEnd; ++effItr) {
                                            if (*effItr == litID) {
                                                TILoffset -= ((insLayer - earliestTILtime) + EPSILON);
                                                if (TILoffset < stateTS) {
                                                    if (evaluateDebug) cout << "Dead end found: TIL definitely going to delete invariant of open action\n";
                                                    return false;
                                                }
                                                break;
                                            }

                                        }
                                    }
                                }

                            }
                        }
                    }
                    if (nextTIL < tilCount) {
                        list<Literal*> & invs = (*actionsToEndPreconditions)[currActID];
                        list<Literal*>::iterator precItr = invs.begin();
                        const list<Literal*>::iterator precEnd = invs.end();

                        for (; precItr != precEnd; ++precItr) {
                            const int litID = (*precItr)->getID();
                            for (int i = nextTIL; i < tilCount; ++i) {
                                const double earliestTILtime = tilTimes[i] - TILoffset;

                                if (earliestTILtime < insLayer) {
                                    list<int>::iterator effItr = tilNegativeEffects[i].begin();
                                    const list<int>::iterator effEnd = tilNegativeEffects[i].end();

                                    for (; effItr != effEnd; ++effItr) {
                                        if (*effItr == litID) {
                                            TILoffset -= ((insLayer - earliestTILtime) + EPSILON);
                                            if (TILoffset < stateTS) {
                                                if (evaluateDebug) cout << "Dead end found: TIL definitely going to delete invariant of open action\n";
                                                return false;
                                            }
                                            break;
                                        }

                                    }
                                }
                            }
                        }

                    }
                }

                if (!affectedStarts.empty() || !affectedEnds.empty()) {
                    assert(insLayer >= 0.000);
                    payload->factLayers[insLayer].endOfJustApplied = &(setsToForbid.back());
                    {
                        set<int>::iterator sItr = affectedStarts.begin();
                        const set<int>::iterator sEnd = affectedStarts.end();
                        for (; sItr != sEnd; ++sItr) {
                            ++(payload->forbiddenStart.insert(pair<int, int>(*sItr, 0)).first->second);
                            if (evaluateDebug) {
                                cout << "Start of " << *sItr << " now delayed because of " << payload->forbiddenStart[*sItr] << " actions\n";
                            }
                        }
                    }
                    {
                        set<int>::iterator sItr = affectedEnds.begin();
                        const set<int>::iterator sEnd = affectedEnds.end();
                        for (; sItr != sEnd; ++sItr) {
                            ++(payload->forbiddenEnd.insert(pair<int, int>(*sItr, 0)).first->second);
                            if (evaluateDebug) {
                                cout << "End of " << *sItr << " now delayed because of " << payload->forbiddenEnd[*sItr] << " actions\n";
                            }
                        }
                    }

                }
                if (!affectedEnds.empty()) {
                    set<int>::iterator sItr = affectedEnds.begin();
                    const set<int>::iterator sEnd = affectedEnds.end();

                    for (; sItr != sEnd; ++sItr) {
                        const double actDur = RPGBuilder::getOpMaxDuration(*sItr, 0);
                        const double earlierIns = insLayer - actDur;
                        if (earlierIns > 0.0) {
                            assert(earlierIns >= 0.000);
                            FactLayerEntry & fle = payload->factLayers[earlierIns];
                            if (!fle.endOfJustApplied) {
                                setsToForbid.push_back(pair<set<int>, set<int> >());
                                fle.endOfJustApplied = &(setsToForbid.back());
                                fle.endOfJustApplied->first.insert(*sItr);
                                ++(payload->forbiddenStart.insert(pair<int, int>(*sItr, 0)).first->second);
                            } else if (fle.endOfJustApplied->first.insert(*sItr).second) {
                                ++(payload->forbiddenStart.insert(pair<int, int>(*sItr, 0)).first->second);
                            }

                        }

                    }


                }
            }
            {
                list<pair<int, int> >::iterator liItr = jaItr->second.begin();
                const list<pair<int, int> >::iterator liEnd = jaItr->second.end();

                for (; liItr != liEnd; ++liItr) {
                    const int currActID = liItr->first;
                    double & eas = payload->openEndActionSchedule[currActID];
                    if (eas == -1.0) {
                        eas = insLayer;
                        if (evaluateDebug) cout << "Delaying end of " << currActID << " until " << insLayer << "\n";
                    }

                }

            }
        }

        return true;
    }

    void performTILInitialisation() {

        if (tilInitialised) return;

        tilInitialised = true;
        list<RPGBuilder::FakeTILAction> & TILs = RPGBuilder::getTILs();
        tilCount = TILs.size();
        tilEffects = vector<list<int> >(tilCount);
        tilNegativeEffects = vector<list<int> >(tilCount);
        tilTemporaryNegativeEffects = vector<list<int> >(tilCount);
        tilTimes = vector<double>(tilCount);

        {
            const int loopLim = processedPreconditionsToActions->size();
            deadlineAtTime = vector<double>(loopLim);
            for (int i = 0; i < loopLim; ++i) deadlineAtTime[i] = DBL_MAX;

        }

        earliestDeadlineRelevancyStart = vector<double>(initialUnsatisfiedEndPreconditions->size());
        earliestDeadlineRelevancyEnd = vector<double>(initialUnsatisfiedEndPreconditions->size());


        list<RPGBuilder::FakeTILAction>::reverse_iterator tilItr = TILs.rbegin();
        const list<RPGBuilder::FakeTILAction>::reverse_iterator tilEnd = TILs.rend();

        set<int> addedLater;

        for (int i = tilCount - 1; tilItr != tilEnd; ++tilItr, --i) {

            tilTimes[i] = tilItr->duration;


            {
                list<Literal*>::iterator effItr = tilItr->addEffects.begin();
                const list<Literal*>::iterator effEnd = tilItr->addEffects.end();

                for (; effItr != effEnd; ++effItr) {
                    const int currEffID = (*effItr)->getID();
                    tilEffects[i].push_back(currEffID);
                    addedLater.insert(currEffID);
                }
            }

            {
                list<Literal*>::iterator effItr = tilItr->delEffects.begin();
                const list<Literal*>::iterator effEnd = tilItr->delEffects.end();

                for (; effItr != effEnd; ++effItr) {
                    const int currEffID = (*effItr)->getID();
                    tilTemporaryNegativeEffects[i].push_back(currEffID);
                    if ((*effectsToActions)[currEffID].empty() && addedLater.find(currEffID) == addedLater.end()) {
                        tilNegativeEffects[i].push_back(currEffID);
                        deadlineAtTime[currEffID] = tilTimes[i];
                    }
                }
            }


        }


    }

    void initialiseLatestArrays() {

        static bool initLatestArrays = false;


        static const int easSize = initialUnsatisfiedEndPreconditions->size();

        if (!initLatestArrays) {
            earliestStartAllowed = vector<double>(easSize);
            earliestEndAllowed = vector<double>(easSize);
            latestStartAllowed = vector<double>(easSize);
            latestEndAllowed = vector<double>(easSize);
            initLatestArrays = true;
        }



        for (int i = 0; i < easSize; ++i) latestStartAllowed[i] = DBL_MAX;
        for (int i = 0; i < easSize; ++i) latestEndAllowed[i] = DBL_MAX;

        if (expandFully) {
            for (int i = 0; i < easSize; ++i) earliestStartAllowed[i] = DBL_MAX;
            for (int i = 0; i < easSize; ++i) earliestEndAllowed[i] = DBL_MAX;
        }
    }

    void addTILsBeforeExpansion(const int & nextTIL, map<double, FactLayerEntry, EpsilonComp > & factLayers, const double & stateTS, const double & tilFrom) {
        if (nextTIL >= tilCount) return;

        static const int easSize = initialUnsatisfiedEndPreconditions->size();

        for (int i = 0; i < easSize; ++i) earliestDeadlineRelevancyStart[i] = DBL_MAX;
        for (int i = 0; i < easSize; ++i) earliestDeadlineRelevancyEnd[i] = DBL_MAX;


        set<int> earlier;

        const double TILoffset = (RPGBuilder::modifiedRPG ? 0.000 : (nextTIL < tilCount ? tilTimes[nextTIL] - tilFrom : 0.0));

        for (int i = nextTIL; i < tilCount; ++i) {
            const double thisTS = tilTimes[i] - TILoffset;

            list<int>::iterator effItr = tilEffects[i].begin();
            const list<int>::iterator effEnd = tilEffects[i].end();

            for (; effItr != effEnd; ++effItr) {

                const int currEff = (*effItr);
                double & currAIL = (*achievedInLayer)[currEff];
                if (currAIL == -1.0 && (*achievedBy)[currEff].first == -1) { // not in initial state
                    if (earlier.insert(currEff).second) {
                        assert(thisTS >= 0.000);
                        factLayers[thisTS].TILs.push_back(pair<int, int>(i, currEff));
                    }
                }

            }

        }

        for (int i = nextTIL; i < tilCount; ++i) {
            const double thisTS = tilTimes[i] - stateTS;

            list<int>::iterator effItr = tilNegativeEffects[i].begin();
            const list<int>::iterator effEnd = tilNegativeEffects[i].end();

            for (; effItr != effEnd; ++effItr) {

                list<pair<int, VAL::time_spec> > & currList = (*preconditionsToActions)[*effItr];

                list<pair<int, VAL::time_spec> >::iterator ioItr = currList.begin();
                const list<pair<int, VAL::time_spec> >::iterator ioEnd = currList.end();

                for (; ioItr != ioEnd; ++ioItr) {
                    const int actID = ioItr->first;
                    if (ioItr->second == VAL::E_AT_START) {
                        if (latestStartAllowed[actID] > thisTS) latestStartAllowed[actID] = thisTS;
                        const double minDur = RPGBuilder::getOpMinDuration(actID, 0);
                        if (latestEndAllowed[actID] > thisTS + minDur) latestEndAllowed[actID] = thisTS + minDur;
                    } else {
                        const double minDur = RPGBuilder::getOpMinDuration(actID, -1);
                        if (latestStartAllowed[actID] > thisTS - minDur) latestStartAllowed[actID] = thisTS - minDur;
                        if (latestEndAllowed[actID] > thisTS) latestEndAllowed[actID] = thisTS;
                    }
                }


            }

        }



    }

    void addBoundsFromTemporalAnalysis(const double & stateTS) {

        const int actCount = latestStartAllowed.size();

        for (int a = 0; a < actCount; ++a) {
            {
                double firmUpper = TemporalAnalysis::getActionTSBounds()[a][0].second;
                if (firmUpper != DBL_MAX) {
                    firmUpper -= stateTS;
                    if (firmUpper < latestStartAllowed[a]) latestStartAllowed[a] = firmUpper;
                }
                if (latestStartAllowed[a] < 0.000) latestEndAllowed[a] = -1.0;
            }
            {
                double firmUpper = TemporalAnalysis::getActionTSBounds()[a][1].second;
                if (firmUpper != DBL_MAX) {
                    firmUpper -= stateTS;
                    if (firmUpper < latestEndAllowed[a]) latestEndAllowed[a] = firmUpper;
                }
                if (latestEndAllowed[a] < 0.001) latestStartAllowed[a] = -1.0;
            }

        }

    }

    double earliestTILForAction(const unsigned int & i, const bool & isStart);
    void findApplicableActions(const MinimalState & theState, const double & stateTime, list<ActionSegment> & applicableActions);
    void filterApplicableActions(const MinimalState & theState, const double & stateTime, list<ActionSegment> & applicableActions);
    bool testApplicability(const MinimalState & theState, const double & stateTime, const ActionSegment & actID, const bool & fail, const bool & ignoreDeletes);

    instantiatedOp* getOp(const unsigned int & i) {

        assert(i >= 0);
        assert(i < actionsToEndEffects->size());
        return RPGBuilder::getInstantiatedOp(i);

    };


    bool sensiblePair(const BuildingPayload * const payload, const pair<int, VAL::time_spec> & achiever, const unsigned int & fact) {
        assert(fact >= 0);
        assert(fact < preconditionsToActions->size());

        if (achiever.second == VAL::E_AT) {
            assert(tilInitialised);
            assert(achiever.first >= 0);
            assert(achiever.first < tilCount);
        } else {
            if (achiever.first < 0) {
                cerr << "\t\tTrying to use a negative indexed action as an achiever for fact " << *(RPGBuilder::getLiteral(fact)) << "\n";
                if (payload->startState.first.find(fact) != payload->startState.first.end()) {
                    cerr << "\t\t\tWas true in the initial state\n";
                }
                assert(achiever.first >= 0);
            }
            assert((unsigned int) achiever.first < actionsToEndPreconditions->size());
        }

        return true;
    }

    void extractRP(BuildingPayload * const payload, int & h, list<pair<double, list<ActionSegment> > > & relaxedPlan, pair<int, VAL::time_spec> & earliestTIL, double & makespanEstimate) {

        makespanEstimate = 0.0;

        const MinimalState & theState = payload->startState;
        map<int, set<int> > & insistUponEnds = payload->insistUponEnds;
        vector<double> & startActionSchedule = payload->startActionSchedule;
        vector<double> & openEndActionSchedule = payload->openEndActionSchedule;

        static const set<int> emptyIntSet;

        map<double, RPGRegress, EpsilonComp> goalsAtLayer;

        {
            set<int>::iterator gsItr = goals.begin();
            const set<int>::iterator gsEnd = goals.end();

            for (; gsItr != gsEnd; ++gsItr) {
                const int currGoal = *gsItr;
                const double insLayer = (*achievedInLayer)[currGoal];
                if (insLayer > 0.0) {
                    const int achiever = (*achievedBy)[currGoal].first;
                    if (achiever != -1) {
                        goalsAtLayer[insLayer].propositionalGoals.insert(pair<int, double>(currGoal, DBL_MAX));
                        if (evaluateDebug) cout << "Goal " << *(RPGBuilder::getLiteral(*gsItr)) << " to be achieved in layer with TS " << insLayer << "\n";
                    } else {
                        assert(payload->startState.first.find(currGoal) != payload->startState.first.end());
                        if (evaluateDebug) cout << "Goal " << *(RPGBuilder::getLiteral(*gsItr)) << " achieved in initial state at layer " << insLayer << "\n";
                    }
                } else if (evaluateDebug) cout << "Goal " << *(RPGBuilder::getLiteral(*gsItr)) << " achieved in initial state, not adding to RPG regression\n";


                if (insLayer > makespanEstimate) {
                    makespanEstimate = insLayer;
                }
            }
        }

        if (!ignoreNumbers) {
            set<int>::iterator gfItr = goalFluents.begin();
            set<int>::iterator gfEnd = goalFluents.end();

            for (; gfItr != gfEnd; ++gfItr) {
                const int currGoal = *gfItr;
                const double insLayer = (*numericAchievedInLayer)[currGoal];
                if (insLayer > 0.0) {
                    if ((*numericAchievedBy)[currGoal]) {
                        goalsAtLayer[insLayer].numericGoals.insert(pair<int, double>(currGoal, DBL_MAX));
                        if (evaluateDebug) cout << "Numeric goal to be achieved in layer with TS " << insLayer << "\n";
                    } else {
                        if (evaluateDebug) cout << "Numeric goal to be achieved in initial state at layer " << insLayer << "\n";
                    }
                } else if (evaluateDebug) cout << "Numeric goal achieved in initial state, not adding to RPG regression\n";

                if (insLayer > makespanEstimate) {
                    makespanEstimate = insLayer;
                }

            }
        }


        if (!RPGBuilder::nonTemporalProblem()) {
            map<int, set<int> >::iterator ueItr = insistUponEnds.begin();
            const map<int, set<int> >::iterator ueEnd = insistUponEnds.end();

            for (; ueItr != ueEnd; ++ueItr) {
                const int currAct = ueItr->first;
                const double insLayer = openEndActionSchedule[currAct];
                const double tilR = earliestTILForAction(currAct, false);
                (goalsAtLayer[insLayer + EPSILON].actionEnds.insert(pair<int, pair<int, double> >(currAct, pair<int, double>(0, tilR))).first->second.first) += ueItr->second.size();
                // HACK
            }
        }


        while (!goalsAtLayer.empty()) {

            const double currTS = goalsAtLayer.rbegin()->first;
            map<int, double> & currGAL = goalsAtLayer.rbegin()->second.propositionalGoals;
            map<int, double> & currNGAL = goalsAtLayer.rbegin()->second.numericGoals;
            map<int, double> & actionStarts = goalsAtLayer.rbegin()->second.actionStarts;
            map<int, pair<int, double> > & actionEnds = goalsAtLayer.rbegin()->second.actionEnds;
            bool alreadyPushed = false;

            {


                map<int, double>::iterator asItr = actionStarts.begin();
                const map<int, double>::iterator asEnd = actionStarts.end();

                if (evaluateDebug && asItr != asEnd) cout << "Adding starts at TS " << currTS << "\n";

                for (; asItr != asEnd; ++asItr) {

                    if (evaluateDebug) cout << "\t\tAdding start of " << asItr->first << "\n";

                    double tilR = earliestTILForAction(asItr->first, true);
                    if (tilR > asItr->second) tilR = asItr->second;

                    if (!alreadyPushed) {
                        relaxedPlan.push_front(pair<double, list<ActionSegment> >(currTS - EPSILON, list<ActionSegment>()));
                        alreadyPushed = true;
                    }

                    relaxedPlan.front().second.push_back(ActionSegment(getOp(asItr->first), VAL::E_AT_START, -1, emptyIntList));
                    ++h;

//              if (currTS == EPSILON) {
//                  helpfulActions.push_back(pair<int, VAL::time_spec>(*asItr, VAL::E_AT_START));
//                  if (evaluateDebug) cout << "\t\tIs a helpful action\n";
//              }

                    {
                        list<Literal*> & actionEffectsList = (*actionsToStartEffects)[asItr->first];
                        list<Literal*>::iterator aelItr = actionEffectsList.begin();
                        const list<Literal*>::iterator aelEnd = actionEffectsList.end();

                        for (; aelItr != aelEnd; ++aelItr) {
                            map<int, double>::iterator cgItr = currGAL.find((*aelItr)->getID());
                            if (cgItr != currGAL.end()) {
                                if (tilR > cgItr->second) tilR = cgItr->second;
                                currGAL.erase(cgItr);
                            }
                        }
                    }

                    {

                        map<int, double>::iterator ngalItr = currNGAL.begin();
                        const map<int, double>::iterator ngalEnd = currNGAL.end();

                        while (ngalItr != ngalEnd) {
                            ActionFluentModification* const afm = (*numericAchievedBy)[ngalItr->first];
                            if (afm && afm->act == asItr->first && afm->ts == VAL::E_AT_START) {
                                if (tilR > ngalItr->second) tilR = ngalItr->second;
                                map<int, double>::iterator tItr = ngalItr;
                                ++tItr;
                                currNGAL.erase(ngalItr);
                                ngalItr = tItr;
                            } else {
                                ++ngalItr;
                            }
                        }
                    }

                    bool isHelpful = true;

                    {
                        list<Literal*> & actionPreconditionlist = (*actionsToProcessedStartPreconditions)[asItr->first];
                        list<Literal*>::iterator aplItr = actionPreconditionlist.begin();
                        const list<Literal*>::iterator aplEnd = actionPreconditionlist.end();

                        if (evaluateDebug) cout << "\t\tPreconditions:\n";

                        for (; aplItr != aplEnd; ++aplItr) {

                            const int currPrec = (*aplItr)->getID();
                            const double acIn = (*achievedInLayer)[currPrec];
                            if (acIn > 0.0) {
                                if ((*achievedBy)[currPrec].first != -1) {
                                    map<int, double>::iterator insItr = goalsAtLayer[acIn].propositionalGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                    if (insItr->second > tilR) insItr->second = tilR;
                                    if (evaluateDebug) cout << "\t\tAdding requirement for " << currPrec << " at time " << acIn << "\n";
                                    assert(acIn < currTS);
                                    isHelpful = false;
                                } else {
                                    if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state at layer " << acIn << "\n";
                                }
                            } else {
                                if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state\n";
                            }

                        }

                        if (evaluateDebug) cout << "\t\tPreconditions done.\n";
                    }

                    if (!ignoreNumbers) {
                        list<int> & actionPreconditionlist = (*actionsToProcessedStartNumericPreconditions)[asItr->first];
                        list<int>::iterator aplItr = actionPreconditionlist.begin();
                        const list<int>::iterator aplEnd = actionPreconditionlist.end();

                        if (evaluateDebug) cout << "\t\tNumeric preconditions:\n";

                        for (; aplItr != aplEnd; ++aplItr) {

                            const int currPrec = (*aplItr);
                            const double acIn = (*numericAchievedInLayer)[currPrec];
                            if (acIn > 0.0) {
                                if ((*numericAchievedBy)[currPrec]) {
                                    map<int, double>::iterator insItr = goalsAtLayer[acIn].numericGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                    if (insItr->second > tilR) insItr->second = tilR;
                                    if (evaluateDebug) cout << "\t\tAdding requirement for numeric precondition " << currPrec << " at time " << acIn << "\n";
                                    assert(acIn < currTS);
                                    isHelpful = false;
                                } else {
                                    if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state at layer " << acIn << "\n";
                                }
                            } else {
                                if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state\n";
                            }

                        }

                        if (evaluateDebug) cout << "\t\tPreconditions done.\n";
                    }


                    if (isHelpful) {
                        payload->helpfulActions.push_back(ActionSegment(RPGBuilder::getInstantiatedOp(asItr->first), VAL::E_AT_START, -1, emptyIntSet));
                        if (evaluateDebug) cout << "\t\tIs a helpful action\n";
                    }

                    double & oldR = earliestDeadlineRelevancyStart[asItr->first];
                    if (oldR > tilR) oldR = tilR;

                }
            }

            {


                map<int, pair<int, double> >::iterator asItr = actionEnds.begin();
                const map<int, pair<int, double> >::iterator asEnd = actionEnds.end();

                if (evaluateDebug && asItr != asEnd) cout << "Adding ends at TS " << currTS << "\n";

                for (; asItr != asEnd; ++asItr) {

                    if (evaluateDebug) cout << "\t\tAdding end of " << asItr->first << "\n";

                    double tilR = earliestTILForAction(asItr->first, false);
                    if (tilR > asItr->second.second) tilR = asItr->second.second;


                    if (!alreadyPushed) {
                        relaxedPlan.push_front(pair<double, list<ActionSegment> >(currTS - EPSILON, list<ActionSegment>()));
                        alreadyPushed = true;
                    }

                    if (!RPGBuilder::canSkipToEnd(asItr->first)) {
                        const int loopLim = asItr->second.first;
                        const ActionSegment theOp(getOp(asItr->first), VAL::E_AT_END, -1, emptyIntList);
                        for (int pb = 0; pb < loopLim; ++pb) {
                            relaxedPlan.front().second.push_back(theOp);
                        }
                        h += loopLim;
                    }


                    {
                        list<Literal*> & actionEffectsList = (*actionsToEndEffects)[asItr->first];
                        list<Literal*>::iterator aelItr = actionEffectsList.begin();
                        const list<Literal*>::iterator aelEnd = actionEffectsList.end();

                        for (; aelItr != aelEnd; ++aelItr) {
                            map<int, double>::iterator cgItr = currGAL.find((*aelItr)->getID());
                            if (cgItr != currGAL.end()) {
                                if (tilR > cgItr->second) tilR = cgItr->second;
                                currGAL.erase(cgItr);
                            }
                        }
                    }

                    {

                        map<int, double>::iterator ngalItr = currNGAL.begin();
                        const map<int, double>::iterator ngalEnd = currNGAL.end();

                        while (ngalItr != ngalEnd) {
                            ActionFluentModification* const afm = (*numericAchievedBy)[ngalItr->first];
                            if (afm && afm->act == asItr->first && afm->ts == VAL::E_AT_END) {
                                if (tilR > ngalItr->second) tilR = ngalItr->second;
                                map<int, double>::iterator tItr = ngalItr;
                                ++tItr;
                                currNGAL.erase(ngalItr);
                                ngalItr = tItr;
                            } else {
                                ++ngalItr;
                            }
                        }
                    }

                    bool isHelpful = true;

                    {
                        list<Literal*> & actionPreconditionlist = (*actionsToEndPreconditions)[asItr->first];
                        list<Literal*>::iterator aplItr = actionPreconditionlist.begin();
                        const list<Literal*>::iterator aplEnd = actionPreconditionlist.end();

                        for (; aplItr != aplEnd; ++aplItr) {

                            const int currPrec = (*aplItr)->getID();
                            const double acIn = (*achievedInLayer)[currPrec];
                            if (acIn > 0.0) {
                                if ((*achievedBy)[currPrec].first != -1) {
                                    map<int, double>::iterator galItr = goalsAtLayer[acIn].propositionalGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                    if (galItr->second > tilR) galItr->second = tilR;
                                    isHelpful = false;
                                }
                            }

                        }


                    }

                    if (!ignoreNumbers) {
                        list<int> & actionPreconditionlist = (*actionsToNumericEndPreconditions)[asItr->first];
                        list<int>::iterator aplItr = actionPreconditionlist.begin();
                        const list<int>::iterator aplEnd = actionPreconditionlist.end();

                        if (evaluateDebug) cout << "\t\tNumeric preconditions:\n";

                        for (; aplItr != aplEnd; ++aplItr) {

                            const int currPrec = (*aplItr);
                            const double acIn = (*numericAchievedInLayer)[currPrec];
                            if (acIn > 0.0) {
                                if ((*numericAchievedBy)[currPrec]) {
                                    map<int, double>::iterator galItr = goalsAtLayer[acIn].numericGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                    if (galItr->second > tilR) galItr->second = tilR;
                                    if (evaluateDebug) cout << "\t\tAdding requirement for numeric precondition " << currPrec << " at time " << acIn << "\n";
                                    assert(acIn < currTS);
                                    isHelpful = false;
                                } else {
                                    if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state at time " << acIn << "\n";
                                }
                            } else {
                                if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state\n";
                            }

                        }


                    }



                    if (evaluateDebug) cout << "\t\tPreconditions done.\n";


                    if (isHelpful) {
                        if (payload->startState.startedActions.find(asItr->first) != payload->startState.startedActions.end()) {
                            payload->helpfulActions.push_back(ActionSegment(RPGBuilder::getInstantiatedOp(asItr->first), VAL::E_AT_END, -1, emptyIntSet));
                            if (evaluateDebug) cout << "\t\tIs a helpful action\n";
                        }
                    }

                    double & oldR = earliestDeadlineRelevancyEnd[asItr->first];
                    if (oldR > tilR) oldR = tilR;

                }
            }

            {


                if ((!currGAL.empty() || !currNGAL.empty()) && !alreadyPushed) {
                    relaxedPlan.push_front(pair<double, list<ActionSegment> >(currTS - EPSILON, list<ActionSegment>()));
                    alreadyPushed = true;
                }

                if (evaluateDebug && !currGAL.empty()) cout << "Finding achievers for goals at TS " << currTS << "\n";
                while (!currGAL.empty()) {
                    const map<int, double>::iterator nta = currGAL.begin();
                    const int nextToAchieve = nta->first;
                    double tilR = nta->second;
                    currGAL.erase(nta);
                    if (evaluateDebug) cout << "\tGoal " << nextToAchieve << "\n";
                    const pair<int, VAL::time_spec> currAchievedBy = (*achievedBy)[nextToAchieve];

                    assert(sensiblePair(payload, currAchievedBy, nextToAchieve));

                    if (currAchievedBy.second == VAL::E_AT_START) {
                        if (evaluateDebug) cout << "\t\tUsing start of " << currAchievedBy.first << "\n";
                        if (evaluateDebug && currTS == EPSILON) cout << "\t\tIs a helpful action\n";
                        relaxedPlan.front().second.push_back(ActionSegment(getOp(currAchievedBy.first), VAL::E_AT_START, -1, emptyIntList));
                        ++h;

                        const double sTIL = earliestTILForAction(currAchievedBy.first, true);
                        if (tilR > sTIL) tilR = sTIL;
//                  if (currTS == EPSILON) {
//                      helpfulActions.push_back(currAchievedBy);
                        //
//                  }

                        {
                            list<Literal*> & actionEffectsList = (*actionsToStartEffects)[currAchievedBy.first];
                            list<Literal*>::iterator aelItr = actionEffectsList.begin();
                            const list<Literal*>::iterator aelEnd = actionEffectsList.end();

                            for (; aelItr != aelEnd; ++aelItr) {
                                map<int, double>::iterator cgItr = currGAL.find((*aelItr)->getID());
                                if (cgItr != currGAL.end()) {
                                    if (tilR > cgItr->second) tilR = cgItr->second;
                                    currGAL.erase(cgItr);
                                }
                            }
                        }

                        {

                            map<int, double>::iterator ngalItr = currNGAL.begin();
                            const map<int, double>::iterator ngalEnd = currNGAL.end();

                            while (ngalItr != ngalEnd) {
                                ActionFluentModification* const afm = (*numericAchievedBy)[ngalItr->first];
                                if (afm && afm->act == currAchievedBy.first && afm->ts == VAL::E_AT_START) {
                                    if (tilR > ngalItr->second) tilR = ngalItr->second;
                                    map<int, double>::iterator tItr = ngalItr;
                                    ++tItr;
                                    currNGAL.erase(ngalItr);
                                    ngalItr = tItr;
                                } else {
                                    ++ngalItr;
                                }
                            }
                        }

                        bool isHelpful = true;

                        {
                            list<Literal*> & actionPreconditionlist = (*actionsToProcessedStartPreconditions)[currAchievedBy.first];
                            list<Literal*>::iterator aplItr = actionPreconditionlist.begin();
                            const list<Literal*>::iterator aplEnd = actionPreconditionlist.end();
                            if (evaluateDebug) cout << "\t\tPreconditions:\n";
                            for (; aplItr != aplEnd; ++aplItr) {

                                const int currPrec = (*aplItr)->getID();
                                const double acIn = (*achievedInLayer)[currPrec];

                                if (acIn > 0.0) {
                                    if ((*achievedBy)[currPrec].first != -1) {
                                        map<int, double>::iterator galItr = goalsAtLayer[acIn].propositionalGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                        if (galItr->second > tilR) galItr->second = tilR;
                                        if (evaluateDebug) cout << "\t\tAdding requirement for " << currPrec << " at time " << acIn << "\n";
                                        assert(acIn < currTS);
                                        isHelpful = false;
                                    } else {
                                        if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state at time " << acIn << "\n";
                                    }
                                } else {
                                    if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state\n";
                                }

                            }

                            if (evaluateDebug) cout << "\t\tPreconditions done\n";

                        }

                        if (!ignoreNumbers) {
                            list<int> & actionPreconditionlist = (*actionsToProcessedStartNumericPreconditions)[currAchievedBy.first];
                            list<int>::iterator aplItr = actionPreconditionlist.begin();
                            const list<int>::iterator aplEnd = actionPreconditionlist.end();

                            if (evaluateDebug) cout << "\t\tNumeric preconditions:\n";

                            for (; aplItr != aplEnd; ++aplItr) {

                                const int currPrec = (*aplItr);
                                const double acIn = (*numericAchievedInLayer)[currPrec];
                                if (acIn > 0.0) {
                                    if ((*numericAchievedBy)[currPrec]) {
                                        map<int, double>::iterator galItr = goalsAtLayer[acIn].numericGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                        if (galItr->second > tilR) galItr->second = tilR;
                                        if (evaluateDebug) cout << "\t\tAdding requirement for numeric precondition " << currPrec << " at time " << acIn << "\n";
                                        assert(acIn < currTS);
                                        isHelpful = false;
                                    } else {
                                        if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state at time " << acIn << "\n";
                                    }
                                } else {
                                    if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state\n";
                                }

                            }


                        }

                        if (evaluateDebug) cout << "\t\tPreconditions done.\n";

                        if (isHelpful) {
                            payload->helpfulActions.push_back(ActionSegment(RPGBuilder::getInstantiatedOp(currAchievedBy.first), VAL::E_AT_START, -1, emptyIntSet));
                            if (evaluateDebug) cout << "\t\tIs a helpful action\n";
                        }

                        double & oldR = earliestDeadlineRelevancyStart[currAchievedBy.first];
                        if (oldR > tilR) oldR = tilR;


                    } else if (currAchievedBy.second == VAL::E_AT_END) {
                        if (evaluateDebug) cout << "\t\tUsing end of " << currAchievedBy.first << "\n";
                        if (evaluateDebug && currTS == EPSILON) cout << "\t\tIs a helpful action\n";
                        relaxedPlan.front().second.push_back(ActionSegment(getOp(currAchievedBy.first), VAL::E_AT_END, -1, emptyIntList));
//                  if (currTS == EPSILON) {
//                      helpfulActions.push_back(currAchievedBy);
//                  }
                        if (!RPGBuilder::canSkipToEnd(currAchievedBy.first)) ++h;

                        const double sTIL = earliestTILForAction(currAchievedBy.first, false);
                        if (tilR > sTIL) tilR = sTIL;


                        {
                            list<Literal*> & actionEffectsList = (*actionsToEndEffects)[currAchievedBy.first];
                            list<Literal*>::iterator aelItr = actionEffectsList.begin();
                            const list<Literal*>::iterator aelEnd = actionEffectsList.end();

                            for (; aelItr != aelEnd; ++aelItr) {
                                map<int, double>::iterator cgItr = currGAL.find((*aelItr)->getID());
                                if (cgItr != currGAL.end()) {
                                    if (tilR > cgItr->second) tilR = cgItr->second;
                                    currGAL.erase(cgItr);
                                }
                            }
                        }

                        {

                            map<int, double>::iterator ngalItr = currNGAL.begin();
                            const map<int, double>::iterator ngalEnd = currNGAL.end();

                            while (ngalItr != ngalEnd) {
                                ActionFluentModification* const afm = (*numericAchievedBy)[ngalItr->first];
                                if (afm && afm->act == currAchievedBy.first && afm->ts == VAL::E_AT_END) {
                                    if (tilR > ngalItr->second) tilR = ngalItr->second;
                                    map<int, double>::iterator tItr = ngalItr;
                                    ++tItr;
                                    currNGAL.erase(ngalItr);
                                    ngalItr = tItr;
                                } else {
                                    ++ngalItr;
                                }
                            }
                        }

                        bool isHelpful = true;
                        {
                            list<Literal*> & actionPreconditionlist = (*actionsToEndPreconditions)[currAchievedBy.first];
                            list<Literal*>::iterator aplItr = actionPreconditionlist.begin();
                            const list<Literal*>::iterator aplEnd = actionPreconditionlist.end();

                            if (evaluateDebug) cout << "\t\tPreconditions:\n";

                            for (; aplItr != aplEnd; ++aplItr) {

                                const int currPrec = (*aplItr)->getID();
                                const double acIn = (*achievedInLayer)[currPrec];
                                if (acIn > 0.0) {
                                    if ((*achievedBy)[currPrec].first != -1) {
                                        map<int, double>::iterator galItr = goalsAtLayer[acIn].propositionalGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                        if (galItr->second > tilR) galItr->second = tilR;
                                        if (evaluateDebug) cout << "\t\tAdding requirement for " << currPrec << " at time " << acIn << "\n";
                                        assert(acIn < currTS);
                                        isHelpful = false;
                                    } else {
                                        if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state at layer " << acIn << "\n";
                                    }
                                } else {
                                    if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state\n";
                                }

                            }

                            if (evaluateDebug) cout << "\t\tPreconditions done\n";

                            if (theState.startedActions.find(currAchievedBy.first) == theState.startedActions.end()) {
                                const double sas = startActionSchedule[currAchievedBy.first];
                                if (sas == -1.0) {
                                    assert(RPGBuilder::canSkipToEnd(currAchievedBy.first));
                                    if (!RPGBuilder::canSkipToEnd(currAchievedBy.first)) {
                                        cerr << "Critical error - trying to schedule goal to before start of plan\n";
                                        exit(1);
                                    }
                                } else {
                                    map<int, double>::iterator galItr = goalsAtLayer[sas + EPSILON].actionStarts.insert(pair<int, double>(currAchievedBy.first, tilR)).first;
                                    if (galItr->second > tilR) galItr->second = tilR;
                                    if (evaluateDebug) cout << "\t\tAdding requirement for start of action at time " << (startActionSchedule[currAchievedBy.first] + EPSILON) << "\n";
                                }

                                isHelpful = false;
                            } else {
                                if (evaluateDebug) cout << "\t\tAction has already been started, do not need to schedule start in RPG\n";
                            }
                        }

                        if (!ignoreNumbers) {
                            list<int> & actionPreconditionlist = (*actionsToNumericEndPreconditions)[currAchievedBy.first];
                            list<int>::iterator aplItr = actionPreconditionlist.begin();
                            const list<int>::iterator aplEnd = actionPreconditionlist.end();

                            if (evaluateDebug) cout << "\t\tNumeric preconditions:\n";

                            for (; aplItr != aplEnd; ++aplItr) {

                                const int currPrec = (*aplItr);
                                const double acIn = (*numericAchievedInLayer)[currPrec];
                                if (acIn > 0.0) {
                                    if ((*numericAchievedBy)[currPrec]) {
                                        map<int, double>::iterator galItr = goalsAtLayer[acIn].numericGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                        if (galItr->second > tilR) galItr->second = tilR;
                                        if (evaluateDebug) cout << "\t\tAdding requirement for numeric precondition " << currPrec << " at time " << acIn << "\n";
                                        assert(acIn < currTS);
                                    } else {
                                        if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state at time " << acIn << "\n";
                                    }
                                } else {
                                    if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state\n";
                                }

                            }


                        }


                        if (evaluateDebug) cout << "\t\tPreconditions done.\n";

                        if (isHelpful) {
                            payload->helpfulActions.push_back(ActionSegment(RPGBuilder::getInstantiatedOp(currAchievedBy.first), VAL::E_AT_END, -1, emptyIntSet));
                            if (evaluateDebug) cout << "\t\tIs a helpful action\n";
                        }


                        double & oldR = earliestDeadlineRelevancyEnd[currAchievedBy.first];
                        if (oldR > tilR) oldR = tilR;

                    } else { // aha, is a timed initial literal!

//                  if (currTS == EPSILON) {
//                      helpfulActions.push_back(currAchievedBy);
//                  }
                        earliestTIL = currAchievedBy;

                        {
                            list<int> & tilEffectsList = tilEffects[currAchievedBy.first];
                            list<int>::iterator aelItr = tilEffectsList.begin();
                            const list<int>::iterator aelEnd = tilEffectsList.end();

                            for (; aelItr != aelEnd; ++aelItr) {
                                currGAL.erase(*aelItr);
                            }
                        }

                    }

                }

                if (evaluateDebug && !currNGAL.empty()) cout << "Finding achievers for numeric goals at TS " << currTS << "\n";
                while (!currNGAL.empty()) {
                    const map<int, double>::iterator nta = currNGAL.begin();
                    const int nextToAchieve = nta->first;
                    double tilR = nta->second;
                    currNGAL.erase(nta);
                    if (evaluateDebug) cout << "\tFluent goal " << nextToAchieve << "\n";
                    ActionFluentModification * const currAchievedBy = (*numericAchievedBy)[nextToAchieve];

                    assert(currAchievedBy);


                    if (currAchievedBy->ts == VAL::E_AT_START) {
                        if (evaluateDebug) cout << "\t\tUsing start of " << currAchievedBy->act << "\n";
                        if (evaluateDebug && currTS == EPSILON) cout << "\t\tIs a helpful action\n";
                        relaxedPlan.front().second.push_back(ActionSegment(getOp(currAchievedBy->act), VAL::E_AT_START, -1, emptyIntList));
                        ++h;

//                  if (currTS == EPSILON) {
//                      helpfulActions.push_back(pair<int, VAL::time_spec>(currAchievedBy->act, VAL::E_AT_START));
                        //
//                  }

                        const double sTIL = earliestTILForAction(currAchievedBy->ts, true);
                        if (tilR > sTIL) tilR = sTIL;

                        {

                            map<int, double>::iterator ngalItr = currNGAL.begin();
                            const map<int, double>::iterator ngalEnd = currNGAL.end();

                            while (ngalItr != ngalEnd) {
                                ActionFluentModification* const afm = (*numericAchievedBy)[ngalItr->first];
                                if (afm && afm->act == currAchievedBy->act && afm->ts == VAL::E_AT_START) {
                                    if (tilR > ngalItr->second) tilR = ngalItr->second;
                                    map<int, double>::iterator tItr = ngalItr;
                                    ++tItr;
                                    currNGAL.erase(ngalItr);
                                    ngalItr = tItr;
                                } else {
                                    ++ngalItr;
                                }
                            }
                        }


                        bool isHelpful = true;

                        {
                            list<Literal*> & actionPreconditionlist = (*actionsToProcessedStartPreconditions)[currAchievedBy->act];
                            list<Literal*>::iterator aplItr = actionPreconditionlist.begin();
                            const list<Literal*>::iterator aplEnd = actionPreconditionlist.end();
                            if (evaluateDebug) cout << "\t\tPreconditions:\n";
                            for (; aplItr != aplEnd; ++aplItr) {

                                const int currPrec = (*aplItr)->getID();
                                const double acIn = (*achievedInLayer)[currPrec];

                                if (acIn > 0.0) {
                                    if ((*achievedBy)[currPrec].first != -1) {
                                        map<int, double>::iterator galItr = goalsAtLayer[acIn].propositionalGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                        if (galItr->second > tilR) galItr->second = tilR;
                                        if (evaluateDebug) cout << "\t\tAdding requirement for " << currPrec << " at time " << acIn << "\n";
                                        assert(acIn < currTS);
                                        isHelpful = false;
                                    } else {
                                        if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state at time " << acIn << "\n";
                                    }
                                } else {
                                    if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state\n";
                                }

                            }

                            if (evaluateDebug) cout << "\t\tPreconditions done\n";

                        }

                        if (!ignoreNumbers) {
                            list<int> & actionPreconditionlist = (*actionsToProcessedStartNumericPreconditions)[currAchievedBy->act];
                            list<int>::iterator aplItr = actionPreconditionlist.begin();
                            const list<int>::iterator aplEnd = actionPreconditionlist.end();

                            if (evaluateDebug) cout << "\t\tNumeric preconditions:\n";

                            for (; aplItr != aplEnd; ++aplItr) {

                                const int currPrec = (*aplItr);
                                const double acIn = (*numericAchievedInLayer)[currPrec];
                                if (acIn > 0.0) {
                                    if ((*numericAchievedBy)[currPrec]) {
                                        map<int, double>::iterator galItr = goalsAtLayer[acIn].numericGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                        if (galItr->second > tilR) galItr->second = tilR;
                                        if (evaluateDebug) cout << "\t\tAdding requirement for numeric precondition " << currPrec << " at time " << acIn << ", to be achieved by " << (*numericAchievedBy)[currPrec]->act << "\n";
                                        assert(acIn < currTS);
                                        isHelpful = false;
                                    } else {
                                        if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state at time " << acIn << "\n";
                                    }
                                } else {
                                    if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state\n";
                                }

                            }

                            if (evaluateDebug) cout << "\t\tPreconditions done.\n";
                        }

                        if (isHelpful) {
                            payload->helpfulActions.push_back(ActionSegment(RPGBuilder::getInstantiatedOp(currAchievedBy->act), VAL::E_AT_START, -1, emptyIntSet));
                        }

                        double & oldR = earliestDeadlineRelevancyStart[currAchievedBy->act];
                        if (oldR > tilR) oldR = tilR;

                    } else {
                        if (evaluateDebug) cout << "\t\tUsing end of " << currAchievedBy->act << "\n";
                        if (evaluateDebug && currTS == EPSILON) cout << "\t\tIs a helpful action\n";
                        relaxedPlan.front().second.push_back(ActionSegment(getOp(currAchievedBy->act), VAL::E_AT_END, -1, emptyIntList));
//                  if (currTS == EPSILON) {
//                      helpfulActions.push_back(pair<int, VAL::time_spec>(currAchievedBy->act, VAL::E_AT_END));
//                  }
                        ++h;

                        const double sTIL = earliestTILForAction(currAchievedBy->act, false);
                        if (tilR > sTIL) tilR = sTIL;


                        {

                            map<int, double>::iterator ngalItr = currNGAL.begin();
                            const map<int, double>::iterator ngalEnd = currNGAL.end();

                            while (ngalItr != ngalEnd) {
                                ActionFluentModification* const afm = (*numericAchievedBy)[ngalItr->first];
                                if (afm && afm->act == currAchievedBy->act && afm->ts == VAL::E_AT_END) {
                                    if (ngalItr->second > tilR) tilR = ngalItr->second;
                                    map<int, double>::iterator tItr = ngalItr;
                                    ++tItr;
                                    currNGAL.erase(ngalItr);
                                    ngalItr = tItr;
                                } else {
                                    ++ngalItr;
                                }
                            }
                        }

                        bool isHelpful = true;

                        {
                            list<Literal*> & actionPreconditionlist = (*actionsToEndPreconditions)[currAchievedBy->act];
                            list<Literal*>::iterator aplItr = actionPreconditionlist.begin();
                            const list<Literal*>::iterator aplEnd = actionPreconditionlist.end();

                            if (evaluateDebug) cout << "\t\tPreconditions:\n";

                            for (; aplItr != aplEnd; ++aplItr) {

                                const int currPrec = (*aplItr)->getID();
                                const double acIn = (*achievedInLayer)[currPrec];
                                if (acIn > 0.0) {
                                    if ((*achievedBy)[currPrec].first != -1) {
                                        map<int, double>::iterator galItr = goalsAtLayer[acIn].propositionalGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                        if (galItr->second > tilR) galItr->second = tilR;

                                        if (evaluateDebug) cout << "\t\tAdding requirement for " << currPrec << " at time " << acIn << "\n";
                                        assert(acIn < currTS);
                                        isHelpful = false;
                                    } else {
                                        if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state at time " << acIn << "\n";
                                    }
                                } else {
                                    if (evaluateDebug) cout << "\t\tPrecondition " << currPrec << " in initial state\n";
                                }

                            }

                            if (evaluateDebug) cout << "\t\tPreconditions done\n";

                            if (!currAchievedBy->openEnd) {
                                map<int, double>::iterator galItr = goalsAtLayer[startActionSchedule[currAchievedBy->act] + EPSILON].actionStarts.insert(pair<int, double>(currAchievedBy->act, tilR)).first;
                                if (galItr->second > tilR) galItr->second = tilR;
                                if (evaluateDebug) cout << "\t\tAdding requirement for start of action at time " << (startActionSchedule[currAchievedBy->act] + EPSILON) << "\n";
                                isHelpful = false;
                            } else {
                                if (evaluateDebug) cout << "\t\tAction has already been started, do not need to schedule start in RPG\n";
                            }
                        }

                        if (!ignoreNumbers) {
                            list<int> & actionPreconditionlist = (*actionsToNumericEndPreconditions)[currAchievedBy->act];
                            list<int>::iterator aplItr = actionPreconditionlist.begin();
                            const list<int>::iterator aplEnd = actionPreconditionlist.end();

                            if (evaluateDebug) cout << "\t\tNumeric preconditions:\n";

                            for (; aplItr != aplEnd; ++aplItr) {

                                const int currPrec = (*aplItr);
                                const double acIn = (*numericAchievedInLayer)[currPrec];
                                if (acIn > 0.0) {
                                    if ((*numericAchievedBy)[currPrec]) {
                                        map<int, double>::iterator galItr = goalsAtLayer[acIn].numericGoals.insert(pair<int, double>(currPrec, tilR)).first;
                                        if (galItr->second > tilR) galItr->second = tilR;
                                        if (evaluateDebug) cout << "\t\tAdding requirement for numeric precondition " << currPrec << " at time " << acIn << "\n";
                                        isHelpful = false;
                                    } else {
                                        if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state at time " << acIn << "\n";
                                    }
                                } else {
                                    if (evaluateDebug) cout << "\t\tNumeric precondition " << currPrec << " satisfied in initial state\n";
                                }

                            }


                        }


                        if (evaluateDebug) cout << "\t\tPreconditions done.\n";

                        if (isHelpful) {
                            payload->helpfulActions.push_back(ActionSegment(RPGBuilder::getInstantiatedOp(currAchievedBy->act), VAL::E_AT_END, -1, emptyIntSet));
                        }


                        double & oldR = earliestDeadlineRelevancyEnd[currAchievedBy->act];
                        if (oldR > tilR) oldR = tilR;

                    }

                }



                if (evaluateDebug) cout << "All goals at this TS now satisfied\n";
            }
            goalsAtLayer.erase(currTS);
        }
    }

    bool applyPropositionalEffects(Private::BuildingPayload * const payload, const int & currAct, double & actCost, const VAL::time_spec & currTS, const double & nlTime, MaxDependentInfo & POtime);

    bool checkPreconditionsAreSatisfied(const int & currAct, const VAL::time_spec & ts, const double & layer);

    bool updateActionsForNewLiteralFact(BuildingPayload * const payload, const int & toPropagate, const double & factLayerTime);
    bool updateActionsForNewNumericFact(BuildingPayload * const payload, const int & toPropagate, const double & factLayerTime);
    bool applyEndEffectNow(BuildingPayload * const payload, const int & currAct, const bool & openAct, const double & factLayerTime);

};

#ifdef MDIDEBUG
RPGHeuristic::Private::BuildingPayload * RPGHeuristic::Private::MaxDependentInfo::referTo = 0;
bool RPGHeuristic::Private::MaxDependentInfo::debug = false;
#endif

vector<double> RPGHeuristic::Private::earliestStartAllowed;
vector<double> RPGHeuristic::Private::earliestEndAllowed;
vector<double> RPGHeuristic::Private::latestStartAllowed;
vector<double> RPGHeuristic::Private::latestEndAllowed;
vector<double> RPGHeuristic::Private::deadlineAtTime;
vector<double> RPGHeuristic::Private::earliestDeadlineRelevancyStart;
vector<double> RPGHeuristic::Private::earliestDeadlineRelevancyEnd;

vector<list<int> > RPGHeuristic::Private::tilEffects;
vector<list<int> > RPGHeuristic::Private::tilNegativeEffects;
vector<list<int> > RPGHeuristic::Private::tilTemporaryNegativeEffects;
vector<double> RPGHeuristic::Private::tilTimes;
bool RPGHeuristic::Private::tilInitialised = false;
int RPGHeuristic::Private::tilCount = 0;

vector<double> RPGHeuristic::Private::earliestPropositionPOTimes;
vector<double> RPGHeuristic::Private::earliestNumericPOTimes;
//vector<double> RPGHeuristic::Private::earliestNumericPrePOTimes;

vector<vector<set<int> > > RPGHeuristic::Private::actionsAffectedByFluent;

const double RPGHeuristic::Private::EPSILON = 0.001;

vector<double> & RPGHeuristic::getEarliestForStarts()
{
    return Private::earliestStartAllowed;
};
vector<double> & RPGHeuristic::getEarliestForEnds()
{
    return Private::earliestEndAllowed;
};

double & RPGHeuristic::getDeadlineRelevancyStart(const int & i)
{
    return Private::earliestDeadlineRelevancyStart[i];
}

double & RPGHeuristic::getDeadlineRelevancyEnd(const int & i)
{
    return Private::earliestDeadlineRelevancyEnd[i];
}


void RPGHeuristic::doFullExpansion(MinimalState & refState)
{
    set<int> dummyGoals;
    set<int> dummyGoalFluents;
    list<ActionSegment> dummyHelpful;
    list<pair<double, list<ActionSegment> > > dummyRP;
    vector<double> minTimestamps(1, 0.0);
    double dummyEstimate;
    d->buildEmptyActionFluentLookupTable();
    
    const bool wasBlind = blindSearch;
    const bool wasNoNumbers = ignoreNumbers;
    
    d->expandFully = true;
    blindSearch = false;
    ignoreNumbers = false;
    
    getRelaxedPlan(refState, minTimestamps, 0.0, dummyHelpful, dummyRP, dummyEstimate);
    
    d->expandFully = false;    
    blindSearch = wasBlind;
    ignoreNumbers = wasNoNumbers;
}

RPGHeuristic::RPGHeuristic(const bool & b,
                           vector<list<Literal*> > * atse,
                           vector<list<Literal*> > * atee,
                           vector<list<pair<int, VAL::time_spec> > > * eta,
                           vector<list<Literal*> > * atsne,
                           vector<list<Literal*> > * atene,
                           vector<list<pair<int, VAL::time_spec> > > * neta,
                           vector<list<pair<int, VAL::time_spec> > > * pta,
                           vector<list<Literal*> > * atsp,
                           vector<list<Literal*> > * ati,
                           vector<list<Literal*> > * atep,
                           vector<list<RPGBuilder::NumericEffect> > * atnuse,
                           vector<list<RPGBuilder::NumericEffect> > * atnuee,
                           vector<list<int> > * atrnuse,
                           vector<list<int> > * atrnuee,
                           vector<list<int> > * atnusp,
                           vector<list<int> > * atnui,
                           vector<list<int> > * atnuep,
                           vector<list<int> > * atpnuep,
                           vector<int> * iusp,
                           vector<int> * iuip,
                           vector<int> * iuep,
                           vector<double> * ail,
                           vector<double> * ailr,
                           vector<pair<int, VAL::time_spec> > * ab,
                           vector<pair<int, VAL::time_spec> > * abr,
                           vector<double> * nail,
                           vector<double> * nailr,
                           vector<ActionFluentModification*> * nab,
                           vector<ActionFluentModification*> * nabr,
                           vector<int> * iunsp,
                           vector<int> * iuni,
                           vector<int> * iunep,
                           vector<RPGBuilder::RPGNumericPrecondition> * rnp,
                           vector<RPGBuilder::RPGNumericEffect> * rne,
                           vector<list<pair<int, VAL::time_spec> > > * ppta,
                           vector<list<pair<int, VAL::time_spec> > > * nppta,
                           vector<list<Literal*> > * atpsp,
                           vector<int> * iupsp,
                           vector<int> * iupsnp,
                           list<pair<int, VAL::time_spec> > * pla,
                           list<pair<int, VAL::time_spec> > * onpa)
        :   d(new Private(b,
                          atse,
                          atee,
                          eta,
                          atsne,
                          atene,
                          neta,
                          pta,
                          atsp,
                          ati,
                          atep,
                          atnuse,
                          atnuee,
                          atrnuse,
                          atrnuee,
                          atnusp,
                          atnui,
                          atnuep,
                          atpnuep,
                          iusp,
                          iuip,
                          iuep,
                          ail,
                          ailr,
                          ab,
                          abr,
                          nail,
                          nailr,
                          nab,
                          nabr,
                          iunsp,
                          iuni,
                          iunep,
                          rnp,
                          rne,
                          ppta,
                          nppta,
                          atpsp,
                          iupsp,
                          iupsnp,
                          pla,
                          onpa))
{

    {
        list<Literal*>::iterator gsItr = RPGBuilder::getLiteralGoals().begin();
        const list<Literal*>::iterator gsEnd = RPGBuilder::getLiteralGoals().end();

        for (; gsItr != gsEnd; ++gsItr) {
            if (!RPGBuilder::isStatic(*gsItr).first) {
                d->goals.insert((*gsItr)->getID());
            }
        }

        d->gsEnd = d->goals.end();
    }

    {
        list<pair<int, int> >::iterator gsItr = RPGBuilder::getNumericRPGGoals().begin();
        const list<pair<int, int> >::iterator gsEnd = RPGBuilder::getNumericRPGGoals().end();

        for (; gsItr != gsEnd; ++gsItr) {
            if (gsItr->first != -1) {
                d->goalFluents.insert(gsItr->first);
            }
            if (gsItr->second != -1) {
                d->goalFluents.insert(gsItr->second);
            }
        }

        d->gfEnd = d->goalFluents.end();
    }

};

RPGHeuristic::~RPGHeuristic()
{
    if (d->deleteArrays) {
        assert(false);
    }
}



RPGHeuristic* RPGBuilder::generateRPGHeuristic()
{

    // for now, don't prune the RPG

    return new RPGHeuristic(false,  // subproblem does not own the arrays
                            &actionsToStartEffects,
                            &actionsToEndEffects,
                            &effectsToActions,
                            &actionsToStartNegativeEffects,
                            &actionsToEndNegativeEffects,
                            &negativeEffectsToActions,
                            &preconditionsToActions,
                            &actionsToStartPreconditions,
                            &actionsToInvariants,
                            &actionsToEndPreconditions,
                            &actionsToStartNumericEffects,
                            &actionsToEndNumericEffects,
                            &actionsToRPGNumericStartEffects,
                            &actionsToRPGNumericEndEffects,
                            &actionsToRPGNumericStartPreconditions,
                            &actionsToRPGNumericInvariants,
                            &actionsToRPGNumericEndPreconditions,
                            &actionsToProcessedStartRPGNumericPreconditions,
                            &initialUnsatisfiedStartPreconditions,
                            &initialUnsatisfiedInvariants,
                            &initialUnsatisfiedEndPreconditions,
                            &achievedInLayer,
                            &achievedInLayerReset,
                            &achievedBy,
                            &achievedByReset,
                            &numericAchievedInLayer,
                            &numericAchievedInLayerReset,
                            &numericAchievedBy,
                            &numericAchievedByReset,
                            &initialUnsatisfiedNumericStartPreconditions,
                            &initialUnsatisfiedNumericInvariants,
                            &initialUnsatisfiedNumericEndPreconditions,
                            &rpgNumericPreconditions,
                            &rpgNumericEffects,
                            &processedPreconditionsToActions,
                            &processedRPGNumericPreconditionsToActions,
                            &actionsToProcessedStartPreconditions,
                            &initialUnsatisfiedProcessedStartPreconditions,
                            &initialUnsatisfiedProcessedStartNumericPreconditions,
                            &preconditionlessActions,
                            &onlyNumericPreconditionActions);

};

bool RPGHeuristic::Private::EndPrecRescale::operator <(const RPGHeuristic::Private::EndPrecRescale & r) const
{

    if (var < r.var) return true;
    if (r.var > var) return false;

    if (offset < r.offset) return true;
    if (offset > r.offset) return false;

    if (totalchange < r.totalchange) return true;
    if (totalchange > r.totalchange) return false;

    if (duration < r.duration) return true;
    if (duration > r.duration) return false;

    return false;


};


set<int> RPGHeuristic::emptyIntList;



int RPGHeuristic::getRelaxedPlan(const MinimalState & theState, const vector<double> & minTimestamps, const double & stateTS, list<ActionSegment> & helpfulActions, list<pair<double, list<ActionSegment> > > & relaxedPlan, double & finalPlanMakespanEstimate, map<double, list<pair<int, int> > > * justApplied, double tilFrom)
{

//    const int vCount = theState.secondMin.size();
//    const int avCount = RPGBuilder::getAVCount();

    const int nextTIL = theState.nextTIL;

    finalPlanMakespanEstimate = 0.0;

    d->integrateContinuousEffects();

//  startedActionExtraEndPreconditions.clear();

    static const double EPSILON = 0.001;
    bool evaluateDebug = Globals::globalVerbosity & 64;

    d->setDebugFlag(evaluateDebug);

    d->populateActionFluentLookupTable();

    if (evaluateDebug) cout << "Evaluating a state with " << theState.startedActions.size() << " sorts of actions on the go\n";

    //rpprintState(theState);

    auto_ptr<Private::BuildingPayload> payload(d->spawnNewPayload(theState, minTimestamps, helpfulActions));

    d->giveUsTheEffectsOfExecutingActions(payload.get());

    /*if (evaluateDebug || eeDebug) {
    const int eeuSize = extraEndUnsatisfied.size();
    for (int i = 0; i < eeuSize; ++i) {
    if (extraEndUnsatisfied[i]) cout << "Still extra ends attached to " << *(RPGBuilder::getInstantiatedOp(i)) << "\n";
    }
    }*/
//  MILPRPG(theState.second);

    int heuristicOffset = 0;

    d->addRequirementToHaveSeenTheEndOfAllCurrentlyExecutingActions(payload.get());

    if (evaluateDebug) {
        cout << "Aiming for number of goals satisfied: " << payload->unsatisfiedGoals << "\n";
    }

    d->seeWhatGoalsAreTrueToStartWith(payload.get());

    if (evaluateDebug) {
        cout << "Goals unsatisfied in initial state: " << payload->unsatisfiedGoals << "\n";
        cout << "Ends not appeared so far: " << payload->unappearedEnds << "\n";
    }

    if (d->expandFully) payload->unsatisfiedGoals = INT_MAX;

    if (!payload->unsatisfiedGoals && !payload->unappearedEnds) return heuristicOffset;


    if (blindSearch) return 1;
    
    d->resetAchievedBy();
    d->recordFactLayerZero(payload.get());
    d->performTILInitialisation();

    d->initialiseLatestArrays();

    d->addTemporalConstraintsFromActiveActions(payload.get());

    if (!d->addTemporalConstraintsFromActiveActions(payload.get(), justApplied, stateTS, nextTIL, tilFrom)) {

        // will return false in cases where there's a direct contradiction between a TIL
        // and an executing action's invariants

        return -1;
    }
    d->addTILsBeforeExpansion(nextTIL, payload->factLayers, stateTS, tilFrom);
    d->addBoundsFromTemporalAnalysis(stateTS);


    d->noLongerForbidden.clear();


    {
        if (evaluateDebug) cout << "Considering preconditionless actions\n";
        d->updateActionsForNewLiteralFact(payload.get(), -1, 0.0);
    }






    if (payload->unsatisfiedGoals || payload->unappearedEnds) {

        if (!RPGBuilder::modifiedRPG && RPGBuilder::sortedExpansion) {

            map<double, list<int> > expansionOrder;
            map<int, PropositionAnnotation>::const_iterator stateItr = theState.first.begin();
            const map<int, PropositionAnnotation>::const_iterator stateEnd = theState.first.end();

            for (; stateItr != stateEnd; ++stateItr) {
                const int factID = stateItr->first;
                const double poTS = d->earliestPropositionPOTimes[factID];
                expansionOrder[poTS].push_back(stateItr->first);
            }

            map<double, list<int> >::const_iterator eoItr = expansionOrder.begin();
            const map<double, list<int> >::const_iterator eoEnd = expansionOrder.end();

            for (; eoItr != eoEnd; ++eoItr) {
                list<int>::const_iterator fItr = eoItr->second.begin();
                const list<int>::const_iterator fEnd = eoItr->second.end();

                for (; fItr != fEnd; ++fItr) {
                    if (d->updateActionsForNewLiteralFact(payload.get(), *fItr, 0.0)) break;
                }
                if (fItr != fEnd) break;
            }


        } else {
            if (evaluateDebug) cout << "Considering " << theState.first.size() << " initial propositional facts\n";
            map<int, PropositionAnnotation>::const_iterator stateItr = theState.first.begin();
            const map<int, PropositionAnnotation>::const_iterator stateEnd = theState.first.end();

            for (; stateItr != stateEnd; ++stateItr) {
                const int factID = stateItr->first;
                //const double poTS = (RPGBuilder::modifiedRPG ? 0.0 : d->earliestPropositionPOTimes[factID]);
                const double layer = (*(d->achievedInLayer))[factID];
                if (layer == 0.0) {
                    if (evaluateDebug) cout << "Updating from fact " << factID << " " << *(RPGBuilder::getLiteral(factID)) << "\n";
                    if (d->updateActionsForNewLiteralFact(payload.get(), factID, 0.0)) break;
                } else {
                    if (evaluateDebug) cout << "Modified RPG: Not updating from fact " << factID << " until " << layer << "\n";
                    payload->factLayers[layer].first.push_back(factID);
                }
            }
        }

    }

    if (payload->unsatisfiedGoals || payload->unappearedEnds) {
        if (evaluateDebug) cout << "Considering initial numeric facts\n";
        const int loopLim = d->rpgNumericPreconditions->size();

        if (loopLim) {
            vector<double> * const maxFluentTable = &(payload->fluentLayers.begin()->second);
            for (int i = 0; i < loopLim; ++i) {
                RPGBuilder::RPGNumericPrecondition & currPre = (*(d->rpgNumericPreconditions))[i];
                if (currPre.isSatisfied(*maxFluentTable)) {
                    const double layer = (*(d->numericAchievedInLayer))[i];
#ifndef NDEBUG
                    ActionFluentModification * const ac = (*(d->numericAchievedBy))[i];
#endif
                    if (layer == 0.0) {
                        if (evaluateDebug) {
                            cout << "Updating from numeric fact " << i << ":  " << (*(d->rpgNumericPreconditions))[i];
                            if ((*(d->numericAchievedBy))[i]) {
                                cout << " achieved by action " << (*(d->numericAchievedBy))[i]->act << "\n";
                            } else {
                                cout << "; was achieved in the initial state\n";
                            }
                        }
                        if (d->updateActionsForNewNumericFact(payload.get(), i, 0.0)) {
                            assert(ac == (*(d->numericAchievedBy))[i]);
                            break;
                        } else {
                            assert(ac == (*(d->numericAchievedBy))[i]);
                        }
                    } else {
                        if (evaluateDebug) cout << "Modified RPG: Not updating from numeric fact " << i << ":  " << (*(d->rpgNumericPreconditions))[i] << " until " << layer << "\n";
                        payload->factLayers[layer].second.push_back(i);
                    }
                }
            }
        }
    }

    if (evaluateDebug) {
        cout << "Unsatisfied goals: " << payload->unsatisfiedGoals << ", Unappeared ends: " << payload->unappearedEnds << "\n";
    }




    while ((payload->unsatisfiedGoals || payload->unappearedEnds) && (!payload->factLayers.empty() || !payload->endActionsAtTime.empty())) {

        if (evaluateDebug) {
            cout << "Unsatisfied goals: " << payload->unsatisfiedGoals << ", Unappeared ends: " << payload->unappearedEnds << "\n";
            cout << "Expanding RPG forwards\n";
        }

        if (!payload->endActionsAtTime.empty() && (payload->factLayers.empty() || payload->endActionsAtTime.begin()->first <= payload->factLayers.begin()->first)) {

            if (evaluateDebug) cout << "ACTION LAYER AT TIME " << payload->endActionsAtTime.begin()->first << "\n";

            {
                if (payload->fluentLayers.find(payload->endActionsAtTime.begin()->first + EPSILON) == payload->fluentLayers.end()) {
                    const vector<double> & backFluents = payload->fluentLayers.rbegin()->second;
                    payload->fluentLayers.insert(pair<double, vector<double> >(payload->endActionsAtTime.begin()->first + EPSILON, backFluents));

                }
            }

            const map<double, list<int>, EpsilonComp >::iterator eaatItr = payload->endActionsAtTime.begin();

            list<int>::iterator eaItr = eaatItr->second.begin();
            const list<int>::iterator eaEnd = eaatItr->second.end();

            const double cTime = eaatItr->first;

            for (; eaItr != eaEnd; ++eaItr) {
                int actToPass = *eaItr;
                bool actIsOpen = false;
                if (actToPass < 0) {
                    actToPass = -actToPass - 1;
                    actIsOpen = true;
                }
                if (cTime > Private::latestEndAllowed[actToPass]) {
                    if (evaluateDebug) {
                        cout << "End of action has been cancelled: invariant or one-way end precondition deleted by TIL\n";
                    }
                } else {
                    if (evaluateDebug) {
                        cout << "Applying ";
                        if (actIsOpen) cout << "open ";
                        cout << "end effect of " << actToPass << " " << *(RPGBuilder::getInstantiatedOp(actToPass)) << "\n";
                    }
                    if (d->applyEndEffectNow(payload.get(), actToPass, actIsOpen, cTime)) break;
                }
            }
            if (evaluateDebug) cout << "Finished action layer\n";
            payload->endActionsAtTime.erase(eaatItr);

        } else {

            const map<double, Private::FactLayerEntry, EpsilonComp >::iterator currFactLayerItr = payload->factLayers.begin();
            if (evaluateDebug) cout << "FACT LAYER AT TIME " << currFactLayerItr->first << "\n";

            const double cTime = currFactLayerItr->first;

            {
                if (payload->fluentLayers.find(cTime) == payload->fluentLayers.end()) {
                    assert(!payload->fluentLayers.empty());
                    const vector<double> & backFluents = payload->fluentLayers.rbegin()->second;
                    payload->fluentLayers.insert(pair<double, vector<double> >(cTime, backFluents));

                }
            }


            {
                if (payload->fluentLayers.find(cTime + EPSILON) == payload->fluentLayers.end()) {
                    assert(!payload->fluentLayers.empty());
                    const vector<double> & backFluents = payload->fluentLayers.rbegin()->second;
                    payload->fluentLayers.insert(pair<double, vector<double> >(cTime + EPSILON, backFluents));

                }
            }




            {
                list<int>::iterator stateItr = currFactLayerItr->second.first.begin();
                const list<int>::iterator stateEnd = currFactLayerItr->second.first.end();

                for (; stateItr != stateEnd; ++stateItr) {
                    if (evaluateDebug) cout << "Updating from fact " << *stateItr << "\n";
                    if (d->updateActionsForNewLiteralFact(payload.get(), *stateItr, cTime)) break;
                }

            }

            {
                list<int>::iterator stateItr = currFactLayerItr->second.second.begin();
                const list<int>::iterator stateEnd = currFactLayerItr->second.second.end();

                for (; stateItr != stateEnd; ++stateItr) {

#ifndef NDEBUG
                    ActionFluentModification * const ac = (*(d->numericAchievedBy))[*stateItr];
#endif

                    if (evaluateDebug) {
                        cout << "Updating from numeric fact " << *stateItr << ":  " << (*(d->rpgNumericPreconditions))[*stateItr];
                        if ((*(d->numericAchievedBy))[*stateItr]) {
                            cout << "; achieved by " << (*(d->numericAchievedBy))[*stateItr]->act << "\n";
                        } else {
                            cout << "; was achieved in the initial state\n";
                        }
                    }
                    if (d->updateActionsForNewNumericFact(payload.get(), *stateItr, cTime)) {
                        assert(ac == (*(d->numericAchievedBy))[*stateItr]);
                        break;
                    } else {
                        assert(ac == (*(d->numericAchievedBy))[*stateItr]);
                    }
                }

            }


            {
                list<pair<int, int> >::iterator stateItr = currFactLayerItr->second.TILs.begin();
                const list<pair<int, int> >::iterator stateEnd = currFactLayerItr->second.TILs.end();

                for (; stateItr != stateEnd; ++stateItr) {
                    if (evaluateDebug) cout << "Updating from TIL " << stateItr->first << ", fact " << stateItr->second << "\n";

                    double & currAIL = (*(d->achievedInLayer))[stateItr->second];
                    if (currAIL == -1.0) { // this is new
                        currAIL = cTime;
                        (*(d->achievedBy))[stateItr->second] = pair<int, VAL::time_spec>(stateItr->first, VAL::E_AT);
                        if (d->updateActionsForNewLiteralFact(payload.get(), stateItr->second, cTime)) break;
                    }


                }
            }

            if (currFactLayerItr->second.endOfJustApplied) {
                d->noLongerForbidden.clear();
                set<int> & startSet = currFactLayerItr->second.endOfJustApplied->first;
                set<int> & endSet = currFactLayerItr->second.endOfJustApplied->second;

                bool spawn = false;

                {
                    set<int>::iterator fItr = startSet.begin();
                    const set<int>::iterator fEnd = startSet.end();

                    for (; fItr != fEnd; ++fItr) {
                        map<int, int>::iterator nrItr = payload->forbiddenStart.find(*fItr);
                        if (!(--(nrItr->second))) {
                            payload->forbiddenStart.erase(nrItr);
                            if (!payload->startPreconditionCounts[*fItr]) {
                                d->noLongerForbidden.push_back(pair<int, VAL::time_spec>(*fItr, VAL::E_AT_START));
                                spawn = true;
                            }
                        }
                    }
                }
                {
                    set<int>::iterator fItr = endSet.begin();
                    const set<int>::iterator fEnd = endSet.end();

                    for (; fItr != fEnd; ++fItr) {
                        map<int, int>::iterator nrItr = payload->forbiddenEnd.find(*fItr);
                        if (!(--(nrItr->second))) {
                            payload->forbiddenEnd.erase(nrItr);
                            if (!payload->endPreconditionCounts[*fItr]) {
                                d->noLongerForbidden.push_back(pair<int, VAL::time_spec>(*fItr, VAL::E_AT_END));
                                spawn = true;
                            }
                        }
                    }
                }

                if (spawn) {

                    if (payload->fluentLayers.find(cTime) == payload->fluentLayers.end()) {
                        assert(!payload->fluentLayers.empty());
                        const vector<double> & backFluents = payload->fluentLayers.rbegin()->second;
                        payload->fluentLayers.insert(pair<double, vector<double> >(cTime, backFluents));
                        if (evaluateDebug) cout << "Created fluent layer at time " << (cTime) << "\n";
                    }


                    if (payload->fluentLayers.find(cTime + EPSILON) == payload->fluentLayers.end()) {
                        assert(!payload->fluentLayers.empty());
                        const vector<double> & backFluents = payload->fluentLayers.rbegin()->second;
                        payload->fluentLayers.insert(pair<double, vector<double> >(cTime + EPSILON, backFluents));
                        if (evaluateDebug) cout << "Created fluent layer at time " << (cTime + EPSILON) << "\n";
                    }
                }

                if (d->updateActionsForNewLiteralFact(payload.get(), -2, cTime)) break;
            }

            {
                list<int>::iterator negItr = currFactLayerItr->second.negativeTILs.begin();
                const list<int>::iterator negEnd = currFactLayerItr->second.negativeTILs.end();

                for (; negItr != negEnd; ++negItr) {

                    list<pair<int, VAL::time_spec> > & currList = (*(d->preconditionsToActions))[*negItr];

                    list<pair<int, VAL::time_spec> >::iterator ioItr = currList.begin();
                    const list<pair<int, VAL::time_spec> >::iterator ioEnd = currList.end();

                    for (; ioItr != ioEnd; ++ioItr) {
                        if (ioItr->second == VAL::E_AT_START) {
                            {
                                int & checkAndIncr = payload->startPreconditionCounts[ioItr->first];
                                if (checkAndIncr) {
                                    ++checkAndIncr;
                                    ++(payload->endPreconditionCounts[ioItr->first]);
                                }
                            }
                        } else {
                            ++(payload->startPreconditionCounts[ioItr->first]);
                            ++(payload->endPreconditionCounts[ioItr->first]);
                        }
                    }

                }
            }

            if (evaluateDebug) cout << "Finished fact layer\n";


            payload->factLayers.erase(currFactLayerItr);

        }
    }


    if (payload->unsatisfiedGoals || payload->unappearedEnds) {
        if (evaluateDebug) {
            cout << "Dead end found in RPG\n";
            set<int>::const_iterator gItr = d->goals.begin();
            const set<int>::const_iterator gEnd = d->goals.end();

            for (; gItr != gEnd; ++gItr) {
                if ((*(d->achievedInLayer))[*gItr] < 0.0) {
                    cout << "Goal " << *gItr << ", " << *(RPGBuilder::getLiteral(*gItr)) << ", was never seen. ";
                    assert(!(RPGBuilder::isStatic(RPGBuilder::getLiteral(*gItr)).first));
                    cout << "Possible achievers: " << (*(d->effectsToActions))[*gItr].size() << endl;
                    list<pair<int, VAL::time_spec> >::const_iterator aItr = (*(d->effectsToActions))[*gItr].begin();
                    const list<pair<int, VAL::time_spec> >::const_iterator aEnd = (*(d->effectsToActions))[*gItr].end();

                    for (; aItr != aEnd; ++aItr) {
                        if (aItr->second == VAL::E_AT_START) {
                            if (payload->startActionSchedule[aItr->first] == -1.0) {
                                cout << "- Start of " << aItr->first << ", " << *(RPGBuilder::getInstantiatedOp(aItr->first)) << ", was never seen\n";
                            }
                            list<Literal*> & actPres = (*(d->actionsToProcessedStartPreconditions))[aItr->first];
                            list<Literal*>::const_iterator preItr = actPres.begin();
                            const list<Literal*>::const_iterator preEnd = actPres.end();
                            for (; preItr != preEnd; ++preItr) {
                                if ((*(d->achievedInLayer))[(*preItr)->getID()] < 0.0) {
                                    cout << "   * Start precondition " << *(*preItr) << " never appeared\n";
                                }
                            }
                        } else {
                            if (payload->endActionSchedule[aItr->first] == -1.0) {
                                if (payload->startActionSchedule[aItr->first] != -1.0) {
                                    cout << "- End of " << aItr->first << ", " << *(RPGBuilder::getInstantiatedOp(aItr->first)) << ", was never seen\n";
                                } else {
                                    cout << "- End of " << aItr->first << ", " << *(RPGBuilder::getInstantiatedOp(aItr->first)) << " was never seen (nor was its start)\n";

                                    list<Literal*> & actPres = (*(d->actionsToProcessedStartPreconditions))[aItr->first];
                                    list<Literal*>::const_iterator preItr = actPres.begin();
                                    const list<Literal*>::const_iterator preEnd = actPres.end();
                                    for (; preItr != preEnd; ++preItr) {
                                        if ((*(d->achievedInLayer))[(*preItr)->getID()] < 0.0) {
                                            cout << "   * Start precondition " << *(*preItr) << " never appeared\n";
                                        }
                                    }
                                }
                                list<Literal*> & actPres = (*(d->actionsToEndPreconditions))[aItr->first];
                                list<Literal*>::const_iterator preItr = actPres.begin();
                                const list<Literal*>::const_iterator preEnd = actPres.end();
                                for (; preItr != preEnd; ++preItr) {
                                    if ((*(d->achievedInLayer))[(*preItr)->getID()] < 0.0) {
                                        cout << "   * End precondition " << *(*preItr) << " never appeared\n";
                                    }
                                }

                            }
                        }
                    }
                }
            }

        }
        return -1;
    } else {
        if (evaluateDebug) cout << "RPG found all goals and ends\n";
    }



    int h = heuristicOffset;

//  evaluateDebug = true;


    pair<int, VAL::time_spec> earliestTIL(-1, VAL::E_AT);

    d->extractRP(payload.get(), h, relaxedPlan, earliestTIL, finalPlanMakespanEstimate);




    {
        // HACK

        map<int, int> started;

        {
            bool firstIns = true;

            map<int, int>::iterator insItr;

            map<int, set<int> >::const_iterator saItr = theState.startedActions.begin();
            const map<int, set<int> >::const_iterator saEnd = theState.startedActions.end();

            for (; saItr != saEnd; ++saItr) {
                if (firstIns) {
                    insItr = started.insert(make_pair(saItr->first, saItr->second.size())).first;
                    firstIns = false;
                } else {
                    insItr = started.insert(insItr, make_pair(saItr->first, saItr->second.size()));
                }
            }

        }
        list<pair<double, list<ActionSegment> > >::iterator rpItr = relaxedPlan.begin();
        const list<pair<double, list<ActionSegment> > >::iterator rpEnd = relaxedPlan.end();

        for (; rpItr != rpEnd; ++rpItr) {

            list<ActionSegment>::iterator rlItr = rpItr->second.begin();
            const list<ActionSegment>::iterator rlEnd = rpItr->second.end();

            for (; rlItr != rlEnd; ++rlItr) {
                if (rlItr->second == VAL::E_AT) continue;
                if (rlItr->second == VAL::E_AT_END) {
                    const map<int, int>::iterator sItr = started.find(rlItr->first->getID());
                    if (!(--(sItr->second))) started.erase(sItr);
                } else {
                    const map<int, int>::iterator innerItr = started.insert(make_pair(rlItr->first->getID(), 0)).first;
                    ++(innerItr->second);
                }
            }

        }

        map<int, int>::iterator sItr = started.begin();
        const map<int, int>::iterator sEnd = started.end();

        for (; sItr != sEnd; ++sItr) h += sItr->second;


    }

    list<double> haWeights;

    if (RPGBuilder::fullFFHelpfulActions) {

        set<int> startsIn;
        set<int> endsIn;

        {
            list<ActionSegment>::iterator haItr = helpfulActions.begin();
            const list<ActionSegment>::iterator haEnd = helpfulActions.end();

            for (; haItr != haEnd; ++haItr) {
                if (haItr->second == VAL::E_AT_START) {
                    startsIn.insert(haItr->first->getID());
                } else if (haItr->second == VAL::E_AT_END) {
                    endsIn.insert(haItr->first->getID());
                }
            }
        }

        {
            list<ActionSegment>::iterator haItr = helpfulActions.begin();
            const list<ActionSegment>::iterator haEnd = helpfulActions.end();

            for (; haItr != haEnd; ++haItr) {
                list<ActionSegment>::iterator haNext = haItr;
                ++haNext;
                const list<Literal*> * effs = 0;
                if (haItr->second == VAL::E_AT_START) {
                    effs = &((*(d->actionsToStartEffects))[haItr->first->getID()]);
                } else {
                    continue;
                }

                const double sas = payload->startActionSchedule[haItr->first->getID()];

                list<Literal*>::const_iterator eItr = effs->begin();
                const list<Literal*>::const_iterator eEnd = effs->end();

                for (; eItr != eEnd; ++eItr) {
                    const list<pair<int, VAL::time_spec> > & acBy = (*(d->effectsToActions))[(*eItr)->getID()];
                    list<pair<int, VAL::time_spec> >::const_iterator candItr = acBy.begin();
                    const list<pair<int, VAL::time_spec> >::const_iterator candEnd = acBy.end();

                    for (; candItr != candEnd; ++candItr) {
                        set<int> * checkIn = 0;
                        const list<Literal*> * deps = 0;
                        if (candItr->second == VAL::E_AT_START) {
                            const double thisSAS = payload->startActionSchedule[candItr->first];
                            if (thisSAS == -1.0) continue;
                            if (thisSAS > sas) continue;
                            checkIn = &startsIn;
                            deps = &((*(d->actionsToProcessedStartPreconditions))[candItr->first]);
                        } else {
                            continue;
                        }
                        if (checkIn->insert(candItr->first).second) {
                            /*                            // first time we've had to work out if this is helpful
                                                        list<Literal*>::const_iterator depItr = deps->begin();
                                                        const list<Literal*>::const_iterator depEnd = deps->end();

                                                        for (; depItr != depEnd; ++depItr) {
                                                            const int factID = (*depItr)->getID();
                                                            const double acIn = (*(d->achievedInLayer))[factID];
                                                            if (acIn == -1.0) break;
                                                            if (acIn > 0.0 && (*(d->achievedBy))[factID].first == -1) break;
                                                        }
                                                        if (depItr == depEnd) {*/
                            helpfulActions.insert(haNext, ActionSegment(RPGBuilder::getInstantiatedOp(candItr->first), candItr->second, -1, emptyIntList));
                            /*}*/
                        }
                    }
                }

                if (haItr->second == VAL::E_AT_START) {
                    effs = &((*(d->actionsToEndEffects))[haItr->first->getID()]);

                    const double endStamp = payload->endActionSchedule[haItr->first->getID()];
                    list<Literal*>::const_iterator eItr = effs->begin();
                    const list<Literal*>::const_iterator eEnd = effs->end();

                    for (; eItr != eEnd; ++eItr) {
                        const list<pair<int, VAL::time_spec> > & acBy = (*(d->effectsToActions))[(*eItr)->getID()];
                        list<pair<int, VAL::time_spec> >::const_iterator candItr = acBy.begin();
                        const list<pair<int, VAL::time_spec> >::const_iterator candEnd = acBy.end();
                        for (; candItr != candEnd; ++candItr) {
                            if (candItr->second != VAL::E_AT_END) continue;
                            if (fabs(payload->endActionSchedule[candItr->first] - endStamp) < 0.0001) {
                                if (endsIn.insert(candItr->first).second && startsIn.insert(candItr->first).second) {
                                    /*const list<Literal*> * deps = &((*(d->actionsToProcessedStartPreconditions))[candItr->first]);

                                    list<Literal*>::const_iterator depItr = deps->begin();
                                    const list<Literal*>::const_iterator depEnd = deps->end();

                                    for (; depItr != depEnd; ++depItr) {
                                        const int factID = (*depItr)->getID();
                                        const double acIn = (*(d->achievedInLayer))[factID];
                                        if (acIn == -1.0) break;
                                        if (acIn > 0.0 && (*(d->achievedBy))[factID].first == -1) break;
                                    }
                                    if (depItr == depEnd) {                                                                */
                                    helpfulActions.insert(haNext, ActionSegment(RPGBuilder::getInstantiatedOp(candItr->first), VAL::E_AT_START, -1, emptyIntList));
                                    /*}*/
                                }

                            }
                        }
                    }

                }

            }
        }
    }


    if (false && !relaxedPlan.empty()) {

        list<ActionSegment> unsortedList;
        unsortedList.swap(helpfulActions);

        if (h && earliestTIL.first != -1) {

            helpfulActions.push_back(ActionSegment(0, earliestTIL.second, earliestTIL.first, emptyIntList));
            haWeights.push_back(DBL_MAX);

        }


        list<ActionSegment>::iterator rlItr = unsortedList.begin();
        const list<ActionSegment>::iterator rlEnd = unsortedList.begin();

        for (; rlItr != rlEnd; ++rlItr) {
            //const int thisIOp = rlItr->first;
            if (nextTIL < Private::tilCount && rlItr->second != VAL::E_AT) {
// HACK
                const double w = (rlItr->second == VAL::E_AT_START ? Private::earliestDeadlineRelevancyStart[rlItr->first->getID()] : Private::earliestDeadlineRelevancyEnd[rlItr->first->getID()]);

                list<ActionSegment>::iterator haItr = helpfulActions.begin();
                const list<ActionSegment>::iterator haEnd = helpfulActions.end();
                list<double>::iterator wItr = haWeights.begin();

                for (; haItr != haEnd && (*wItr) < w; ++haItr, ++wItr);

                helpfulActions.insert(haItr, *rlItr);
                haWeights.insert(wItr, w);
                if (w == DBL_MAX) {
//                  cout << thisID << " " << "inf\n";
                } else {
//                  cout << thisID << " " << w << "\n";
                }


            } else {
                helpfulActions.push_back(*rlItr);
                haWeights.push_back(DBL_MAX);
            }
        }


    }

    if (false) {
        cout << "Relaxed plan for state:\n";
        list<pair<double, list<ActionSegment> > >::iterator rpItr = relaxedPlan.begin();
        const list<pair<double, list<ActionSegment> > >::iterator rpEnd = relaxedPlan.end();

        for (; rpItr != rpEnd; ++rpItr) {
            cout << "\t" << rpItr->first << ":\n";
            list<ActionSegment>::iterator rlItr = rpItr->second.begin();
            const list<ActionSegment>::iterator rlEnd = rpItr->second.end();

            for (; rlItr != rlEnd; ++rlItr) {
                cout << "\t\t" << *(rlItr->first);
                if (rlItr->second == VAL::E_AT_END) {
                    cout << ", end\n";
                } else {
                    cout << ", start\n";
                }
            }


        }
    }


    if (evaluateDebug || Globals::globalVerbosity & 1048576) {

        cout << "Helpful actions:\n";

        list<ActionSegment>::iterator haItr = helpfulActions.begin();
        const list<ActionSegment>::iterator haEnd = helpfulActions.end();

        for (; haItr != haEnd; ++haItr) {
            if (haItr->second == VAL::E_AT) {
                cout << "Timed initial literal action " << haItr->divisionID << "\n";
            } else {
                cout << *(haItr->first) << ", ";
                if (haItr->second == VAL::E_AT_START) cout << "start\n";
                if (haItr->second == VAL::E_AT_END) cout << "end\n";
                if (haItr->second == VAL::E_OVER_ALL) cout << "middle point " << haItr->divisionID << "\n";
            }
        }


    }

    return h;

};

double RPGHeuristic::Private::earliestTILForAction(const unsigned int & i, const bool & isStart)
{

    assert(i >= 0);
    assert(i < actionsToEndPreconditions->size());
    double toReturn = DBL_MAX;

    list<Literal*> & inspect = (isStart ? (*actionsToProcessedStartPreconditions)[i] : (*actionsToEndPreconditions)[i]);
    list<Literal*>::iterator insItr = inspect.begin();
    const list<Literal*>::iterator insEnd = inspect.end();

//  cout << "\t\tRequires precs:\n";
    for (; insItr != insEnd; ++insItr) {
//      cout << "\t\t\t" << *(*insItr);
        assert(*insItr);
        double & thisD = deadlineAtTime[(*insItr)->getID()];
        if (toReturn > thisD) toReturn = thisD;
        if (thisD == DBL_MAX) {
//          cout << ", which is never a deadline\n";
        } else {
//          cout << ", which is a deadline at " << thisD << "\n";
        }
    }

    return toReturn;

};

double RPGHeuristic::earliestTILForAction(const int & i, const bool & isStart)
{

    return d->earliestTILForAction(i, isStart);

};

double RPGHeuristic::Private::calculateActCost(Private::BuildingPayload * const payload, const int & currAct, const VAL::time_spec & currTS)
{
    if (!RPGHeuristic::hAddCostPropagation) {
        return 0.0;
    }
    
    if (currTS == VAL::E_AT) {
        return 1.0;
    }
    
    double toReturn = 1.0;
    
    for (int pass = 0; pass < 2; ++pass) {
      
        if (currTS == VAL::E_AT_START) {
            if (pass == 1) {
                break;
            }
        } else {
            if (payload->startedActions.find(currAct) != payload->startedActions.end()) {
                continue;
            }
        }
        
        list<Literal*> & inspect = (pass ? (*actionsToEndPreconditions)[currAct] : (*actionsToProcessedStartPreconditions)[currAct]);
        
        list<Literal*>::iterator insItr = inspect.begin();
        const list<Literal*>::iterator insEnd = inspect.end();
        
        for (; insItr != insEnd; ++insItr) {
            toReturn += hAddCostOfFact[(*insItr)->getID()];
        }
    }
    
    return toReturn;
}

bool RPGHeuristic::Private::applyPropositionalEffects(Private::BuildingPayload * const payload, const int & currAct, double & actCost, const VAL::time_spec & currTS, const double & nlTime, MaxDependentInfo & POtime)
{
    static const bool updateDebug = Globals::globalVerbosity & 64;

    list<Literal*> & addEffects = (currTS == VAL::E_AT_START ? RPGBuilder::getStartPropositionAdds()[currAct] : RPGBuilder::getEndPropositionAdds()[currAct]);

    list<Literal*>::iterator addEffItr = addEffects.begin();
    const list<Literal*>::iterator addEffEnd = addEffects.end();

    for (; addEffItr != addEffEnd; ++addEffItr) {
        const int currEff = (*addEffItr)->getID();
        double & currAIL = (*achievedInLayer)[currEff];
        if (currAIL == -1.0) {
            currAIL = nlTime;
            (*achievedBy)[currEff] = make_pair(currAct, currTS);
            if (actCost == -1.0) {
                actCost = calculateActCost(payload, currAct, currTS);
            }

            hAddCostOfFact[currEff] = actCost;

            payload->factLayers[nlTime].first.push_back(currEff);
            //earliestPropositionPOTimes[currEff] = POtime.get();
            if (updateDebug) cout << "\t\tFact " << currEff << " is new\n";
            if (goals.find(currEff) != gsEnd) {
                if (!(--(payload->unsatisfiedGoals)) && !(payload->unappearedEnds)) return true;
            }
        } else if (currAIL == nlTime) {            
            
            if (actCost == -1.0) {
                actCost = calculateActCost(payload, currAct, currTS);
            }
            
            if (actCost < hAddCostOfFact[currEff]) {
                (*achievedBy)[currEff] = make_pair(currAct, currTS);
                hAddCostOfFact[currEff] = actCost;
                if (updateDebug) cout << "\t\tFact " << currEff << " re-achieved at layer " << currAIL << " for lower cost\n";
            } else {
                if (updateDebug) cout << "\t\tFact " << currEff << " was already achieved in this layer, i.e. " << currAIL << ", and the new achiever isn't any cheaper\n";
            }

        } else {
            if (updateDebug) cout << "\t\tFact " << currEff << " was already achieved in layer, " << currAIL << "\n";
        }
    }

    return false;
}

bool RPGHeuristic::Private::checkPreconditionsAreSatisfied(const int & currAct, const VAL::time_spec & ts, const double & layer)
{

    {
        const list<Literal*> & precList = (ts == VAL::E_AT_START
                                           ? (*actionsToProcessedStartPreconditions)[currAct]
                                           : (*actionsToEndPreconditions)[currAct]
                                          );


        list<Literal*>::const_iterator pItr = precList.begin();
        const list<Literal*>::const_iterator pEnd = precList.end();


        for (; pItr != pEnd; ++pItr) {
            assert((*achievedInLayer)[(*pItr)->getID()] != -1.0);
            assert((*achievedInLayer)[(*pItr)->getID()] <= layer);
            assert((*achievedBy)[(*pItr)->getID()] != make_pair(currAct, ts));
            if (evaluateDebug) cout << "\t\t\t\tPrecondition " << *(*pItr) << " became true at " << (*achievedInLayer)[(*pItr)->getID()] << "\n";
        }
    }

    {
        const list<int> & precList = (ts == VAL::E_AT_START
                                      ? (*actionsToProcessedStartNumericPreconditions)[currAct]
                                      : (*actionsToNumericEndPreconditions)[currAct]
                                     );

        list<int>::const_iterator pItr = precList.begin();
        const list<int>::const_iterator pEnd = precList.end();


        for (; pItr != pEnd; ++pItr) {
            assert((*numericAchievedInLayer)[(*pItr)] != -1.0);
            assert((*numericAchievedInLayer)[(*pItr)] <= layer);
            ActionFluentModification * const ac = (*numericAchievedBy)[(*pItr)];
            if (ac) {
                assert(!(ac->act == currAct && ac->ts == ts));
            }
            if (evaluateDebug) cout << "\t\t\t\tNumeric precondition " << (*pItr) << " became true at " << (*numericAchievedInLayer)[(*pItr)] << "\n";
        }
    }

    return true;
}

bool RPGHeuristic::Private::updateActionsForNewLiteralFact(Private::BuildingPayload * const payload, const int & toPropagate, const double & factLayerTime)
{

    static const double EPSILON = 0.001;

    const MinimalState & startState = payload->startState;
    vector<int> & startPreconditionCounts = payload->startPreconditionCounts;
    vector<int> & endPreconditionCounts = payload->endPreconditionCounts;
    vector<int> & numericStartPreconditionCounts = payload->numericStartPreconditionCounts;
    vector<int> & numericEndPreconditionCounts = payload->numericEndPreconditionCounts;
    map<double, FactLayerEntry, EpsilonComp > & factLayer = payload->factLayers;
    map<double, vector<double>, EpsilonComp > & fluentLayers = payload->fluentLayers;
    map<double, map<int, list<ActionFluentModification> >, EpsilonComp> & fluentModifications = payload->fluentModifications;
    map<double, list<int>, EpsilonComp > & endActionsAtTime = payload->endActionsAtTime;
    vector<double> & startActionSchedule = payload->startActionSchedule;
    vector<double> & endActionSchedule = payload->endActionSchedule;
    vector<double> & openEndActionSchedule = payload->openEndActionSchedule;
    const map<int, set<int> > & startedActions = payload->startedActions;
    int & unsatisfiedGoals = payload->unsatisfiedGoals;
    int & unappearedEnds = payload->unappearedEnds;
    map<int, set<int> > & insistUponEnds = payload->insistUponEnds;
    map<int, int> & forbiddenStart = payload->forbiddenStart;
    map<int, int> & forbiddenEnd = payload->forbiddenEnd;


    const double nlTime = factLayerTime + EPSILON;
    const bool updateDebug = Globals::globalVerbosity & 64;
    const bool preconditionless = (toPropagate < 0);
    list<pair<int, VAL::time_spec> > & dependents = ((toPropagate == -1) ? (*preconditionlessActions) : ((toPropagate == -2) ? (noLongerForbidden) : (*processedPreconditionsToActions)[toPropagate]));

    if (evaluateDebug) {
        if (toPropagate == -1) {
            cout << "*Special case: preconditionless actions\n";
        } else if (toPropagate == -2) {
            cout << "*Special case: actions that are no longer forbidden\n";
        }
    }

    list<pair<int, VAL::time_spec> >::iterator depItr = dependents.begin();
    const list<pair<int, VAL::time_spec> >::iterator depEnd = dependents.end();

    if (updateDebug) cout << "\tAffects " << dependents.size() << " actions\n";

    const map<double, vector<double> >::iterator mfItr = fluentLayers.find(factLayerTime);
    assert(mfItr != fluentLayers.end());

    const vector<double> & maxFluents = mfItr->second;

    for (; depItr != depEnd; ++depItr) {
        const int currAct = depItr->first;
        const bool startAct = (depItr->second == VAL::E_AT_START);
        assert(depItr->second != VAL::E_OVER_ALL);
        if (updateDebug) cout << "\tAffects " << currAct << ", " << (startAct ? "start" : "end") << " " << *(RPGBuilder::getInstantiatedOp(currAct)) << "\n";
        if (startAct) {
            int & spc = startPreconditionCounts[currAct];
            if (!preconditionless) --spc;
            if ((!spc && !numericStartPreconditionCounts[currAct]) && forbiddenStart.find(currAct) == forbiddenStart.end() && nlTime < latestStartAllowed[currAct]) {
                assert(!RPGBuilder::rogueActions[currAct]);
                if (updateDebug) {
                    cout << "\tStart of action " << currAct << " is now applicable: " << startPreconditionCounts[currAct] << "/" << numericStartPreconditionCounts[currAct];
                    if (preconditionless) cout << ", considered preconditionless";
                    cout << "\n";
                }

                assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));

                if (expandFully) earliestStartAllowed[currAct] = factLayerTime;

                MAKESTARTMDI(POtime, currAct);

                double actCostTmp = -1.0;
                
                if (applyPropositionalEffects(payload, currAct, actCostTmp, VAL::E_AT_START, nlTime, POtime)) return true;




                list<double>::iterator contChangeItr = integratedCTSEffectChange[currAct].begin();
//  vector<list<int> > integratedCTSEffectVar;
//  vector<list<double> > integratedCTSEffectChange;


                for (int numPass = 0; numPass < 2; ++numPass) {

                    list<int> & numericEffects = (numPass ? integratedCTSEffectVar[currAct] : (*actionsToRPGNumericStartEffects)[currAct]);;

                    list<int>::iterator numEffItr = numericEffects.begin();
                    const list<int>::iterator numEffEnd = numericEffects.end();

                    if (updateDebug) {
                        if (numPass) {
                            cout << "\t\tHas " << numericEffects.size() << " continuous numeric effects\n";
                        } else {
                            cout << "\t\tHas " << numericEffects.size() << " start numeric effects\n";
                        }
                    }

                    for (; numEffItr != numEffEnd; ++numEffItr) {

                        double signedEffect;
                        int howManyTimes;
                        int varToAffect;
                        bool isAssignment;

                        if (numPass) {
                            signedEffect = *contChangeItr;
                            howManyTimes = RPGBuilder::howManyTimes(currAct, startState);
                            varToAffect = *numEffItr;
                            isAssignment = false;
                        } else {
                            RPGBuilder::RPGNumericEffect & currEff = (*rpgNumericEffects)[*numEffItr];

                            signedEffect = currEff.evaluate(maxFluents, RPGBuilder::getOpMinDuration(currAct, 0), RPGBuilder::getOpMaxDuration(currAct, 0));
                            howManyTimes = RPGBuilder::howManyTimes(currAct, startState);

                            varToAffect = currEff.fluentIndex;
                            isAssignment = currEff.isAssignment;
                        }
                        if (updateDebug) cout << "\t\tEffect upon " << *(RPGBuilder::getPNE(varToAffect)) << ": ";

                        {

                            list<pair<int, ActionFluentModification*> > actuallyChanged;
                            map<double, vector<double>, EpsilonComp >::iterator flItr = fluentLayers.insert(pair<double, vector<double> >(nlTime, maxFluents)).first;

                            if (isAssignment) {
                                map<int, list<ActionFluentModification> > & afMap = fluentModifications[factLayerTime];


                                for (int pass = 0; pass < 2; ++pass) {
                                    ActionFluentModification afm(currAct, depItr->second, false, signedEffect, howManyTimes, true);
                                    list<ActionFluentModification> & afmList = afMap[varToAffect];
                                    afmList.push_back(afm);

                                    double & oldValue = flItr->second[varToAffect];

                                    if (updateDebug) {
                                        if (!pass) {
                                            if (signedEffect == DBL_MAX) {
                                                cout << "assign to inf";
                                            } else {
                                                cout << "assign to " << signedEffect;
                                            }
                                        }
                                        if (pass) {
                                            if (oldValue == DBL_MAX) {
                                                cout << ", old minimum value of -inf\n";
                                            } else {
                                                cout << ", old minimum value of -" << oldValue << "\n";
                                            }
                                        } else {
                                            if (oldValue == DBL_MAX) {
                                                cout << ", old max value of inf";
                                            } else {
                                                cout << ", old max value of " << oldValue;
                                            }
                                            cout.flush();
                                        }
                                    }

                                    if (oldValue < signedEffect) {

                                        oldValue = signedEffect;
                                        actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                                    }

                                    varToAffect += RPGBuilder::getPNECount(); // for pass 2
                                }

                            } else if (signedEffect != 0.0) {
                                double effectMagnitude = signedEffect;
                                if (signedEffect < 0.0) {
                                    effectMagnitude = 0.0 - effectMagnitude;
                                    varToAffect += RPGBuilder::getPNECount();
                                    if (updateDebug) cout << "-";
                                }
                                if (updateDebug) {
                                    if (effectMagnitude == DBL_MAX) {
                                        cout << "infinite\n";
                                    } else {
                                        cout << "increase by " << effectMagnitude << "\n";
                                    }
                                }

                                ActionFluentModification afm(currAct, depItr->second, false, effectMagnitude, howManyTimes, false);

                                map<int, list<ActionFluentModification> > & afMap = fluentModifications[factLayerTime];

                                list<ActionFluentModification> & afmList = afMap[varToAffect];
                                afmList.push_back(afm);



                                if (updateDebug) {
                                    cout << "\t\t\tCan be done " << howManyTimes << " times\n";
                                }

                                if (effectMagnitude == DBL_MAX || howManyTimes == INT_MAX) {
                                    double & oldValue = flItr->second[varToAffect];
                                    if (oldValue != DBL_MAX) {
                                        flItr->second[varToAffect] = DBL_MAX; // incrementing by infinity or assigning infinity - as if it matters....
                                        actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                                    }
                                } else {
                                    const double overallChange = howManyTimes * effectMagnitude;
                                    double & oldValue = flItr->second[varToAffect];
                                    if (oldValue != DBL_MAX) {
                                        oldValue += overallChange;
                                        actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                                    }
                                }
                            } else {
                                if (updateDebug) cout << "0.0\n";
                            }

                            list<pair<int, ActionFluentModification*> >::iterator acItr = actuallyChanged.begin();
                            const list<pair<int, ActionFluentModification*> >::iterator acEnd = actuallyChanged.end();

                            for (; acItr != acEnd; ++acItr) {

                                {
                                    list<int> & affectedPres = RPGBuilder::affectsRPGNumericPreconditions(acItr->first);

                                    list<int>::iterator apItr = affectedPres.begin();
                                    const list<int>::iterator apEnd = affectedPres.end();

                                    for (; apItr != apEnd; ++apItr) {

                                        const int currEff = *apItr;
                                        double & currAIL = (*numericAchievedInLayer)[currEff];
                                        if (currAIL == -1.0) {
                                            assert(!((*numericAchievedBy)[currEff]));
                                            if ((*rpgNumericPreconditions)[currEff].isSatisfied(flItr->second)) {
                                                currAIL = nlTime;
                                                (*numericAchievedBy)[currEff] = acItr->second;
                                                factLayer[nlTime].second.push_back(currEff);
                                                if (goalFluents.find(currEff) != gfEnd) {
                                                    if (!(--unsatisfiedGoals) && !unappearedEnds) return true;
                                                }
                                            }
                                        }

                                    }

                                }




                                {
                                    list<int> & affectedAVs = RPGBuilder::getVariableDependencies(acItr->first);

                                    list<int>::iterator aavItr = affectedAVs.begin();
                                    const list<int>::iterator aavEnd = affectedAVs.end();

                                    for (; aavItr != aavEnd; ++aavItr) {

                                        double & oldVal = flItr->second[*aavItr];
                                        const double newVal = RPGBuilder::getArtificialVariable(*aavItr).evaluate(flItr->second);

                                        if (newVal > oldVal) {

                                            oldVal = newVal;

                                            list<int> & affectedPres = RPGBuilder::affectsRPGNumericPreconditions(*aavItr);

                                            list<int>::iterator apItr = affectedPres.begin();
                                            const list<int>::iterator apEnd = affectedPres.end();

                                            for (; apItr != apEnd; ++apItr) {

                                                const int currEff = *apItr;
                                                double & currAIL = (*numericAchievedInLayer)[currEff];
                                                if (currAIL == -1.0) {
                                                    assert(!((*numericAchievedBy)[currEff]));
                                                    if ((*rpgNumericPreconditions)[currEff].isSatisfied(flItr->second)) {
                                                        currAIL = nlTime;
                                                        (*numericAchievedBy)[currEff] = acItr->second;
                                                        factLayer[nlTime].second.push_back(currEff);
                                                        if (goalFluents.find(currEff) != gfEnd) {
                                                            if (!(--unsatisfiedGoals) && !unappearedEnds) return true;
                                                        }
                                                    }
                                                }

                                            }

                                        }

                                    }
                                }
                            }

                        }


                        if (numPass) ++contChangeItr;
                    }
                }

                {
                    const double endTime = factLayerTime + RPGBuilder::getOpMinDuration(currAct, 0);

                    startActionSchedule[currAct] = factLayerTime;
                    endActionSchedule[currAct] = endTime;

                    if (!endPreconditionCounts[currAct]) {
                        if (updateDebug) cout << "\t\tEnd of action can be put at fact layer sufficiently far in future (" << endTime << "\n";

                        endActionsAtTime[endTime].push_back(currAct);
                    }
                }

                assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));

            } else {
                if (updateDebug) cout << "\tStart of action " << currAct << " now only has " << startPreconditionCounts[currAct] << " unsatisfied propositional preconditions and " << numericStartPreconditionCounts[currAct] << " numeric\n";
            }
        } else {
            int & epc = endPreconditionCounts[currAct];
            if (!preconditionless) --epc;
            if ((!epc && !numericEndPreconditionCounts[currAct]) && forbiddenEnd.find(currAct) == forbiddenEnd.end() && nlTime < latestEndAllowed[currAct]) {
                if (RPGBuilder::rogueActions[currAct]) {
                    cout << "Critical Error: Trying to apply end of action " << currAct << ", " << *(RPGBuilder::getInstantiatedOp(currAct)) << ", which is invalid or irrelevant\n";
                    if (&dependents == preconditionlessActions) {
                        cout << "\tFound it on the list of preconditionless actions\n";
                    } else {
                        cout << "\tFound it on the list of actions depending on " << RPGBuilder::getLiteral(toPropagate) << "\n";
                    }
                    assert(!RPGBuilder::rogueActions[currAct]);
                }
                if (updateDebug) cout << "\tEnd of action " << currAct << " is now applicable\n";

                assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));

                if (expandFully) earliestEndAllowed[currAct] = factLayerTime;

                bool insistOnThisEnd = (insistUponEnds.find(currAct) != insistUponEnds.end());
                if (insistOnThisEnd) {
                    if (updateDebug) cout << "\tEnd is insisted upon, had " << unappearedEnds << " remaining\n";
                    if (factLayerTime >= openEndActionSchedule[currAct]) {
                        openEndActionSchedule[currAct] = factLayerTime;
                        if (!(--unappearedEnds) && !unsatisfiedGoals) return true;
                        if (updateDebug) cout << "\tNow down to " << unappearedEnds << "\n";
                    } else {
                        if (updateDebug) cout << "\tOpen end cannot appear quite yet\n";
                        insistOnThisEnd = false;
                        endActionsAtTime[openEndActionSchedule[currAct]].push_back(-1 - currAct);
                        if (!(--unappearedEnds) && !unsatisfiedGoals) return true;
                        if (updateDebug) cout << "\tNow down to " << unappearedEnds << "\n";
                    }
                }




                for (int epPass = 0; epPass < 2; ++epPass) {
                    bool doLoop = false;
                    bool limitToOnce = false;
                    if (!epPass) {
                        doLoop = insistOnThisEnd;
                        limitToOnce = true;
                    } else {
                        if (!startPreconditionCounts[currAct] && !numericStartPreconditionCounts[currAct] && startActionSchedule[currAct] > -1.0) {
                            const double enforcedEnd = endActionSchedule[currAct];
                            if (factLayerTime >= enforcedEnd) {
                                endActionSchedule[currAct] = factLayerTime;
                                doLoop = true;
                            } else {
                                endActionsAtTime[enforcedEnd].push_back(currAct);
                            }
                        }
                    }

                    if (doLoop) {
                        if (updateDebug) {
                            if (limitToOnce) {
                                cout << "\tStarted before RPG\n";
                            } else {
                                cout << "\tStart is sufficiently earlier\n";
                            }
                        }

                        MAKEENDMDI(POtime, (limitToOnce ? payload->earliestStartOf[currAct] : 0.0), RPGBuilder::getOpMinDuration(currAct, 0), currAct);

                        double actCostTmp = -1.0;
                        
                        if (applyPropositionalEffects(payload, currAct, actCostTmp, VAL::E_AT_END, nlTime, POtime)) return true;


                        list<int> & numericEffects = (*actionsToRPGNumericEndEffects)[currAct];

                        list<int>::iterator numEffItr = numericEffects.begin();
                        const list<int>::iterator numEffEnd = numericEffects.end();

                        for (; numEffItr != numEffEnd; ++numEffItr) {
                            RPGBuilder::RPGNumericEffect & currEff = (*rpgNumericEffects)[*numEffItr];

                            const double signedEffect = currEff.evaluate(maxFluents, RPGBuilder::getOpMinDuration(currAct, 0), RPGBuilder::getOpMaxDuration(currAct, 0));
                            int howManyTimes = 0;

                            if (limitToOnce) {
                                const map<int, set<int> >::const_iterator saItr = startedActions.find(currAct);
                                if (saItr != startedActions.end()) howManyTimes += saItr->second.size();
                                // HACK
                            } else {
                                howManyTimes = RPGBuilder::howManyTimes(currAct, startState);
                            }


                            int varToAffect = currEff.fluentIndex;

                            if (updateDebug) cout << "\t\tEffect upon " << *(RPGBuilder::getPNE(varToAffect)) << ": ";

                            {

                                list<pair<int, ActionFluentModification*> > actuallyChanged;
                                map<double, vector<double>, EpsilonComp >::iterator flItr = fluentLayers.insert(pair<double, vector<double> >(nlTime, maxFluents)).first;

                                if (currEff.isAssignment) {
                                    map<int, list<ActionFluentModification> > & afMap = fluentModifications[factLayerTime];


                                    for (int pass = 0; pass < 2; ++pass) {
                                        ActionFluentModification afm(currAct, depItr->second, limitToOnce, signedEffect, howManyTimes, true);
                                        list<ActionFluentModification> & afmList = afMap[varToAffect];
                                        afmList.push_back(afm);

                                        double & oldValue = flItr->second[varToAffect];

                                        if (updateDebug) {
                                            if (!pass) {
                                                if (signedEffect == DBL_MAX) {
                                                    cout << "assign to inf";
                                                } else {
                                                    cout << "assign to " << signedEffect;
                                                }
                                            }
                                            if (pass) {
                                                if (oldValue == DBL_MAX) {
                                                    cout << ", old minimum value of -inf\n";
                                                } else {
                                                    cout << ", old minimum value of -" << oldValue << "\n";
                                                }
                                            } else {
                                                if (oldValue == DBL_MAX) {
                                                    cout << ", old max value of inf";
                                                } else {
                                                    cout << ", old max value of " << oldValue;
                                                }
                                                cout.flush();
                                            }
                                        }

                                        if (oldValue < signedEffect) {
                                            oldValue = signedEffect;
                                            actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                                        }

                                        varToAffect += RPGBuilder::getPNECount(); // for pass 2
                                    }

                                } else if (signedEffect != 0.0) {
                                    double effectMagnitude = signedEffect;
                                    if (signedEffect < 0.0) {
                                        effectMagnitude = 0.0 - effectMagnitude;
                                        varToAffect += RPGBuilder::getPNECount();
                                        if (updateDebug) cout << "-";
                                    }
                                    if (updateDebug) {
                                        if (effectMagnitude == DBL_MAX) {
                                            cout << "infinite\n";
                                        } else {
                                            cout << effectMagnitude << "\n";
                                        }
                                    }

                                    ActionFluentModification afm(currAct, depItr->second, limitToOnce, effectMagnitude, howManyTimes, false);

                                    map<int, list<ActionFluentModification> > & afMap = fluentModifications[factLayerTime];

                                    list<ActionFluentModification> & afmList = afMap[varToAffect];
                                    afmList.push_back(afm);



                                    if (updateDebug) {
                                        cout << "Can be done " << howManyTimes << " times\n";
                                    }

                                    if (effectMagnitude == DBL_MAX || howManyTimes == INT_MAX) {
                                        double & oldValue = flItr->second[varToAffect];
                                        if (oldValue != DBL_MAX) {
                                            flItr->second[varToAffect] = DBL_MAX; // incrementing by infinity or assigning infinity - as if it matters....
                                            actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                                        }
                                    } else {
                                        const double overallChange = howManyTimes * effectMagnitude;
                                        double & oldValue = flItr->second[varToAffect];
                                        if (oldValue != DBL_MAX) {
                                            oldValue += overallChange;
                                            actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                                        }
                                    }
                                } else {
                                    if (updateDebug) cout << "0.0\n";
                                }

                                list<pair<int, ActionFluentModification*> >::iterator acItr = actuallyChanged.begin();
                                const list<pair<int, ActionFluentModification*> >::iterator acEnd = actuallyChanged.end();

                                for (; acItr != acEnd; ++acItr) {

                                    {
                                        list<int> & affectedPres = RPGBuilder::affectsRPGNumericPreconditions(acItr->first);

                                        list<int>::iterator apItr = affectedPres.begin();
                                        const list<int>::iterator apEnd = affectedPres.end();

                                        for (; apItr != apEnd; ++apItr) {

                                            const int currEff = *apItr;
                                            double & currAIL = (*numericAchievedInLayer)[currEff];
                                            if (currAIL == -1.0) {
                                                assert(!((*numericAchievedBy)[currEff]));
                                                if ((*rpgNumericPreconditions)[currEff].isSatisfied(flItr->second)) {
                                                    if (updateDebug) cout << "\t\t" << (*rpgNumericPreconditions)[currEff] << " now satisfied\n";
                                                    currAIL = nlTime;
                                                    (*numericAchievedBy)[currEff] = acItr->second;
                                                    factLayer[nlTime].second.push_back(currEff);
                                                    if (goalFluents.find(currEff) != gfEnd) {
                                                        if (!(--unsatisfiedGoals) && !unappearedEnds) return true;
                                                    }
                                                }
                                            }

                                        }

                                    }

                                    {
                                        list<int> & affectedAVs = RPGBuilder::getVariableDependencies(acItr->first);

                                        list<int>::iterator aavItr = affectedAVs.begin();
                                        const list<int>::iterator aavEnd = affectedAVs.end();

                                        for (; aavItr != aavEnd; ++aavItr) {

                                            double & oldVal = flItr->second[*aavItr];
                                            const double newVal = RPGBuilder::getArtificialVariable(*aavItr).evaluate(flItr->second);

                                            if (newVal > oldVal) {

                                                oldVal = newVal;

                                                list<int> & affectedPres = RPGBuilder::affectsRPGNumericPreconditions(*aavItr);

                                                list<int>::iterator apItr = affectedPres.begin();
                                                const list<int>::iterator apEnd = affectedPres.end();

                                                for (; apItr != apEnd; ++apItr) {

                                                    const int currEff = *apItr;
                                                    double & currAIL = (*numericAchievedInLayer)[currEff];
                                                    if (currAIL == -1.0) {
                                                        assert(!((*numericAchievedBy)[currEff]));
                                                        if ((*rpgNumericPreconditions)[currEff].isSatisfied(flItr->second)) {
                                                            currAIL = nlTime;
                                                            (*numericAchievedBy)[currEff] = acItr->second;
                                                            factLayer[nlTime].second.push_back(currEff);
                                                            if (goalFluents.find(currEff) != gfEnd) {
                                                                if (!(--unsatisfiedGoals) && !unappearedEnds) return true;
                                                            }
                                                        }
                                                    }

                                                }

                                            }

                                        }
                                    }
                                }

                            }

                        }

                    }

                }

                assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));


            } else {
                if (updateDebug) cout << "\tEnd of action " << currAct << " now only has " << endPreconditionCounts[currAct] << " unsatisfied preconditions\n";
            }
        }
    }

    return false;

};

bool RPGHeuristic::Private::updateActionsForNewNumericFact(Private::BuildingPayload * const payload, const int & toPropagate, const double & factLayerTime)
{

    static const double EPSILON = 0.001;

    const MinimalState & startState = payload->startState;
    vector<int> & startPreconditionCounts = payload->startPreconditionCounts;
    vector<int> & endPreconditionCounts = payload->endPreconditionCounts;
    vector<int> & numericStartPreconditionCounts = payload->numericStartPreconditionCounts;
    vector<int> & numericEndPreconditionCounts = payload->numericEndPreconditionCounts;
    map<double, FactLayerEntry, EpsilonComp > & factLayer = payload->factLayers;
    map<double, vector<double>, EpsilonComp > & fluentLayers = payload->fluentLayers;
    map<double, map<int, list<ActionFluentModification> >, EpsilonComp> & fluentModifications = payload->fluentModifications;
    map<double, list<int>, EpsilonComp > & endActionsAtTime = payload->endActionsAtTime;
    vector<double> & startActionSchedule = payload->startActionSchedule;
    vector<double> & endActionSchedule = payload->endActionSchedule;
    vector<double> & openEndActionSchedule = payload->openEndActionSchedule;
    const map<int, set<int> > & startedActions = payload->startedActions;

    int & unsatisfiedGoals = payload->unsatisfiedGoals;
    int & unappearedEnds = payload->unappearedEnds;
    map<int, set<int> > & insistUponEnds = payload->insistUponEnds;
    map<int, int> & forbiddenStart = payload->forbiddenStart;
    map<int, int> & forbiddenEnd = payload->forbiddenEnd;


    const double nlTime = factLayerTime + EPSILON;
    const bool updateDebug = Globals::globalVerbosity & 64;

    static bool initDummy = false;
    list<pair<int, VAL::time_spec> > dummyList;
    if (!initDummy) {
        dummyList.push_back(pair<int, VAL::time_spec>(-1, VAL::E_AT_END));
        initDummy = true;
    }

    list<pair<int, VAL::time_spec> > & dependents = ((toPropagate >= 0) ? (*processedNumericPreconditionsToActions)[toPropagate] : dummyList);

    list<pair<int, VAL::time_spec> >::iterator depItr = dependents.begin();
    const list<pair<int, VAL::time_spec> >::iterator depEnd = dependents.end();

    if (updateDebug && (toPropagate >= 0)) cout << "\tAffects " << dependents.size() << " actions\n";

    const vector<double> & maxFluents = fluentLayers[factLayerTime];

    for (; depItr != depEnd; ++depItr) {
        const int currAct = ((toPropagate >= 0) ? depItr->first : (-1 - toPropagate));
        const bool startAct = ((toPropagate >= 0) ? (depItr->second == VAL::E_AT_START) : false);
        assert(depItr->second != VAL::E_OVER_ALL);
        if (updateDebug) cout << "\tAffects " << currAct << ", " << (startAct ? "start" : "end") << " " << *(RPGBuilder::getInstantiatedOp(currAct)) << "\n";
        if (startAct) {

            if (!(--numericStartPreconditionCounts[currAct]) && !startPreconditionCounts[currAct] && forbiddenStart.find(currAct) == forbiddenStart.end() && nlTime < latestStartAllowed[currAct]) {
                assert(!RPGBuilder::rogueActions[currAct]);
                if (updateDebug) cout << "\tStart of action " << currAct << " is now applicable\n";

                assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));

                if (expandFully) earliestStartAllowed[currAct] = factLayerTime;

                MAKESTARTMDI(POtime, currAct);

                double actCostTmp = -1.0;
                
                if (applyPropositionalEffects(payload, currAct, actCostTmp, VAL::E_AT_START, nlTime, POtime)) return true;

                list<double>::iterator contChangeItr = integratedCTSEffectChange[currAct].begin();

                for (int numPass = 0; numPass < 2; ++numPass) {

                    list<int> & numericEffects = (numPass ? integratedCTSEffectVar[currAct] : (*actionsToRPGNumericStartEffects)[currAct]);;

                    list<int>::iterator numEffItr = numericEffects.begin();
                    const list<int>::iterator numEffEnd = numericEffects.end();


                    for (; numEffItr != numEffEnd; ++numEffItr) {

                        double signedEffect;
                        int howManyTimes;
                        int varToAffect;
                        bool isAssignment;

                        if (numPass) {
                            signedEffect = *contChangeItr;
                            howManyTimes = RPGBuilder::howManyTimes(currAct, startState);
                            varToAffect = *numEffItr;
                            isAssignment = false;
                        } else {

                            RPGBuilder::RPGNumericEffect & currEff = (*rpgNumericEffects)[*numEffItr];

                            signedEffect = currEff.evaluate(maxFluents, RPGBuilder::getOpMinDuration(currAct, 0), RPGBuilder::getOpMaxDuration(currAct, 0));
                            howManyTimes = RPGBuilder::howManyTimes(currAct, startState);

                            varToAffect = currEff.fluentIndex;
                            isAssignment = currEff.isAssignment;
                        }

                        if (updateDebug) cout << "\t\tEffect upon " << *(RPGBuilder::getPNE(varToAffect)) << ": ";

                        {

                            list<pair<int, ActionFluentModification*> > actuallyChanged;
                            map<double, vector<double>, EpsilonComp >::iterator flItr = fluentLayers.insert(pair<double, vector<double> >(nlTime, maxFluents)).first;

                            if (isAssignment) {
                                map<int, list<ActionFluentModification> > & afMap = fluentModifications[factLayerTime];

                                if (updateDebug) cout << " = " << signedEffect << "\n";

                                for (int pass = 0; pass < 2; ++pass) {
                                    ActionFluentModification afm(currAct, depItr->second, false, signedEffect, howManyTimes, true);
                                    list<ActionFluentModification> & afmList = afMap[varToAffect];
                                    afmList.push_back(afm);

                                    double & oldValue = flItr->second[varToAffect];

                                    if (updateDebug) {
                                        if (!pass) {
                                            if (signedEffect == DBL_MAX) {
                                                cout << "assign to inf";
                                            } else {
                                                cout << "assign to " << signedEffect;
                                            }
                                        }
                                        if (pass) {
                                            if (oldValue == DBL_MAX) {
                                                cout << ", old minimum value of -inf\n";
                                            } else {
                                                cout << ", old minimum value of -" << oldValue << "\n";
                                            }
                                        } else {
                                            if (oldValue == DBL_MAX) {
                                                cout << ", old max value of inf";
                                            } else {
                                                cout << ", old max value of " << oldValue;
                                            }
                                            cout.flush();
                                        }
                                    }

                                    if (oldValue < signedEffect) {
                                        oldValue = signedEffect;
                                        actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                                    }

                                    varToAffect += RPGBuilder::getPNECount(); // for pass 2
                                }

                            } else if (signedEffect != 0.0) {
                                double effectMagnitude = signedEffect;
                                if (signedEffect < 0.0) {
                                    effectMagnitude = 0.0 - effectMagnitude;
                                    varToAffect += RPGBuilder::getPNECount();
                                    if (updateDebug) cout << "-";
                                }
                                if (updateDebug) {
                                    if (effectMagnitude == DBL_MAX) {
                                        cout << "infinite\n";
                                    } else {
                                        cout << effectMagnitude << "\n";
                                    }
                                }

                                ActionFluentModification afm(currAct, depItr->second, false, effectMagnitude, howManyTimes, false);

                                map<int, list<ActionFluentModification> > & afMap = fluentModifications[factLayerTime];

                                list<ActionFluentModification> & afmList = afMap[varToAffect];
                                afmList.push_back(afm);



                                if (updateDebug) {
                                    cout << "\t\t\tCan be done " << howManyTimes << " times\n";
                                }

                                if (effectMagnitude == DBL_MAX || howManyTimes == INT_MAX) {
                                    double & oldValue = flItr->second[varToAffect];
                                    if (oldValue != DBL_MAX) {
                                        flItr->second[varToAffect] = DBL_MAX; // incrementing by infinity or assigning infinity - as if it matters....
                                        actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                                    }
                                } else {
                                    const double overallChange = howManyTimes * effectMagnitude;
                                    double & oldValue = flItr->second[varToAffect];
                                    if (oldValue != DBL_MAX) {
                                        oldValue += overallChange;
                                        actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                                    }
                                }
                            } else {
                                if (updateDebug) cout << "0.0\n";
                            }

                            list<pair<int, ActionFluentModification*> >::iterator acItr = actuallyChanged.begin();
                            const list<pair<int, ActionFluentModification*> >::iterator acEnd = actuallyChanged.end();

                            for (; acItr != acEnd; ++acItr) {

                                {
                                    list<int> & affectedPres = RPGBuilder::affectsRPGNumericPreconditions(acItr->first);

                                    list<int>::iterator apItr = affectedPres.begin();
                                    const list<int>::iterator apEnd = affectedPres.end();

                                    for (; apItr != apEnd; ++apItr) {

                                        const int currEff = *apItr;
                                        double & currAIL = (*numericAchievedInLayer)[currEff];
                                        if (currAIL == -1.0) {
                                            assert(!((*numericAchievedBy)[currEff]));
                                            if ((*rpgNumericPreconditions)[currEff].isSatisfied(flItr->second)) {
                                                currAIL = nlTime;
                                                (*numericAchievedBy)[currEff] = acItr->second;
                                                factLayer[nlTime].second.push_back(currEff);
                                                if (goalFluents.find(currEff) != gfEnd) {
                                                    if (!(--unsatisfiedGoals) && !unappearedEnds) return true;
                                                }
                                            }
                                        }

                                    }

                                }



                                {
                                    list<int> & affectedAVs = RPGBuilder::getVariableDependencies(acItr->first);

                                    list<int>::iterator aavItr = affectedAVs.begin();
                                    const list<int>::iterator aavEnd = affectedAVs.end();

                                    for (; aavItr != aavEnd; ++aavItr) {

                                        double & oldVal = flItr->second[*aavItr];
                                        const double newVal = RPGBuilder::getArtificialVariable(*aavItr).evaluate(flItr->second);

                                        if (newVal > oldVal) {

                                            oldVal = newVal;

                                            list<int> & affectedPres = RPGBuilder::affectsRPGNumericPreconditions(*aavItr);

                                            list<int>::iterator apItr = affectedPres.begin();
                                            const list<int>::iterator apEnd = affectedPres.end();

                                            for (; apItr != apEnd; ++apItr) {

                                                const int currEff = *apItr;
                                                double & currAIL = (*numericAchievedInLayer)[currEff];
                                                if (currAIL == -1.0) {
                                                    assert(!((*numericAchievedBy)[currEff]));
                                                    if ((*rpgNumericPreconditions)[currEff].isSatisfied(flItr->second)) {
                                                        currAIL = nlTime;
                                                        (*numericAchievedBy)[currEff] = acItr->second;
                                                        factLayer[nlTime].second.push_back(currEff);
                                                        if (goalFluents.find(currEff) != gfEnd) {
                                                            if (!(--unsatisfiedGoals) && !unappearedEnds) return true;
                                                        }
                                                    }
                                                }

                                            }

                                        }

                                    }
                                }
                            }

                        }

                        if (numPass) ++contChangeItr;
                    }

                }

                {
                    const double endTime = factLayerTime + RPGBuilder::getOpMinDuration(currAct, 0);

                    startActionSchedule[currAct] = factLayerTime;
                    endActionSchedule[currAct] = endTime;

                    if (!endPreconditionCounts[currAct]) {
                        if (updateDebug) cout << "\t\tEnd of action can be put at fact layer sufficiently far in future (" << endTime << ")\n";

                        endActionsAtTime[endTime].push_back(currAct);
                    }
                }

                assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));

            } else {
                if (updateDebug) cout << "\tStart of action " << currAct << " now only has " << startPreconditionCounts[currAct] << " unsatisfied propositional preconditions\n";
            }
        } else {
            if (toPropagate >= 0) --numericEndPreconditionCounts[currAct];
            if (!numericEndPreconditionCounts[currAct] && !endPreconditionCounts[currAct] && forbiddenEnd.find(currAct) == forbiddenEnd.end() && nlTime < latestEndAllowed[currAct]) {
                assert(!RPGBuilder::rogueActions[currAct]);
                if (updateDebug) cout << "\tEnd of action " << currAct << " is now applicable\n";

                assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));

                if (expandFully) earliestEndAllowed[currAct] = factLayerTime;

                bool insistOnThisEnd = (insistUponEnds.find(currAct) != insistUponEnds.end());
                if (insistOnThisEnd) {
                    if (factLayerTime >= openEndActionSchedule[currAct]) {
                        openEndActionSchedule[currAct] = factLayerTime;
                        if (!(--unappearedEnds) && !unsatisfiedGoals) return true;
                        if (updateDebug) cout << "\tNow down to " << unappearedEnds << "\n";
                    } else {
                        if (updateDebug) cout << "\tOpen end cannot appear quite yet\n";
                        insistOnThisEnd = false;
                        endActionsAtTime[openEndActionSchedule[currAct]].push_back(-1 - currAct);
                        if (!(--unappearedEnds) && !unsatisfiedGoals) return true;
                        if (updateDebug) cout << "\tNow down to " << unappearedEnds << "\n";
                    }
                }

                for (int epPass = 0; epPass < 2; ++epPass) {
                    bool doLoop = false;
                    bool limitToOnce = false;
                    if (!epPass) {
                        doLoop = insistOnThisEnd;
                        limitToOnce = true;
                    } else {
                        if (!startPreconditionCounts[currAct] && !numericStartPreconditionCounts[currAct] && startActionSchedule[currAct] > -1.0) {
                            const double enforcedEnd = endActionSchedule[currAct];
                            if (factLayerTime >= enforcedEnd) {
                                endActionSchedule[currAct] = factLayerTime;
                                doLoop = true;
                            } else {
                                endActionsAtTime[enforcedEnd].push_back(currAct);
                            }
                        }
                    }

                    if (doLoop) {
                        if (updateDebug) {
                            if (limitToOnce) {
                                cout << "\tStarted before RPG\n";
                            } else {
                                cout << "\tStart is sufficiently earlier\n";
                            }
                        }

                        MAKEENDMDI(POtime, (limitToOnce ? payload->earliestStartOf[currAct] : 0.0), RPGBuilder::getOpMinDuration(currAct, 0), currAct);

                        double actCostTmp = -1.0;
                        
                        if (applyPropositionalEffects(payload, currAct, actCostTmp, VAL::E_AT_END, nlTime, POtime)) return true;

                        list<int> & numericEffects = (*actionsToRPGNumericEndEffects)[currAct];

                        list<int>::iterator numEffItr = numericEffects.begin();
                        const list<int>::iterator numEffEnd = numericEffects.end();

                        for (; numEffItr != numEffEnd; ++numEffItr) {
                            RPGBuilder::RPGNumericEffect & currEff = (*rpgNumericEffects)[*numEffItr];

                            const double signedEffect = currEff.evaluate(maxFluents, RPGBuilder::getOpMinDuration(currAct, 0), RPGBuilder::getOpMaxDuration(currAct, 0));
                            int howManyTimes = 0;
                            if (limitToOnce) {
                                const map<int, set<int> >::const_iterator saItr = startedActions.find(currAct);
                                if (saItr != startedActions.end()) howManyTimes += saItr->second.size();
                            } else {
                                howManyTimes += RPGBuilder::howManyTimes(currAct, startState);
                            }

                            int varToAffect = currEff.fluentIndex;

                            if (updateDebug) cout << "\t\tEffect upon " << *(RPGBuilder::getPNE(varToAffect)) << ": ";

                            {

                                list<pair<int, ActionFluentModification*> > actuallyChanged;
                                map<double, vector<double>, EpsilonComp >::iterator flItr = fluentLayers.insert(pair<double, vector<double> >(nlTime, maxFluents)).first;

                                if (currEff.isAssignment) {
                                    map<int, list<ActionFluentModification> > & afMap = fluentModifications[factLayerTime];

                                    if (updateDebug) cout << "= " << signedEffect << "\n";
                                    for (int pass = 0; pass < 2; ++pass) {
                                        ActionFluentModification afm(currAct, depItr->second, limitToOnce, signedEffect, howManyTimes, true);
                                        list<ActionFluentModification> & afmList = afMap[varToAffect];
                                        afmList.push_back(afm);

                                        double & oldValue = flItr->second[varToAffect];

                                        if (updateDebug) {
                                            if (!pass) {
                                                if (signedEffect == DBL_MAX) {
                                                    cout << "assign to inf";
                                                } else {
                                                    cout << "assign to " << signedEffect;
                                                }
                                            }
                                            if (pass) {
                                                if (oldValue == DBL_MAX) {
                                                    cout << ", old minimum value of -inf\n";
                                                } else {
                                                    cout << ", old minimum value of -" << oldValue << "\n";
                                                }
                                            } else {
                                                if (oldValue == DBL_MAX) {
                                                    cout << ", old max value of inf";
                                                } else {
                                                    cout << ", old max value of " << oldValue;
                                                }
                                                cout.flush();
                                            }
                                        }

                                        if (oldValue < signedEffect) {

                                            oldValue = signedEffect;
                                            actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                                        }

                                        varToAffect += RPGBuilder::getPNECount(); // for pass 2
                                    }

                                } else if (signedEffect != 0.0) {
                                    double effectMagnitude = signedEffect;
                                    if (signedEffect < 0.0) {
                                        effectMagnitude = 0.0 - effectMagnitude;
                                        varToAffect += RPGBuilder::getPNECount();
                                        if (updateDebug) cout << "-";
                                    }
                                    if (updateDebug) {
                                        if (effectMagnitude == DBL_MAX) {
                                            cout << "infinite\n";
                                        } else {
                                            cout << effectMagnitude << "\n";
                                        }
                                    }

                                    ActionFluentModification afm(currAct, depItr->second, limitToOnce, effectMagnitude, howManyTimes, false);

                                    map<int, list<ActionFluentModification> > & afMap = fluentModifications[factLayerTime];

                                    list<ActionFluentModification> & afmList = afMap[varToAffect];
                                    afmList.push_back(afm);



                                    if (updateDebug) {
                                        cout << "\t\t\tCan be done " << howManyTimes << " times\n";
                                    }

                                    if (effectMagnitude == DBL_MAX || howManyTimes == INT_MAX) {
                                        double & oldValue = flItr->second[varToAffect];
                                        if (oldValue != DBL_MAX) {
                                            flItr->second[varToAffect] = DBL_MAX; // incrementing by infinity or assigning infinity - as if it matters....
                                            actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                                        }
                                    } else {
                                        const double overallChange = howManyTimes * effectMagnitude;
                                        double & oldValue = flItr->second[varToAffect];
                                        if (oldValue != DBL_MAX) {
                                            oldValue += overallChange;
                                            actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                                        }
                                    }
                                } else {
                                    if (updateDebug) cout << "0.0\n";
                                }

                                list<pair<int, ActionFluentModification*> >::iterator acItr = actuallyChanged.begin();
                                const list<pair<int, ActionFluentModification*> >::iterator acEnd = actuallyChanged.end();

                                for (; acItr != acEnd; ++acItr) {

                                    {
                                        list<int> & affectedPres = RPGBuilder::affectsRPGNumericPreconditions(acItr->first);

                                        list<int>::iterator apItr = affectedPres.begin();
                                        const list<int>::iterator apEnd = affectedPres.end();

                                        for (; apItr != apEnd; ++apItr) {

                                            const int currEff = *apItr;
                                            double & currAIL = (*numericAchievedInLayer)[currEff];
                                            if (currAIL == -1.0) {
                                                assert(!((*numericAchievedBy)[currEff]));
                                                if ((*rpgNumericPreconditions)[currEff].isSatisfied(flItr->second)) {
                                                    currAIL = nlTime;
                                                    (*numericAchievedBy)[currEff] = acItr->second;
                                                    factLayer[nlTime].second.push_back(currEff);
                                                    if (goalFluents.find(currEff) != gfEnd) {
                                                        if (!(--unsatisfiedGoals) && !unappearedEnds) return true;
                                                    }
                                                }
                                            }

                                        }

                                    }


                                    {
                                        list<int> & affectedAVs = RPGBuilder::getVariableDependencies(acItr->first);

                                        list<int>::iterator aavItr = affectedAVs.begin();
                                        const list<int>::iterator aavEnd = affectedAVs.end();

                                        for (; aavItr != aavEnd; ++aavItr) {

                                            double & oldVal = flItr->second[*aavItr];
                                            const double newVal = RPGBuilder::getArtificialVariable(*aavItr).evaluate(flItr->second);

                                            if (newVal > oldVal) {

                                                oldVal = newVal;

                                                list<int> & affectedPres = RPGBuilder::affectsRPGNumericPreconditions(*aavItr);

                                                list<int>::iterator apItr = affectedPres.begin();
                                                const list<int>::iterator apEnd = affectedPres.end();

                                                for (; apItr != apEnd; ++apItr) {

                                                    const int currEff = *apItr;
                                                    double & currAIL = (*numericAchievedInLayer)[currEff];
                                                    if (currAIL == -1.0) {
                                                        assert(!((*numericAchievedBy)[currEff]));
                                                        if ((*rpgNumericPreconditions)[currEff].isSatisfied(flItr->second)) {
                                                            currAIL = nlTime;
                                                            (*numericAchievedBy)[currEff] = acItr->second;
                                                            factLayer[nlTime].second.push_back(currEff);
                                                            if (goalFluents.find(currEff) != gfEnd) {
                                                                if (!(--unsatisfiedGoals) && !unappearedEnds) return true;
                                                            }
                                                        }
                                                    }

                                                }

                                            }

                                        }
                                    }
                                }

                            }



                        }



                    }

                }

                assert(checkPreconditionsAreSatisfied(depItr->first, depItr->second, nlTime));

            } else {

                if (updateDebug) {
                    cout << "\tEnd of action " << currAct;
                    cout << " now only has " << endPreconditionCounts[currAct] << " unsatisfied preconditions\n";
                }
            }
        }
    }

    return false;

};

bool RPGHeuristic::Private::applyEndEffectNow(Private::BuildingPayload * const payload, const int & currAct, const bool & openEnd, const double & factLayerTime)
{

    static const double EPSILON = 0.001;

    const MinimalState & startState = payload->startState;
    map<double, FactLayerEntry, EpsilonComp > & factLayer = payload->factLayers;
    map<double, vector<double>, EpsilonComp > & fluentLayers = payload->fluentLayers;
    map<double, map<int, list<ActionFluentModification> >, EpsilonComp> & fluentModifications = payload->fluentModifications;
    const map<int, set<int> > & startedActions = payload->startedActions;

    int & unsatisfiedGoals = payload->unsatisfiedGoals;
    int & unappearedEnds = payload->unappearedEnds;


    const double nlTime = factLayerTime + EPSILON;
    const bool updateDebug = Globals::globalVerbosity & 64;


    if (updateDebug) cout << "\tEnd of action " << currAct << " is now applicable\n";

    assert(checkPreconditionsAreSatisfied(currAct, VAL::E_AT_END, nlTime));

    if (expandFully) earliestEndAllowed[currAct] = factLayerTime;

    const vector<double> & maxFluents = fluentLayers[factLayerTime];

    MAKEENDMDI(POtime, (openEnd ? payload->earliestStartOf[currAct] : 0.0), RPGBuilder::getOpMinDuration(currAct, 0), currAct);

    double actCostTmp = -1.0;
    
    if (applyPropositionalEffects(payload, currAct, actCostTmp, VAL::E_AT_END, nlTime, POtime)) return true;


    list<int> & numericEffects = (*actionsToRPGNumericEndEffects)[currAct];

    list<int>::iterator numEffItr = numericEffects.begin();
    const list<int>::iterator numEffEnd = numericEffects.end();

    for (; numEffItr != numEffEnd; ++numEffItr) {
        RPGBuilder::RPGNumericEffect & currEff = (*rpgNumericEffects)[*numEffItr];

        const double signedEffect = currEff.evaluate(maxFluents, RPGBuilder::getOpMinDuration(currAct, 0), RPGBuilder::getOpMaxDuration(currAct, 0));
        int howManyTimes = 0;

        if (openEnd) {
//HACK
            const map<int, set<int> >::const_iterator saItr = startedActions.find(currAct);
            if (saItr != startedActions.end()) howManyTimes = saItr->second.size();
        } else {
            howManyTimes = RPGBuilder::howManyTimes(currAct, startState);
        }

        int varToAffect = currEff.fluentIndex;

        if (updateDebug) cout << "\t\tEffect upon " << *(RPGBuilder::getPNE(varToAffect)) << ": ";

        {

            list<pair<int, ActionFluentModification*> > actuallyChanged;
            map<double, vector<double>, EpsilonComp >::iterator flItr = fluentLayers.insert(pair<double, vector<double> >(nlTime, maxFluents)).first;

            if (currEff.isAssignment) {

                if (updateDebug) cout << " = " << signedEffect << "\n";

                map<int, list<ActionFluentModification> > & afMap = fluentModifications[factLayerTime];


                for (int pass = 0; pass < 2; ++pass) {
                    ActionFluentModification afm(currAct, VAL::E_AT_END, openEnd, signedEffect, howManyTimes, true);
                    list<ActionFluentModification> & afmList = afMap[varToAffect];
                    afmList.push_back(afm);

                    double & oldValue = flItr->second[varToAffect];

                    if (updateDebug) {
                        if (!pass) {
                            if (signedEffect == DBL_MAX) {
                                cout << "assign to inf";
                            } else {
                                cout << "assign to " << signedEffect;
                            }
                        }
                        if (pass) {
                            if (oldValue == DBL_MAX) {
                                cout << ", old minimum value of -inf\n";
                            } else {
                                cout << ", old minimum value of -" << oldValue << "\n";
                            }
                        } else {
                            if (oldValue == DBL_MAX) {
                                cout << ", old max value of inf";
                            } else {
                                cout << ", old max value of " << oldValue;
                            }
                            cout.flush();
                        }
                    }

                    if (oldValue < signedEffect) {

                        oldValue = signedEffect;
                        actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                    }

                    varToAffect += RPGBuilder::getPNECount(); // for pass 2
                }

            } else if (signedEffect != 0.0) {
                double effectMagnitude = signedEffect;
                if (signedEffect < 0.0) {
                    effectMagnitude = 0.0 - effectMagnitude;
                    varToAffect += RPGBuilder::getPNECount();
                    if (updateDebug) cout << "-";
                }
                if (updateDebug) {
                    if (effectMagnitude == DBL_MAX) {
                        cout << "infinite\n";
                    } else {
                        cout << effectMagnitude << "\n";
                    }
                }

                ActionFluentModification afm(currAct, VAL::E_AT_END, openEnd, effectMagnitude, howManyTimes, false);

                map<int, list<ActionFluentModification> > & afMap = fluentModifications[factLayerTime];

                list<ActionFluentModification> & afmList = afMap[varToAffect];
                afmList.push_back(afm);



                if (updateDebug) {
                    cout << "\t\t\tCan be done " << howManyTimes << " times\n";
                }

                if (effectMagnitude == DBL_MAX || howManyTimes == INT_MAX) {
                    double & oldValue = flItr->second[varToAffect];
                    if (oldValue != DBL_MAX) {
                        flItr->second[varToAffect] = DBL_MAX; // incrementing by infinity or assigning infinity - as if it matters....
                        actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                    }
                } else {
                    const double overallChange = howManyTimes * effectMagnitude;
                    double & oldValue = flItr->second[varToAffect];
                    if (oldValue != DBL_MAX) {
                        oldValue += overallChange;
                        actuallyChanged.push_back(pair<int, ActionFluentModification*>(varToAffect, &(afmList.back())));
                    }
                }
            } else {
                if (updateDebug) cout << "0.0\n";
            }

            list<pair<int, ActionFluentModification*> >::iterator acItr = actuallyChanged.begin();
            const list<pair<int, ActionFluentModification*> >::iterator acEnd = actuallyChanged.end();

            for (; acItr != acEnd; ++acItr) {

                {
                    list<int> & affectedPres = RPGBuilder::affectsRPGNumericPreconditions(acItr->first);

                    list<int>::iterator apItr = affectedPres.begin();
                    const list<int>::iterator apEnd = affectedPres.end();

                    for (; apItr != apEnd; ++apItr) {

                        const int currEff = *apItr;
                        double & currAIL = (*numericAchievedInLayer)[currEff];
                        if (currAIL == -1.0) {
                            assert(!((*numericAchievedBy)[currEff]));
                            if ((*rpgNumericPreconditions)[currEff].isSatisfied(flItr->second)) {
                                currAIL = nlTime;
                                (*numericAchievedBy)[currEff] = acItr->second;
                                factLayer[nlTime].second.push_back(currEff);
                                if (goalFluents.find(currEff) != gfEnd) {
                                    if (!(--unsatisfiedGoals) && !unappearedEnds) return true;
                                }
                            }
                        }

                    }

                }


                {
                    list<int> & affectedAVs = RPGBuilder::getVariableDependencies(acItr->first);

                    list<int>::iterator aavItr = affectedAVs.begin();
                    const list<int>::iterator aavEnd = affectedAVs.end();

                    for (; aavItr != aavEnd; ++aavItr) {

                        double & oldVal = flItr->second[*aavItr];
                        const double newVal = RPGBuilder::getArtificialVariable(*aavItr).evaluate(flItr->second);

                        if (newVal > oldVal) {

                            oldVal = newVal;

                            list<int> & affectedPres = RPGBuilder::affectsRPGNumericPreconditions(*aavItr);

                            list<int>::iterator apItr = affectedPres.begin();
                            const list<int>::iterator apEnd = affectedPres.end();

                            for (; apItr != apEnd; ++apItr) {

                                const int currEff = *apItr;
                                double & currAIL = (*numericAchievedInLayer)[currEff];
                                if (currAIL == -1.0) {
                                    assert(!((*numericAchievedBy)[currEff]));
                                    if ((*rpgNumericPreconditions)[currEff].isSatisfied(flItr->second)) {
                                        currAIL = nlTime;
                                        (*numericAchievedBy)[currEff] = acItr->second;
                                        factLayer[nlTime].second.push_back(currEff);
                                        if (goalFluents.find(currEff) != gfEnd) {
                                            if (!(--unsatisfiedGoals) && !unappearedEnds) return true;
                                        }
                                    }
                                }

                            }

                        }

                    }
                }
            }

        }



    }

    assert(checkPreconditionsAreSatisfied(currAct, VAL::E_AT_END, nlTime));

    return false;

};

void RPGHeuristic::findApplicableActions(const MinimalState & theState, const double & stateTime, list<ActionSegment> & applicableActions)
{
    d->findApplicableActions(theState, stateTime, applicableActions);
}

bool negativesAreOkay(const list<Literal*> & checkFor, const map<int, PropositionAnnotation> & checkIn)
{

    list<Literal*>::const_iterator cfItr = checkFor.begin();
    const list<Literal*>::const_iterator cfEnd = checkFor.end();

    for (; cfItr != cfEnd; ++cfItr) {
        if (checkIn.find((*cfItr)->getID()) != checkIn.end()) return false;
    }

    return true;
}

void RPGHeuristic::Private::findApplicableActions(const MinimalState & theState, const double & stateTime, list<ActionSegment> & applicableActions)
{

    static const bool debug = (Globals::globalVerbosity & 16);


    static const set<int> emptyIntSet;


    vector<int> startPreconditionCounts(*initialUnsatisfiedProcessedStartPreconditions);
    vector<int> endPreconditionCounts(*initialUnsatisfiedEndPreconditions);

    list<ActionSegment> toFilter;

    for (int pass = 0; pass < 2; ++pass) {
        list<pair<int, VAL::time_spec> > & dependents = (pass ? *onlyNumericPreconditionActions : *preconditionlessActions);

        if (debug) {
            if (pass) {
                cout << "Considering the " << dependents.size() << " actions with only numeric preconditions\n";
            } else {
                cout << "Considering the " << dependents.size() << " propositionally preconditionless actions\n";
            }
        }


        list<pair<int, VAL::time_spec> >::iterator depItr = dependents.begin();
        const list<pair<int, VAL::time_spec> >::iterator depEnd = dependents.end();

        for (; depItr != depEnd; ++depItr) {
            const int currAct = depItr->first;
            const VAL::time_spec startOrEnd = depItr->second;

            if (!negativesAreOkay((startOrEnd == VAL::E_AT_START
                                   ? RPGBuilder::getProcessedStartNegativePropositionalPreconditions()[currAct]
                                   : RPGBuilder::getEndNegativePropositionalPreconditions()[currAct]) ,
                                  theState.first)) {
                continue;
            }

            ActionSegment candidate(getOp(currAct), startOrEnd, -1, emptyIntSet);
            const bool nonMutex = RPGBuilder::stepNeedsToHaveFinished(candidate, theState, candidate.needToFinish);

            if (nonMutex) {
                toFilter.push_back(candidate);
                if (debug) {
                    cout << *(getOp(currAct));
                    if (startOrEnd == VAL::E_AT_START) {
                        cout << " start is a propositional candidate\n";
                    } else {
                        cout << " end is a propositional candidate\n";
                    }
                }
            } else {
                if (debug) {
                    cout << *(getOp(currAct));
                    if (startOrEnd == VAL::E_AT_START) {
                        cout << " start breaks a propositional invariant\n";
                    } else {
                        cout << " end breaks a propositional invariant\n";
                    }
                }
            }

        }
    }

    {
        map<int, PropositionAnnotation>::const_iterator stateItr = theState.first.begin();
        const map<int, PropositionAnnotation>::const_iterator stateEnd = theState.first.end();

        for (; stateItr != stateEnd; ++stateItr) {

            list<pair<int, VAL::time_spec> > & dependents = (*processedPreconditionsToActions)[stateItr->first];

            list<pair<int, VAL::time_spec> >::const_iterator depItr = dependents.begin();
            const list<pair<int, VAL::time_spec> >::const_iterator depEnd = dependents.end();

            for (; depItr != depEnd; ++depItr) {
                const int currAct = depItr->first;
                const VAL::time_spec startOrEnd = depItr->second;
                int & toManipulate = (startOrEnd == VAL::E_AT_START ? startPreconditionCounts[currAct] : endPreconditionCounts[currAct]);

                if (!(--toManipulate)) {

                    if (negativesAreOkay((startOrEnd == VAL::E_AT_START
                                          ? RPGBuilder::getProcessedStartNegativePropositionalPreconditions()[currAct]
                                          : RPGBuilder::getEndNegativePropositionalPreconditions()[currAct]) ,
                                         theState.first)) {
                        ActionSegment candidate(getOp(currAct), startOrEnd, -1, emptyIntSet);
                        const bool nonMutex = RPGBuilder::stepNeedsToHaveFinished(candidate, theState, candidate.needToFinish);

                        if (nonMutex) toFilter.push_back(candidate);
                    }
                }
            }
        }

    }



    list<ActionSegment>::iterator fItr = toFilter.begin();
    const list<ActionSegment>::iterator fEnd = toFilter.end();

    for (; fItr != fEnd; ++fItr) {
        /*const pair<int, VAL::time_spec> currAct = *fItr;
        list<RPGBuilder::NumericPrecondition> & currList = (*actionsToNumericPreconditions)[currAct->first];
        bool isApplicable = true;
        list<RPGBuilder::NumericPrecondition>::iterator npItr = currList.begin();
        const list<RPGBuilder::NumericPrecondition>::iterator npEnd = currList.end();

        for (; npItr != npEnd; ++npItr) {
        if (!npItr->isSatisfied(theState.second)) {
        isApplicable = false;
        break;
        }
        }
        if (isApplicable) {
        applicableActions.push_back(currAct);
        }*/

        bool isApplicable = true;

        if (fItr->second == VAL::E_AT_START) {
            isApplicable = TemporalAnalysis::okayToStart(fItr->first->getID(), stateTime);
        } else if (fItr->second == VAL::E_AT_END) {
            isApplicable = TemporalAnalysis::okayToEnd(fItr->first->getID(), stateTime);
        }

        vector<list<int> > * const toQuery = (fItr->second == VAL::E_AT_START ? actionsToProcessedStartNumericPreconditions : actionsToNumericEndPreconditions);
        if (isApplicable) {

            list<int> & nprecs = (*toQuery)[fItr->first->getID()];

            list<int>::iterator npItr = nprecs.begin();
            const list<int>::iterator npEnd = nprecs.end();

            for (; npItr != npEnd; ++npItr) {

                if (!((*rpgNumericPreconditions)[*npItr]).isSatisfiedWCalculate(theState.secondMin, theState.secondMax)) {
                    if (debug) cout << (*rpgNumericPreconditions)[*npItr] << " isn't satisfied, so " << *(fItr->first) << " isn't applicable\n";
                    isApplicable = false;
                    break;
                }

            }

        }

        /*        if (isApplicable && false) { //HACK
                    vector<list<int> > * const checkEffects = (fItr->second == VAL::E_AT_START ? actionsToRPGNumericStartEffects : actionsToRPGNumericEndEffects);

                    {

                        list<int> & neffs = (*checkEffects)[fItr->first->getID()];

                        list<int>::iterator neItr = neffs.begin();
                        const list<int>::iterator neEnd = neffs.end();

                        for (; neItr != neEnd; ++neItr) {

                            if (theState.fluentInvariants.find(((*rpgNumericEffects)[*neItr]).fluentIndex) != theState.fluentInvariants.end()) {
                                isApplicable = false;
                                break;
                            }

                        }


                    }

                }*/

        if (isApplicable) {


            map<int, set<int> >::const_iterator saOneItr = theState.startedActions.find(fItr->first->getID());
            if (fItr->second == VAL::E_AT_START) {
                if (!RPGBuilder::noSelfOverlaps || saOneItr == theState.startedActions.end()) {
                    if (RPGBuilder::isInteresting(fItr->first->getID(), theState.first, theState.startedActions)) {
                        applicableActions.push_back(*fItr);
                    }
                }
            } else {
                if (saOneItr != theState.startedActions.end()) {
                    applicableActions.push_back(*fItr);
                }
            }
        }
    }

    /*{
        map<int, map<int,int> >::iterator saItr = theState.startedActions.begin();
        const map<int, map<int,int> >::iterator saEnd = theState.startedActions.end();
        for (; saItr != saEnd; ++saItr) {
            RPGBuilder::LinearEffects * const lEffs = RPGBuilder::getLinearDiscretisation()[saItr->first];
            if (lEffs) {
                const int lLim = lEffs->divisions - 1;
                map<int,int>::iterator saOneItr = saItr->second.begin();
                const map<int,int>::iterator saOneEnd = saItr->second.end();

                for (; saOneItr != saOneEnd && saOneItr->first < lLim; ++saOneItr) {
                    applicableActions.push_back(ActionSegment(getOp(saItr->first), VAL::E_OVER_ALL, saOneItr->first, emptyIntList));
                }
            }
        }
    }*/

    {

        const int nextTIL = theState.nextTIL;
        static list<RPGBuilder::FakeTILAction> & tilActs = RPGBuilder::getTILs();
        static const list<RPGBuilder::FakeTILAction>::iterator tilEnd = tilActs.end();

        list<RPGBuilder::FakeTILAction>::iterator tilItr = tilActs.begin();

        int i = 0;

        for (; i < nextTIL; ++i, ++tilItr);

        for (; i == nextTIL && tilItr != tilEnd; ++tilItr, ++i) {

            ActionSegment candidate(0, VAL::E_AT, i, emptyIntSet);

            const bool isApplicable = RPGBuilder::stepNeedsToHaveFinished(candidate, theState, candidate.needToFinish);

            if (isApplicable) {
                //cout << "TIL " << i << " is applicable\n";
                applicableActions.push_back(candidate);
            } else {
                break;
            }

        }

    }

};


bool RPGHeuristic::testApplicability(const MinimalState & theState, const double & stateTime, const ActionSegment & actID, bool fail, bool ignoreDeletes)
{
    return d->testApplicability(theState, stateTime, actID, fail, ignoreDeletes);
}

bool RPGHeuristic::Private::testApplicability(const MinimalState & theState, const double & stateTime, const ActionSegment & actID, const bool & fail, const bool & ignoreDeletes)
{

    if (actID.second == VAL::E_AT_START) {

        if (RPGBuilder::noSelfOverlaps) {

            if (fail) {
                assert(theState.startedActions.find(actID.first->getID()) == theState.startedActions.end());
            } else {
                if (theState.startedActions.find(actID.first->getID()) != theState.startedActions.end()) return false;
            }
        }


        if (fail) {
            assert(TemporalAnalysis::okayToStart(actID.first->getID(), stateTime));
        } else {
            if (!TemporalAnalysis::okayToStart(actID.first->getID(), stateTime)) return false;
        }

        {
            set<int> ntf;
            const bool passesPropositional = RPGBuilder::stepNeedsToHaveFinished(actID, theState, ntf);
            if (!passesPropositional) {
                if (fail) {
                    assert(false);
                } else {
                    return false;
                }
            }
        }

        if (!negativesAreOkay(RPGBuilder::getProcessedStartNegativePropositionalPreconditions()[actID.first->getID()],
                              theState.first)) {
            if (fail) {
                assert(false);
            } else {
                return false;
            }
        }


        if (!ignoreDeletes) {

            list<int> & checkFor = (*actionsToProcessedStartNumericPreconditions)[actID.first->getID()];
            list<int>::iterator fItr = checkFor.begin();
            const list<int>::iterator fEnd = checkFor.end();

            for (; fItr != fEnd; ++fItr) {

                if (!((*rpgNumericPreconditions)[*fItr]).isSatisfiedWCalculate(theState.secondMin, theState.secondMax)) {
                    if (fail) {
                        assert(false);
                    } else {
                        return false;
                    }
                }

            }
        }



    } else if (actID.second == VAL::E_AT_END) {


        if (fail) {
            assert(TemporalAnalysis::okayToEnd(actID.first->getID(), stateTime));
        } else {
            if (!TemporalAnalysis::okayToEnd(actID.first->getID(), stateTime)) return false;
        }


        {
            set<int> ntf;
            const bool passesPropositional = RPGBuilder::stepNeedsToHaveFinished(actID, theState, ntf);
            if (!passesPropositional) {
                if (fail) {
                    assert(false);
                } else {
                    return false;
                }
            }
        }

        if (!negativesAreOkay(RPGBuilder::getEndNegativePropositionalPreconditions()[actID.first->getID()],
                              theState.first)) {
            if (fail) {
                assert(false);
            } else {
                return false;
            }
        }


        if (!ignoreDeletes) {

            list<int> & checkFor = (*actionsToNumericEndPreconditions)[actID.first->getID()];
            list<int>::iterator fItr = checkFor.begin();
            const list<int>::iterator fEnd = checkFor.end();



            for (; fItr != fEnd; ++fItr) {

                if (!((*rpgNumericPreconditions)[*fItr]).isSatisfiedWCalculate(theState.secondMin, theState.secondMax)) {
                    if (fail) {
                        assert(false);
                    } else {
                        return false;
                    }
                }

            }
        }


    } else if (actID.second != VAL::E_AT) {
        assert(false);

    } else { // til action

        const int & nextTIL = theState.nextTIL;

        if (fail) {
            if (actID.divisionID < nextTIL) {
                cout << "Trying to apply " << actID.divisionID << ", but next one is " << nextTIL << "\n";
            }
            assert(actID.divisionID >= nextTIL);
        } else {
            if (actID.divisionID < nextTIL) return false;
        }

        static list<RPGBuilder::FakeTILAction> & tilActs = RPGBuilder::getTILs();
        static const list<RPGBuilder::FakeTILAction>::iterator tilEnd = tilActs.end();

        list<RPGBuilder::FakeTILAction>::iterator tilItr = tilActs.begin();

        int i = 0;

        static const set<int> emptyIntSet;

        for (; i < nextTIL; ++i, ++tilItr);

        for (; i <= actID.divisionID; ++tilItr, ++i) {
            if (tilItr == tilEnd) {
                cout << "Error: " << i << " TILs in the domain, but trying to access the one with index " << i << "\n";
                assert(tilItr != tilEnd);
            }

            ActionSegment candidate(actID.first, VAL::E_AT, i, emptyIntSet);

            const bool passesPropositional = RPGBuilder::stepNeedsToHaveFinished(candidate, theState, candidate.needToFinish);
            if (!passesPropositional) {
                if (fail) {
                    assert(false);
                } else {
                    return false;
                }
            }

        }


    }

    return true;


}
void RPGHeuristic::filterApplicableActions(const MinimalState & theState, const double & stateTime, list<ActionSegment> & applicableActions)
{
    return d->filterApplicableActions(theState, stateTime, applicableActions);
}

void RPGHeuristic::Private::filterApplicableActions(const MinimalState & theState, const double & stateTime, list<ActionSegment> & applicableActions)
{


    const bool filterDebug = false;

    if (filterDebug) cout << "Filtering applicable actions\n";

    list<ActionSegment> toFilter(applicableActions);

    list<ActionSegment> toNumericFilter;

    applicableActions.clear();

    if (filterDebug) cout << "Input consists of " << toFilter.size() << " actions\n";

    list<ActionSegment>::iterator tfItr = toFilter.begin();
    const list<ActionSegment>::iterator tfEnd = toFilter.end();

    /*for (; fItr != fEnd; ++fItr) {
    const int currAct = *fItr;
    list<RPGBuilder::NumericPrecondition> & currList = (*actionsToNumericPreconditions)[currAct];
    bool isApplicable = true;
    list<RPGBuilder::NumericPrecondition>::iterator npItr = currList.begin();
    const list<RPGBuilder::NumericPrecondition>::iterator npEnd = currList.end();

    for (; npItr != npEnd; ++npItr) {
    if (!npItr->isSatisfied(theState.second)) {
    isApplicable = false;
    break;
    }
    }
    if (isApplicable) {
    applicableActions.push_back(currAct);
    }
    }*/

    for (; tfItr != tfEnd; ++tfItr) {


        bool isApplicable = true;

        if (tfItr->second == VAL::E_AT_START) {

            if (!RPGBuilder::noSelfOverlaps || theState.startedActions.find(tfItr->first->getID()) == theState.startedActions.end()) {

                if (filterDebug) cout << "Considering start of " << *(tfItr->first) << "\n";

                isApplicable = RPGBuilder::isInteresting(tfItr->first->getID(), theState.first, theState.startedActions);

                if (isApplicable) isApplicable = TemporalAnalysis::okayToStart(tfItr->first->getID(), stateTime);

                if (isApplicable) isApplicable = RPGBuilder::stepNeedsToHaveFinished(*tfItr, theState, tfItr->needToFinish);

                if (isApplicable) isApplicable = negativesAreOkay(RPGBuilder::getProcessedStartNegativePropositionalPreconditions()[tfItr->first->getID()], theState.first);

            } else {
                isApplicable = false;
            }

        } else if (tfItr->second == VAL::E_AT_END) {
            const map<int, set<int> >::const_iterator saItr = theState.startedActions.find(tfItr->first->getID());
            if (saItr != theState.startedActions.end() && TemporalAnalysis::okayToEnd(tfItr->first->getID(), stateTime)) {
                isApplicable = RPGBuilder::stepNeedsToHaveFinished(*tfItr, theState, tfItr->needToFinish);
            } else {
                isApplicable = false;
            }
            if (isApplicable) isApplicable = negativesAreOkay(RPGBuilder::getEndNegativePropositionalPreconditions()[tfItr->first->getID()], theState.first);

        } else if (tfItr->second != VAL::E_AT) {

            assert(false);
        } else if (tfItr->second == VAL::E_AT) { // TIL action

            static const set<int> emptyIntSet;

            static bool cachedTILs = false;
            static vector<RPGBuilder::FakeTILAction*> tilActs;
            if (!cachedTILs) {
                cachedTILs = true;
                list<RPGBuilder::FakeTILAction> & rpgTILs = RPGBuilder::getTILs();
                tilActs = vector<RPGBuilder::FakeTILAction*>(rpgTILs.size());

                list<RPGBuilder::FakeTILAction>::iterator rItr = rpgTILs.begin();
                const list<RPGBuilder::FakeTILAction>::iterator rEnd = rpgTILs.end();

                for (int i = 0; rItr != rEnd; ++rItr, ++i) {
                    tilActs[i] = &(*rItr);
                }
            }

            const int nextTIL = theState.nextTIL;

            if (tfItr->divisionID >= nextTIL) {

                bool safe = true;

                for (int i = nextTIL; safe && i <= tfItr->divisionID; ++i) {

                    ActionSegment candidate(tfItr->first, VAL::E_AT, i, emptyIntSet);

                    safe = RPGBuilder::stepNeedsToHaveFinished(candidate, theState, tfItr->needToFinish);

                }

                if (safe) {
                    applicableActions.push_back(*tfItr);
                    // skip straight to the destination list - no numeric interactions,
                    // as these are timed literals, not timed fluents
                }

            }
            isApplicable = false;
        } else {
            isApplicable = false;
        }

        if (isApplicable) {
            toNumericFilter.push_back(*tfItr);
        }
    }



    list<ActionSegment>::iterator tnfItr = toNumericFilter.begin();
    const list<ActionSegment>::iterator tnfEnd = toNumericFilter.end();

    for (; tnfItr != tnfEnd; ++tnfItr) {
        bool isApplicable = true;

        if (filterDebug) cout << "Considering numerically " << *(tnfItr->first) << "\n";

        vector<list<int> > * const toQuery = (tnfItr->second == VAL::E_AT_START ? actionsToProcessedStartNumericPreconditions : actionsToNumericEndPreconditions);
        {

            list<int> & nprecs = (*toQuery)[tnfItr->first->getID()];

            list<int>::iterator npItr = nprecs.begin();
            const list<int>::iterator npEnd = nprecs.end();

            for (; npItr != npEnd; ++npItr) {

                if (!((*rpgNumericPreconditions)[*npItr]).isSatisfiedWCalculate(theState.secondMin, theState.secondMax)) {
                    isApplicable = false;
                    break;
                } else {
                    if (filterDebug) cout << "\t" << (*rpgNumericPreconditions)[*npItr] << "satisfied\n";
                }

            }

        }

        if (isApplicable) {
            applicableActions.push_back(*tnfItr);
        }
    }


};


list<instantiatedOp*> * RPGHeuristic::makePlan(list<int> & steps)
{

    list<instantiatedOp*> * toReturn = new list<instantiatedOp*>();

    list<int>::iterator sItr = steps.begin();
    const list<int>::iterator sEnd = steps.end();
    cout << "\n";
    for (; sItr != sEnd; ++sItr) {
        toReturn->push_back(RPGBuilder::getInstantiatedOp(*sItr));
    }

    return toReturn;
}

instantiatedOp* RPGHeuristic::getOp(const int & i)
{

    return RPGBuilder::getInstantiatedOp(i);

};


list<Literal*> & RPGHeuristic::getDeleteEffects(const int & i, const VAL::time_spec & t)
{
    if (t == VAL::E_AT_START) {
        return (*(d->actionsToStartNegativeEffects))[i];
    } else {
        return (*(d->actionsToEndNegativeEffects))[i];
    }
}

list<Literal*> & RPGHeuristic::getAddEffects(const int & i, const VAL::time_spec & t)
{
    if (t == VAL::E_AT_START) {
        return (*(d->actionsToStartEffects))[i];
    } else {
        return (*(d->actionsToEndEffects))[i];
    }
};

list<Literal*> & RPGHeuristic::getPreconditions(const int & i, const VAL::time_spec & t)
{
    if (t == VAL::E_AT_START) {
        return (*(d->actionsToStartPreconditions))[i];
    } else {
        return (*(d->actionsToEndPreconditions))[i];
    }
};


list<int> & RPGHeuristic::getNumericEffects(const int & i, const VAL::time_spec & t)
{
    if (t == VAL::E_AT_START) {
        return (*(d->actionsToRPGNumericStartEffects))[i];
    } else {
        return (*(d->actionsToRPGNumericEndEffects))[i];
    }

};

list<Literal*> & RPGHeuristic::getInvariants(const int & i)
{
    return (*(d->actionsToInvariants))[i];
}

RPGBuilder::RPGNumericEffect & RPGHeuristic::getRPGNE(const int & i)
{
    return (*(d->rpgNumericEffects))[i];
};


bool RPGHeuristic::hAddCostPropagation = false;
bool RPGHeuristic::blindSearch = false;
bool RPGHeuristic::makeCTSEffectsInstantaneous = false;    
bool RPGHeuristic::ignoreNumbers = false;
    
void RPGHeuristic::setGuidance(const char * config)
{
    const string asString(config);
    
    if (asString == "blind") {
        blindSearch = true;                
    } else if (asString == "nonumbers") {
        ignoreNumbers = true;
    } else if (asString == "makectsinstantaneous") {
        makeCTSEffectsInstantaneous = true;
    } else {
        cerr << "Possible options for the -g parameter are:\n";
        cerr << "\t-gblind                - use blind search (0 heuristic for goal states, otherwise 1)\n";
        cerr << "\t-gnonumbers            - ignore numeric preconditions and effects\n";
        cerr << "\t-gmakectsinstantaneous - make continuous effects instantaneous (as in the Colin IJCAI paper)\n";
        exit(1);
    }
}
    


};
