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

#ifndef __RPGBUILDER
#define __RPGBUILDER

#include <vector>
#include <list>
#include <set>
#include <map>
#include <limits>

using std::vector;
using std::list;
using std::set;
using std::map;

#include "minimalstate.h"

#include "instantiation.h"

#include "ptree.h"

#include <assert.h>
#include <values.h>
#include <math.h>

using namespace Inst;

namespace Planner
{

class RPGHeuristic;
class StartEvent;

struct EpsilonComp {

    bool operator()(const double & a, const double & b) const {
        if (fabs(b - a) < 0.0005) return false;
        return (a < b);
    }

};

class MinimalState;


struct ActionFluentModification {

    int act;
    VAL::time_spec ts;
    bool openEnd;
    double change;
    int howManyTimes;
    bool assignment;

    ActionFluentModification(const int & a, const VAL::time_spec & startOrEnd, const bool & oe, const double & incr, const int & shots, const bool & ass)
            : act(a), ts(startOrEnd), openEnd(oe), change(incr), howManyTimes(shots), assignment(ass) {};

};

class ActionSegment
{

public:
    static int tilLimit;

    instantiatedOp* first;
    VAL::time_spec second;
    int divisionID;

    set<int> needToFinish;

    ActionSegment()
        : first(0), second(VAL::E_OVER_ALL), divisionID(-1)
    {
    }
    
    ActionSegment(instantiatedOp* const f, const VAL::time_spec & s, const int & d, const set<int> & n)
        : first(f), second(s), divisionID(d), needToFinish(n)
    {
        assert(second != VAL::E_AT || divisionID <= tilLimit);
    }

    ActionSegment(const ActionSegment & o)
        : first(o.first), second(o.second), divisionID(o.divisionID), needToFinish(o.needToFinish)
    {
    }

    virtual ~ActionSegment()
    {
    }
    
};


struct InvData;

class RPGBuilder
{

public:
    enum math_op { NE_ADD, NE_SUBTRACT, NE_MULTIPLY, NE_DIVIDE, NE_CONSTANT, NE_FLUENT, NE_VIOLATION};

    static bool doSkipAnalysis;

    struct Operand {
        math_op numericOp;
        int fluentValue;
        double constantValue;
        string isviolated;
        Operand(const math_op & o) : numericOp(o), fluentValue(-1), constantValue(NAN) {};
        Operand(const int & f) : numericOp(NE_FLUENT), fluentValue(f), constantValue(NAN) {};
        Operand(const double & c) : numericOp(NE_CONSTANT), fluentValue(-1), constantValue(c) {};
        Operand(const string & s) : numericOp(NE_VIOLATION), fluentValue(-1), constantValue(NAN), isviolated(s) {};
    };

    static double calculateRHS(const list<Operand> & formula, vector<double> & fluents);
    static pair<double, bool> constRHS(const list<Operand> & formula);

    class NumericEffect
    {

    public:

        int fluentIndex;
        VAL::assign_op op; // how to update given fluent index (add to it, assign to it, etc.)
        list<Operand> formula; // formula in postfix notation
        NumericEffect(const VAL::assign_op & opIn, const int & fIn, VAL::expression * formulaIn, VAL::FastEnvironment * f, VAL::TypeChecker * t = 0);
        double applyEffect(vector<double> & fluents) const;
        void display(ostream & o) const;
    };

    class NumericPrecondition
    {

    public:
        VAL::comparison_op op; // how to compare given fluent index to calculated RHS
        list<Operand> LHSformula; // formula for LHS in postfix notation
        list<Operand> RHSformula; // formula for RHS postfix notation
        bool valid;
        bool polarity; // false = it needs to be negated
        NumericPrecondition(const VAL::comparison_op & opIn, VAL::expression * LHSformulaIn, VAL::expression * RHSformulaIn, VAL::FastEnvironment * f, VAL::TypeChecker * t = 0, const bool negated = false);
        bool isSatisfied(vector<double> & fluents) const;
        void display(ostream & o) const;

        double evaluateRHS(vector<double> & fluentTable) const;
        pair<double, bool> constRHS() const;
    };

    class ArtificialVariable
    {

    public:
        int ID;
        int size;
        vector<double> weights;
        vector<int> fluents;
        double constant;

        double maxNeed;

        ArtificialVariable() : ID(-1), size(0), constant(0.0), maxNeed(-DBL_MAX) {};
        ArtificialVariable(const int & id, const int & s, const vector<double> & w, const vector<int> & f, const double & d, const double & maxIn) : ID(id), size(s), weights(w), fluents(f), constant(d), maxNeed(maxIn) {};

        double evaluate(const vector<double> & fluentTable) {
            double toReturn = constant;
            for (int i = 0; i < size; ++i) {
                if (fluents[i] < 0) {
                    return std::numeric_limits<double>::signaling_NaN();
                }
                toReturn += weights[i] * fluentTable[fluents[i]];
            }
            return toReturn;
        };

        double evaluateWCalculate(const vector<double> & fluentTable, const int & pneCount);
        double evaluateWCalculate(const vector<double> & minFluentTable, const vector<double> & maxFluentTable, const int & pneCount);

        bool operator <(const ArtificialVariable & v) const;
        void display(ostream & o) const;

        void updateMax(const double & m) {
            if (m > maxNeed) maxNeed = m;
        }
    };

    class RPGNumericPrecondition
    {

    public:
        int ID;

        int LHSVariable;
        double LHSConstant;

        VAL::comparison_op op;

        int RHSVariable;
        double RHSConstant;

        RPGNumericPrecondition() : ID(-1), LHSVariable(-1), LHSConstant(0.0), op(VAL::E_GREATEQ), RHSVariable(-1), RHSConstant(0.0) {};
        RPGNumericPrecondition(const int & i, const int & lv, const double & lw, const VAL::comparison_op & o, const double & rw) : ID(i), LHSVariable(lv), LHSConstant(lw), op(o), RHSVariable(-1), RHSConstant(rw) {};

        bool isSatisfied(vector<double> & maxFluents) const {            
            if (LHSVariable < 0) return false;
            
            const double lv = maxFluents[LHSVariable];
            if (lv == std::numeric_limits<double>::signaling_NaN()) return false;
            
            if (op == VAL::E_GREATER) {                
                return (lv > RHSConstant);
            } else {
                return (lv >= RHSConstant);
            }
        };

        bool isSatisfiedWCalculate(const vector<double> & maxFluents) const;
        bool isSatisfiedWCalculate(const vector<double> & minFluents, const vector<double> & maxFluents) const;

        bool operator <(const RPGNumericPrecondition & r) const;

        void display(ostream & o) const;

    };

    class RPGNumericEffect
    {

    public:
        int ID;

        int fluentIndex;

        bool isAssignment; // if it's an assignmentOp - assume it's a += otherwise

        vector<double> weights;
        vector<int> variables;

        double constant;

        int size;

        RPGNumericEffect() : ID(-1), fluentIndex(-1), isAssignment(false), constant(NAN), size(-1) {};

        RPGNumericEffect(const int & idIn, const int & fluent, const bool & ass,
                         const vector<double> & weightsIn, const vector<int> & vars, const int & s,
                         const double & con) :
                ID(idIn), fluentIndex(fluent), isAssignment(ass),
                weights(weightsIn), variables(vars), constant(con), size(s) {};

        double evaluate(const vector<double> & maxFluents, const double & minDur, const double & maxDur) const {
            double toReturn = constant;
            for (int i = 0; i < size; ++i) {
                if (variables[i] >= 0) {
                    const double val = maxFluents[variables[i]];
                    if (val == DBL_MAX) return DBL_MAX;
                    if (val == -DBL_MAX) return -DBL_MAX;
                    toReturn += weights[i] * val;
                } else if (variables[i] == -3) {
                    double lw = weights[i];
                    double toUse = maxDur;
                    if (lw < 0) {
                        lw *= -1.0;
                        toUse = -minDur;
                    }
                    if (toUse == DBL_MAX) return DBL_MAX;
                    if (toUse == -DBL_MAX) return -DBL_MAX;
                    toReturn += lw * toUse;
                } else {
                    assert(false);
                }
            }
            return toReturn;
        };

        pair<double, double> applyEffectMinMax(const vector<double> & minFluents, const vector<double> & maxFluents, const double & minDur, const double & maxDur);

        bool operator <(const RPGNumericEffect & e) const;

        void display(ostream & o) const;


    };

    /** A class defining a single duration constraint, in LNF. */
    class DurationExpr
    {

    public:
        /** The weights of the variables in the duration expression */
        vector<double> weights;
        
        /** The IDs of the variables in the duration expression.  @see RPGBuilder::getPNE */
        vector<int> variables;

        /** The comparison operator for this duration expression.  Is one of:
         *  - <code>VAL::E_GREATEQ</code> for <code>(>= ?duration ...)</code>
         *  - <code>VAL::E_GREATER</code> for <code>(> ?duration ...)</code>
         *  - <code>VAL::E_EQUALS</code> for <code>(= ?duration ...)</code>
         *  - <code>VAL::E_LESS</code> for <code>(< ?duration ...)</code>
         *  - <code>VAL::E_LESSEQ</code> for <code>(<= ?duration ...)</code>
         */
        VAL::comparison_op op;

        /** The constant term for the duration expression */
        double constant;

        DurationExpr() : weights(), variables(), constant(0.0) {};
        
        /**
         * Create a duration expression, with unknown operator and empty weights
         * and variables.
         *
         * @param d  The constant term for the duration expression
         */
        DurationExpr(const double & d) : weights(0), variables(0), constant(d) {} ;


        /**
         * Evaluate the minimum possible value that satisfies this duration expression,
         * given the provided lower- and upper-bounds on the values of the task numeric
         * variables.
         *
         * @param minVars  Minimum values of the task variables
         * @param maxvars  Maximum values of the task variables
         * 
         * @return  The lowest permissible duration, according to this duration expression.
         */
        double minOf(const vector<double> & minVars, const vector<double> & maxVars);
        
        /**
         * Evaluate the maximum possible value that satisfies this duration expression,
         * given the provided lower- and upper-bounds on the values of the task numeric
         * variables.
         *
         * @param minVars  Minimum values of the task variables
         * @param maxvars  Maximum values of the task variables
         * 
         * @return  The greatest permissible duration, according to this duration expression.
         */
        double maxOf(const vector<double> & minVars, const vector<double> & maxVars);
    };

    /** A class containing all the duration constraints imposed on an action. */
    class RPGDuration
    {

    public:
        /** The fixed durations of the action, those of the form <code>(= ?duration ...)</code> */
        list<DurationExpr *> fixed;

        /** The minimum durations of the action, those of the form <code>(>= ?duration ...)</code> or <code>(> ?duration ...)</code> */        
        list<DurationExpr *> min;
        
        /** The maximum durations of the action, those of the form <code>(<= ?duration ...)</code> or <code>(< ?duration ...)</code> */        
        list<DurationExpr *> max;

//      LPDuration() : fixed(0), min(0), max(0) {};
        RPGDuration(list<DurationExpr *> & eq, list<DurationExpr *> & low, list<DurationExpr *> & high)
                : fixed(eq), min(low), max(high) {};

        /**
         * Return one of the lists of durations contained in the object.
         * 
         * @param i  Which list of durations to return:
         *           - 0 returns the <code>fixed</code> duration expressions;
         *           - 1 returns the <code>min</code> duration expressions;
         *           - 2 returns the <code>max</code> duration expressions.
         *
         * @return  The requested list of duration expressions.
         */
        const list<DurationExpr *> & operator[](const int & i) const {
            switch (i) {
            case 0:
                return fixed;
            case 1:
                return min;
            case 2:
                return max;
            }
            assert(false);
        }
    };


    /**
     *  Class to represent the 'action' whose application corresponds to a timed initial literal.
     */
    class FakeTILAction
    {

    public:
        /** The time-stamp of the timed initial literal */
        const double duration;

        /** The facts added at the specified time */
        list<Literal*> addEffects;

        /** The facts deleted at the specified time */
        list<Literal*> delEffects;


        /**
         *   Add the specified add and delete effects to the timed initial literal action.  Is used when
         *   multiple TILs are found at a given time-stamp.
         *
         *   @param adds  Add effects to include in this TIL action
         *   @param dels  Delete effects to include in this TIL action
         */
        void mergeIn(const LiteralSet & adds, const LiteralSet & dels) {

            {
                LiteralSet::iterator lsItr = adds.begin();
                const LiteralSet::iterator lsEnd = adds.end();

                for (; lsItr != lsEnd; ++lsItr) {
                    addEffects.push_back(*lsItr);
                }
            }

            {
                LiteralSet::iterator lsItr = dels.begin();
                const LiteralSet::iterator lsEnd = dels.end();

                for (; lsItr != lsEnd; ++lsItr) {
                    delEffects.push_back(*lsItr);
                }
            }
        }

        /**
         *   Constructor for an action corresponding to a Timed Initial Literal.
         *
         *   @param dur  The time at which the timed initial occurs
         *   @param adds The facts added at time <code>dur</code>
         *   @param dels The facts deleted at time <code>dur</code>
         */
        FakeTILAction(const double & dur, const LiteralSet & adds, const LiteralSet & dels)
                : duration(dur) {
            mergeIn(adds, dels);
        }

    };




    class KShotFormula
    {

    public:
        KShotFormula() {};
        virtual int getLimit(const MinimalState & s) const = 0;
        virtual int getOptimisticLimit(const MinimalState & s) const = 0;
        virtual ~KShotFormula() {};
    };

    class UnlimitedKShotFormula : public KShotFormula
    {

    public:
        UnlimitedKShotFormula() : KShotFormula() {};
        virtual int getLimit(const MinimalState &) const {
            return INT_MAX;
        };
        virtual int getOptimisticLimit(const MinimalState &) const {
            return INT_MAX;
        };
    };

    class OneShotKShotFormula : public KShotFormula
    {

    private:
        list<int> watchedLiterals;
    public:
        OneShotKShotFormula(list<int> & toWatch) : KShotFormula(), watchedLiterals(toWatch) {};
        virtual int getLimit(const MinimalState & s) const;
        virtual int getOptimisticLimit(const MinimalState & s) const;
    };

    struct ShotCalculator {

        int variable;
        double greaterThan;
        double decreaseBy;

        ShotCalculator(const int & v, const double & g, const double & d) : variable(v), greaterThan(g), decreaseBy(d) {};
    };

    class KShotKShotFormula : public KShotFormula
    {

    private:
        list<ShotCalculator> formulae;
    public:
        KShotKShotFormula(list<ShotCalculator> & c) : KShotFormula(), formulae(c) {};
        virtual int getLimit(const MinimalState & s) const;
        virtual int getOptimisticLimit(const MinimalState & s) const;
    };

    /** A class defining the linear continuous effects of an action */
    class LinearEffects
    {

    public:
        /**
         * A single linear continuous numeric effect expression.  The gradient of the
         * effect is stored in LNF.
         */
        struct EffectExpression {
            /** The weights for the variables in the LNF expression of the gradient. */
            vector<double> weights;
            
            /** The variables in the LNF expression of the gradient. */
            vector<int> variables;

            /** The constant term of the LNF expression of the gradient. */
            double constant;

            /**
             * Constructor for a constant-gradient linear continuous effect
             *
             * @param g  The gradient of the effect
             */
            EffectExpression(const double & g) : weights(0), variables(0), constant(g) {};
        };

//      vector<double> durations;

        /** The IDs of the variables upon which this action has continuous effects.  @see RPGBuilder::getPNE */
        vector<int> vars;
        
        /**
         * The effects themselves. Each entry is a vector describing the effect upon the corresponding
         * variable in <code>vars</code>.  For now, at most one such vector exists, but in the future
         * this may change if support for piecewise-linear effects is added.
         */        
        vector<vector<EffectExpression> > effects;

//      double totalDuration;

        /**
         *  The number of divisions for the continuous numeric effects of the action.  At present this
         *  is always 1, i.e. the gradients of the effects remain fixed across the execution of the
         *  action, but this may change if support for piecewise-linear effects is added.
         */
        int divisions;

    };

    class Metric
    {

    public:
        bool minimise;

        list<double> weights;
        list<int> variables;

        Metric(const bool & m) : minimise(m) {};

    };

    static Metric * theMetric;
    static set<int> metricVars;

    struct Constraint {

        string name;

        VAL::constraint_sort cons;
        list<Literal*> goal;
        list<Literal*> trigger;

        list<NumericPrecondition> goalNum;
        list<NumericPrecondition> triggerNum;

        list<int> goalRPGNum;
        list<int> triggerRPGNum;

        double deadline;
        double from;

        double cost;
        bool neverTrue;

        Constraint() : deadline(0.0), from(0.0), cost(0.0), neverTrue(false) {};
        Constraint(const string & n) : name(n), deadline(0.0), from(0.0), cost(0.0), neverTrue(false) {};

    };

    /**
     *  Class to represent a conditional effect.  The class is capable of storing more fields than can
     *  currently be used by the planner - currently, only conditional effects upon metric tracking
     *  variables are supported, conditional on either propositions controlled exclusively by
     *  timed initial literals, or on numeric values.
     */
    class ConditionalEffect
    {

    private:

        /**
         * Propositional preconditions on the conditional effect.  Each entry is pair: the first
         * entry denotes the fact that must hold, the second denotes the time at which it must
         * hold (either <code>VAL::E_AT_START</code>, <code>VAL::E_OVER_ALL</code> or <code>VAL::E_AT_END</code>.
         * For now, the literal must be controlled exclusively by timed initial literals.
         */
        list<pair<Literal*, VAL::time_spec> > propositionalConditions;

        /**
         * Numeric preconditions on the conditional effect.  Each entry is pair: the first
         * entry denotes an entry in <code>RPGBuilder::rpgNumericPreconditions</code>; the second denotes the time at which it must
         * hold (either <code>VAL::E_AT_START</code>, <code>VAL::E_OVER_ALL</code> or <code>VAL::E_AT_END</code>.
         */
        list<pair<int, VAL::time_spec> > numericPreconditions;

        /**
         * Conditional numeric effects.  For now, these must act exclusively on metric tracking variables.
         * Each entry is pair: the first is an index in <code>RPGBuilder::rpgNumericEffects</code>; the second denotes the time at which
         * it occurs (either <code>VAL::E_AT_START</code> or <code>VAL::E_AT_END</code>).
         */
        list<pair<int, VAL::time_spec> > numericEffects;

        /** Conditional propositional add effects.  For now, must be empty. */
        list<pair<Literal*, VAL::time_spec> > propositionalAddEffects;
        /** Conditional propositional delete effects.  For now, must be empty. */
        list<pair<Literal*, VAL::time_spec> > propositionalDeleteEffects;

    public:
        ConditionalEffect() {
        }

        void addCondition(Literal * const l, const VAL::time_spec & t) {
            propositionalConditions.push_back(make_pair(l, t));
        }

        void addCondition(const int & p, const VAL::time_spec & t) {
            numericPreconditions.push_back(make_pair(p, t));
        }

        void addNumericEffect(const int & p, const VAL::time_spec & t) {
            numericEffects.push_back(make_pair(p, t));
        }

        void addAddEffect(Literal * const l, const VAL::time_spec & t) {
            propositionalAddEffects.push_back(make_pair(l, t));
        }

        void addDeleteEffect(Literal * const l, const VAL::time_spec & t) {
            propositionalDeleteEffects.push_back(make_pair(l, t));
        }

        const list<pair<Literal*, VAL::time_spec> > & getPropositionalConditions() const {
            return propositionalConditions;
        }

        const list<pair<int, VAL::time_spec> > & getNumericPreconditions() const {
            return numericPreconditions;
        }

        const list<pair<int, VAL::time_spec> > & getNumericEffects() const {
            return numericEffects;
        }

        const list<pair<Literal*, VAL::time_spec> > & getPropositionalAddEffects() const {
            return propositionalAddEffects;
        }

        const list<pair<Literal*, VAL::time_spec> > & getPropositionalDeleteEffects() const {
            return propositionalDeleteEffects;
        }


    };

    class NoDuplicatePair
    {

    protected:

        list<Literal*> * first;
        LiteralSet * second;

    public:

        NoDuplicatePair()
                : first((list<Literal*>*)0), second((LiteralSet*)0) {
        }

        NoDuplicatePair(list<Literal*> * const listIn, LiteralSet* const setIn)
                : first(listIn), second(setIn) {
        }

        void push_back(Literal* const toAdd) {
            if (second->insert(toAdd).second) {
                first->push_back(toAdd);
            }
        }

        Literal* back() const {
            return first->back();
        }

        bool operator!() const {
            return (!first);
        }

        template<typename T>
        void insert(T itr, const T & itrEnd) {
            for (; itr != itrEnd; ++itr) {
                push_back(*itr);
            }
        };
    };


    class ProtoConditionalEffect
    {

    public:

        list<Literal*> startPrec;
        LiteralSet startPrecSet;
        list<Literal*> inv;
        LiteralSet invSet;
        list<Literal*> endPrec;
        LiteralSet endPrecSet;

        list<Literal*> startNegPrec;
        LiteralSet startNegPrecSet;
        list<Literal*> negInv;
        LiteralSet negInvSet;
        list<Literal*> endNegPrec;
        LiteralSet endNegPrecSet;


        list<RPGBuilder::NumericPrecondition> startPrecNumeric;
        list<RPGBuilder::NumericPrecondition> invNumeric;
        list<RPGBuilder::NumericPrecondition> endPrecNumeric;

        list<Literal*> startAddEff;
        LiteralSet startAddEffSet;
        list<Literal*> startDelEff;
        LiteralSet startDelEffSet;
        list<RPGBuilder::NumericEffect> startNumericEff;

        list<Literal*> endAddEff;
        LiteralSet endAddEffSet;
        list<Literal*> endDelEff;
        LiteralSet endDelEffSet;
        list<RPGBuilder::NumericEffect> endNumericEff;

    };

private:



    static bool RPGdebug;
    static bool problemIsNotTemporal;
// static vector<list<pair<int, double> > > actionsToNegativeNumericEffects;
// static vector<list<pair<int, double> > > negativeNumericEffectsToActions;

    static vector<list<pair<int, VAL::time_spec> > > preconditionsToActions;
    static vector<list<pair<int, VAL::time_spec> > > negativePreconditionsToActions;

    static list<pair<int, VAL::time_spec> > preconditionlessActions;
    static list<pair<int, VAL::time_spec> > onlyNumericPreconditionActions;

    static vector<list<Literal*> > actionsToStartPreconditions;
    static vector<list<Literal*> > actionsToInvariants;
    static vector<list<Literal*> > actionsToEndPreconditions;

    static vector<list<Literal*> > actionsToStartNegativePreconditions;
    static vector<list<Literal*> > actionsToNegativeInvariants;
    static vector<list<Literal*> > actionsToEndNegativePreconditions;


    static vector<LiteralSet> actionsToEndOneShots;

    static vector<list<Literal*> > actionsToStartEffects;
    static vector<list<Literal*> > actionsToStartNegativeEffects;
    static vector<list<Literal*> > actionsToEndEffects;
    static vector<list<Literal*> > actionsToEndNegativeEffects;

    static vector<list<pair<int, VAL::time_spec> > > effectsToActions;
    static vector<list<pair<int, VAL::time_spec> > > negativeEffectsToActions;

    static vector<double> actionsToMinDurations;
    static vector<double> actionsToMaxDurations;
    static vector<list<NumericPrecondition*> > fixedDurationExpressions;
    static vector<list<NumericPrecondition*> > minDurationExpressions;
    static vector<list<NumericPrecondition*> > maxDurationExpressions;

    static vector<bool> startEndSkip;

    // one for each instantiated action; within that - one for each point along the action, minus the last
    // entry [a][0] specifies duration between a0 and a1
    static vector<vector<RPGDuration*> > rpgDurationExpressions;
    static vector<double> nonTemporalDuration;


    static vector<LinearEffects*> linearDiscretisation;

    static vector<list<ProtoConditionalEffect*> > actionsToRawConditionalEffects;

    static vector<list<NumericPrecondition> > actionsToStartNumericPreconditions;
    static vector<list<NumericPrecondition> > actionsToNumericInvariants;
    static vector<list<NumericPrecondition> > actionsToEndNumericPreconditions;

    static vector<list<NumericEffect> > actionsToStartNumericEffects;
    static vector<list<NumericEffect> > actionsToEndNumericEffects;

    static vector<list<int> > actionsToRPGNumericStartPreconditions;
    static vector<list<int> > actionsToRPGNumericInvariants;
    static vector<list<int> > actionsToRPGNumericEndPreconditions;

    static vector<list<int> > actionsToRPGNumericStartEffects;
    static vector<list<int> > actionsToRPGNumericEndEffects;

    static vector<list<ConditionalEffect> > actionsToConditionalEffects;

// static vector<list<pair<int, double> > > numericPreconditionsToActions;
// static vector<list<pair<int, double> > > actionsToNumericPreconditions;

    static vector<int> initialUnsatisfiedStartPreconditions;
    static vector<int> initialUnsatisfiedInvariants;
    static vector<int> initialUnsatisfiedEndPreconditions;

    static vector<double> achievedInLayer;
    static vector<double> achievedInLayerReset;
    static vector<pair<int, VAL::time_spec> > achievedBy;
    static vector<pair<int, VAL::time_spec> > achievedByReset;

    static vector<double> negativeAchievedInLayer;
    static vector<double> negativeAchievedInLayerReset;
    static vector<pair<int, VAL::time_spec> > negativeAchievedBy;
    static vector<pair<int, VAL::time_spec> > negativeAchievedByReset;

    static vector<double> numericAchievedInLayer;
    static vector<double> numericAchievedInLayerReset;
    static vector<ActionFluentModification*> numericAchievedBy;
    static vector<ActionFluentModification*> numericAchievedByReset;

// static vector<int> increasedInLayer;
// static vector<pair<int, double> > increasedBy;
// static vector<pair<int, double> > increasedReset;

    static vector<instantiatedOp*> instantiatedOps;

    static vector<Literal*> literals;
    static vector<PNE*> pnes;
    static vector<pair<bool, bool> > staticLiterals;

    static vector<RPGNumericPrecondition> rpgNumericPreconditions;
    static vector<RPGNumericEffect> rpgNumericEffects;
    static vector<list<pair<int, VAL::time_spec> > > rpgNumericEffectsToActions;
    static vector<list<pair<int, VAL::time_spec> > > rpgNumericPreconditionsToActions;

    static vector<ArtificialVariable> rpgArtificialVariables;
    static vector<list<int> > rpgVariableDependencies;

    static vector<list<int> > rpgArtificialVariablesToPreconditions;
    static vector<list<int> > rpgNegativeVariablesToPreconditions;
    static vector<list<int> > rpgPositiveVariablesToPreconditions;

    static vector<int> initialUnsatisfiedNumericStartPreconditions;
    static vector<int> initialUnsatisfiedNumericInvariants;
    static vector<int> initialUnsatisfiedNumericEndPreconditions;

    static vector<list<pair<int, VAL::time_spec> > > processedPreconditionsToActions;
    static vector<list<pair<int, VAL::time_spec> > > processedNegativePreconditionsToActions;
    static vector<list<Literal*> > actionsToProcessedStartPreconditions;
    static vector<list<Literal*> > actionsToProcessedStartNegativePreconditions;
    static vector<int> initialUnsatisfiedProcessedStartPreconditions;
    static vector<list<int> > realVariablesToRPGEffects;

    static vector<list<pair<int, VAL::time_spec> > > processedRPGNumericPreconditionsToActions;

    static vector<list<NumericPrecondition> > actionsToProcessedStartNumericPreconditions;

    static list<Literal*> literalGoals;
    static list<double> literalGoalDeadlines;
    static list<NumericPrecondition> numericGoals;
    static list<double> numericGoalDeadlines;
    static list<pair<int, int> > numericRPGGoals;


    static vector<Constraint> preferences;
    static map<string, int> prefNameToID;

    static vector<Constraint> constraints;


    static vector<list<int> > actionsToProcessedStartRPGNumericPreconditions;
    static vector<int> initialUnsatisfiedProcessedStartNumericPreconditions;

    static vector<list<int> > mentionedInFluentInvariants;

    static list<FakeTILAction> timedInitialLiterals;
    static vector<FakeTILAction*> timedInitialLiteralsVector;

    static list<FakeTILAction> optimisationTimedInitialLiterals;
    static vector<FakeTILAction*> optimisationTimedInitialLiteralsVector;

    static vector<FakeTILAction*> allTimedInitialLiteralsVector;
    
    static map<int, set<int> > tilsThatAddFact;
    static map<int, set<int> > tilsThatDeleteFact;

    static vector<KShotFormula*> kShotFormulae;
    static vector<bool> selfMutexes;
    static vector<bool> oneShotLiterals;

    static vector<double> maxNeeded;
    static map<int, int> uninterestingnessCriteria;

    static void buildRPGNumericPreconditions();
    static void buildRPGNumericEffects();
// static void simplify(pair<list<double>, list<int> > & s);
// static void makeOneSided(pair<list<double>, list<int> > & LHSvariable, pair<list<double>, list<int> > & RHSvariable, const int & negOffset);
// static void makeWeightedSum(list<Operand> & formula, pair<list<double>, list<int> > & result);
    static void processPreconditions(set<ArtificialVariable> & artificialVariableSet,
                                     map<RPGNumericPrecondition, list<pair<int, VAL::time_spec> > > & rpgNumericPreconditionSet,
                                     list<NumericPrecondition> & currPreList, list<int> & destList, int & toIncrement,
                                     const int & negOffset, const int & offset, int & precCount, int & avCount,
                                     vector<double> & localMaxNeed, const int & i, const VAL::time_spec & passTimeSpec);

    static void buildDurations(vector<list<NumericPrecondition*> > & d, vector<list<NumericPrecondition*> > & e, vector<list<NumericPrecondition*> > & f);

    static bool pushInvariantBackThroughStartEffects(const RPGNumericPrecondition & pre, list<int> & startEffs, InvData & commonData, pair<const RPGNumericPrecondition *, bool> & preResult, pair<ArtificialVariable *, bool> & avResult);

    static void handleNumericInvariants();
    static void findSelfMutexes();
    static void oneShotInferForTILs();
    static void kshotInferForAction(const int & i, MinimalState & refState, LiteralSet & maybeOneShotLiteral, vector<double> & initialFluents, const int & fluentCount);
    static void doSomeUsefulMetricRPGInference();
    static void checkConditionalNumericEffectsAreOnlyOnMetricTrackingVariables();
    static void separateOptimisationTILs();
    static void findUninterestingnessCriteria();
    static DurationExpr * buildDE(NumericPrecondition * d);
    static list<DurationExpr *> buildDEList(list<NumericPrecondition *> & d);
    static LinearEffects * buildLE(const int & i);

    static bool considerAndFilter(LiteralSet & initialState, LiteralSet & revisit, const int & operatorID);
    static void postFilterUnreachableActions();
    static void buildMetric(VAL::metric_spec*);

    /**
     *  Visit the conditional effects noted during action instantiation, and
     *  make ConditionalEffect objects to represent each of them.  For now,
     *  the function also calls <code>postmortem_noADL</code> if there are
     *  any conditional propositional effects, or preconditions on facts that
     *  aren't exclusively controlled by timed initial literals.
     *  @see ConditionalEffect
     */
    static void buildThePropositionalBitOfConditionalEffects();

    static void findStaticLiterals();
    static void pruneStaticPreconditions(list<Literal*> & toPrune, int & toDec);
    static void pruneStaticPreconditions();

    class CommonRegressionData;

    static void postFilterIrrelevantActions();
    static void findActionTimestampLowerBounds();
    static void pruneIrrelevant(const int & operatorID);
    static void findCompressionSafeActions();

    static RPGHeuristic * globalHeuristic;

public:


    static bool canSkipToEnd(const int & i) {
        return startEndSkip[i];
    };
    static pair<bool, bool> & isStatic(Literal* l);

    static void simplify(pair<list<double>, list<int> > & s);
    static void makeOneSided(pair<list<double>, list<int> > & LHSvariable, pair<list<double>, list<int> > & RHSvariable, const int & negOffset);
    static void makeWeightedSum(list<Operand> & formula, pair<list<double>, list<int> > & result);

    static const vector<double> & getNonTemporalDurationToPrint() {
        return nonTemporalDuration;
    }
    
    static const vector<FakeTILAction*> & getAllTimedInitialLiterals() {
        return allTimedInitialLiteralsVector;
    }
    
    static vector<list<NumericPrecondition*> > & getFixedDEs() {
        return fixedDurationExpressions;
    }
    
    static vector<list<NumericPrecondition*> > & getMinDEs() {
        return minDurationExpressions;
    }
    
    static vector<list<NumericPrecondition*> > & getMaxDEs() {
        return maxDurationExpressions;
    }
    
    static vector<RPGDuration*> & getRPGDEs(const int & a) {
        return rpgDurationExpressions[a];
    }
    
    static vector<LinearEffects*> & getLinearDiscretisation() {
        return linearDiscretisation;
    }

    static vector<list<int> > & getStartPreNumerics() {
        return actionsToRPGNumericStartPreconditions;
    }
    
    static vector<list<int> > & getInvariantNumerics() {
        return actionsToRPGNumericInvariants;
    }
    
    static vector<list<int> > & getEndPreNumerics() {
        return actionsToRPGNumericEndPreconditions;
    }

    static vector<list<int> > & getStartEffNumerics() {
        return actionsToRPGNumericStartEffects;
    }
    
    static vector<list<int> > & getEndEffNumerics() {
        return actionsToRPGNumericEndEffects;
    }

    static const vector<list<ConditionalEffect> > & getActionsToConditionalEffects() {
        return actionsToConditionalEffects;
    }
    
    static vector<list<Literal*> > & getStartPropositionAdds() {
        return actionsToStartEffects;
    }
    
    static vector<list<Literal*> > & getStartPropositionDeletes() {
        return actionsToStartNegativeEffects;
    }

    static vector<list<Literal*> > & getEndPropositionAdds() {
        return actionsToEndEffects;
    };
    static vector<list<Literal*> > & getEndPropositionDeletes() {
        return actionsToEndNegativeEffects;
    };

    static vector<list<Literal*> > & getProcessedStartPropositionalPreconditions() {
        return actionsToProcessedStartPreconditions;
    };
    static vector<list<Literal*> > & getStartPropositionalPreconditions() {
        return actionsToStartPreconditions;
    };
    static vector<list<Literal*> > & getStartNegativePropositionalPreconditions() {
        return actionsToStartNegativePreconditions;
    };
    static vector<list<Literal*> > & getInvariantPropositionalPreconditions() {
        return actionsToInvariants;
    };
    static vector<list<Literal*> > & getInvariantNegativePropositionalPreconditions() {
        return actionsToNegativeInvariants;
    };
    static vector<list<Literal*> > & getEndPropositionalPreconditions() {
        return actionsToEndPreconditions;
    };
    static vector<list<Literal*> > & getEndNegativePropositionalPreconditions() {
        return actionsToEndNegativePreconditions;
    };

    static vector<list<Literal*> > & getProcessedStartNegativePropositionalPreconditions() {
        return actionsToProcessedStartNegativePreconditions;
    };

    /** @Return A reference to <code>rpgNumericPreconditions</code>, the preconditions used in the problem, in LNF. */
    static vector<RPGNumericPrecondition> & getNumericPreTable() {
        return rpgNumericPreconditions;
    }
    
    /** @Return A reference to <code>rpgNumericEffects</code>, the effects used in the problem, in LNF. */
    static vector<RPGNumericEffect> & getNumericEff() {
        return rpgNumericEffects;
    }
    
    /** @Return a reference to <code>rpgNumericEffectsToActions</code>: for each numeric effects, which actions have it. */
    static const vector<list<pair<int, VAL::time_spec> > > & getRpgNumericEffectsToActions() {
        return rpgNumericEffectsToActions;
    }

    /**
     * @Return A const reference to <code>rpgArtificialVariables</code>, the artificial variables built
     *         so that multi-variable preconditions and can be written in terms of a single 'artificial'
     *         variable.
     */
    static const vector<ArtificialVariable> & getArtificialVariableTable() {
        return rpgArtificialVariables;
    }
    
    static vector<list<pair<int, VAL::time_spec> > > & getPresToActions() {
        return processedPreconditionsToActions;
    }
    
    static vector<list<pair<int, VAL::time_spec> > > & getRawPresToActions() {
        return preconditionsToActions;
    };

    static const vector<bool> & rogueActions;
    static vector<bool> realRogueActions;
    static bool sortedExpansion;
    static bool fullFFHelpfulActions;
    static bool modifiedRPG;
    static bool noSelfOverlaps;
    static bool doTemporalAnalysis;

    static void initialise();

    static bool nonTemporalProblem() {
        return problemIsNotTemporal;
    };

    static list<Literal*> & getLiteralGoals() {
        return literalGoals;
    };

    static RPGHeuristic * generateRPGHeuristic();
    static RPGHeuristic * getHeuristic() {
        return globalHeuristic;
    }
    static void getInitialState(LiteralSet & initialState, vector<double> & initialFluents);
    static void getNonStaticInitialState(LiteralSet & initialState, vector<double> & initialFluents);
    static instantiatedOp* getInstantiatedOp(const int & i) {
        return instantiatedOps[i];
    };
    static Literal* getLiteral(const int & i) {
        return literals[i];
    };
    static list<FakeTILAction> & getTILs() {
        return timedInitialLiterals;
    };
    static list<pair<int, VAL::time_spec> > & getEffectsToActions(const int & i) {
        return effectsToActions[i];
    };
    static vector<FakeTILAction*> & getTILVec() {
        return timedInitialLiteralsVector;
    };

    static void getEffects(instantiatedOp* op, const bool & start, list<Literal*> & add, list<Literal*> & del, list<NumericEffect> & numeric);
    static void getPrecInv(instantiatedOp* op, const bool & start, list<Literal*> & precs, list<Literal*> & inv, list<NumericPrecondition> & numericPrec, list<NumericPrecondition> & numericInv);
    // static void getCollapsedAction(instantiatedOp* op, list<Literal*> & pre, list<Literal*> & add, list<Literal*> & del, list<NumericPrecondition> & numericPre, list<NumericEffect> & numericEff);

    static Metric * getMetric() {
        return theMetric;
    }
    
    static list<int> & getMentioned(const int & i) {
        return mentionedInFluentInvariants[i];
    };

    static bool stepNeedsToHaveFinished(const ActionSegment & act, const MinimalState & s, set<int> & dest);

    static double getOpMinDuration(instantiatedOp* op, const int & div);
    static double getOpMinDuration(const int & op, const int & div);

    static double getOpMaxDuration(instantiatedOp* op, const int & div);
    static double getOpMaxDuration(const int & op, const int & div);


    static pair<double, double> getOpDuration(instantiatedOp* op, const int & div, const vector<double> & minFluents, const vector<double> & maxFluents);
    static pair<double, double> getOpDuration(const int & op, const int & div, const vector<double> & minFluents, const vector<double> & maxFluents);

    static PNE* getPNE(const int & i) {
        return pnes[i];
    };
    static int getPNECount() {
        return pnes.size();
    };
    static int getAVCount() {
        return rpgArtificialVariables.size();
    };
    static ArtificialVariable & getArtificialVariable(const int & i) {
        return rpgArtificialVariables[i - (2 * getPNECount())];
    };

    static list<int> & getVariableDependencies(const int & i) {
        return rpgVariableDependencies[i];
    };
    static list<int> & affectsRPGNumericPreconditions(int i) {

        static const int off = getPNECount();
        if (i < off) {
            return rpgPositiveVariablesToPreconditions[i];
        };
        i -= off;
        if (i < off) {
            return rpgNegativeVariablesToPreconditions[i];
        }
        i -= off;
        return rpgArtificialVariablesToPreconditions[i];

    }


    static int howManyTimes(const int & actID, const MinimalState & e) {
        return kShotFormulae[actID]->getLimit(e);
    };
    static int howManyTimesOptimistic(const int & actID, const MinimalState & e) {
        return kShotFormulae[actID]->getOptimisticLimit(e);
    };
    static bool literalIsOneShot(const int & lID) {
        return oneShotLiterals[lID];
    };
    static bool isSelfMutex(const int & actID) {
        return selfMutexes[actID];
    };
    static list<pair<int, int> > & getNumericRPGGoals() {
        return numericRPGGoals;
    };
    static list<NumericPrecondition> & getNumericGoals() {
        return numericGoals;
    };

    static bool isInteresting(const int & act, const map<int, PropositionAnnotation> & facts, const map<int, set<int> > & started);
    static LiteralSet & getEndOneShots(const int & i) {
        return actionsToEndOneShots[i];
    };
};



class RPGHeuristic
{

private:
    class Private;

    Private * const d;

public:

    /** @brief If set to true, the heuristic uses the cheapest hadd-cost achiever at a given layer. */
    static bool hAddCostPropagation;
    
    /** @brief If set to true, the heuristic returns a value of 1 for goal states, or 0 otherwise. */
    static bool blindSearch;
    
    /** @brief If set to true, the RPG ignores numeric preconditions and effects. */
    static bool ignoreNumbers;
    
    /** @brief If set to true, the RPG integrates continuous effects, so they occur in full at the start of the action. */
    static bool makeCTSEffectsInstantaneous;
    
    /**
     *  Configure the heuristic guidance provided by the RPG heuristic, according to the
     *  given string (usually passed with the prefix '-g' on the command line).
     *
     *  @param config  Configuration name to use
     */
    static void setGuidance(const char * config);
    
    static set<int> emptyIntList;


    RPGHeuristic(const bool & b,
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
                 list<pair<int, VAL::time_spec> > * onpa);

    ~RPGHeuristic();

    static vector<double> & getEarliestForStarts();
    static vector<double> & getEarliestForEnds();


    int getRelaxedPlan(const MinimalState & theState, const vector<double> & minTimestamps, const double & stateTS, list<ActionSegment> & helpfulActions, list<pair<double, list<ActionSegment> > > & relaxedPlan, double & finalPlanMakespanEstimate, map<double, list<pair<int, int> > > * justApplied = 0, double tilFrom = 0.001);

    void findApplicableActions(const MinimalState & theState, const double & stateTime, list<ActionSegment> & applicableActions);
    void filterApplicableActions(const MinimalState & theState, const double & stateTime, list<ActionSegment> & applicableActions);

    bool testApplicability(const MinimalState & theState, const double & stateTime, const ActionSegment & actID, bool fail = false, bool ignoreDeletes = false);

    list<Literal*> & getDeleteEffects(const int & i, const VAL::time_spec & t);
    list<Literal*> & getAddEffects(const int & i, const VAL::time_spec & t);
    list<Literal*> & getPreconditions(const int & i, const VAL::time_spec & t);
    list<Literal*> & getInvariants(const int & i);

    list<int> & getNumericEffects(const int & i, const VAL::time_spec & t);
    RPGBuilder::RPGNumericEffect & getRPGNE(const int & i);

    list<instantiatedOp*> * makePlan(list<int> & steps);

    instantiatedOp* getOp(const int & i);
    double earliestTILForAction(const int & i, const bool & isStart);

    static double & getDeadlineRelevancyStart(const int & i);

    static double & getDeadlineRelevancyEnd(const int & i);


    void doFullExpansion(MinimalState & refState);

};

ostream & operator <<(ostream & o, const RPGBuilder::NumericPrecondition & p);
ostream & operator <<(ostream & o, const RPGBuilder::NumericEffect & p);

ostream & operator <<(ostream & o, const RPGBuilder::RPGNumericPrecondition & p);
ostream & operator <<(ostream & o, const RPGBuilder::ArtificialVariable & p);
ostream & operator <<(ostream & o, const RPGBuilder::RPGNumericEffect & p);

enum whereAreWe { PARSE_UNKNOWN, PARSE_PRECONDITION, PARSE_EFFECT, PARSE_DURATION, PARSE_GOAL, PARSE_INITIAL, PARSE_CONDITIONALEFFECT, PARSE_CONTINUOUSEFFECT, PARSE_METRIC, PARSE_DERIVATION_RULE, PARSE_CONSTRAINTS };

extern whereAreWe WhereAreWeNow;

void validatePNE(PNE * c);
void validateLiteral(Literal * l);

void postmortem_noNestedWhens();
void postmortem_noADL();
void postmortem_nonLinearCTS(const string & actName, const string & worksOutAs);
void postmortem_noQuadratic(const string & theOp);
void postmortem_noTimeSpecifierOnAPropPrecondition(const string & actname, const string & effect);
void postmortem_fixedAndNotTimeSpecifiers(const string & actname, const bool & multipleEquals);
void postmortem_noTimeSpecifierOnAPropEffect(const string & actname, const string & effect);
void postmortem_noTimeSpecifierOnInstantNumericEffect(const string & actname, const string & effect, const string & suggested, const bool & isAssign);
void postmortem_wrongNumberOfFluentArguments(const string & actname, const bool & haveActName, const whereAreWe & w, const string & predname, const string & lit, const int & givenArgs, const set<int> & realargs);
void postmortem_wrongNumberOfPredicateArguments(const string & actname, const bool & haveActName, const whereAreWe & w, const string & predname, const string & lit, const int & givenargs, const set<int> & realargs);
void postmortem_mathsError(const string & description, const string & help, const whereAreWe & w);
void postmortem_noConstraints(const bool unsupportedPref = false, const char * n = 0);
void postmortem_isViolatedNotExist(const string & s);
void postmortem_fatalConstraint(const string & whichOne);

};



#endif
// kate: indent-mode cstyle; space-indent on; indent-width 4; replace-tabs on;
