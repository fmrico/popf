#ifndef NUMERICANALYSIS_H
#define NUMERICANALYSIS_H

#include <ptree.h>

#include <vector>
using std::vector;

#include <cfloat>
#include <utility>

using std::make_pair;

#include "globals.h"

namespace Planner {
    
class NumericAnalysis
{
    public:
        /** @brief Enum denoting the identified dominance constraint on a given task variable.        
        *   - <code>E_NODOMINANCE</code> denotes that no dominance constraint has been identified
        *   - <code>E_IRRELEVANT</code> denotes that the task variable never appears as a precondition of
        *     an action, or within one of the goals.
        *   - <code>E_METRICTRACKING<code> denotes that the task variable never appears as a precondition - only in the 
        *     task metric.
        *   - <code>E_SMALLERISBETTER</code> denotes that smaller values of the variable are preferable to larger values
        *     (e.g. all preconditions are of the form <code>(<= v c)</code>, and <code>v</code> can only be increased).
        *   - <code>E_BIGGERISBETTER</code> denotes that smaller values of the variable are preferable to larger values
        *     (e.g. all preconditions are of the form <code>(>= v c)</code>, and <code>v</code> can only be decreased). 
        */
        enum dominance_constraint { E_NODOMINANCE, E_IRRELEVANT, E_METRICTRACKING, E_SMALLERISBETTER, E_BIGGERISBETTER};
        
        /** @brief Enum denoting whether the effects on a variable are order-independent.
        *
        * - <code>E_ORDER_DEPENDENT</code> denotes that the order of the effects could matter
        * - <code>E_ORDER_INDEPENDENT_AT_END</code> denotes that if no actions are executing, the order
        *   of the effects does not matter
        * - <code>E_ORDER_INDEPENDENT</code> denotes that the order of the effects does not matter
        *
        *  For now, order-independence is determined by one of two simple rules.  First, if all the
        *  effects are only increases or decreases by fixed, constant values, then the effects on
        *  a variable are <code>E_ORDER_INDEPENDENT</code>.  Or, if all the other effects are
        *  increases or decreases by a continuous linear effect/duration dependent effect
        *  of actions for which the duration is bounded by fixed, constant values, then the
        *  variable can be classed as <code>E_ORDER_INDEPENDENT_AT_END</code>.  
        */
        enum order_independence { E_ORDER_DEPENDENT, E_ORDER_INDEPENDENT_AT_END, E_ORDER_INDEPENDENT };
        
        #ifdef POPF3ANALYSIS
        /** @brief Struct used to define limits on one-way variable usage, according to the goals.
         *
         * This struct is used to hold the output from <code>NumericAnalysis::findGoalNumericUsageLimits()</code>.
         * The limit is defined on the given variable according to <code>op</code>:
         * - if <code>op = VAL::E_GREATER</code>, then the limit is <code>var > limit</code>, i.e. <code>var</code> must
         *   not be decreased to this point or the relevant goal is unreachable.
         * - if <code>op = VAL::E_GREATEQ</code>, then the limit is <code>var >= limit</code>, i.e. <code>var</code> must
         *   not be decreased beyond this point or the relevant goal is unreachable.
         */
        struct NumericLimitDescriptor {
            /** @brief The variables upon which the limit is defined. */
            map<int,double> var;
            
            /** @brief The operator used when setting the limit. */
            VAL::comparison_op op;
            
            /** @brief The limit on the variable. */
            double limit;
            
            /** @brief If <code>true</code>, the limit is due to the plan metric. */
            bool optimisationLimit;

            NumericLimitDescriptor() {
            }
            
            NumericLimitDescriptor(const int & v, const double & w, const VAL::comparison_op & cOp, const double & lim);
            
            NumericLimitDescriptor(const vector<int> & v, const vector<double> & w, const int & size, const VAL::comparison_op & cOp, const double & lim);
            
            NumericLimitDescriptor(const vector<int> & v, const vector<double> & w, const int & size);
            
            bool operator<(const NumericLimitDescriptor & other) const {
                return (var < other.var);
            }
        };
        
        /** @brief Whether a variable is only in at start conditions.
         *
         * If entry <code>i</code> is <code>true</code>, then variable <code>i</code> is only in
         * <code>at start</code> conditions.
         */
        static vector<bool> onlyAtStartConditionsOnVariable;
        #endif
        
    protected:
        /** @brief Dominance constraints on each task variable.
         * 
         *  These are defined in the order in which they
         *  occur when iterating from <code>Inst::instantiatedOp::pnesBegin()</code> to
         *  <code>Inst::instantiatedOp::pnesEnd()</code>.
         */
        static vector<dominance_constraint> dominanceConstraints;
        
        /** @brief For each task variable, this defines whether the effects on that variable are order-independent. */
        static vector<order_independence> allEffectsAreOrderIndependent;
        
        #ifdef POPF3ANALYSIS        
        /** @brief Ascertain whether it is only ever better for preconditions to have bigger/smaller values of a given fact.
         *
         *  This function is used internally by <code>findDominanceConstraintsAndMetricTrackingVariables()<code>.
         * 
         *  @retval <code>E_BIGGERISBETTER</code>   Bigger values of the variable are preferable
         *  @retval <code>E_SMALLERISBETTER</code>  Smaller values of the variable are preferable
         *  @retval <code>E_NODOMINANCE</code>      Neither bigger nor smaller values are clearly preferable
         */ 
        static dominance_constraint preconditionDominanceInOneDirection(const int & varID);            

        /** @brief Ascertain whether a numeric precondition only supports change the other way
         * 
         *  This captures the modelling idiom that replenishing a resource needs it to be non-full,
         *  which otherwise gives the impression that having less of the resouce might be better;
         *  whereas it's not if:
         *   i) the only actions with a 'less is better' precondition then increase it
         *  ii) there's no gap between what they want it to be less than and what they replace it to
         *      (e.g. < 95, but then assign 100, would make it beneficial to get to below 95).
         * 
         *  Note that this works for both directions (> and <) - it looks at what the precondition
         *  is suggesting.
         * 
         *  @retval <code>true</code>  'Not full' only supports it being replenished
         *  @retval <code>false</code>  Cannot prove this: less might be better.
         */ 
        static bool requiringNotFullOnlySupportsReplenishment(const int & preID);            
        
        
        
        /** @brief Ascertain whether any dominance is broken by effects.
         * 
         * This function is used internally by <code>findDominanceConstraintsAndMetricTrackingVariables()<code>.
         * If a variable is only in preconditions of the form <code>p {>=,>} c</code>, it not necessarily always be preferable
         * for it to be smaller if there is an effect <code>q -= p</code> on a variable which, according to the preconditions,
         * is better to be bigger.  This function considers how the effects might change the dominance rules determined from
         * just the preconditions.  @see preconditionDominanceInOneDirection
         * 
         * @param workingDominanceConstraints  The dominance constraints to be updated based on the effects
         */
        static void considerConflictingDominanceThroughEffects(vector<dominance_constraint> & workingDominanceConstraints);
        
        /** @brief Update dominance constraints according to a duration-dependent effect.
         *
         * This function is used internally by <code>considerConflictingDominanceThroughEffects()</code>.
         * If a variable causes an action's duration to increase, and that action has a continuous effect/duration-dependent
         * effect on another variable, then this affects the dominance constraints:
         * - if the effect is on a variable for which bigger values are preferred:
         *   - if the effect is positive-weighted, bigger durations are preferred; otherwise,
         *   - if the effect is negative-weighted, smaller durations are preferred.
         * - if the effect is on a variable for which smaller values are preferred:
         *   - if the effect is positive-weighted, smaller durations are preferred; otherwise,
         *   - if the effect is negative-weighted, durations durations are preferred.
         * - if the effect is on a variable for which neither smaller nor bigger values are clearly preferable, then
         *   there is no clear preference for the duration.
         * 
         * In all cases, the preference for the duration is used update the dominance constraints on the variables
         * affecting the duration:
         * - if increasing a variable causes the duration to increase/decrease, and that is not what would be preferred,
         *   then bigger values of that variable can no longer be considered better;
         * - if decreasing a variable causes the duration to increase/decrease, and that is not what would be preferred,
         *   then smaller values of that variable can no longer be considered better.
         * 
         * @param v  The variable upon which the effect is acting
         * @param w  The weight of the effect (i.e. positive or negative)
         * @param durVar  The variable affecting the duration
         * @param asIncreasedCausesDurationToDecrease  If <code>true</code>, increasing the variable's value causes the duration
         *                                             of the effect to increase; otherwise, it causes it to decrease.
         * @param workingDominanceConstraints          The current dominance constraints, to be updated as above.
         */
        static void updateForDurationDependence(const int & v, const double & w,
                                                const int & durVar, const bool & asIncreasedCausesDurationToDecrease,
                                                vector<dominance_constraint> & workingDominanceConstraints);
        
                                                
        static void considerCostsOfLiteralGoalsReachedByGoalOnlyOperators();
                                                
        /** @brief Vector to store results of <code>NumericAnalysis::findGoalNumericUsageLimits()</code>. */
        static vector<NumericLimitDescriptor> goalNumericUsageLimits;
                                               
        /** @brief Map (keys are goal indices) to store results of <code>NumericAnalysis::considerCostsOfLiteralGoalsReachedByGoalOnlyOperators()</code>. */
        static vector<map<int, list<int> > > goalHasIndependentCostOnLimit;
        
        static bool thereAreIndependentGoalCosts;
        
        /** @brief Result of the analysis to determine whether the metric is monotonically worsening. */
        static bool metricIsMonotonicallyWorsening;
        
        #endif
        
        static vector<double> maximumPositiveGradientOnVariable;
        static vector<double> maximumNegativeGradientOnVariable;

        static vector<bool> variableOnlyIncreasesByGradients;
        static vector<bool> variableOnlyDecreasesByGradients;

        
        
    public:                
        #ifdef POPF3ANALYSIS
        static bool doGoalLimitAnalysis;
        #endif
        
        /** @brief Use static analysis to find the maximum gradients that can act on a variable
         *
         * This is currently calculated by looping over the actions that have a continuous effect on v,
         * and adding their contribution the maximum positive/negative gradient (for increase/decrease effects,
         * respectively) of v.  If an action can self-overlap arbitrarily many times, this causes
         * the gradient to become infinite.
         */
        static void findMaximumGradients();

        
        static const vector<double> & getMaximumPositiveGradientOnVariable() {
            return maximumPositiveGradientOnVariable;
        }
        
        static const vector<double> & getMaximumNegativeGradientOnVariable() {
            return maximumNegativeGradientOnVariable;
        }
        
        static const vector<bool> & getWhetherVariableOnlyIncreaseByGradients() {
            return variableOnlyIncreasesByGradients;
        }
        
        static const vector<bool> & getWhetherVariableOnlyDecreaseByGradients() {
            return variableOnlyDecreasesByGradients;
        }
        
        
        /** @brief Use static analysis to identify the dominance constraints on the task numeric variables.  
         * 
         *  Each variable is given one of the values from
         *  <code>NumericAnalysis::dominance_constraint</code>.  To obtain the results of this analysis,
         *  use <code>getDominanceConstraints()</code>.
         */
        static void findDominanceConstraintsAndMetricTrackingVariables();
        
        /** @brief Use static analysis to identify the variables for which all effects are order-independent.
         *
         *  The effects on a variable are order-independent if the effects can be 
         *  interleaved arbitrarily without affecting the final change on the 
         *  variable.  To obtain the results of this analysis, use
         *  <code>getDataOnWhichVariablesHaveOrderIndependentEffects()</code>.
         */
        static void findWhichVariablesHaveOrderIndependentEffects();
        
        #ifdef POPF3ANALYSIS
        /** @brief Find which variables have prescribed usage limits in the goal.
         *
         *  This function identifies cases where the numeric goals impose limits on the usage of some
         *  variable in solutions.  Such limits arise in one of two cases:
         *  - <code>v {<=,<} c</code>, where <code>v</code> can only ever be increased;
         *  - <code>v {>=,>} c</code>, where <code>v</code> can only ever be decreased.
         *
         *  In both cases <code>v</code> must have a defined initial value.
         *
         *  To obtain the results of this analysis, call <code>getGoalNumericUsageLimits()</code>.
         */
        static void findGoalNumericUsageLimits();
        
        /** @brief Return the results of the analysis to find usage limits on numeric variables.
        *
        *  @return A const reference to <code>NumericAnalysis::goalNumericUsageLimits</code>,
        *          where each entry is a <code>NumericAnalysis::NumericLimitDescriptor</code> detailing
        *          a limit on a numeric variable according to the goals.
        */        
        static const vector<NumericLimitDescriptor> & getGoalNumericUsageLimits() {
            return goalNumericUsageLimits;
        }
        
        /** @brief Return the results of the analysis to find which literal goals have direct-achivement costs independent of the rest of the problem. */
        static const vector<map<int, list<int> > > & getIndependentGoalCosts() {
            return goalHasIndependentCostOnLimit;
        }
        
        static const bool & areThereAnyIndependentGoalCosts() {
            return thereAreIndependentGoalCosts;
        }
        
        /** @brief Find which variables are only in at start preconditions. */
        static void findWhichVariablesAreOnlyInAtStarts();
        
        /** @brief Return the results of the analysis to find variables only in at-start preconditions. */
        static const vector<bool> & getWhichVariablesAreOnlyInStartPreconditions() {
            return onlyAtStartConditionsOnVariable;
        };
        
        
        static void findWhetherTheMetricIsMonotonicallyWorsening();
        
        static const bool & theMetricIsMonotonicallyWorsening() {
            return metricIsMonotonicallyWorsening;
        }
        
        #endif
        
        /**
         *  Return the results of the analysis to determine which variables have only order-independent
         *  effects, as performed by <code>findWhichVariablesHaveOrderIndependentEffects()</code>.
         *
         *  @return A const reference to <code>NumericAnalysis::allEffectsAreOrderIndependent</code>,
         *          where the entries denote whether the corresponding variable's
         *          effects are order-independent.
         */        
        static const vector<order_independence> & getDataOnWhichVariablesHaveOrderIndependentEffects() {
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
