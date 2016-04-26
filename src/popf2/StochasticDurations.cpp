#include "StochasticDurations.h"
#include "temporalconstraints.h"
#include "RPGBuilder.h"
#include "temporalanalysis.h"

#include "colours.h"

#include <sstream>
#include <cassert>
#include <utility>
#include <fstream>
#include <sstream>
#include <cstdlib>

using std::istringstream;
using std::stringstream;
using std::pair;
using std::endl;
using std::cerr;
using std::ifstream;
using std::getline;

#include <instantiation.h>

using namespace Inst;
using namespace VAL;

#include <gsl/gsl_cdf.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_rng.h>


namespace Planner {

extern bool exceedsSimpleMetricBound(const MinimalState & theState, const bool & verbose);
    
StochasticDurations::StochasticDurations()    
{
}

void lazyError(const instantiatedOp * const op, const PNE * const pne) {
    
    cerr << "Found a state dependent duration on " << *op << ", based on " << *pne << ".  Due to lazy programming, these are not currently supported.\n";
    exit(1);
}

double StochasticDurations::evaluateActionDurationAtPercentile(const int & act)
{
    const RPGBuilder::RPGDuration* const dur = RPGBuilder::getRPGDEs(act)[0];
    
    list<RPGBuilder::DurationExpr *>::const_iterator dItr = dur->fixed.begin();
    
    #ifndef NDEBUG
    const list<RPGBuilder::DurationExpr *>::const_iterator dEnd = dur->fixed.end();    
    assert(dItr != dEnd);
    #endif
    
    double toReturn = 0.0;
            
    if (!(*dItr)->variables.empty()) {
        
        PNE * currPNE;
        if ((*dItr)->variables[0].first != -1) {
            currPNE = RPGBuilder::getPNE((*dItr)->variables[0].first);
            //cout << "Duration governed by variable " << *currPNE << endl;
        } else {
            currPNE = (*dItr)->variables[0].second;
            //cout << "Duration governed by stochastic constant " << *currPNE << endl;
        }    
        
        PDF * const currPDF = PDFsForVariable[currPNE->getGlobalID()];
        
        if (currPDF) {
            toReturn = currPDF->getValueAtPercentile();
        } else {
            if ((*dItr)->variables[0].first != -1) {
                cout << "Duration governed by variable " << *currPNE << endl;
            } else {           
                cout << "Duration governed by stochastic constant " << *currPNE << endl;
            }    
            
            lazyError(RPGBuilder::getInstantiatedOp(act), currPNE);
        }

        toReturn *= (*dItr)->weights[0];
    }
    
    toReturn += (*dItr)->constant;
    
    return toReturn;
}

class SinglePercentileData : public StochasticTimestampData {
  
public:
    double timeAtPercentile;    
    bool hasBeenSampledToGoalAccuracy;
    
    SinglePercentileData(const double & t, const bool & accurate)
        : timeAtPercentile(t), hasBeenSampledToGoalAccuracy(accurate)
    {
    }
    
    /*SinglePercentileData()
        : timeAtPercentile(-1.0), hasBeenSampledToGoalAccuracy(false) {
    }*/
    
    StochasticTimestampData * clone() const {
        return new SinglePercentileData(*this);
    }
    
    double getTimestampForRPGHeuristic() const {
        return timeAtPercentile;
    }
};
    



class MonteCarloDurationManager : public StochasticDurations {
  
protected:
    
    /** @brief The number of samples to use when computing non-goal CDFs.
     *
     *  If this variable equals -1, then use expected durations rather than
     *  sampling for anything but goal facts.
     */
    int nonGoalSamples;
    
    /** @brief The number of samples to use when computing goal CDFs. */
    int samples;
    
    /** @brief Whether to assume durations are deterministic. */
    bool onlyEverUseExpectedValues;
    
    /** @brief Use goal sampling as each goal is achieved, rather than when all are achieved. */
    bool useGoalSamplingForUnitGoalAchievement;
    
    /** @brief  A stochastic duration, repeated a number of times. */
    struct StochasticDurationAndNumberOfTimes {
        /** @brief  The distribution of the duration. */        
        int distributionID;
        
        /** @brief  The parameter controlling the distribution. */
        double rateParameter;
        
        /** @brief  Number of times in which the duration occurs in sequence. */
        int numberOfTimes;
        
        StochasticDurationAndNumberOfTimes()
            : distributionID(-1) {
        }
        
        StochasticDurationAndNumberOfTimes(const int & dist, const double & param, const double & n)
            : distributionID(dist), rateParameter(param), numberOfTimes(n) {
        }
        
        bool operator <(const StochasticDurationAndNumberOfTimes & s) const {
            if (distributionID < s.distributionID) return true;
            if (distributionID > s.distributionID) return false;
            
            if (rateParameter < s.rateParameter) return true;
            if (rateParameter > s.rateParameter) return false;
            
            if (numberOfTimes < s.numberOfTimes) return true;
            
            return false;
        }
    };
    
    
    
    
    /** @brief A cache of the time taken to execute various sequences of activities.
     *
     * The keys of the map correspond to sequences of actions.  They are represented as sets,
     * containing duration definitions (the distribution, the rate parameter, and the number
     * of times it appears in total in the sequence).  The value associated with each sequence
     * is the total time taken for the sequence to complete, at the percentile at which the
     * solution deadline must hold.
     */
    map<set<StochasticDurationAndNumberOfTimes>, double> sequenceCache;
    
    

    
    /** @brief  If <code>true</code>, print debugging information.*/    
    bool debug;
    
    
    double sampleDuration(const int & act) {
        const RPGBuilder::RPGDuration* const dur = RPGBuilder::getRPGDEs(act)[0];
        
        list<RPGBuilder::DurationExpr *>::const_iterator dItr = dur->fixed.begin();
        
        #ifndef NDEBUG
        const list<RPGBuilder::DurationExpr *>::const_iterator dEnd = dur->fixed.end();    
        assert(dItr != dEnd);                
        #endif

        if ((*dItr)->variables.empty()) {
            return (*dItr)->constant;
        }
                
        PNE * currPNE;
        if ((*dItr)->variables[0].first != -1) {
            currPNE = RPGBuilder::getPNE((*dItr)->variables[0].first);
        } else {
            currPNE = (*dItr)->variables[0].second;
        }
        
        PDF * const currPDF = PDFsForVariable[currPNE->getGlobalID()];
        
        if (currPDF) {
            return currPDF->sample();
        } else {
            lazyError(RPGBuilder::getInstantiatedOp(act), currPNE);
            return std::numeric_limits< double >::signaling_NaN();
        }
                
       
    }
        
        
    double expectedDuration(const int & act) {
        const RPGBuilder::RPGDuration* const dur = RPGBuilder::getRPGDEs(act)[0];
        
        list<RPGBuilder::DurationExpr *>::const_iterator dItr = dur->fixed.begin();
        
        #ifndef NDEBUG
        const list<RPGBuilder::DurationExpr *>::const_iterator dEnd = dur->fixed.end();    
        assert(dItr != dEnd);                
        #endif
                
                
        if ((*dItr)->variables.empty()) {
            return (*dItr)->constant;
        }
        
        PNE * currPNE;
        if ((*dItr)->variables[0].first != -1) {
            currPNE = RPGBuilder::getPNE((*dItr)->variables[0].first);
        } else {
            currPNE = (*dItr)->variables[0].second;
        }
        
        PDF * const currPDF = PDFsForVariable[currPNE->getGlobalID()];
        if (currPDF) {
            return currPDF->getExpectedValue();
        } else {
            lazyError(RPGBuilder::getInstantiatedOp(act), currPNE);
            return std::numeric_limits< double >::signaling_NaN();
        }
        
    }
            
    /*double sampleTimestamp(vector<FFEvent*> & planVec, const int & stepID, const TemporalConstraints * const orderings, const bool & deterministicSoFar) {
        
        const map<int,bool> * const cons = orderings->stepsBefore(stepID);
        
        if (!cons) {
            if (debug) {
                cout << stepID << ": No precedence constraints, returning 0.0" << endl;
            }
            return 0.0;
        }
        
        if (planVec[stepID]->time_spec == VAL::E_AT_START) {
            
            if (debug) {
                cout << stepID << ": Start of an action, looking at back edges\n";
            }
            // simple case: all back edges are deterministic - 0 or epsilon
            
            double latestSoFar = 0.0;
            
            map<int,bool>::const_iterator cItr = cons->begin();
            const map<int,bool>::const_iterator cEnd = cons->end();
            
            double newLatest;
            for (; cItr != cEnd; ++cItr) {
                if (deterministicSoFar) {
                    newLatest = (cItr->second ? EPSILON : 0.0) + planVec[cItr->first]->stochasticTimestamp->getTimestampForRPGHeuristic(); //sampleTimestamp(planVec, cItr->first, orderings);
                } else {
                    newLatest = (cItr->second ? EPSILON : 0.0) + sampleTimestamp(planVec, cItr->first, orderings, false);
                } 
            
                if (debug) {
                    cout << " -> " << cItr->first << ": would put it at " << newLatest << endl;
                }
                
                if (newLatest > latestSoFar) {
                    latestSoFar = newLatest;
                }
            }
            
            if (debug) {
                cout << "* Returning " << latestSoFar << endl;
            }
            return latestSoFar;
            
        }
        
        assert(planVec[stepID]->time_spec == VAL::E_AT_END);
        
        if (debug) {
            cout << stepID << ": End of an action, looking at back edges\n";
        }
        
        double latestSoFar = 0.0;
        
        map<int,bool>::const_iterator cItr = cons->begin();
        const map<int,bool>::const_iterator cEnd = cons->end();
                
        double newLatest;
        
        for (; cItr != cEnd; ++cItr) {        
            if (cItr->first == planVec[stepID]->pairWithStep) {
                newLatest = sampleTimestamp(planVec, cItr->first, orderings, false);
                if (debug) {
                    cout << " -> corresponding start " << cItr->first << " was at " << newLatest << ", so with duration = ";
                }                
                newLatest += sampleDuration(planVec[stepID]->action->getID());
                if (debug) {
                    cout << newLatest << endl;
                }
            } else {
                if (deterministicSoFar) {
                    newLatest = (cItr->second ? EPSILON : 0.0) + planVec[cItr->first]->stochasticTimestamp->getTimestampForRPGHeuristic();// sampleTimestamp(planVec, cItr->first, orderings);
                } else {
                    newLatest = (cItr->second ? EPSILON : 0.0) + sampleTimestamp(planVec, cItr->first, orderings, false);
                } 
            
                if (debug) {
                    cout << " -> " << cItr->first << ": would put it at " << newLatest << endl;
                }
                
            }
            if (newLatest > latestSoFar) {
                latestSoFar = newLatest;
            }            
        }
        
        if (debug) {
            cout << "* Returning " << latestSoFar << endl;
        }
        
        return latestSoFar;
    }*/

    double sampleSeveralTimestamps(vector<FFEvent*> & planVec, const int & stepID, const TemporalConstraints * const orderings, const bool & deterministicSoFar) {
        
        if (stepTimestampSampleDirtyBits[stepID] == currentDirtyBit) {
            if (debug) {
                cout << stepID << ": Reusing previous result for step " << stepID << ", returning " << stepTimestampSample[stepID] << endl;
            }
            
            return stepTimestampSample[stepID];
        }
        
        const map<int,bool> * const cons = orderings->stepsBefore(stepID);
        
        if (!cons) {
            if (debug) {
                cout << stepID << ": No precedence constraints on step " << stepID << ", returning 0.0" << endl;
            }
            
            // we should never ask to sample the timestamp of a step with nothing before it
            assert(!recordTemporaryDistributions[stepID]);
            
            stepTimestampSampleDirtyBits[stepID] = currentDirtyBit;
            stepTimestampSample[stepID] = 0.0;
            
            return 0.0;
        }
        
        if (planVec[stepID]->time_spec == VAL::E_AT_START) {
            
            if (debug) {
                cout << stepID << ": Start of an action, looking at back edges\n";
            }
            // simple case: all back edges are deterministic - 0 or epsilon
            
            double latestSoFar = 0.0;
            
            map<int,bool>::const_iterator cItr = cons->begin();
            const map<int,bool>::const_iterator cEnd = cons->end();
            
            double newLatest;
            for (; cItr != cEnd; ++cItr) {
                if (deterministicSoFar) {
                    newLatest = (cItr->second ? EPSILON : 0.0) + planVec[cItr->first]->stochasticTimestamp->getTimestampForRPGHeuristic(); //sampleTimestamp(planVec, cItr->first, orderings);
                } else {
                    newLatest = (cItr->second ? EPSILON : 0.0) + sampleSeveralTimestamps(planVec, cItr->first, orderings, false);
                } 
            
                if (debug) {
                    cout << " -> " << cItr->first << ": would put it at " << newLatest << endl;
                }
                
                if (newLatest > latestSoFar) {
                    latestSoFar = newLatest;
                }
            }
            
            if (debug) {
                cout << "* Returning " << latestSoFar << " for start step " << stepID << endl;
            }
            
            if (recordTemporaryDistributions[stepID]) {
                if (debug) {
                    cout << "Incrementing bucket " << latestSoFar << " for start step " << stepID << endl;
                }
                ++(temporaryDistributions[stepID].insert(make_pair(latestSoFar,0)).first->second);
            }
            
            stepTimestampSampleDirtyBits[stepID] = currentDirtyBit;
            stepTimestampSample[stepID] = latestSoFar;
            
            return latestSoFar;
            
        }
        
        assert(planVec[stepID]->time_spec == VAL::E_AT_END);
        
        if (debug) {
            cout << stepID << ": End of an action, looking at back edges\n";
        }
        
        double latestSoFar = 0.0;
        
        map<int,bool>::const_iterator cItr = cons->begin();
        const map<int,bool>::const_iterator cEnd = cons->end();
                
        double newLatest;
        
        for (; cItr != cEnd; ++cItr) {        
            if (cItr->first == planVec[stepID]->pairWithStep) {
                newLatest = sampleSeveralTimestamps(planVec, cItr->first, orderings, false);
                if (debug) {
                    cout << " -> corresponding start " << cItr->first << " was at " << newLatest << ", so with duration = ";
                }                
                newLatest += sampleDuration(planVec[stepID]->action->getID());
                if (debug) {
                    cout << newLatest << endl;
                }
            } else {
                if (deterministicSoFar) {
                    newLatest = (cItr->second ? EPSILON : 0.0) + planVec[cItr->first]->stochasticTimestamp->getTimestampForRPGHeuristic();// sampleTimestamp(planVec, cItr->first, orderings);
                } else {
                    newLatest = (cItr->second ? EPSILON : 0.0) + sampleSeveralTimestamps(planVec, cItr->first, orderings, false);
                } 
            
                if (debug) {
                    cout << " -> " << cItr->first << ": would put it at " << newLatest << endl;
                }
                
            }
            if (newLatest > latestSoFar) {
                latestSoFar = newLatest;
            }            
        }
        
        if (debug) {
            cout << "* Returning " << latestSoFar << " for end step " << stepID << endl;
        }
        
        if (recordTemporaryDistributions[stepID]) {
            if (debug) {
                cout << "Incrementing bucket " << latestSoFar << " for end step " << stepID << endl;
            }
            ++(temporaryDistributions[stepID].insert(make_pair(latestSoFar,0)).first->second);
        }
        
        stepTimestampSampleDirtyBits[stepID] = currentDirtyBit;
        stepTimestampSample[stepID] = latestSoFar;
        
        return latestSoFar;
    }
    
    
    double expectedTimestamp(vector<FFEvent*> & planVec, const int & stepID, const TemporalConstraints * const orderings) {
        
        const map<int,bool> * const cons = orderings->stepsBefore(stepID);
        
        if (!cons) {
            if (debug) {
                cout << stepID << ": No precedence constraints, returning 0.0" << endl;
            }
            return 0.0;
        }
        
        if (planVec[stepID]->time_spec == VAL::E_AT_START) {
            
            if (debug) {
                cout << stepID << ": Start of an action, looking at back edges\n";
            }
            // simple case: all back edges are deterministic - 0 or epsilon
            
            double latestSoFar = 0.0;
            
            map<int,bool>::const_iterator cItr = cons->begin();
            const map<int,bool>::const_iterator cEnd = cons->end();
            
            double newLatest;
            for (; cItr != cEnd; ++cItr) {
                if (planVec[cItr->first]->stochasticTimestamp) {
                    newLatest = (cItr->second ? EPSILON : 0.0) + planVec[cItr->first]->stochasticTimestamp->getTimestampForRPGHeuristic();
                } else {
                    newLatest = (cItr->second ? EPSILON : 0.0) + expectedTimestamp(planVec, cItr->first, orderings);
                }
                
                if (debug) {
                    cout << " -> " << cItr->first << ": would put it at " << newLatest << endl;
                }
                
                if (newLatest > latestSoFar) {
                    latestSoFar = newLatest;
                }
            }
            
            if (debug) {
                cout << "* Returning " << latestSoFar << endl;
            }
            return latestSoFar;
            
        }
        
        assert(planVec[stepID]->time_spec == VAL::E_AT_END);
        
        if (debug) {
            cout << stepID << ": End of an action, looking at back edges\n";
        }
        
        double latestSoFar = 0.0;
        
        map<int,bool>::const_iterator cItr = cons->begin();
        const map<int,bool>::const_iterator cEnd = cons->end();
                
        double newLatest;
        
        for (; cItr != cEnd; ++cItr) {        
            if (cItr->first == planVec[stepID]->pairWithStep) {
                if (planVec[cItr->first]->stochasticTimestamp) {
                    newLatest = planVec[cItr->first]->stochasticTimestamp->getTimestampForRPGHeuristic();
                } else {
                    newLatest = expectedTimestamp(planVec, cItr->first, orderings);
                }
                if (debug) {
                    cout << " -> corresponding start " << cItr->first << " was at " << newLatest << ", so with duration = ";
                }                
                newLatest += expectedDuration(planVec[stepID]->action->getID());
                if (debug) {
                    cout << newLatest << endl;
                }
            } else {
                
                if (planVec[cItr->first]->stochasticTimestamp) {
                    newLatest = (cItr->second ? EPSILON : 0.0) + planVec[cItr->first]->stochasticTimestamp->getTimestampForRPGHeuristic();
                } else {
                    newLatest = (cItr->second ? EPSILON : 0.0) + expectedTimestamp(planVec, cItr->first, orderings);
                }
                                
                if (debug) {
                    cout << " -> " << cItr->first << ": would put it at " << newLatest << endl;
                }
                
            }
            if (newLatest > latestSoFar) {
                latestSoFar = newLatest;
            }            
        }
        
        if (debug) {
            cout << "* Returning " << latestSoFar << endl;
        }
        
        return latestSoFar;
    }
    
    void usage() {
        cerr << "Error initialising monte-carlo duration manager.  Command line options must fit\n";
        cerr << "one of the following formats:\n\n";
        cerr << "-Mmontecarlo                 - 5000 samples for goal state CDFs; expected durations otherwise\n";
        cerr << "-Mmontecarlo[n]              - [n] samples for goal state CDFs; expected durations otherwise\n";
        cerr << "-Mmontecarlo[n1][n2]         - [n1] samples for goal state CDFs; [n2] samples otherwise\n";
        cerr << "-Mmontecarlo[n1][n2][yes|no] - If 'no', performs the same as -Mmontecarlo[n1][n2].  Otherwise,\n";
        cerr << "                               also uses goal sampling for unit goals in non-goal states.\n";
    }
    
    void status() {
        
        cout << "Using Monte-Carlo duration manager.  Parameters:\n";
        cout << "\tNumber of samples for goals = " << samples << endl;
        if (nonGoalSamples == -1) {
            cout << "\tDo not use sampling for non-goal-achieving steps\n";
        } else {
            cout << "\tNumber of samples for non-goal-achieving steps = " << nonGoalSamples << endl;
        }
        
        if (useGoalSamplingForUnitGoalAchievement) {
            cout << "\tUse goal-sampling for unit goals, as they are achieved\n";
        } else {
            cout << "\tOnly use goal-sampling for states achieving all goals\n";
        }
        
    }
    
public:
    
    MonteCarloDurationManager(const bool & relax)
        : nonGoalSamples(-1), samples(5000), onlyEverUseExpectedValues(relax), useGoalSamplingForUnitGoalAchievement(false), temporaryDistributionSize(0) {
        debug = false;
        
        resizeTemporaryStores(10);
        status();
    }

    MonteCarloDurationManager(const string & params)
        : nonGoalSamples(-1), samples(5000), onlyEverUseExpectedValues(false), useGoalSamplingForUnitGoalAchievement(false), temporaryDistributionSize(0) {

        resizeTemporaryStores(10);
        
        stringstream stream(params);
        string token;
        
        for (int currArg = 0; getline(stream,token,','); ++currArg) {
        
            switch (currArg) {
                case 0:
                {
                    istringstream cnv(token);
                    
                    if (!cnv >> samples) {
                        usage();
                        exit(1);
                    }
                                        
                    break;
                }
                case 1:
                {
                    istringstream cnv(token);
                    
                    if (!cnv >> nonGoalSamples) {
                        usage();
                        exit(1);
                    }
                    
                    break;
                }
                case 2:
                {
                    if (token == "yes") {
                        useGoalSamplingForUnitGoalAchievement = true;
                    } else if (token != "no") {
                        usage();
                        exit(1);
                    }
                    break;
                }
                default:
                {
                    usage();
                }
            }
            
        }
        
        debug = false;
        status();
    }
    
    ~MonteCarloDurationManager() {
    }

    bool migrateMaxDeadlineConstraints(map<double, map<int,double> > & compressionSafeEndEffectOfStepsAtTime,
                                       double & effectMagnitudeExcluded,
                                       list<pair<list<int>,double> > & maxDeadlineConstraints,
                                       const double & deadline, bool pushedYet) {
        
        static const pair<list<int>,double> dummyPair;
        
        effectMagnitudeExcluded = 0.0;
        
        pair<list<int>,double> * dest = (pushedYet ? &(maxDeadlineConstraints.back()) : 0);        
        
        if (!compressionSafeEndEffectOfStepsAtTime.empty()) {
            
            map<double, map<int,double> >::const_iterator csItr = compressionSafeEndEffectOfStepsAtTime.begin();
            const map<double, map<int,double> >::const_iterator csEnd = compressionSafeEndEffectOfStepsAtTime.end();
            
            for (; csItr != csEnd; ++csItr) {
                
                if (csItr->first == -1.0) {
                    
                    // compression-safe end effects are (at this point) recorded as having started at
                    // t=-1.0 - exclude them if requested to do so
                    
                    //cout << "Excluding compression-safe numeric end effect\n";
                    
                    map<int,double>::const_iterator tItr = csItr->second.begin();
                    const map<int,double>::const_iterator tEnd = csItr->second.end();
                    
                    for (; tItr != tEnd; ++tItr) {
                        effectMagnitudeExcluded += tItr->second;
                    }
                    continue;
                }
                
                if (!pushedYet) {
                    maxDeadlineConstraints.push_back(dummyPair);
                    dest = &(maxDeadlineConstraints.back());                                                
                    dest->second = deadline;
                    pushedYet = true;
                }
                                                        
                map<int,double>::const_iterator tItr = csItr->second.begin();
                const map<int,double>::const_iterator tEnd = csItr->second.end();
                
                for (; tItr != tEnd; ++tItr) {
                    dest->first.push_back(tItr->first);
                }
            }            
        }
                              
        return pushedYet;
                                    
    }
    
    /** @brief  Find the SLAs that need checking on new plan steps, due to goals becoming true.
     *
     * @return <code>true</code> if all goals are met; <code>false</code> otherwise.
     */
    bool findDeadlineConstraints(const MinimalState & prevState,  MinimalState & newState,
                                 const vector<FFEvent*> & planVec, const int & appliedStepID,
                                 map<int,double> & deadlineConstraints,
                                 list<pair<list<int>,double> > & maxDeadlineConstraints) {
        
        bool allGoalsMet = true;
        
        static vector<bool> litGoalAlreadyPresent(RPGBuilder::getLiteralGoals().size());
        static vector<bool> numGoalAlreadyPresent(RPGBuilder::getNumericRPGGoals().size() * 2);
        
        
        {
            
            const list<Literal*> & literalGoals = RPGBuilder::getLiteralGoals();
            const list<double> & literalGoalDeadlines = RPGBuilder::getLiteralGoalDeadlines();
            
            list<Literal*>::const_iterator gItr = literalGoals.begin();        
            const list<Literal*>::const_iterator gEnd = literalGoals.end();
            
            list<double>::const_iterator gdItr = literalGoalDeadlines.begin();
            
            int gID = 0;
            
            for (int fID; gItr != gEnd; ++gItr, ++gdItr, ++gID) {
                
                if (RPGBuilder::isStatic(*gItr).first) {
                    litGoalAlreadyPresent[gID] = true;
                    continue;
                }
                                
                fID = (*gItr)->getStateID();
                
                {
                    // first, see if we have it
                    
                    const map<int, PropositionAnnotation>::const_iterator fItr = newState.first.find(fID);
                
                    if (fItr != newState.first.end()) {
                        
                        if (prevState.first.find(fID) == prevState.first.end()) {
                            
                            litGoalAlreadyPresent[gID] = false;
                            
                            if (fItr->second.availableFrom.beforeOrAfter == StepAndBeforeOrAfter::BEFORE) {
                                // special case - is still true from the initial state
                            } else {
                                
                                if (!useGoalSamplingForUnitGoalAchievement) {
                                    newState.literalGoalHoldsFromStep(gID, fItr->second.availableFrom.stepID);
                                }
                                
                                
                                const map<int,double>::iterator insItr = deadlineConstraints.insert(make_pair(fItr->second.availableFrom.stepID, *gdItr)).first;
                                
                                if (insItr->second > *gdItr) {
                                    insItr->second = *gdItr;
                                }
                            }
                                                            
                            continue;
                            
                        } else {
                            litGoalAlreadyPresent[gID] = true;
                            assert(useGoalSamplingForUnitGoalAchievement || newState.stepFromWhichLiteralGoalsHold);
                        }
                        
                    } else {
                        allGoalsMet = false;
                    }
                }
                
                {
                    // then, if we don't have it, see if it's been deleted, as that restricts when it could be added again
                    
                    const map<int, PropositionAnnotation>::const_iterator fItr = newState.retired.find(fID);
                    
                    if (fItr != newState.retired.end()) {
                        
                        map<StepAndBeforeOrAfter, bool>::const_iterator afterItr = fItr->second.addableFrom.begin();
                        const map<StepAndBeforeOrAfter, bool>::const_iterator afterEnd = fItr->second.addableFrom.end();
                        
                        for (; afterItr != afterEnd; ++afterItr) {
                            
                            const map<int,double>::iterator insItr = deadlineConstraints.insert(make_pair(afterItr->first.stepID, *gdItr)).first;
                            
                            if (insItr->second > *gdItr) {
                                insItr->second = *gdItr;
                            }                            
                        }
                    }
                }
                
                // otherwise, it could in theory be added at time 0
            }
        }
           
        {
         
            const list<pair<int,int> > & numericGoals = RPGBuilder::getNumericRPGGoals();
            const list<double> & numericGoalDeadlines = RPGBuilder::getNumericRPGGoalDeadlines();
            
            
            static const int varCount = RPGBuilder::getPNECount();
            
            list<pair<int,int> >::const_iterator gItr = numericGoals.begin();
            const list<pair<int,int> >::const_iterator gEnd = numericGoals.end();
            
            list<double>::const_iterator gdItr = numericGoalDeadlines.begin();
            
            int gID = 0;
            
            for (int fID; gItr != gEnd; ++gItr, ++gdItr) {
                
                for (int ipass = 0; ipass < 2; ++ipass, ++gID) {
                    fID = (ipass ? gItr->second : gItr->first);
                    
                    if (fID < 0) continue;
                    
                    const RPGBuilder::RPGNumericPrecondition & currPre = RPGBuilder::getNumericPreTable()[fID];
                                        
                    if (!currPre.isSatisfiedWCalculate(newState.secondMin, newState.secondMax)) {
                        allGoalsMet = false;
                        continue;
                    }

                    if (currPre.isSatisfiedWCalculate(prevState.secondMin, prevState.secondMax)) {
                        numGoalAlreadyPresent[gID] = true;
                        assert(useGoalSamplingForUnitGoalAchievement || (prevState.stepsFromWhichNumericGoalsHold && prevState.stepsFromWhichNumericGoalsHold[gID]));                        
                        continue;                        
                    }
                                        
                    numGoalAlreadyPresent[gID] = false;
                                                            
                    // if we get this far it's only just become satisfied
                    
                    list<int> trueFromStep;
                    bool pushed = false;
                                                            

                    double effectMagnitudeExcluded = 0.0;
                    
                    {
                        int var = currPre.LHSVariable;
                        
                        // first, we forbid the inclusion of compression-safe end numeric effects from the end of an action that
                        // has just been started
                                                                                                            
                                                                                                            
                                                                                                            
                        if (var >= 2 * varCount) {
                            const RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(var);
                            
                            for (int i = 0; i < currAV.size; ++i) {
                                int varTwo = currAV.fluents[i];
                                double exW = currAV.weights[i];
                                
                                if (varTwo >= varCount) {
                                    varTwo -= varCount;
                                    exW = -exW;
                                }
                                
                                const int stepID = newState.temporalConstraints->lastStepToTouchPNE[varTwo].lastInstantaneousEffect;
                                if (stepID >= 0) {                                    
                                    trueFromStep.push_back(stepID);                                                                        
                                }                                       
                    
                                double localExcluded = 0.0;
                    
                                pushed = migrateMaxDeadlineConstraints(newState.temporalConstraints->lastStepToTouchPNE[varTwo].compressionSafeEndEffectOfStepsAtTime,
                                                                       localExcluded, maxDeadlineConstraints, *gdItr, pushed);                                                                                                                      
                                                                       
                                
                                effectMagnitudeExcluded += (exW * localExcluded);
                            }
                            
                        } else {
                            double exW = 1.0;
                            if (var >= varCount) {
                                var -= varCount;
                                exW = -1.0;
                            }
                            const int stepID = newState.temporalConstraints->lastStepToTouchPNE[var].lastInstantaneousEffect;
                            if (stepID >= 0) {
                                trueFromStep.push_back(stepID);
                            }
                            
                            pushed = migrateMaxDeadlineConstraints(newState.temporalConstraints->lastStepToTouchPNE[var].compressionSafeEndEffectOfStepsAtTime,
                                                                   effectMagnitudeExcluded, maxDeadlineConstraints, *gdItr, pushed);
                            effectMagnitudeExcluded *= exW;
                        }
                        
                        
                        // then, if anything was excluded, we see if it's actually needed
                        
                        if (effectMagnitudeExcluded != 0.0) {
                            
                            var = currPre.LHSVariable;
                            double stateValue;
                            
                            if (var >= 2 * varCount) {
                                const RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(var);
                                
                                stateValue = currAV.evaluateWCalculate(newState.secondMin, newState.secondMax, varCount);                                                                
                            } else if (var >= varCount) {
                                stateValue = newState.secondMin[var - varCount];
                                if (stateValue != 0.0) {
                                    stateValue = -stateValue;
                                }
                            } else {
                                stateValue = newState.secondMax[var];
                            }
                            
                            const double valueWithExclusions = stateValue - effectMagnitudeExcluded;
                            
                            const bool stillSatisfied = (currPre.op == VAL::E_GREATER ? valueWithExclusions > currPre.RHSConstant
                                                                                      : valueWithExclusions >= currPre.RHSConstant);
                                                                                      
                            if (!stillSatisfied) {
                                
                                // we know anything excluded is the end of the compression safe action just applied,
                                // the effects of which are available after the corresponding end (whose step index is
                                // one higher than the start)
                                
                                //cout << "Needed to include the CS end effect of " << *(planVec[appliedStepID]->action) << " to satisfy " << currPre << endl;
                                
                                trueFromStep.push_back(appliedStepID + 1);
                                
                            }
                        }
                                                                                
                                                                                
                    }

                    if (pushed) {
                        maxDeadlineConstraints.back().first.insert(maxDeadlineConstraints.back().first.end(), trueFromStep.begin(), trueFromStep.end());
                    } else {
                        
                        const int tfsSize = trueFromStep.size();
                        if (tfsSize > 1) {
                            pair<list<int>,double> dummyPair;
                            
                            dummyPair.second = *gdItr;
                            dummyPair.first.swap(trueFromStep);
                            
                            maxDeadlineConstraints.push_back(dummyPair);
                            pushed = true;
                        } else if (tfsSize == 1) {
                            
                            list<int>::const_iterator scItr = trueFromStep.begin();
                            const list<int>::const_iterator scEnd = trueFromStep.end();
                            
                            for (; scItr != scEnd; ++scItr) {
                                const map<int,double>::iterator insItr = deadlineConstraints.insert(make_pair(*scItr, *gdItr)).first;
                                
                                if (insItr->second > *gdItr) {
                                    insItr->second = *gdItr;
                                }  
                            }
                        }
                    }
                                                                                                                
                                                                                                                
                    
                    if (!useGoalSamplingForUnitGoalAchievement) {
                        if (pushed) {
                            newState.numericGoalHoldsFromSteps(gID,maxDeadlineConstraints.back().first);
                        } else {
                            newState.numericGoalHoldsFromSteps(gID,trueFromStep);
                        } 
                    
                    }
                    
                    assert(useGoalSamplingForUnitGoalAchievement || (newState.stepsFromWhichNumericGoalsHold && newState.stepsFromWhichNumericGoalsHold[gID]));                        
                                                                
                }
                
            }
        }              

        if (allGoalsMet && !useGoalSamplingForUnitGoalAchievement) {
            // add in extra constraints for previously achieved goals
            
            {
            
                const list<Literal*> & literalGoals = RPGBuilder::getLiteralGoals();
                const list<double> & literalGoalDeadlines = RPGBuilder::getLiteralGoalDeadlines();
                
                list<Literal*>::const_iterator gItr = literalGoals.begin();        
                const list<Literal*>::const_iterator gEnd = literalGoals.end();
                
                list<double>::const_iterator gdItr = literalGoalDeadlines.begin();
                
                for (int gID = 0; gItr != gEnd; ++gItr, ++gdItr, ++gID) {
                    
                    
                    // only add the deadline constraints for non-static, previously satisfied goals
                    
                    if (RPGBuilder::isStatic(*gItr).first || !litGoalAlreadyPresent[gID]) {                       
                        continue;
                    }
                    
                    assert(newState.stepFromWhichLiteralGoalsHold);
                    
                    const int stepID = newState.stepFromWhichLiteralGoalsHold[gID];
                    
                    if (stepID >= 0) {                                    
                        const map<int,double>::iterator insItr = deadlineConstraints.insert(make_pair(stepID, *gdItr)).first;
                        
                        if (insItr->second > *gdItr) {
                            insItr->second = *gdItr;
                        }                                                                       
                    }              
                }
            }
            
            {
                const list<pair<int,int> > & numericGoals = RPGBuilder::getNumericRPGGoals();
                const list<double> & numericGoalDeadlines = RPGBuilder::getNumericRPGGoalDeadlines();
                
                
                list<pair<int,int> >::const_iterator gItr = numericGoals.begin();
                const list<pair<int,int> >::const_iterator gEnd = numericGoals.end();
                
                list<double>::const_iterator gdItr = numericGoalDeadlines.begin();
                
                int gID = 0;
                
                for (int fID; gItr != gEnd; ++gItr, ++gdItr) {
                    
                    for (int ipass = 0; ipass < 2; ++ipass, ++gID) {
                        fID = (ipass ? gItr->second : gItr->first);
                        
                        if (fID < 0) continue;
                        
                        if (!numGoalAlreadyPresent[gID]) {
                            // only add the deadline constraints from previously satisfied goals
                            continue;                            
                        }
                        
                        const int * const orderAfter = newState.stepsFromWhichNumericGoalsHold[gID];
                        
                        assert(orderAfter);
                        
                        if (!orderAfter[0]) {
                            // true initially, so no orderings needed
                            continue;
                        }
                        
                        if (orderAfter[0] == 1) {
                            // need to order after just a single step
                            const map<int,double>::iterator insItr = deadlineConstraints.insert(make_pair(orderAfter[1], *gdItr)).first;
                                                        
                            if (insItr->second > *gdItr) {
                                insItr->second = *gdItr;
                            } 
                        } else {
                            pair<list<int>,double> dummyPair;
                            
                            dummyPair.second = *gdItr;
                            
                            maxDeadlineConstraints.push_back(dummyPair);
                            
                            list<int> & dest = maxDeadlineConstraints.back().first;
                            
                            for (int i = 1; i <= orderAfter[0]; ++i) {
                                dest.push_back(orderAfter[i]);
                            }
                        }
                    }
                }
            }
            
        }
        
        return allGoalsMet;                       
    }
    
    /*bool staysWithinGoalDeadlines(const MinimalState & prevState, const MinimalState & newState, const vector<FFEvent*> & planVec) {

        {
            
            const list<Literal*> & literalGoals = RPGBuilder::getLiteralGoals();
            const list<double> & literalGoalDeadlines = RPGBuilder::getLiteralGoalDeadlines();
            
            list<Literal*>::const_iterator gItr = literalGoals.begin();        
            const list<Literal*>::const_iterator gEnd = literalGoals.end();
            
            list<double>::const_iterator gdItr = literalGoalDeadlines.begin();
            
            for (int fID; gItr != gEnd; ++gItr, ++gdItr) {
                
                if (RPGBuilder::isStatic(*gItr).first) {
                    continue;
                }
                
                fID = (*gItr)->getStateID();
                                                
                {
                    // first, see if we have it
                    
                    const map<int, PropositionAnnotation>::const_iterator fItr = newState.first.find(fID);
                
                    if (fItr != newState.first.end()) {
                        
                        if (prevState.first.find(fID) == prevState.first.end()) {
                            if (fItr->second.availableFrom.beforeOrAfter == StepAndBeforeOrAfter::BEFORE) {
                                // special case - is still true from the initial state
                            } else {
                                
                                const FFEvent* const step = planVec[fItr->second.availableFrom.stepID];
                                
                                if (step->stochasticTimestamp->getTimestampForRPGHeuristic() > *gdItr) {
                                    return false;
                                }
                                
                            }
                            
                            continue;
                        }
                    }                                
                }
                
                {
                    // then, if we don't have it, see if it's been deleted, as that restricts when it could be added again
                    
                    const map<int, PropositionAnnotation>::const_iterator fItr = newState.retired.find(fID);
                    
                    if (fItr != newState.retired.end()) {
                        
                        map<StepAndBeforeOrAfter, bool>::const_iterator afterItr = fItr->second.addableFrom.begin();
                        const map<StepAndBeforeOrAfter, bool>::const_iterator afterEnd = fItr->second.addableFrom.end();
                        
                        for (; afterItr != afterEnd; ++afterItr) {
                            const FFEvent* const step = planVec[afterItr->first.stepID];
                            
                            if (step->stochasticTimestamp->getTimestampForRPGHeuristic() > *gdItr) {
                                
                                // if the condition holds, then the earliest point at which it could be added,
                                // i.e. after a deletor, is too late
                                
                                return false;
                            }
                        }
                    }
                }
                
                // otherwise, it could in theory be added at time 0
            }
            
        }
        
        
        {
         
            const list<pair<int,int> > & numericGoals = RPGBuilder::getNumericRPGGoals();
            const list<double> & numericGoalDeadlines = RPGBuilder::getNumericRPGGoalDeadlines();
            
            
            static const int varCount = RPGBuilder::getPNECount();
            
            list<pair<int,int> >::const_iterator gItr = numericGoals.begin();
            const list<pair<int,int> >::const_iterator gEnd = numericGoals.end();
            
            list<double>::const_iterator gdItr = numericGoalDeadlines.begin();
            
            for (int fID; gItr != gEnd; ++gItr, ++gdItr) {
                
                for (int ipass = 0; ipass < 2; ++ipass) {
                    fID = (ipass ? gItr->second : gItr->first);
                    
                    if (fID < 0) continue;
                    
                    const RPGBuilder::RPGNumericPrecondition & currPre = RPGBuilder::getNumericPreTable()[fID];
                                        
                    if (   currPre.isSatisfiedWCalculate(prevState.secondMin, prevState.secondMax)
                        && currPre.isSatisfiedWCalculate(newState.secondMin, newState.secondMax)) {
                        
                        // if it was previously satisfied and is still satisfied
                        // that's fine
                        
                        continue;
                    }
                    
                    for (int pass = 0; pass < 2; ++pass) {
                        int var = (pass ? currPre.RHSVariable : currPre.LHSVariable);
                        if (var == -1) continue;
                        if (var >= 2 * varCount) {
                            const RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(var);
                            
                            for (int i = 0; i < currAV.size; ++i) {
                                int varTwo = currAV.fluents[i];
                                if (varTwo >= varCount) varTwo -= varCount;
                                
                                const int stepID = newState.temporalConstraints->lastStepToTouchPNE[varTwo].lastInstantaneousEffect;
                                if (stepID >= 0) {
                                    
                                    const FFEvent* const step = planVec[stepID];
                                    
                                    if (step->stochasticTimestamp->getTimestampForRPGHeuristic() > *gdItr) {
                                        return false;
                                    }
                                }                                                                
                            }
                            
                        } else {
                            if (var >= varCount) var -= varCount;
                            const int stepID = newState.temporalConstraints->lastStepToTouchPNE[var].lastInstantaneousEffect;
                            if (stepID >= 0) {
                                
                                const FFEvent* const step = planVec[stepID];
                                
                                if (step->stochasticTimestamp->getTimestampForRPGHeuristic() > *gdItr) {
                                    return false;
                                }
                            }                    
                        }
                    }
                }
                
            }
        }
         
        return true;
    }*/

    template <typename T>
    void findAllBefore(const int & stepID,
                       int & maxStepID,
                       list<int> & toVisit, const TemporalConstraints * const orderings,
                       T tmpItr, const T & stepItrEnd,
                       pair<set<int>, set<int> > & dest) {
            
        static const bool localdebug = false;
        
        set<int> allBefore;
        
        while (!toVisit.empty()) {

            const int currStep = toVisit.front();
            
            toVisit.pop_front();
            
            if (localdebug && currStep != stepID) {
                cout << currStep << " ";
                cout.flush();
            }

            if (maxStepID < currStep) {
                maxStepID = currStep;
            }


            const map<int, bool> * const cons = orderings->stepsBefore(currStep);

            if (!cons) {
                continue;
            }

            map<int, bool>::const_iterator cItr = cons->begin();
            const map<int, bool>::const_iterator cEnd = cons->end();

            for (; cItr != cEnd; ++cItr) {
                if (allBefore.insert(cItr->first).second) {
                    toVisit.push_back(cItr->first);
                }
            }
        }                        
         
        set<int>::const_iterator cItr = allBefore.begin();
        const set<int>::const_iterator cEnd = allBefore.end();

        while (tmpItr != stepItrEnd && cItr != cEnd) {
            if (*cItr < tmpItr->first) {
                ++cItr;
            } else if (tmpItr->first < *cItr) {
                ++tmpItr;
            } else {
                if (localdebug) {
                    cout << " [" << *cItr << "]";
                    cout.flush();
                }
                dest.first.insert(*cItr);
                ++cItr;
                ++tmpItr;
            }
        }
        
        if (stepID >= 0) {
            allBefore.insert(stepID);
        }
        
        allBefore.swap(dest.second);
        
    }
    
    int temporaryDistributionSize;
    vector<map<double,int> > temporaryDistributions;
    vector<bool> recordTemporaryDistributions;
    
    vector<bool> stepTimestampSampleDirtyBits;
    bool currentDirtyBit;
    vector<double> stepTimestampSample;
    
    void resizeTemporaryStores(const int & newSize) {
        
        if (newSize < temporaryDistributionSize) {
            return;
        }
        
        temporaryDistributionSize = newSize + 10;
        
        temporaryDistributions.resize(temporaryDistributionSize);
        recordTemporaryDistributions.resize(temporaryDistributionSize, false);
        
        stepTimestampSampleDirtyBits.resize(temporaryDistributionSize);
        
        #ifndef NDEBUG
        stepTimestampSample.resize(temporaryDistributionSize, std::numeric_limits< double >::signaling_NaN());        
        #else
        stepTimestampSample.resize(temporaryDistributionSize);        
        #endif
        
    }
    
    inline void setDirtyBitsToTrue() {
        
        for (int m = 0; m < temporaryDistributionSize; ++m) {
            stepTimestampSampleDirtyBits[m] = true;
        }
        
    }
    
    /** @brief  Update the estimates of the times at which a given sequence of steps are reached.
     *
     * @param theseAreGoalSteps  If <code>true</code> use goal-quality sampling for the steps; otherwise, use
     *                           the other method specified (expected values, sampling, etc.)
     */
    template <typename T>
    bool updateSeveralTimestampsQuickly(T stepItr, const T & stepItrEnd,
                                        const list<pair<list<int>, double> > & maxDeadlineConstraints,
                                        vector<FFEvent*> & planVec, const TemporalConstraints * const orderings,
                                        const bool & theseAreGoalSteps)
    {
        
        const bool localdebug = false;

        if (!theseAreGoalSteps && nonGoalSamples == -1) {
             
            // if these aren't goal-achieving steps, and we aren't using sampling for non-goal steps, plug in
            // expected values and call it good.

            if (debug || localdebug) {
                cout << "Finding expected timestamps of one or more steps:\n";
            }

            for (; stepItr != stepItrEnd; ++stepItr) {
             
                const double est = expectedTimestamp(planVec, stepItr->first, orderings);

                delete planVec[stepItr->first]->stochasticTimestamp;

                planVec[stepItr->first]->stochasticTimestamp = new SinglePercentileData(est, false);

                if (debug) {
                    cout << "\tStep " << stepItr->first << ": " << est << std::endl;
                }

            }

            return true;

        }
         
        const int samplesToUse = (theseAreGoalSteps ? samples : nonGoalSamples);

        int maxStepID = 0;
        
        map<int, pair<set<int>, set<int> > > numberOfStepsBeforeIt;

        const T stepBegin = stepItr;
         
        for (; stepItr != stepItrEnd; ++stepItr) {
            
            if (   planVec[stepItr->first]->stochasticTimestamp
                && (!theseAreGoalSteps || dynamic_cast<SinglePercentileData*>(planVec[stepItr->first]->stochasticTimestamp)->hasBeenSampledToGoalAccuracy) ) {
                
                if (localdebug) {
                    cout << COLOUR_yellow << "Already have samples of the time to reach step " << stepItr->first << " with the necessary accuracy\n" << COLOUR_default;
                }
                
                continue;
            }
            
            if (localdebug) {
                cout << COLOUR_yellow << "Looking at consequences of sampling step " << stepItr->first;
                if (theseAreGoalSteps) {
                    cout << " due to being in a goal state\n";
                }
                cout << COLOUR_default << endl;                
            }            

            {
                const map<int, bool> * const cons = orderings->stepsBefore(stepItr->first);

                if (!cons) {
                    pair<set<int>, set<int> > dummyPair;
                    numberOfStepsBeforeIt[stepItr->first] = dummyPair;
                    if (maxStepID < stepItr->first) {
                        maxStepID = stepItr->first;
                    }
                    if (localdebug) {
                        cout << " - No predecessors; so none at all.\n";
                    }
                    continue;
                }
            }
 
            list<int> toVisit;
             
            toVisit.push_back(stepItr->first);

            pair<set<int>, set<int> > & dest = numberOfStepsBeforeIt[stepItr->first];
            
            findAllBefore<T>(stepItr->first, maxStepID, toVisit, orderings, stepBegin, stepItrEnd, dest);
            
            if (localdebug) {
                cout << endl;
            }
                          
        }
        
        vector<const pair<list<int>, double>* > fakeStepsForMDCs;                
        
        int fakeCount = 0;
        
        if (!maxDeadlineConstraints.empty()) {
            
            fakeCount = maxDeadlineConstraints.size();
            fakeStepsForMDCs.resize(fakeCount, 0);
            
            list<pair<list<int>, double> >::const_iterator mdcItr = maxDeadlineConstraints.begin();
            const list<pair<list<int>, double> >::const_iterator mdcEnd = maxDeadlineConstraints.end();
            
            for (int offset = -1; mdcItr != mdcEnd; ++mdcItr, --offset) {
                
                list<int> toVisit;
                
                if (localdebug) {
                    cout << "Finding orderings before";
                }
                
                list<int>::const_iterator sItr = mdcItr->first.begin();
                const list<int>::const_iterator sEnd = mdcItr->first.end();
                                            
                
                for (; sItr != sEnd; ++sItr) {
                    const map<int, bool> * const cons = orderings->stepsBefore(*sItr);
                                                
                    if (cons && !cons->empty()) {
                        if (localdebug) {
                            cout << " " << *sItr;
                        }
                        
                        toVisit.push_back(*sItr);
                    } else {
                        if (localdebug) {
                            cout << " [not " << *sItr << "]";
                        }
                    }
                }
                
                if (toVisit.empty()) {
                    if (localdebug) {
                        cout << " - No predecessors; so none at all.\n";
                    }
                    continue;
                }

                if (localdebug) {
                    cout << endl;
                }   
                
                
                
                fakeStepsForMDCs[-1-offset] = &(*mdcItr);
                
                pair<set<int>, set<int> > & dest = numberOfStepsBeforeIt[offset];
                
                findAllBefore<T>(offset, maxStepID, toVisit, orderings, stepBegin, stepItrEnd, dest);
                
            }
        }
        
        if (localdebug) {
            cout << COLOUR_light_green << "Now looping over Bayesian network estimation\n" << COLOUR_default;
        }
        
        resizeTemporaryStores(maxStepID);
        
        const int stopAt = ceil(samplesToUse * solutionDeadlinePercentage);
                
        while (!numberOfStepsBeforeIt.empty()) {
            
            map<int, pair<set<int>,set<int> > >::iterator biggestItr = numberOfStepsBeforeIt.begin();
            
            map<int, pair<set<int>,set<int> > >::iterator bsItr = biggestItr;
            const map<int, pair<set<int>,set<int> > >::iterator bsEnd = numberOfStepsBeforeIt.end();
            
            ++bsItr;
            
            for (; bsItr != bsEnd; ++bsItr) {
                if (bsItr->second.first.size() > biggestItr->second.first.size()) {
                    biggestItr = bsItr;
                }
            }

            assert(   biggestItr->first <= -1
                   || !planVec[biggestItr->first]->stochasticTimestamp
                   || !dynamic_cast<SinglePercentileData*>(planVec[biggestItr->first]->stochasticTimestamp)->hasBeenSampledToGoalAccuracy);
            
            
            if (localdebug) {
                cout << "Running a Bayesian Network trace back from ";
                if (biggestItr->first >= 0) {
                    cout << "step " << biggestItr->first << endl;
                } else {
                    cout << "artificial step " << -1 - biggestItr->first << endl;
                }
            }

            {
                set<int>::const_iterator duItr = biggestItr->second.second.begin();
                const set<int>::const_iterator duEnd = biggestItr->second.second.end();
                
                for (; duItr != duEnd; ++duItr) {
                    if (  !planVec[*duItr]->stochasticTimestamp
                       || !dynamic_cast<SinglePercentileData*>(planVec[*duItr]->stochasticTimestamp)->hasBeenSampledToGoalAccuracy) {
                        
                        if (localdebug) {
                            cout << "* Updating recorded CDF for " << *duItr << endl;
                        }

                        recordTemporaryDistributions[*duItr] = true;
                        
                    }
                    
                }
            }
            
            if (biggestItr->first >= 0 ) {
            
                setDirtyBitsToTrue();
                
                for (int s = 0; s < samplesToUse; ++s) {
                    
                    currentDirtyBit = (s & 1);
                    
                    if (localdebug && s < 2) {
                        cout << COLOUR_light_red << "Sample run " << s;
                        if (currentDirtyBit) {
                            cout << " - dirty bit true\n";
                        } else {
                            cout << " - dirty bit false\n";
                        }
                        cout << COLOUR_default;
                        debug = true;
                    }                                        
                    
                    sampleSeveralTimestamps(planVec, biggestItr->first, orderings, false);
                    debug = false;
                }
                
            } else {
                
                const list<int> & maxOf = fakeStepsForMDCs[-1 - biggestItr->first]->first;
                
                list<int>::const_iterator mItr;
                const list<int>::const_iterator mEnd = maxOf.end();
                
                double latestSoFar;
                double newlatest;
                
                map<double,int> pdf;
                
                setDirtyBitsToTrue();
                
                for (int s = 0; s < samplesToUse; ++s) {
                    currentDirtyBit = (s & 1);
                    
                    if (localdebug && s < 2) {
                        cout << COLOUR_light_red << "Sample run " << s;
                        if (currentDirtyBit) {
                            cout << " - dirty bit true\n";
                        } else {
                            cout << " - dirty bit false\n";
                        }
                        cout << COLOUR_default;
                        debug = true;
                    }
                    latestSoFar = 0.0;
                    
                    for (mItr = maxOf.begin(); mItr != mEnd; ++mItr) {
                        newlatest = sampleSeveralTimestamps(planVec, *mItr, orderings, false);
                        
                        if (newlatest > latestSoFar) {
                            latestSoFar = newlatest;
                        }
                    }
                    ++(pdf.insert(make_pair(latestSoFar,0)).first->second);
                    debug = false;
                }
                
                int s = 0;
                
                map<double,int>::iterator dItr = pdf.begin();
                const map<double,int>::iterator dEnd = pdf.end();
                
                for (; dItr != dEnd; ++dItr) {
                    s += dItr->second;
                    if (s >= stopAt) {
                        break;
                    }
                }

                if (dItr->first > fakeStepsForMDCs[-1 - biggestItr->first]->second) {
                    return false;
                }
                
            } 
        
            
            set<int> updated;
            updated.swap(biggestItr->second.second);
            
            if (biggestItr->first <= -1) {            
                numberOfStepsBeforeIt.erase(biggestItr);
            }
            
            set<int>::const_iterator duItr = updated.begin();
            const set<int>::const_iterator duEnd = updated.end();
            
            for (; duItr != duEnd; ++duItr) {
                
                if (!recordTemporaryDistributions[*duItr]) {
                    continue;
                }
                
                int s = 0;
                
                assert(!temporaryDistributions[*duItr].empty());

                map<double,int>::iterator dItr = temporaryDistributions[*duItr].begin();
                const map<double,int>::iterator dEnd = temporaryDistributions[*duItr].end();
                
                for (; dItr != dEnd; ++dItr) {
                    s += dItr->second;
                    if (s >= stopAt) {
                        break;
                    }
                }
                
                delete planVec[*duItr]->stochasticTimestamp;

                planVec[*duItr]->stochasticTimestamp = new SinglePercentileData(dItr->first, theseAreGoalSteps);

                temporaryDistributions[*duItr].clear();
                recordTemporaryDistributions[*duItr] = false;
                
                numberOfStepsBeforeIt.erase(*duItr);

                if (localdebug) {
                    cout << "> Sampled CDF for step " << *duItr << " at percentile is " << dItr->first << endl;
                }
            }
            

            
        }
        
        return true;
        
    }
    
    /** @brief  The main way in which the planner interacts with the Monte-Carlo sampling. */  
    bool updateTimestampsOfNewPlanStep(const MinimalState & prevState, MinimalState & newState, list<FFEvent> & header, FFEvent * const newStep, FFEvent* const newStepPairedWith, const int & stepID, bool & isAGoalState) {
      
        static const bool localdebug = false;
        
        if (localdebug) {
            cout << COLOUR_yellow << "::updateTimestampsOfNewPlanStep()" << COLOUR_default << endl;            
        }
        
        isAGoalState = false;
        
        map<int, pair<int, set<int> > > stepsThatNeedUpdating;
        set<int> relevantStepIDs;
        
        static pair<int, set<int> > emptyUpdateEntry;
        emptyUpdateEntry.first = 0;
        
        relevantStepIDs.insert(stepID);
        
        if (newStep->time_spec == VAL::E_AT_START) {

            if (localdebug) {                
                cout << "Starting action " << *(newStep->action) << " at step " << stepID << std::endl;                
            }
            
            const map<int,bool> * const cons = newState.temporalConstraints->stepsBefore(stepID);
            
            if (!cons) {
                // simple case - action with no predecessors can go at time zero           
                newStep->stochasticTimestamp = new SinglePercentileData(0.0, true);
                if (localdebug) {                
                    cout << "- Can go at time 0\n";
                }                
            } else {
                if (localdebug) {                
                    cout << "- Start has predecessors, so needs to be computed\n";
                }                
                stepsThatNeedUpdating.insert(make_pair(stepID, emptyUpdateEntry));
            }
            
            if (newStepPairedWith) {
                relevantStepIDs.insert(stepID + 1);
                const map<int,bool> * const consTwo = newState.temporalConstraints->stepsBefore(stepID + 1);
                bool simple = (!consTwo && !cons);
                
                if (!simple && !cons && consTwo->size() == 1) {
                    const int before = consTwo->begin()->first;
                    if (before == stepID) {
                        simple = true;
                    }
                }
                
                if (simple) {       
                    double durCalc;
                    
                    if (onlyEverUseExpectedValues) {
                        durCalc = expectedDuration(newStep->action->getID());
                    } else {
                        durCalc = evaluateActionDurationAtPercentile(newStep->action->getID());
                    }
                    
                    newStepPairedWith->stochasticTimestamp = new SinglePercentileData(durCalc, !onlyEverUseExpectedValues);
                    if (localdebug) {                
                        cout << "- Corresponding end is simply the duration after it = " << newStepPairedWith->stochasticTimestamp->getTimestampForRPGHeuristic() << endl;
                    } 
                } else {
                    if (localdebug) {                
                        cout << "- Corresponding end needs to be computed\n";
                    }                
                    
                    stepsThatNeedUpdating.insert(make_pair(stepID + 1, emptyUpdateEntry));
                }
            } else {
                assert(RPGBuilder::getRPGDEs(newStep->action->getID()).empty());
            }
                                                        
        } else {
            
            assert(newStep->time_spec == VAL::E_AT_END);
                
            relevantStepIDs.insert(stepID - 1);
            
            const map<int,bool> * const cons = newState.temporalConstraints->stepsBefore(stepID - 1);            
            const map<int,bool> * const consTwo = newState.temporalConstraints->stepsBefore(stepID);
            bool simple = (!consTwo && !cons);
            
            if (!simple && !cons && consTwo->size() == 1) {
                const int before = consTwo->begin()->first;
                if (before == stepID - 1) {
                    simple = true;
                }
            }
            
            if (simple) {
                double durCalc;
                
                if (onlyEverUseExpectedValues) {
                    durCalc = expectedDuration(newStep->action->getID());
                } else {
                    durCalc = evaluateActionDurationAtPercentile(newStep->action->getID());
                }
                
                newStep->stochasticTimestamp = new SinglePercentileData(durCalc, !onlyEverUseExpectedValues);
            } else {
                stepsThatNeedUpdating.insert(make_pair(stepID, emptyUpdateEntry));
            }
            
        }
        
        const unsigned int stepCount = newState.temporalConstraints->size();
        
        for (unsigned int s = 0; s < stepCount; ++s) {
            const map<int, bool> * const cons = newState.temporalConstraints->stepsBefore(s);
            if (!cons) continue;
            
            if (relevantStepIDs.find(s) != relevantStepIDs.end()) continue;
            
            map<int, bool>::const_iterator pItr = cons->begin();
            const map<int, bool>::const_iterator pEnd = cons->end();
            
            set<int>::const_iterator rItr = relevantStepIDs.begin();
            const set<int>::const_iterator rEnd = relevantStepIDs.end();
            
            while (pItr != pEnd && rItr != rEnd) {
                if (pItr->first < *rItr) {
                    ++pItr;
                } else if (*rItr < pItr->first) {
                    ++rItr;
                } else {
                    stepsThatNeedUpdating.insert(make_pair(s, emptyUpdateEntry));
                    break;
                }                                
            }
        }

        const int hSize = header.size();
        
        vector<FFEvent*> planVec;
        
        if (newStep && newStepPairedWith) {
            planVec.resize(hSize + 2);
        } else if (newStep && newStep->time_spec != VAL::E_AT_END) {
            planVec.resize(hSize + 1);
        } else {
            planVec.resize(hSize);
        }
        
        {
            int s = 0;
            
            list<FFEvent>::iterator pItr = header.begin();
            const list<FFEvent>::iterator pEnd = header.end();
            
            for (; pItr != pEnd; ++pItr, ++s) {
                planVec[s] = &(*pItr);
            }
            
            if (newStep && newStepPairedWith) {
                planVec[s++] = newStep;
                planVec[s++] = newStepPairedWith;
            } else if (newStep && newStep->time_spec != VAL::E_AT_END) {
                planVec[s++] = newStep;
            }
        }
        
        map<int,double> deadlineConstraints;
        list<pair<list<int>,double> > maxDeadlineConstraints;
        
        isAGoalState = findDeadlineConstraints(prevState, newState, planVec, stepID, deadlineConstraints, maxDeadlineConstraints) && newState.startedActions.empty();
        
        const bool haveAnyConstraints = !(deadlineConstraints.empty() && maxDeadlineConstraints.empty());
        
        if (!haveAnyConstraints || (!useGoalSamplingForUnitGoalAchievement && !isAGoalState)) {
            
            // short-circuit anything clever and just skip to whatever means is used to estimate
            // step timestamps during search if we aren't at a point where goal-quality sampling is needed
            
            debug = localdebug;

            const bool retVal = updateSeveralTimestampsQuickly(stepsThatNeedUpdating.begin(), stepsThatNeedUpdating.end(),
                                                               maxDeadlineConstraints,
                                                               planVec, newState.temporalConstraints, false);
            
            if (!retVal) return false;
           
            if (haveAnyConstraints) {
                {
                    map<int,double>::const_iterator dcItr = deadlineConstraints.begin();
                    const map<int,double>::const_iterator dcEnd = deadlineConstraints.end();

                    for (; dcItr != dcEnd; ++dcItr) {
                        if (planVec[dcItr->first]->stochasticTimestamp->getTimestampForRPGHeuristic() > dcItr->second) {
                            return false;
                        }
                    }
                }
                
                {
                
                    list<pair<list<int>, double> >::const_iterator mdcItr = maxDeadlineConstraints.begin();
                    const list<pair<list<int>, double> >::const_iterator mdcEnd = maxDeadlineConstraints.end();
                    
                    for (; mdcItr != mdcEnd; ++mdcItr) {
                        list<int>::const_iterator sItr = mdcItr->first.begin();
                        const list<int>::const_iterator sEnd = mdcItr->first.end();
                        
                        for (; sItr != sEnd; ++sItr) {
                            if (planVec[*sItr]->stochasticTimestamp->getTimestampForRPGHeuristic() > mdcItr->second) {
                                return false;
                            }
                        }
                    }
                }
            }
                                                               
            debug = false;

            return true;
            
        }
        
        if (isAGoalState && !useGoalSamplingForUnitGoalAchievement && Globals::globalVerbosity & 1) {
            cout << COLOUR_light_blue << "G" << COLOUR_default;
            cout.flush();
        }
        
        const bool checkVal = updateSeveralTimestampsQuickly(deadlineConstraints.begin(), deadlineConstraints.end(),
                                                             maxDeadlineConstraints,
                                                             planVec, newState.temporalConstraints, !onlyEverUseExpectedValues);
        
        if (!checkVal) {
            return false;
        }

        {
            map<int,double>::const_iterator dcItr = deadlineConstraints.begin();
            const map<int,double>::const_iterator dcEnd = deadlineConstraints.end();

            for (; dcItr != dcEnd; ++dcItr) {
                if (localdebug) {
                    cout << "Samplied timestamp of step " << dcItr->first << " = " << dcItr->second << endl;
                }
                if (planVec[dcItr->first]->stochasticTimestamp->getTimestampForRPGHeuristic() > dcItr->second) {
                    return false;
                }
            }

        }
        
        map<int,bool> tmpMap;

        map<int,bool>::iterator insItr = tmpMap.end();

        map<int, pair<int, set<int> > >::const_iterator sItr = stepsThatNeedUpdating.begin();
        const map<int, pair<int, set<int> > >::const_iterator sEnd = stepsThatNeedUpdating.end();

        map<int,double>::const_iterator dcItr = deadlineConstraints.begin();
        const map<int,double>::const_iterator dcEnd = deadlineConstraints.end();


        while (dcItr != dcEnd && sItr != sEnd) {
            if (dcItr->first < sItr->first) {
                ++dcItr;
            } else if (sItr->first < dcItr->first) {
                insItr = tmpMap.insert(insItr, make_pair(sItr->first,false));
                ++sItr;
            } else {
                ++dcItr; ++sItr;
            }
        }

        for (; sItr != sEnd; ++sItr) {
            insItr = tmpMap.insert(insItr, make_pair(sItr->first,false));
        }

        return updateSeveralTimestampsQuickly(tmpMap.begin(), tmpMap.end(), maxDeadlineConstraints, planVec, newState.temporalConstraints, false);
            

    };
    
    virtual void prepareTheInitialState(MinimalState & initialState) {
        
        if (useGoalSamplingForUnitGoalAchievement) {
            // we only need to do this to mark initially-true goals as 'past achieved' (persisting up to the point at which they're destroyed)
            // for if we're only looking whether the deadlines have been met in complete goal states
            
            return;            
        }
        
        {
            
            const list<Literal*> & literalGoals = RPGBuilder::getLiteralGoals();
            const list<double> & literalGoalDeadlines = RPGBuilder::getLiteralGoalDeadlines();
            
            list<Literal*>::const_iterator gItr = literalGoals.begin();        
            const list<Literal*>::const_iterator gEnd = literalGoals.end();
            
            list<double>::const_iterator gdItr = literalGoalDeadlines.begin();
            
            int gID = 0;
            
            for (int fID; gItr != gEnd; ++gItr, ++gdItr, ++gID) {
                
                if (!RPGBuilder::isStatic(*gItr).first) {                    
                    
                    fID = (*gItr)->getStateID();
                    
                    const map<int, PropositionAnnotation>::const_iterator fItr = initialState.first.find(fID);
                    if (fItr == initialState.first.end()) {
                        continue;
                    }
                }
                
                // if we get this far, we have the goal, either because it's static, or in the initial state
                initialState.literalGoalHoldsFromStep(gID, -1);
            }
        }
        
        {
            
            const list<pair<int,int> > & numericGoals = RPGBuilder::getNumericRPGGoals();
            const list<double> & numericGoalDeadlines = RPGBuilder::getNumericRPGGoalDeadlines();
            
            list<pair<int,int> >::const_iterator gItr = numericGoals.begin();
            const list<pair<int,int> >::const_iterator gEnd = numericGoals.end();
            
            list<double>::const_iterator gdItr = numericGoalDeadlines.begin();
            
            int gID = 0;
            
            for (int fID; gItr != gEnd; ++gItr, ++gdItr) {
                
                for (int ipass = 0; ipass < 2; ++ipass, ++gID) {
                    fID = (ipass ? gItr->second : gItr->first);
                    
                    if (fID < 0) continue;
                                        
                    const RPGBuilder::RPGNumericPrecondition & currPre = RPGBuilder::getNumericPreTable()[fID];
                    
                    if (currPre.isSatisfiedWCalculate(initialState.secondMin, initialState.secondMax)) {                        
                        
                        static list<int> emptyList;
                        
                        initialState.numericGoalHoldsFromSteps(gID, emptyList);
                        
                    }
                }
            }
        }
        
    }
    
};
    
StochasticDurations * durationManager = 0;
double solutionDeadlineTime = DBL_MAX;
double solutionDeadlinePercentage;

void setDurationManager(const string & name)
{
    assert(durationManager == 0);
    
    if (name.find("montecarlo") == 0) {
        if (name.length() == 10) {
            durationManager = new MonteCarloDurationManager(false);
        } else {
            durationManager = new MonteCarloDurationManager(name.substr(10));
        }
    } else if (name.find("deterministic") == 0) {
        durationManager = new MonteCarloDurationManager(true);
    } else {
        cerr << "Unknown duration manager configuration '" << name << "'\n";
        cerr << "Possible options are:\n";
        cerr << "\tmontecarlo[options] - monte-carlo sampling on a Bayes net\n";
        cerr << "\tdeterministic - use mean durations, assume model is deterministic\n";
        cerr << endl;
        exit(1);
    }
}

void setSolutionDeadlineTimeToLatestGoal()
{
    
    solutionDeadlineTime = 0;
    
    for (int pass = 0; pass < 2; ++pass) {
        const list<double> & loopOver = (pass ? RPGBuilder::getNumericRPGGoalDeadlines() : RPGBuilder::getLiteralGoalDeadlines());
        
        list<double>::const_iterator loItr = loopOver.begin();
        const list<double>::const_iterator loEnd = loopOver.end();
        
        for (; loItr != loEnd; ++loItr) {
            if (solutionDeadlineTime < *loItr) {
                solutionDeadlineTime = *loItr;
            }
        }
        
    }
}


class CustomPDF : public PDF {
    
    
protected:
    
    map<double, int> samples;
    map<int,double> cdf;
    
    int numberOfSamples;    
    double valueAtPercentile;
    bool haveCalculatedValueAtPercentile;
    
public:
    
    CustomPDF(const string & filename);
      
    double getValueAtPercentile();            
    double sample();
};


CustomPDF::CustomPDF(const string & filename)
{

    /*
    
    map<double, int> samples;
    map<int,double> cdf;
    
    double expectedValue;
    
    */
    
    
    ifstream current_in_stream(filename.c_str());
    
    if (!current_in_stream.good()) {
        cout << "Exiting: could not open plan file " << filename << "\n";
        exit(1);
    }
    
    
    {
        map<double, int>::iterator insItr = samples.end();
            
        string line;
        int comma;
        
        while (!current_in_stream.eof()) {
            
            getline(current_in_stream, line);
            
            
            comma = line.find(',');
            
            const string bucket = line.substr(0, comma);
            const string entryCount = line.substr(comma + 1);
            
            istringstream buffer1(bucket);
            double bucketID;
            
            if (!(buffer1 >> bucketID)) {
                cout << "Exiting: in distribution file " << filename << ", cannot interpret " << bucket << " as a numeric value for a PDF bucket\n";
            }

            istringstream buffer2(entryCount);
            int bucketSize;

            if (!(buffer2 >> bucketSize)) {
                cout << "Exiting: in distribution file " << filename << ", cannot interpret " << entryCount << " as an integral value for the size of a PDF bucket\n";
            }
            
            insItr = samples.insert(insItr, make_pair(bucketID, bucketSize));
            
        }
    }
    
    current_in_stream.close();
    
    {
        expectedValue = 0.0;
        numberOfSamples = 0;
    
        map<int, double>::iterator insItr = cdf.end();
        
        map<double, int>::iterator sItr = samples.begin();
        const map<double, int>::iterator sEnd = samples.end();
        
        for (; sItr != sEnd; ++sItr) {
            numberOfSamples += sItr->second;
            expectedValue += (sItr->first * sItr->second);
            insItr = cdf.insert(insItr, make_pair(numberOfSamples, sItr->first));
        }
        
        expectedValue /= numberOfSamples;
    }
    
    srand(2010);
    haveCalculatedValueAtPercentile = false;
}

double CustomPDF::sample()
{
    const int sampleID = (rand() % numberOfSamples);
    
    const map<int, double>::const_iterator cdfItr = cdf.lower_bound(sampleID);
    
    assert(cdfItr != cdf.end());
    
    return cdfItr->second;
}

double CustomPDF::getValueAtPercentile()
{
    if (haveCalculatedValueAtPercentile) {
        return valueAtPercentile;
    }
    
    const map<int, double>::const_iterator cdfItr = cdf.lower_bound(numberOfSamples * 0.75);
    assert(cdfItr != cdf.end());
    
    valueAtPercentile = cdfItr->second;
    haveCalculatedValueAtPercentile = true;
    
    return valueAtPercentile;
}


class StaticPDF : public PDF {
  
public:
    
    StaticPDF(const double & ev) {
        expectedValue = ev;
    }
    
    double sample() {
        return expectedValue;
    }
    
    double getValueAtPercentile() {
        return expectedValue;
    }
    
};



gsl_rng * gsl_generator = 0;

class GSLAnalyticPDF : public PDF {
    
    
protected:
    
    void initialiseGenerator() {
        
        if (gsl_generator) {
            return;
        }
        
        gsl_generator = gsl_rng_alloc(gsl_rng_random_glibc2);
        gsl_rng_set(gsl_generator, 2010);        
        
    }


public:
    
    GSLAnalyticPDF() {
            
        initialiseGenerator();
        
    }

};

class GSLAnalyticPDFSingleParameter : public GSLAnalyticPDF {
    
protected:
    
    double parameter;    
    double (*sampleFunction)(const gsl_rng*,double);
    double (*cdfFunction)(double,double);
    
public:
    GSLAnalyticPDFSingleParameter(double (*sampleFn)(const gsl_rng*,double),
                                  double (*cdfFn)(double,double),
                                  const double & parameterIn, const double & ev)
        : GSLAnalyticPDF(),
          parameter(parameterIn),
          sampleFunction(sampleFn), cdfFunction(cdfFn)
    {        
        expectedValue = ev;        
    }
    
    double sample() {
        return (*sampleFunction)(gsl_generator,parameter);        
    }
    
    double getValueAtPercentile() {
        return (*cdfFunction)(solutionDeadlinePercentage,parameter);
    }

};

class GSLAnalyticScaledPDFSingleParameter : public GSLAnalyticPDF {
    
protected:
    
    double scale;
    double parameter;    
    double (*sampleFunction)(const gsl_rng*,double);
    double (*cdfFunction)(double,double);
    
public:
    GSLAnalyticScaledPDFSingleParameter(double (*sampleFn)(const gsl_rng*,double),
                                        double (*cdfFn)(double,double),
                                        const double & parameterIn,
                                        const double & multiplier,
                                        const double & ev)
        : GSLAnalyticPDF(),
          scale(multiplier), parameter(parameterIn),
          sampleFunction(sampleFn), cdfFunction(cdfFn)
    {        
        expectedValue = ev;        
    }
    
    double sample() {
        return scale + (*sampleFunction)(gsl_generator,parameter) * scale;        
    }
    
    double getValueAtPercentile() {
        return scale + (*cdfFunction)(solutionDeadlinePercentage,parameter) * scale;
    }

};

class GSLAnalyticPDFTwoParameter : public GSLAnalyticPDF {
    
protected:
    
    double parameterA;    
    double parameterB;
    double (*sampleFunction)(const gsl_rng*,double,double);
    double (*cdfFunction)(double,double,double);
    
public:
    GSLAnalyticPDFTwoParameter(double (*sampleFn)(const gsl_rng*,double,double),
                               double (*cdfFn)(double,double,double),
                               const double & parameterAIn, const double & parameterBIn, const double & ev)
        : GSLAnalyticPDF(),
          parameterA(parameterAIn), parameterB(parameterBIn),
          sampleFunction(sampleFn), cdfFunction(cdfFn)
    {        
        expectedValue = ev;        
    }
    
    double sample() {
        return (*sampleFunction)(gsl_generator,parameterA,parameterB);        
    }
    
    double getValueAtPercentile() {
        return (*cdfFunction)(solutionDeadlinePercentage,parameterA,parameterB);
    }

};

vector<PDF*> PDFsForVariable;

void initialiseDistributions()
{
    
    static const bool verbose = false;
    
    PDFsForVariable.resize(instantiatedOp::howManyPNEsOfAnySort(), 0);
    
    PNEStore::iterator pneItr = instantiatedOp::pnesBegin();
    const PNEStore::iterator pneEnd = instantiatedOp::pnesEnd();
    
    
    for (; pneItr != pneEnd; ++pneItr) {
        PNE* const currPNE = *pneItr;
        
        const double pneValue = EFT(currPNE->getHead())->getInitial(currPNE->begin(), currPNE->end()).second;
        
        if (currPNE->getHead()->getName().find("exponential") == 0) {
            
            PDFsForVariable[currPNE->getGlobalID()] = new GSLAnalyticPDFSingleParameter(&gsl_ran_exponential, &gsl_cdf_exponential_Pinv, pneValue, pneValue);
            
            if (verbose) {
                cout << "Initialised an exponential distribution based on " << *currPNE << ", rate " << pneValue << endl;
            }
            
        } else if (currPNE->getHead()->getName().find("stochastic-gaussian-mean") == 0) {
            
            const string restOfName = currPNE->getHead()->getName().substr(24); // 24 = number of characters in 'stochastic-gaussian-mean'
            
            PNEStore::iterator pne2Itr = instantiatedOp::pnesBegin();
            
            PNE* standardDeviation;
            
            for (; pne2Itr != pneEnd; ++pne2Itr) {
                standardDeviation = *pne2Itr;
                
                if (standardDeviation->getHead()->getName().find("stochastic-gaussian-standarddeviation") == 0) {
                    //cout << "Name is " << standardDeviation->getHead()->getName() << ", length " << standardDeviation->getHead()->getName().size() << endl;
                    const string restOfThisName = standardDeviation->getHead()->getName().substr(37); // 37 = number of characters in 'stochastic-gaussian-standarddeviation'
                    
                    if (restOfThisName == restOfName) {
                        
                        VAL::LiteralParameterIterator<VAL::parameter_symbol_list::const_iterator> meanItr = currPNE->begin();
                        const VAL::LiteralParameterIterator<VAL::parameter_symbol_list::const_iterator> meanEnd = currPNE->end();

                        VAL::LiteralParameterIterator<VAL::parameter_symbol_list::const_iterator> sdItr = standardDeviation->begin();
                        const VAL::LiteralParameterIterator<VAL::parameter_symbol_list::const_iterator> sdEnd = standardDeviation->end();
                        
                        for (; meanItr != meanEnd && sdItr != sdEnd; ++meanItr, ++sdItr) {
                            
                            if (*meanItr != *sdItr) {                                
                                break;
                            }
                            
                        }
                        
                        if (meanItr == meanEnd && sdItr == sdEnd) {
                            break;
                        }
                    }
                }
            }
            
            if (pne2Itr == pneEnd) {
                cerr << "Fatal error: found a Gaussian distribution mean function, " << currPNE->getHead()->getName() << ", but no standard deviation function stochastic-gaussian-standarddeviation" << restOfName << endl;
                exit(1);
            }
            
            const double sd = EFT(standardDeviation->getHead())->getInitial(standardDeviation->begin(), standardDeviation->end()).second;
            
            PDFsForVariable[currPNE->getGlobalID()] = new GSLAnalyticScaledPDFSingleParameter(&gsl_ran_gaussian, &gsl_cdf_gaussian_Pinv, sd / pneValue, pneValue, pneValue);

            if (verbose) {
                cout << "Initialised a Gaussian distribution based on " << *currPNE << ", mean = " << pneValue << ", standard deviation " << *standardDeviation << " = " << sd << endl;
            }
            
            
        } else if (currPNE->getHead()->getName().find("stochastic-gaussian-standarddeviation") == 0) {
            // do nothing
        } else if (currPNE->getHead()->getName().find("stochastic-uniform-lower") == 0) {

            const string restOfName = currPNE->getHead()->getName().substr(24); // 24 = number of characters in 'stochastic-uniform-lower'
            
            PNEStore::iterator pne2Itr = instantiatedOp::pnesBegin();
            
            PNE* standardDeviation;
            
            for (; pne2Itr != pneEnd; ++pne2Itr) {
                standardDeviation = *pne2Itr;
                
                if (standardDeviation->getHead()->getName().find("stochastic-uniform-upper") == 0) {
                    //cout << "Name is " << standardDeviation->getHead()->getName() << ", length " << standardDeviation->getHead()->getName().size() << endl;
                    const string restOfThisName = standardDeviation->getHead()->getName().substr(24); // 37 = number of characters in 'stochastic-uniform-upper'
                    
                    if (restOfThisName == restOfName) {
                        
                        VAL::LiteralParameterIterator<VAL::parameter_symbol_list::const_iterator> meanItr = currPNE->begin();
                        const VAL::LiteralParameterIterator<VAL::parameter_symbol_list::const_iterator> meanEnd = currPNE->end();

                        VAL::LiteralParameterIterator<VAL::parameter_symbol_list::const_iterator> sdItr = standardDeviation->begin();
                        const VAL::LiteralParameterIterator<VAL::parameter_symbol_list::const_iterator> sdEnd = standardDeviation->end();
                        
                        for (; meanItr != meanEnd && sdItr != sdEnd; ++meanItr, ++sdItr) {
                            
                            if (*meanItr != *sdItr) {                                
                                break;
                            }
                            
                        }
                        
                        if (meanItr == meanEnd && sdItr == sdEnd) {
                            break;
                        }
                    }
                }
            }
            
            if (pne2Itr == pneEnd) {
                cerr << "Fatal error: found a uniform distribution lower-bound function, " << currPNE->getHead()->getName() << ", but could not find stochastic-uniform-upper" << restOfName << endl;
                exit(1);
            }
            
            const double upper = EFT(standardDeviation->getHead())->getInitial(standardDeviation->begin(), standardDeviation->end()).second;
            
            PDFsForVariable[currPNE->getGlobalID()] = new GSLAnalyticPDFTwoParameter(&gsl_ran_flat, &gsl_cdf_flat_Pinv, pneValue, upper, (pneValue + upper)/2);

            if (verbose) {
                cout << "Initialised a uniform distribution based on " << *currPNE << ", range = [" << pneValue << "," << upper << "]" << endl;
            }
        } else if (currPNE->getHead()->getName().find("stochastic-uniform-upper") == 0) {
            // do nothing
        } else if (currPNE->getHead()->getName().find("stochastic-external") == 0) {
            VAL::LiteralParameterIterator<VAL::parameter_symbol_list::const_iterator> paramItr = currPNE->begin();
            const VAL::LiteralParameterIterator<VAL::parameter_symbol_list::const_iterator> paramEnd = currPNE->end();
            
            if (paramItr == paramEnd) {
                cerr << "Error: stochastic-external function type should take exactly 1 argument, a filename.\n";
                exit(1);
            }
            
            const VAL::parameter_symbol * const filename = *paramItr;
            
            PDFsForVariable[currPNE->getGlobalID()] = new CustomPDF(filename->getName());
            
            ++paramItr;
            
            if (paramItr != paramEnd) {
                cerr << "Error: stochastic-external function type should take exactly 1 argument, a filename.\n";
                exit(1);
            }
        } else if (EFT(currPNE->getHead())->isStatic()) {
            
            PDFsForVariable[currPNE->getGlobalID()] = new StaticPDF(pneValue);
            
        }
    }
    
}

};
