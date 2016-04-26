#include "lpscheduler.h"
#include "globals.h"
#include "numericanalysis.h"
#include "temporalanalysis.h"
#include "temporalconstraints.h"
#include "colours.h"

/*#include "ClpSimplex.hpp"
#include "CoinHelperFunctions.hpp"
#include "CoinTime.hpp"
#include "CoinBuild.hpp"
#include "CoinModel.hpp"*/

#include "solver.h"

#include <limits>

#include <sstream>
#include <string>
#include <list>
#include <iterator>
#include <algorithm>
#include <iomanip>

using std::ostringstream;
using std::string;
using std::list;
using std::for_each;
using std::endl;

#define NaN std::numeric_limits<double>::signaling_NaN()

namespace Planner
{

bool ignoreABedges = false;
bool checkSanity = false;

bool LPScheduler::hybridBFLP = true;
bool LPScheduler::optimiseOrdering = true;

vector<double> LPScheduler::TILtimestamps;

vector<vector<list<pair<int, RPGBuilder::LinearEffects::EffectExpression> > > > LPScheduler::gradientEffects;

vector<vector<list<RPGBuilder::RPGNumericEffect* > > > LPScheduler::instantEffects;

vector<vector<list<const LPScheduler::Constraint*> > > LPScheduler::constraints;

vector<vector<LPScheduler::InterestingMap> > LPScheduler::interesting;

vector<vector<vector<double> > > LPScheduler::pointsThatWouldBeMutexWithOptimisationTILs;

vector<vector<pair<bool,bool> > > LPScheduler::boringAct;

vector<double> LPScheduler::initialValues;

list<const LPScheduler::Constraint*> LPScheduler::goalConstraints;

int LPScheduler::numVars;
bool LPScheduler::initialised = false;


int LPScheduler::lpDebug = 0;
bool LPScheduler::workOutFactLayerZeroBoundsStraightAfterRecentAction = false;


set<LPScheduler::Constraint> LPScheduler::Constraint::constraintStore;
int LPScheduler::Constraint::constraintCount = 0;

static const double N = 100000.000;
static const double SAFE = 0.01;

void printRow(MILPSolver *, const int &, const int &);

void nicerLPPrint(MILPSolver *lp)
{
    const int cc = lp->getNumCols();
    
    for (int ci = 0; ci < cc; ++ci) {
        cout << "C" << ci << ", " << lp->getColName(ci) << ", has range [" << lp->getColLower(ci) << "," << lp->getColUpper(ci) << "]\n";
    }

    
    const int rc = lp->getNumRows();

    printRow(lp, 0, rc);

};

void printRow(MILPSolver * lp, const int & rs, const int & rc)
{


    for (int r = 0; r < rc; ++r) {
        if (r < rs) continue;

        cout << r << ",\"" << lp->getRowName(r) << "\",\"";

        vector<pair<int,double> > entries;
        
        lp->getRow(r, entries);;
        
        vector<pair<int,double> >::iterator thisRow = entries.begin();
        const vector<pair<int,double> >::iterator thisRowEnd = entries.end();

        for (int v = 0; thisRow != thisRowEnd; ++thisRow, ++v) {
            if (v) {
                if (thisRow->second >= 0.0) {
                    cout << " + ";
                    if (thisRow->second != 1.0) cout << thisRow->second << ".";
                } else {
                    if (thisRow->second == -1.0) {
                        cout << " - ";
                    } else {
                        cout << " " << thisRow->second << ".";
                    }
                }
            } else {
                if (thisRow->second == 1.0) {
                    cout << "";
                } else if (thisRow->second == -1.0) {
                    cout << "-";
                } else {
                    cout << thisRow->second << ".";
                }
            }

            cout << lp->getColName(thisRow->first);
        }
        cout << " in [";
        if (lp->getRowLower(r) == -LPinfinity) {
            cout << "-inf,";
        } else {
            cout << lp->getRowLower(r) << ",";
        }
        if (lp->getRowUpper(r) == LPinfinity) {
            cout << "inf]";
        } else {
            cout << lp->getRowUpper(r) << "]";
        }

        cout << "\"\n";
    }

}

void checkForZeroRows(MILPSolver *lp)
{

    cout << "Warning - checkForZeroRows not yet implemented\n";
    /*
    const int cc = lp->getNumCols();
    const int rc = lp->getNumRows();

    int * varBuf = new int[cc];
    double * wBuf = new double[cc];

    for (int r = 1; r <= rc; ++r) {
    const int vCount = get_rowex(lp, r, wBuf, varBuf);
    for (int v = 0; v < vCount; ++v) {
        if (wBuf[v] == 0.0) {
            cout << "Have a zero weight on row " << get_row_name(lp,r) << "\n";
            printRow(lp,r,r);
            break;
        }
    }
    }

    delete [] varBuf;
    delete [] wBuf;
    */
}

struct BFEdge {

    int from;
    int to;
    double min;
    double max;
    bool implicit;

    BFEdge(const int & i, const int & j, const double & a, const double & b, bool imp = false)
            : from(i), to(j), min(a), max(b), implicit(imp) {
        assert(i != j);
        if (Globals::globalVerbosity & 4096) {
            cout << "BFEdge from " << i << " to " << j;
            if (implicit) cout << ", marked as implicit";
            cout << "\n";
        }
    };

};


bool Propagation(LPQueueSet & Q, BFEdge & e, vector<double> & distFromZero, vector<double> & distToZero,
                 vector<int> & pairWith, const vector<FFEvent*> & events,
                 map<int, IncomingAndOutgoing > & temporaryEdges)
{

    static const double HALFEPSILON = EPSILON / 2;

    const bool bfDebug = (Globals::globalVerbosity & 4096);
    if (bfDebug) cout << "Propagating\n";

    Q.visit(e.from, e.to);
    assert(e.from == -1 || events[e.from]);
    assert(e.to == -1 || events[e.to]);

    if (e.implicit) {
        if (bfDebug) cout << "\tImplicit edge - skipping update\n";
    } else {
        if (bfDebug) cout << "\tExplicit edge - storing\n";
        if (e.to == -1) { // adding a minimum timestamp back to zero
            if (bfDebug) cout << "\tEdge from node " << e.from << " to time zero\n";
            if (e.min > events[e.from]->lpMinTimestamp) events[e.from]->lpMinTimestamp = e.min;
            if (e.max < events[e.from]->lpMaxTimestamp) events[e.from]->lpMaxTimestamp = e.max;
        } else if (e.from == -1) { // adding a maximum timestamp from zero
            if (bfDebug) cout << "\tEdge from time zero to node " << e.to << "\n";
            if (e.min > events[e.to]->lpMinTimestamp) events[e.to]->lpMinTimestamp = e.min;
            if (e.max < events[e.to]->lpMaxTimestamp) events[e.to]->lpMaxTimestamp = e.max;
        } else {

            if (bfDebug) cout << "\tEdge from node " << e.from << " to " << e.to << "\n";

            map<int, IncomingAndOutgoing >::iterator teFrom = temporaryEdges.find(e.from);

            if (teFrom == temporaryEdges.end()) {
                teFrom = temporaryEdges.insert(make_pair(e.from, IncomingAndOutgoing())).first;
            }

            map<int, IncomingAndOutgoing >::iterator teTo = temporaryEdges.find(e.to);
            if (teTo == temporaryEdges.end()) {
                teTo = temporaryEdges.insert(make_pair(e.to, IncomingAndOutgoing())).first;
            }

            IncomingAndOutgoing * const fromPair = &(teFrom->second);
            IncomingAndOutgoing * const toPair = &(teTo->second);

            toPair->addPredecessor(e.from, e.min > HALFEPSILON);
            fromPair->addFollower(e.to, e.min > HALFEPSILON);

            if (Globals::globalVerbosity & 4096) {
                cout << "Have " << fromPair->mustFollowThis().size() << " recorded successors of " << e.from << endl;
                cout << "Have " << toPair->mustPrecedeThis().size() << " recorded predecessors of " << e.to << endl;
            }
        }

        while (!Q.empty()) {

            int u = Q.pop_front();

            if (bfDebug) {
                if (u != -1) {
                    cout << "Next in queue: " << u << "\n";
                } else {
                    cout << "Next in queue: time 0\n";
                }
            }

            if (Q.UB[u]) {

                const double d0u = (u >= 0 ? distFromZero[u] : 0.0);

                if (d0u != DBL_MAX) {

                    if (bfDebug) {
                        if (u >= 0) {
                            if (events[u]->time_spec == VAL::E_AT) {
                                cout << "Edges out of TIL " << events[u]->divisionID << "\n";
                            } else {
                                cout << "Edges out of " << *(events[u]->action);
                                if (events[u]->time_spec == VAL::E_AT_START) {
                                    cout << ", start\n";
                                } else {
                                    if (TemporalAnalysis::canSkipToEnd(events[u]->action->getID())) {
                                        cout << ", implicit end\n";
                                    } else if (events[u]->getEffects == false) {
                                        cout << ", future end\n";
                                    } else {
                                        cout << ", end\n";
                                    }
                                }

                            }
                        } else {
                            cout << "Edges out of time zero\n";
                        }
                    }

                    if (u >= 0) {

                        // First, consider duration edges out of u

                        // For this u has to be a durative action action, so first, we ask
                        // what it's paired with...

                        const int v = pairWith[u];
                        if (v >= 0) {  // and this has to be a sensible value, i.e. u isn't a TIL or instantaneous action
                            double w;


                            if (u < v) {
                                if (bfDebug) {
                                    cout << "\tTo the end of the action (vertex " << v;
                                    cout.flush();
                                }

                                w = events[u]->maxDuration;
                                if (bfDebug) {
                                    cout << ", weight " << w << ")\n";
                                }

                            } else {
                                if (bfDebug) {
                                    cout << "\tTo the start of the action (vertex " << v; cout.flush();
                                }
                                w = -(events[v]->minDuration);
                                if (bfDebug) cout << ", weight " << w << ")\n";

                            }

                            const double d0uplusWuv = d0u + w;
                            double & d0v = distFromZero[v];

                            if (d0uplusWuv < (d0v - HALFEPSILON)) {
                                d0v = d0uplusWuv;
                                if (bfDebug) cout << "\t\td(zero-to-" << v << ") now = " << d0v << "\n";
                                if (d0v + distToZero[v] < -HALFEPSILON) {
                                    Q.cleanup(e.from, e.to);
                                    return false;
                                } else {
                                    if (Q.NEW[u] == v) {
                                        if (Q.UBP[v]) {
                                            Q.cleanup(e.from, e.to);
                                            return false;
                                        } else {
                                            Q.UBP[v] = true;
                                        }
                                    }
                                }
                                Q.UB[v] = true;
                                Q.push_back(v);
                                assert(v == -1 || events[v]);

                            }
                        }

                        // Having considered duration vars, now we look if there
                        // are any other edges - separation between this and
                        // previous plan steps.

                        const map<int, IncomingAndOutgoing >::iterator teFrom = temporaryEdges.find(u);

                        if (teFrom != temporaryEdges.end()) {
                            const map<int, bool> & backEdges = teFrom->second.mustPrecedeThis();

                            if (bfDebug) {
                                cout << "\t" << backEdges.size() << " edges back to earlier points\n";
                            }
                            map<int, bool>::const_iterator beItr = backEdges.begin();
                            const map<int, bool>::const_iterator beEnd = backEdges.end();

                            for (; beItr != beEnd; ++beItr) {
                                const int v = beItr->first;
                                if (v >= 0) {
                                    const double d0uplusWuv = d0u - (beItr->second ? EPSILON : 0.0);
                                    double & d0v = distFromZero[v];
                                    if (d0uplusWuv < (d0v - HALFEPSILON)) {
                                        d0v = d0uplusWuv;
                                        if (bfDebug) {
                                            cout << "\t\td(zero-to-" << v << ") now = " << d0v << "\n";
                                        }
                                        if (d0v + distToZero[v] < -HALFEPSILON) {
                                            Q.cleanup(e.from, e.to);
                                            return false;
                                        } else {
                                            if (Q.NEW[u] == v) {
                                                if (Q.UBP[v]) {
                                                    Q.cleanup(e.from, e.to);
                                                    return false;
                                                } else {
                                                    Q.UBP[v] = true;
                                                }
                                            }
                                        }
                                        Q.UB[v] = true;
                                        Q.push_back(v);


                                    }
                                } else {
                                    if (bfDebug) cout << "One back to time zero, but we can't reduce d00\n";
                                }
                            }
                        }
                    } else { // doing edges out of the zero node - max ts on each node

                        const int loopLim = events.size();

                        for (int v = 0; v < loopLim; ++v) {
                            if (!events[v]) continue;
                            const double w = events[v]->lpMaxTimestamp;
                            if (w != DBL_MAX) {
                                double & d0v = distFromZero[v];
                                if (w < (d0v - HALFEPSILON)) {
                                    d0v = w;
                                    if (bfDebug) cout << "\t\td(zero-to-" << v << ") now = " << d0v << "\n";
                                    if (d0v + distToZero[v] < -HALFEPSILON) {
                                        Q.cleanup(e.from, e.to);
                                        return false;
                                    } else {
                                        if (Q.NEW[u] == v) {
                                            if (Q.UBP[v]) {
                                                Q.cleanup(e.from, e.to);
                                                return false;
                                            } else {
                                                Q.UBP[v] = true;
                                            }
                                        }
                                    }
                                    Q.UB[v] = true;
                                    Q.push_back(v);
                                    assert(v == -1 || events[v]);

                                }
                            }
                        }
                    }
                }
            }

            if (Q.LB[u]) {

                const double du0 = (u >= 0 ? distToZero[u] : 0.0);

                if (du0 != DBL_MAX) {
                    if (bfDebug) {
                        if (u >= 0) {

                            if (events[u]->time_spec == VAL::E_AT) {
                                cout << "Edges into of TIL " << events[u]->divisionID << "\n";
                            } else {

                                cout << "Edges into " << *(events[u]->action);
                                if (events[u]->time_spec == VAL::E_AT_START) {
                                    cout << ", start\n";
                                } else {
                                    if (TemporalAnalysis::canSkipToEnd(events[u]->action->getID())) {
                                        cout << ", implicit end\n";
                                    } else if (!(events[u]->getEffects)) {
                                        cout << ", fake end\n";
                                    } else {
                                        cout << ", end\n";
                                    }
                                }
                            }
                        } else {
                            cout << "Edges into time zero\n";
                        }
                    }

                    if (u >= 0) {

                        {   // If it's a durative action, we'll have an edge in from the end of the action

                            const int v = pairWith[u];

                            if (v >= 0) {

                                // ...then it's the other end of this action - if it's less than 0
                                // that means we have a TIL or instantaneous action

                                double w;

                                if (u < v) {
                                    w = -events[u]->minDuration;
                                    if (bfDebug) cout << "\tFrom the corresponding end: " << w << "\n";
                                } else {
                                    w = events[v]->maxDuration;
                                    if (bfDebug) cout << "\tFrom the corresponding start: " << w << "\n";
                                }

                                if (w != DBL_MAX) {
                                    const double du0plusWvu = du0 + w;
                                    double & dv0 = distToZero[v];

                                    if (du0plusWvu < (dv0 - HALFEPSILON)) {
                                        if (bfDebug) cout << "\t\td(" << v << "-to-zero) was " << dv0;
                                        dv0 = du0plusWvu;
                                        if (bfDebug) cout << ", now = " << dv0 << "\n";

                                        if (distFromZero[v] != DBL_MAX && distFromZero[v] + dv0 < -HALFEPSILON) {
                                            if (bfDebug) cout << "\t\t... but distFromZero = " << distFromZero[v] << " - cycle\n";
                                            Q.cleanup(e.from, e.to);
                                            return false;
                                        } else { // figure 2, lines 20a to 20f
                                            if (Q.NEW[v] == u) {
                                                if (bfDebug) cout << "\t\tVertex from " << v << " to " << u << " was the new one, have revised lower bound\n";
                                                if (Q.LBP[u]) {
                                                    if (bfDebug) cout << "\t\tPreviously LBP tagged" << u << " - cycle\n";
                                                    Q.cleanup(e.from, e.to);
                                                    return false;
                                                } else {
                                                    if (bfDebug) cout << "\t\tLBP tagging " << u << "\n";
                                                    Q.LBP[u] = true;
                                                }
                                            }
                                        }

                                        Q.LB[v] = true;
                                        Q.push_back(v);
                                        assert(v == -1 || events[v]);

                                    }
                                } else {
                                    if (bfDebug) cout << "(i.e. edge weight is infinite)\n";
                                }
                            }
                        } // End of durative action case


                        // Then, in all cases, we need to consider the non-duration edges out of the node

                        const map<int, IncomingAndOutgoing >::iterator teFrom = temporaryEdges.find(u);

                        if (teFrom != temporaryEdges.end()) {

                            const map<int, bool> & foreEdges = teFrom->second.mustFollowThis();


                            map<int, bool>::const_iterator feItr = foreEdges.begin();
                            const map<int, bool>::const_iterator feEnd = foreEdges.end();

                            for (; feItr != feEnd; ++feItr) {
                                const int v = feItr->first;
                                if (v >= 0) {
                                    if (bfDebug) {
                                        cout << "\tEdge of weight ";
                                        if (feItr->second) {
                                            cout << "epsilon";
                                        } else {
                                            cout << "zero";
                                        }

                                        if (events[v]->getEffects) {
                                            cout << " from node " << v << "\n";
                                        } else {
                                            cout << " from future end node " << v << "\n";
                                        }
                                    }
                                    const double du0plusWvu = du0 - (feItr->second ? EPSILON : 0);
                                    double & dv0 = distToZero[v];

                                    if (du0plusWvu < (dv0 - HALFEPSILON)) {
                                        dv0 = du0plusWvu;
                                        if (bfDebug) cout << "\t\td(" << v << "-to-zero) now = " << dv0 << "\n";

                                        if (distFromZero[v] != DBL_MAX && distFromZero[v] + dv0 < -HALFEPSILON) {
                                            if (bfDebug) cout << "\t\t... but distFromZero = " << distFromZero[v] << " - cycle\n";
                                            Q.cleanup(e.from, e.to);
                                            return false;
                                        } else { // figure 2, lines 20a to 20f
                                            if (Q.NEW[v] == u) {
                                                if (bfDebug) {
                                                    cout << "\t\tVertex from " << v << " to " << u << " was the new one\n";
                                                }
                                                if (Q.LBP[u]) {
                                                    if (bfDebug) {
                                                        cout << "\t\tLBP[" << u << "] has been seen before, found a negative cycle\n";
                                                    }
                                                    Q.cleanup(e.from, e.to);
                                                    return false;
                                                } else {
                                                    if (bfDebug) {
                                                        cout << "\t\tMarking LBP[" << u << "]\n";
                                                    }
                                                    Q.LBP[u] = true;
                                                }
                                            }
                                        }

                                        Q.LB[v] = true;
                                        Q.push_back(v);
                                        assert(v == -1 || events[v]);

                                    }
                                }
                            }

                        } // end of the edges to subsequent actions

                    } else { // edges into zero: min timestamps for each action
                        const int loopLim = events.size();
                        for (int v = 0; v < loopLim; ++v) {
                            if (!events[v]) continue;
                            double w = events[v]->lpMinTimestamp;
                            if (w != 0.0) w = -w;

                            if (bfDebug) cout << "\tEdge from node " << v << ", w = " << w << "\n";
                            double & dv0 = distToZero[v];

                            if (w < (dv0 - HALFEPSILON)) {
                                dv0 = w;

                                if (bfDebug) cout << "\t\td(" << v << "-to-zero) now = " << dv0 << "\n";


                                if (distFromZero[v] != DBL_MAX && distFromZero[v] + dv0 < -HALFEPSILON) {
                                    if (bfDebug) {
                                        cout << "\t\t... but distFromZero = " << distFromZero[v] << " - cycle\n";
                                    }
                                    Q.cleanup(e.from, e.to);
                                    return false;
                                } else { // figure 2, lines 20a to 20f
                                    if (Q.NEW[v] == u) {
                                        if (bfDebug) {
                                            cout << "\t\tVertex from " << v << " to " << u << " was the new one\n";
                                        }
                                        if (Q.LBP[u]) {
                                            Q.cleanup(e.from, e.to);
                                            return false;
                                        } else {
                                            Q.LBP[u] = true;
                                        }
                                    }
                                }

                                Q.LB[v] = true;
                                Q.push_back(v);
                                assert(v == -1 || events[v]);

                            }
                        }
                    }
                }
            }

            Q.LB[u] = false;
            Q.UB[u] = false;

        }
    }

    Q.reset(e.from, e.to);

    return true;
}

class ChildData
{

private:
    LPQueueSet * Q;
    vector<double> distFromZero;
    vector<double> distToZero;
    vector<int> pairWith;
    vector<FFEvent*> eventsWithFakes;
    map<int, IncomingAndOutgoing> temporaryEdges;
    list<BFEdge> newEdges;
    list<FFEvent*> garbage;
    //pair<set<int>, set<int> > constraintsFromLastStep;
    bool needsLP;
    bool copyTimestampsOnDestruction;

public:

    map<int, double> autoMinima;

    ChildData(LPQueueSet * QIn,
              const vector<double> & dfz, const vector<double> & dtz, const vector<int> & pw,
              const vector<FFEvent*> & ewf, const map<int, IncomingAndOutgoing > & tes,
              const bool & nlp)
            : Q(QIn),
            distFromZero(dfz), distToZero(dtz), pairWith(pw), eventsWithFakes(ewf),
            temporaryEdges(tes), needsLP(nlp), copyTimestampsOnDestruction(true) {
    };

    void letTheLPSetTimestamps() {
        copyTimestampsOnDestruction = false;
    }
    
    inline const bool & willSetTimestamps() const {
        return copyTimestampsOnDestruction;
    }
    
    inline void updateLPNeed(const bool & b) {
        if (b) needsLP = true;
    };

    ~ChildData() {
        if (copyTimestampsOnDestruction) {
            static const bool cdDebug = (Globals::globalVerbosity & 4096);
            const int ewfCount = eventsWithFakes.size();

            if (cdDebug) cout << "Copying " << ewfCount << " timestamps back\n";

            for (int e = 0; e < ewfCount; ++e) {
                if (eventsWithFakes[e]) {
                    eventsWithFakes[e]->lpMaxTimestamp = distFromZero[e];
                    if (distToZero[e] == 0.0) {
                        eventsWithFakes[e]->lpMinTimestamp = 0.0;
                    } else {
                        eventsWithFakes[e]->lpMinTimestamp = -distToZero[e];
                    }
                    eventsWithFakes[e]->lpTimestamp = eventsWithFakes[e]->lpMinTimestamp;
                    if (cdDebug) cout << "\t" << e << ": " << eventsWithFakes[e]->lpTimestamp << "\n";
                }
            }
        }
        list<FFEvent*>::iterator gItr = garbage.begin();
        const list<FFEvent*>::iterator gEnd = garbage.end();

        for (; gItr != gEnd; ++gItr) delete *gItr;
    }

    inline void gc(FFEvent * const g) {
        garbage.push_back(g);
    };
    inline void pairEventWith(const int & i, const int & j) {
        if (Globals::globalVerbosity & 4096) {
            cout << "Recording that " << i <<  " is paired with " << j << endl;
        }
        pairWith[i] = j; pairWith[j] = i;
    }

    inline void setTil(const int & i) {
        pairWith[i] = -2;
    };
    inline void setNonTemporal(const int & i) {
        pairWith[i] = -3;
    };
    vector<FFEvent*> & getEventsWithFakes() {
        return eventsWithFakes;
    };

    const vector<double> & getDistFromZero() const {
        return distFromZero;
    };
    const vector<double> & getDistToZero() const {
        return distToZero;
    };

    inline void setDFZ(const int & i, const double & d) {
        distFromZero[i] = d;
    };
    inline void setDTZ(const int & i, const double & d) {
        distToZero[i] = d;
    };

    inline void addNewEdge(const BFEdge & e) {
        newEdges.push_back(e);
    };
    inline IncomingAndOutgoing & makeEdgeListFor(const int & i) {
        return temporaryEdges[i];
    };
    inline map<int, IncomingAndOutgoing >::iterator getEdgeListItr(const int & i) {
        return temporaryEdges.find(i);
    }
    inline map<int, IncomingAndOutgoing >::iterator getEdgeListEnd() {
        return temporaryEdges.end();
    }

    const vector<int> & getPairWith() const {
        return pairWith;
    };

    bool propagateNewEdges();
    bool updateLPMinTimestamp(const double & d, const int & stepID);
    void sanityCheck() {
        const int loopLim = distToZero.size();
        for (int i = 0; i < loopLim; ++i) {
            if (eventsWithFakes[i] && distToZero[i] > 0) {
                cout << "Event " << i << " set to come " << -(distToZero[i]) << " before time zero\n";
                assert(distToZero[i] <= 0.0);
            }
            if (eventsWithFakes[i] && eventsWithFakes[i]->time_spec == VAL::E_AT && pairWith[i] != -2) {
                cout << "Event " << i << " is a TIL, but is not paired with -2\n";
                assert(pairWith[i] == -2);
            }
        }
    }
    bool checkItContainsAllTheseEdges(const TemporalConstraints * const cons) const;

    inline void moveZeroDists(const int & from, const int & to) {
        distToZero[to] = distToZero[from];
        distFromZero[to] = distFromZero[from];
        distToZero[from] = 0.0;
        distFromZero[from] = DBL_MAX;
    }
    void clearPairFor(const int & i) {
        pairWith[i] = -1;
        assert(!eventsWithFakes[i]);
    }
    inline const bool & doLPSolve() const {
        return needsLP;
    };

    void distsToLPStamps(const int debug = -1) {
        const int ll = eventsWithFakes.size();

        for (int i = 0; i < ll; ++i) {
            FFEvent * const f = eventsWithFakes[i];
            if (f) {
                double w = distToZero[i];
                if (w != 0.0) w = -w;
                if (debug == i && (!f->action || !TemporalAnalysis::canSkipToEnd(f->action->getID()))) {
                    if (fabs(w - f->lpTimestamp) > 0.0005) {
                        cout << "LP would put event " << i << " at " << f->lpTimestamp << ", but STN puts it at " << w << "\n";
                        assert(fabs(w - f->lpTimestamp) <= 0.0005);
                    }
                }

                f->passInMinMax(w, distFromZero[i]);
            }
        }
    }

    void distsToLPMinStamps() {
        const int ll = eventsWithFakes.size();

        for (int i = 0; i < ll; ++i) {
            FFEvent * const f = eventsWithFakes[i];
            if (f) {
                double w = distToZero[i];
                if (w != 0.0) w = -w;
                f->passInMinMax(w, distFromZero[i]);
            }
        }
    }

    void distsToLPMinStampsAndCheck(vector<FFEvent*> & withLPTimestamps) {
        const int ll = eventsWithFakes.size();

        for (int i = 0; i < ll; ++i) {
            FFEvent * const f = eventsWithFakes[i];
            if (f) {
                double w = distToZero[i];
                if (w != 0.0) w = -w;
                
                #ifndef NDEBUG
                if (w - withLPTimestamps[i]->lpTimestamp > 0.0000001) {
                    std::cerr << std::setprecision(6) << std::fixed << std::endl << "Error: step " << i << " has been given a timestamp of " << withLPTimestamps[i]->lpTimestamp << " by the LP, but the STP reported this had to be at least " << w << endl;
                    exit(1);
                }

                if (withLPTimestamps[i]->lpTimestamp - distFromZero[i] > 0.0000001) {
                    std::cerr << std::setprecision(6) << std::fixed << std::endl << "Error: step " << i << " has been given a timestamp of " << withLPTimestamps[i]->lpTimestamp << " by the LP, but the STP reported this could be no more than " << distFromZero[i] << endl;
                    exit(1);
                }
                #endif
                //f->passInMinMax(w, distFromZero[i]);
            }
        }
    }



    void printDotFile(ostream & o);
};



bool ChildData::checkItContainsAllTheseEdges(const TemporalConstraints * const cons) const
{
    bool toReturn = true;
    const int lim = cons->size();
    for (int i = 0; i < lim; ++i) {
        const map<int, bool> * const anyBefore = cons->stepsBefore(i);
        if (!anyBefore) continue;

        map<int, IncomingAndOutgoing >::const_iterator mItr = temporaryEdges.find(i);

        map<int, bool>::const_iterator sItr = anyBefore->begin();
        const map<int, bool>::const_iterator sEnd = anyBefore->end();

        if (mItr != temporaryEdges.end()) {
            //cout << "Have " << mItr->second.mustPrecedeThis().size() << " recorded predecessors of " << i << endl;
            map<int, bool>::const_iterator exItr = mItr->second.mustPrecedeThis().begin();
            const map<int, bool>::const_iterator exEnd = mItr->second.mustPrecedeThis().end();

            while (exItr != exEnd && sItr != sEnd) {
                if (exItr->first < sItr->first) {
                    //cout << "- Predecessor of " << exItr->first << " isn't in T\n";
                    ++exItr;
                } else if (sItr->first < exItr->first) {
                    if (sItr->first != pairWith[i]) {
                        cout << "Missing edge noting that " << sItr->first << " must precede " << i << endl;
                        toReturn = false;
                    } else {
                        //cout << "- Predecessor of " << sItr->first << " isn't in the data, but that might be okay\n";
                    }
                    ++sItr;
                } else {
                    //cout << "- Both record predecessor of " << sItr->first << endl;
                    ++exItr;
                    ++sItr;
                }
            }

        } else {
            //cout << "Have no recorded predecessors of " << i << endl;
        }

        for (; sItr != sEnd; ++sItr) {
            if (sItr->first != pairWith[i]) {
                cout << "Missing edge noting that " << sItr->first << " must precede " << i << endl;
                toReturn = false;
            }
        }
    }
    return toReturn;
}

void ChildData::printDotFile(ostream & o)
{

    o << "digraph STN {\n\n";
    o << "T0 [label=\"t0\"]\n";
    const int ll = eventsWithFakes.size();

    vector<vector<double> > edgeMatrix(ll + 1, vector<double>(ll + 1, DBL_MAX));
    vector<vector<string> > edgeLabels(ll + 1, vector<string>(ll + 1));

    edgeMatrix[0][0] = 0.0;

    for (int i = 0; i < ll; ++i) {
        if (eventsWithFakes[i]) {
            edgeMatrix[i+1][i+1] = 0.0;
        }
    }

    for (int i = 0; i < ll; ++i) {
        FFEvent * const f = eventsWithFakes[i];
        if (f) {
            double w = distToZero[i];
            if (w != 0.0) w = -w;
            o << "N" << i << " [label=\"" << w << ": ";
            if (f->action) {
                o << *(f->action) << " ";
                if (f->time_spec == VAL::E_AT_START) {
                    o << " S\"];\n";
                } else {
                    if (TemporalAnalysis::canSkipToEnd(f->action->getID())) {
                        o << " E (implicit)\"];\n";
                    } else {
                        if (eventsWithFakes[i]->getEffects) {
                            o << " E\"];\n";
                        } else {
                            o << " E (future)\"];\n";
                        }
                    }
                }
            } else {
                o << "TIL " << f->divisionID << "\"];\n";
            }
        }
    }

    vector<pair<double, double> > endMinMax(ll, make_pair(0.0, DBL_MAX));

    for (int i = 0; i < ll; ++i) {
        FFEvent * const f = eventsWithFakes[i];
        if (f) {
            {
                if (f->action && f->time_spec == VAL::E_AT_START) {
                    const bool nonTemporal = RPGBuilder::getRPGDEs(f->action->getID()).empty();

                    const vector<pair<double, double> > & tsBounds = TemporalAnalysis::getActionTSBounds()[f->action->getID()];

                    double startMin = tsBounds[0].first;
                    double endMin = tsBounds[1].first;

                    double startMax = tsBounds[0].second;
                    double endMax = tsBounds[1].second;


                    if (!nonTemporal) {

                        {
                            const double sEndMin = startMin + f->minDuration;
                            if (sEndMin > endMin) endMin = sEndMin;
                        }
                        {
                            const double sStartMin = endMin - f->maxDuration;
                            if (sStartMin > startMin) startMin = sStartMin;

                        }

                        if (startMax != DBL_MAX && f->maxDuration != DBL_MAX) {
                            const double sEndMax = startMax + f->maxDuration;
                            if (sEndMax < endMax) endMax = sEndMax;
                        }

                        if (endMax != DBL_MAX) {
                            const double sStartMax = endMax - f->minDuration;
                            if (sStartMax < startMax) startMax = sStartMax;
                        }
                    }

                    if (startMin != 0.0) {
                        edgeMatrix[i+1][0] = -startMin;
                        if (startMin == 0.001) {
                            edgeLabels[i+1][0] = "-e, gmin";
                        } else {
                            ostringstream namestream;
                            namestream << -startMin << ", gmin";
                            edgeLabels[i+1][0] = namestream.str();
                        }
                    }
                    if (startMax < DBL_MAX) {
                        edgeMatrix[0][i+1] = startMax;
                        ostringstream namestream;
                        namestream << startMax << ", gmax";
                        edgeLabels[0][i+1] = namestream.str();
                    }

                    if (!nonTemporal) {
                        endMinMax[i] = make_pair(endMin, endMax);
                    }

                } else if (f->action && f->time_spec == VAL::E_AT_END) {
                    const double & startMin = endMinMax[pairWith[i]].first;
                    const double & startMax = endMinMax[pairWith[i]].second;

                    if (startMin != 0.0) {
                        edgeMatrix[i+1][0] = -startMin;
                        if (startMin == 0.001) {
                            edgeLabels[i+1][0] = "-e, gmin";
                        } else {
                            ostringstream namestream;
                            namestream << -startMin << ", gmin";
                            edgeLabels[i+1][0] = namestream.str();
                        }
                    }
                    if (startMax < DBL_MAX) {
                        edgeMatrix[0][i+1] = startMax;
                        ostringstream namestream;
                        namestream << startMax << ", gmax";
                        edgeLabels[0][i+1] = namestream.str();
                    }
                } else if (f->time_spec == VAL::E_AT) {
                    const double tilTime = LPScheduler::getTILTimestamp(f->divisionID);
                    {
                        edgeMatrix[i+1][0] = -tilTime;
                        ostringstream namestream;
                        namestream << -tilTime << ", til";
                        edgeLabels[i+1][0] = namestream.str();
                    }
                    {
                        edgeMatrix[0][i+1] = tilTime;
                        ostringstream namestream;
                        namestream << tilTime << ", til";
                        edgeLabels[0][i+1] = namestream.str();
                    }
                } else {
                    assert(false);
                }

            }
            /*{
                if (!i) {
                    edgeMatrix[i+1][i] = 0;
                    edgeLabels[i+1][i] = "0, seq";
                } else if (pairWith[i] != (i - 1)) {
                    edgeMatrix[i+1][i] = -EPSILON;
                    edgeLabels[i+1][i] = "-e, seq";
                } else {
                    cout << "No sequence edge for N" << i << ", as is paired with N" << pairWith[i] << "\n";
                }

                set<int>::iterator ntfItr = f->needToFinish.begin();
                const set<int>::iterator ntfEnd = f->needToFinish.end();

                for (; ntfItr != ntfEnd; ++ntfItr) {
                    edgeMatrix[i+1][pairWith[*ntfItr] + 1] = -EPSILON;
                    edgeLabels[i+1][pairWith[*ntfItr] + 1] = "-e, ntf";
                }
                if (pairWith[i] > i) {
                    if (f->maxDuration != DBL_MAX) {
                        edgeMatrix[i+1][pairWith[i] + 1] = f->maxDuration;
                        ostringstream namestream;
                        namestream << f->maxDuration;
                        edgeLabels[i+1][pairWith[i] + 1] = namestream.str();
                    }
                    double w = f->minDuration;
                    if (w != 0.0) w = -w;
                    if (w == -EPSILON) {
                        edgeMatrix[pairWith[i]+1][i + 1] = -EPSILON;
                        edgeLabels[pairWith[i]+1][i + 1] = "-e, mindur";
                    } else {
                        edgeMatrix[pairWith[i]+1][i + 1] = w;
                        ostringstream namestream;
                        namestream << w;
                        edgeLabels[pairWith[i]+1][i + 1] = namestream.str();
                    }
                }

            }*/

            {
                if (pairWith[i] > i) {
                    if (f->maxDuration != DBL_MAX) {
                        edgeMatrix[i+1][pairWith[i] + 1] = f->maxDuration;
                        ostringstream namestream;
                        namestream << f->maxDuration;
                        edgeLabels[i+1][pairWith[i] + 1] = namestream.str();
                    }
                    double w = f->minDuration;
                    if (w != 0.0) w = -w;
                    if (w == -EPSILON) {
                        edgeMatrix[pairWith[i] + 1][i + 1] = -EPSILON;
                        edgeLabels[pairWith[i] + 1][i + 1] = "-e, mindur";
                    } else {
                        edgeMatrix[pairWith[i] + 1][i + 1] = w;
                        ostringstream namestream;
                        namestream << w;
                        edgeLabels[pairWith[i] + 1][i + 1] = namestream.str();
                    }
                }

            }

            {

                {
                    map<int, IncomingAndOutgoing >::iterator teItr = temporaryEdges.find(i);
                    if (teItr != temporaryEdges.end()) {
                        map<int, bool>::const_iterator ntfItr = teItr->second.mustPrecedeThis().begin();
                        const map<int, bool>::const_iterator ntfEnd = teItr->second.mustPrecedeThis().end();

                        for (; ntfItr != ntfEnd; ++ntfItr) {
                            //if (ntfItr->first == eventCount - 1) continue;
                            if (ntfItr->second) {
                                edgeMatrix[i+1][ntfItr->first + 1] = -EPSILON;
                                edgeLabels[i+1][ntfItr->first + 1] = "-e, temp";
                            } else {
                                edgeMatrix[i+1][ntfItr->first + 1] = 0;
                                edgeLabels[i+1][ntfItr->first + 1] = "0, temp";
                            }
                        }
                    }
                }
            }

        }
    }

    for (int i = 0; i <= ll; ++i) {
        for (int j = 0; j <= ll; ++j) {
            if (i == j) continue;
            if (edgeMatrix[i][j] != DBL_MAX) {
                if (i == 0) {
                    o << "T0 -> ";
                } else {
                    o << "N" << i - 1 << " -> ";
                }

                if (j == 0) {
                    o << "T0 ";
                } else {
                    o << "N" << j - 1 << " ";
                }
                o << "[label=\"" << edgeLabels[i][j] << "\"];\n";
            }
        }
    }

    o << "\n}\n";

    for (int k = 0; k <= ll; ++k) {
        for (int i = 0; i <= ll; ++i) {
            const double IK = edgeMatrix[i][k];
            if (IK == DBL_MAX) continue;
            for (int j = 0; j <= ll; ++j) {
                double KJ = edgeMatrix[k][j];
                if (KJ == DBL_MAX) continue;
                KJ += IK;
                if (KJ < edgeMatrix[i][j]) {
                    edgeMatrix[i][j] = KJ;
                }
            }
        }
    }
    for (int i = 0; i <= ll; ++i) {
        if (edgeMatrix[i][i] < 0.0) {
            if (!i) {
                cout << "Negative cycle for t0\n";
            } else {
                if (eventsWithFakes[i-1]) {
                    cout << "Negative cycle for ";
                    if (eventsWithFakes[i-1]->action) {
                        cout << *(eventsWithFakes[i-1]->action) << " ";
                        if (eventsWithFakes[i-1]->time_spec == VAL::E_AT_START) {
                            cout << "start\n";
                        } else {
                            cout << "end\n";
                        }
                    } else {
                        cout << "TIL " << eventsWithFakes[i-1]->divisionID << "\n";
                    }
                } else {
                    cout << "Negative cycle for unknown event - this should never happen\n";
                }
            }
        }
    }

}


void LPScheduler::InterestingMap::insertKeepingTrues(const pair<int, bool> & toInsert)
{
    if (toInsert.second) {
        __super::insert(toInsert).first->second = true;
    } else {
        __super::insert(toInsert);
    }
}


void LPScheduler::CountedConstraintSet::insert(const LPScheduler::Constraint* const c)
{
    ++(__super::insert(make_pair(c, 0)).first->second);
}

void LPScheduler::CountedConstraintSet::erase(const LPScheduler::Constraint* const c)
{
    __super::iterator fItr = find(c);
    if (fItr == end()) return;
    if (!(--(fItr->second))) {
        __super::erase(fItr);
    }
}

void LPScheduler::CountedConstraintSet::insert(const LPScheduler::ConstraintSet & c)
{

    iterator lastPos;
    bool firstTime = true;

    ConstraintSet::const_iterator cItr = c.begin();
    const ConstraintSet::const_iterator cEnd = c.end();

    for (; cItr != cEnd; ++cItr) {
        if (firstTime) {
            lastPos = __super::insert(make_pair(*cItr, 0)).first;
            firstTime = false;
        } else {
            lastPos = __super::insert(lastPos, make_pair(*cItr, 0));
        }
        ++(lastPos->second);
    }
}

void LPScheduler::CountedConstraintSet::erase(const LPScheduler::ConstraintSet & c)
{

    ConstraintSet::const_iterator cItr = c.begin();
    const ConstraintSet::const_iterator cEnd = c.end();

    for (; cItr != cEnd; ++cItr) {
        const iterator lastPos = find(*cItr);
        if (lastPos == end()) continue;
        if (!(--(lastPos->second))) {
            __super::erase(lastPos);
        }
    }
}


LPScheduler::LPScheduler(const MinimalState & s, list<FFEvent> & plan)
{
    static list<FFEvent> now;
    static map<int, list<list<StartEvent>::iterator > > compulsaryEnds;
    static list<StartEvent> dummySEQ;
    static list<int>* dummy;
    static ParentData * dummyP;

    LPScheduler(s, plan, now, -1, dummySEQ, dummyP, compulsaryEnds, 0, 0, dummy, false);
}


void LPScheduler::collateRelevantVariablesAndInvariants(InterestingMap & currInterest, CountedConstraintSet & activeInvariants,
        const int & stepID, const VAL::time_spec & currTS, const int & actID,
        vector<set<int> > & activeAncestorsOfStep,
        const vector<bool> & correspondEndHasBeenInserted,
        vector<map<int, ConstraintSet > > & invariantsThisStepStartsOnVariableI)
{
    if (currTS != VAL::E_AT) {
        currInterest = interesting[actID][1];
    }

    if (currTS == VAL::E_AT_START) {
        currInterest.insertKeepingTrues(interesting[actID][0].begin(), interesting[actID][0].end());
    }

    if (currTS == VAL::E_AT_END) {
        currInterest.insertKeepingTrues(interesting[actID][2].begin(), interesting[actID][2].end());
    }

    set<int> & activeAncestors = activeAncestorsOfStep[stepID];

    if (activeAncestors.empty()) {
        if (lpDebug & 1024) {
            cout << COLOUR_yellow << "No active ancestors of this step to cause any invariants" << COLOUR_default << endl;
        }
        return;
    }

    InterestingMap::iterator ciItr = currInterest.begin();
    const InterestingMap::iterator ciEnd = currInterest.end();

    for (; ciItr != ciEnd; ++ciItr) {

        if (!ciItr->second) continue;

        set<int>::iterator aaItr = activeAncestors.begin();
        const set<int>::iterator aaEnd = activeAncestors.end();

        for (; aaItr != aaEnd; ++aaItr) {

            if (correspondEndHasBeenInserted[*aaItr]) continue;
            
            const map<int, ConstraintSet >::iterator onThisVariable = invariantsThisStepStartsOnVariableI[*aaItr].find(ciItr->first);

            if (onThisVariable == invariantsThisStepStartsOnVariableI[*aaItr].end()) {
                if (lpDebug & 1024) {
                    cout << COLOUR_yellow << "No invariants on " << *(RPGBuilder::getPNE(ciItr->first)) << " started at step " << *aaItr << COLOUR_default << endl;
                }
                continue;
            }
            if (lpDebug & 1024) {
                cout << COLOUR_yellow << "Adding invariants on " << *(RPGBuilder::getPNE(ciItr->first)) << " that started at step " << *aaItr << COLOUR_default << endl;
            }

            activeInvariants.insert(onThisVariable->second);

        }
    }


    CountedConstraintSet::iterator csItr = activeInvariants.begin();
    const CountedConstraintSet::iterator csEnd = activeInvariants.end();

    for (; csItr != csEnd; ++csItr) {
        currInterest.insertPreconditions(csItr->first->variables.begin(), csItr->first->variables.end());
    }

}

void LPScheduler::recordVariablesInvolvedInThisStepsInvariants(const list<const Constraint*> & invariants,
                                                               map<int, ConstraintSet> & invariantsOnVariableI)
{

    list<const Constraint*>::const_iterator invItr = invariants.begin();
    const list<const Constraint*>::const_iterator invEnd = invariants.end();
    
    for (; invItr != invEnd; ++invItr) {
        const vector<int> & vars = (*invItr)->variables;
        
        const int lim = vars.size();
        
        for (int s = 0; s < lim; ++s) {
            if (lpDebug & 1024) {
                cout << COLOUR_light_green << "Step has an invariant depending on " << *(RPGBuilder::getPNE(vars[s])) << COLOUR_default << endl;
            }
            invariantsOnVariableI[vars[s]].insert(*invItr);
        }
    }
}


void LPScheduler::addConstraintsToGetValuesOfVariablesNow(InterestingMap & currInterest, const int & stepID, const int & currVar, map<int, int> & beforeStep)
{

    static const vector<pair<int,double> > emptyEntries;


    int colIdx = lp->getNumCols();
    int constrIdx = lp->getNumRows();

    InterestingMap::iterator sItr = currInterest.begin();
    const InterestingMap::iterator sItrEnd = currInterest.end();

    for (; sItr != sItrEnd; ++sItr) {

        FluentTracking & tracker = finalNumericVars[sItr->first];
        
        if (tracker.statusOfThisFluent != FluentTracking::FS_NORMAL) {
            if (lpDebug & 1) {
                cout << "Not adding constraint to get value of " << *(RPGBuilder::getPNE(sItr->first)) << ": it is a metric tracking fluent";
                if (tracker.statusOfThisFluent == FluentTracking::FS_IGNORE) {
                    cout << " that is being ignored\n";
                } else {
                    cout << ", the effects on which are order-independent, and hence will be included directly in the objective function if needed\n";
                }
            }
            continue;
        }
        
        if (lpDebug & 1) {
            cout << "Adding constraint at " << colIdx << " to get value of " << *(RPGBuilder::getPNE(sItr->first)) << " now";
            if (sItr->second) {
                cout << " - intend to write to it";
            }
            cout << "\n";
        }

        lp->addCol(emptyEntries, -LPinfinity, LPinfinity, MILPSolver::C_REAL);

        if (assertions) assert((lp->getNumCols() - 1) == colIdx);

        if (nameLPElements) {
            ostringstream namestream;
            namestream << *(RPGBuilder::getPNE(sItr->first));
            namestream << "b" << stepID;
            string asString = namestream.str();
            lp->setColName(colIdx, asString);
        }

        if (tracker.activeGradientCount && tracker.activeGradient != 0.0) {
            if (lpDebug & 1) cout << "Active gradient = " << tracker.activeGradient << "\n";
            if (tracker.lastEffectValueVariable != -1) {

                static vector<pair<int,double> > entries(4);
                
                entries[0].second = 1.0;
                entries[1].second = -1.0;
                entries[2].second = -tracker.activeGradient;
                entries[3].second = tracker.activeGradient;

                if (assertions) assert(entries[3].second != 0.0);
                if (assertions) assert(entries[2].second != 0.0);

                entries[0].first = colIdx;
                entries[1].first = tracker.lastEffectValueVariable;
                entries[2].first = currVar;
                entries[3].first = tracker.lastEffectTimestampVariable;

                lp->addRow(entries, 0.0, 0.0);

            } else {
                static vector<pair<int,double> > entries(3);
                
                entries[0].second = 1.0;
                entries[1].second = -tracker.activeGradient;
                entries[2].second = tracker.activeGradient;

                if (assertions) assert(entries[2].second != 0.0);
                if (assertions) assert(entries[1].second != 0.0);

                entries[0].first = colIdx;
                entries[1].first = currVar;
                entries[2].first = tracker.lastEffectTimestampVariable;

                lp->addRow(entries, tracker.postLastEffectValue, tracker.postLastEffectValue); // syntax for EQ prev

            }

            if (nameLPElements) {
                ostringstream namestream;
                namestream << stepID << "delta-" << *(RPGBuilder::getPNE(sItr->first));
                string asString = namestream.str();
                lp->setRowName(constrIdx, asString);
            }

            if (assertions) assert((lp->getNumRows() - 1) == constrIdx);

            ++constrIdx;



        } else {

            if (tracker.lastEffectValueVariable != -1) {
                static vector<pair<int,double> > entries(2);
                
                entries[0].second = 1.0;
                entries[1].second = -1.0;

                entries[0].first = colIdx;
                entries[1].first = tracker.lastEffectValueVariable;
                
                lp->addRow(entries, 0.0, 0.0);
                
                if (nameLPElements) {
                    ostringstream namestream;
                    namestream << stepID << "delta-0-" << *(RPGBuilder::getPNE(sItr->first));
                    string asString = namestream.str();
                    lp->setRowName(constrIdx, asString);
                }
                ++constrIdx;
            } else {
                lp->setColBounds(colIdx, tracker.postLastEffectValue, tracker.postLastEffectValue);
            }
        }


        beforeStep[sItr->first] = colIdx;
        ++colIdx;

    }
}


int LPScheduler::generateEndDetails(const VAL::time_spec & currTS, const int & actID, const int & stepID, FFEvent & currEvent,
                                    const vector<FFEvent*> & planAsAVector, int & nextImaginaryEndVar, vector<EndDetails> & imaginaryMinMax)
{

    int dummyEnd = -1;


    if (currTS == VAL::E_AT_START && !RPGBuilder::getRPGDEs(actID).empty()) {

        dummyEnd = timestampVars[stepID] + (currEvent.pairWithStep - stepID);

        if (!planAsAVector[currEvent.pairWithStep]->getEffects) {

            if (RPGBuilder::getRPGDEs(actID).back()->fixed.empty()) {
                imaginaryMinMax[stepID] = EndDetails(dummyEnd, nextImaginaryEndVar, -1);
                ++nextImaginaryEndVar;
                
                static vector<pair<int,double> > entries(2);
                
                entries[0].first = imaginaryMinMax[stepID].imaginaryMin;
                entries[1].first = imaginaryMinMax[stepID].imaginaryMax;
                entries[0].second = -1.0;
                entries[1].second = 1.0;
                assert(entries[0].first < lp->getNumCols());
                assert(entries[1].first < lp->getNumCols());
                lp->addRow(entries, 0.0, LPinfinity);

                if (nameLPElements) {
                    {
                        ostringstream namestream;
                        namestream << "minMax" << stepID;
                        string asString = namestream.str();
                        if (lpDebug & 64) {
                            cout << "R" << lp->getNumRows() - 1 << " = " << asString << "\n";
                            //                            checkRows[lp->getNumRows()] = asString;
                        }
                        lp->setRowName(lp->getNumRows() - 1, asString);
                    }
                    {
                        ostringstream namestream;
                        namestream << "iendmax" << currEvent.pairWithStep;
                        string asString = namestream.str();
                        lp->setColName(imaginaryMinMax[stepID].imaginaryMax, asString);
                    }
                }

            } else {
                imaginaryMinMax[stepID] = EndDetails(dummyEnd, dummyEnd, -1);
            }


        } else {
            imaginaryMinMax[stepID] = EndDetails(dummyEnd, dummyEnd, -1);
        }
    }

    return dummyEnd;
}


class LPScheduler::ConstraintAdder {

protected:
    LPScheduler* const parent;

    FFEvent & currEvent;
    const int stepID;

    InterestingMap * untouched;
    //set<int> * durationalVary;
    list<int> * stableNextTime;
    list<int> * unstableNextTime;

    
    void addNormalEffect(RPGBuilder::RPGNumericEffect* const ceItr) {

        static const vector<pair<int,double> > emptyEntries;
        
        static const int varCount = RPGBuilder::getPNECount();

        const int currVar = ceItr->fluentIndex;
        
        const FluentTracking & tracker = parent->finalNumericVars[currVar];
        
        assert(tracker.statusOfThisFluent == FluentTracking::FS_NORMAL);
        
        int sLim = ceItr->size;

        const bool wasStable = parent->stableVariable[currVar];

        vector<pair<int,double> > entries;
        entries.reserve(sLim);
        
        bool isStable = ((tracker.activeGradientCount == 0) && (!parent->finalNumericVars[currVar].everHadADurationDependentEffect));

        bool hasDurationVar = false;

        for (int s = 0; s < sLim; ++s) {
            double currW = -ceItr->weights[s];
            if (parent->assertions) assert(currW != 0.0);
            int varIdx = ceItr->variables[s];
            if (varIdx >= 0) {
                if (varIdx >= varCount) {
                    varIdx -= varCount;
                    currW = ceItr->weights[s];
                }
                
                {
                    const map<int,int>::iterator atItr = applyTo->find(varIdx);
                    assert(atItr != applyTo->end());
                    entries.push_back(make_pair(atItr->second, currW));
                }
                
                if (!parent->stableVariable[varIdx]) {
                    isStable = false;
                }
                
                if (parent->finalNumericVars[varIdx].everHadADurationDependentEffect) {
                    parent->finalNumericVars[currVar].everHadADurationDependentEffect = true;
                }
                //                        if (outstandingCTS.find(varIdx) != outstandingCTS.end()) stableVariable[currVar] = false;
            } else if (varIdx == -3) {
                hasDurationVar = true;
                int startStep;
                int endStep;

                if (currEvent.time_spec == VAL::E_AT_START) {
                    startStep = parent->timestampVars[stepID];
                    endStep = startStep + 1;
                } else {
                    endStep = parent->timestampVars[stepID];
                    startStep = endStep - 1;
                }
                entries.push_back(make_pair(endStep, currW));                
                entries.push_back(make_pair(startStep, ceItr->weights[s]));
                                
                if (parent->assertions) assert(entries.back().second != 0.0);
                
                isStable = false;
                parent->finalNumericVars[currVar].everHadADurationDependentEffect = true;

            } else {
                cout << "Unprocessed continuous effect in LP scheduler, aborting.\n";
                assert(false);
            }
        }

        parent->lp->addCol(emptyEntries, -LPinfinity, LPinfinity, MILPSolver::C_REAL);

        const int colIdx = parent->lp->getNumCols() - 1;

        if (parent->nameLPElements) {
            ostringstream namestream;
            namestream << *(RPGBuilder::getPNE(currVar));
            namestream << "a" << stepID;
            string asString = namestream.str();
            parent->lp->setColName(colIdx, asString);
        }


        entries.push_back(make_pair(colIdx, 1.0));

        if (!(ceItr->isAssignment)) {
            {
                const map<int,int>::iterator atItr = applyTo->find(currVar);
                assert(atItr != applyTo->end());                
                entries.push_back(make_pair(atItr->second, -1.0));            
            }
            isStable = (isStable && wasStable);
        }

        if (lpDebug & 1) {
            if (hasDurationVar) {
                cout << "Adding effect dependent on duration:";
                const int sLim = entries.size();
                for (int sp = 0; sp < sLim; ++sp) {
                    cout << "\t";
                    if (sp > 0) cout << " + "; else cout << "   ";
                    cout << entries[sp].second << " * " << parent->lp->getColName(entries[sp].first) << "\n";
                }
                cout << "\t = " << ceItr->constant;
            } else {
                cout << "Added effect " << *ceItr << " to get value of " << *(RPGBuilder::getPNE(currVar)) << "\n";
            }
        }

        parent->lp->addRow(entries, ceItr->constant, ceItr->constant);

        if (parent->nameLPElements) {
            const int constrIdx = parent->lp->getNumRows() - 1;
            ostringstream namestream;
            namestream << "eff@" << stepID;
            string asString = namestream.str();
            parent->lp->setRowName(constrIdx, asString);
        }

        (*output)[currVar] = colIdx;

        InterestingMap::iterator untItr = untouched->find(currVar);
        assert(untItr != untouched->end());
        assert(untItr->second);
        untouched->erase(untItr);

        if (isStable && !wasStable) stableNextTime->push_back(currVar);
        
        if (!isStable && wasStable) {
            if (lpDebug & 1) {
                cout << "Effect " << *ceItr << " makes variable unstable next time\n";
            }
                    
            unstableNextTime->push_back(currVar);
        }

    }

    void addOrderIndependentMetricEffect(RPGBuilder::RPGNumericEffect* const ceItr) {

        static const int varCount = RPGBuilder::getPNECount();

        const int currVar = ceItr->fluentIndex;
        
        FluentTracking & tracker = parent->finalNumericVars[currVar];
        
        assert(tracker.statusOfThisFluent == FluentTracking::FS_ORDER_INDEPENDENT);
        assert(!ceItr->isAssignment);
        
        const int sLim = ceItr->size;

        for (int s = 0; s < sLim; ++s) {
            double thisW = ceItr->weights[s];
            int varIdx = ceItr->variables[s];
            if (varIdx >= 0) {
                if (varIdx >= varCount) {
                    varIdx -= varCount;
                    thisW = -thisW;
                }
                
                {
                    const map<int,int>::iterator atItr = applyTo->find(varIdx);
                    assert(atItr != applyTo->end());                    
                    tracker.orderIndependentValueTerms.insert(make_pair(atItr->second, 0.0)).first->second += thisW;
                }

                assert(parent->stableVariable[varIdx]);
                
            } else if (varIdx == -3) {
                int startStep;
                int endStep;

                if (currEvent.time_spec == VAL::E_AT_START) {
                    startStep = parent->timestampVars[stepID];
                    endStep = startStep + 1;
                } else {
                    endStep = parent->timestampVars[stepID];
                    startStep = endStep - 1;
                }
                
                tracker.orderIndependentValueTerms.insert(make_pair(endStep, 0.0)).first->second += thisW;
                tracker.orderIndependentValueTerms.insert(make_pair(startStep, 0.0)).first->second -= thisW;

            } else {
                cout << "Unprocessed continuous effect in LP scheduler, aborting.\n";
                assert(false);
            }
        }

        tracker.orderIndependentValueConstant += ceItr->constant;

    }

    
    const char * label;
    mutable int suffix;
        
public:
    map<int, int> * applyTo;    
    map<int, int> * output;
    
    
    
    ConstraintAdder(LPScheduler * const parentIn, FFEvent & ev, const char * const labelIn, const int & stepIDIn, map<int, int> & applyToIn)
        : parent(parentIn), currEvent(ev), stepID(stepIDIn),
          untouched((InterestingMap*)0), stableNextTime((list<int>*)0), unstableNextTime((list<int>*)0),
          label(labelIn), suffix(0), applyTo(&applyToIn), output((map<int, int>*)0) {
    }

    void supplyEffectData(map<int, int> & atStep, InterestingMap & unto, list<int> & snt, list<int> & unt) {
        output = &atStep;
        untouched = &unto;
        stableNextTime = &snt;
        unstableNextTime = &unt;
    }

    void changeLabel(const char * l) {
        label = l;
        suffix = 0;
    }

    void operator()(const Constraint * const csItr) const {

        const int cSize = csItr->weights.size();

        vector<pair<int,double> > entries(cSize);
        
        if (LPScheduler::lpDebug & 1024) {
            cout << "Adding constraint: ";
            for (int s = 0 ; s < cSize; ++s) {
                if (s) cout << " + ";
                cout << csItr->weights[s] << "*" << parent->lp->getColName((*applyTo)[csItr->variables[s]]);

            }
            if (csItr->lower != -DBL_MAX) {
                cout << ", >= " << csItr->lower;
            }
            if (csItr->upper != DBL_MAX) {
                cout << ", <= " << csItr->upper;
            }
            cout << std::endl;
        }

        for (int s = 0 ; s < cSize; ++s) {
            entries[s].second = csItr->weights[s];
            if (parent->assertions) assert(entries[s].second != 0.0);
            entries[s].first = (*applyTo)[csItr->variables[s]];
        }

        parent->lp->addRow(entries,
                           (csItr->lower != -DBL_MAX ? csItr->lower : -LPinfinity),
                           (csItr->upper != DBL_MAX  ? csItr->upper : LPinfinity ) );

        if (parent->nameLPElements) {
            const int constrIdx = parent->lp->getNumRows() - 1;
            ostringstream namestream;
            namestream << label << stepID << "n" << suffix;
            string asString = namestream.str();
            parent->lp->setRowName(constrIdx, asString);
            ++suffix;
        }
    }

    void operator()(const pair<const Constraint*, unsigned int> & csItr) const {
        operator()(csItr.first);
    }

    void operator()(RPGBuilder::RPGNumericEffect* const ceItr) {

        const int currVar = ceItr->fluentIndex;
        
        FluentTracking & tracker = parent->finalNumericVars[currVar];

        switch (tracker.statusOfThisFluent) {
            case FluentTracking::FS_NORMAL:
            {
                addNormalEffect(ceItr);
                return;
            }
            case FluentTracking::FS_ORDER_INDEPENDENT:
            {
                addOrderIndependentMetricEffect(ceItr);
                return;
            }
            default:
            {
                return;
            }
        }
        
    }

};

struct LPScheduler::DurationAdder {

    LPScheduler * const parent;
    int durationID;
    FFEvent & currEvent;
    const int stepID;
    map<int, int> * beforeStep;
    const vector<bool> & stableVariable;
    
    int startToUse;
    int endToUse;

    VAL::comparison_op durationType;

    bool durationIsFixed;
    
    DurationAdder(LPScheduler * const parentIn, FFEvent & ev, const int & stepIDIn, map<int, int> & beforeStepIn, const vector<bool> & stableVariableIn)
            : parent(parentIn), durationID(0), currEvent(ev), stepID(stepIDIn), beforeStep(&beforeStepIn), stableVariable(stableVariableIn),
            startToUse(-1), endToUse(-1), durationType(VAL::E_EQUALS), durationIsFixed(true) {
    }

    void setStartEnd(const int & s, const int & e, const VAL::comparison_op & dt) {
        startToUse = s;
        endToUse = e;
        durationType = dt;
    }

    void operator()(RPGBuilder::DurationExpr* const currDE) {

        const int vSize = currDE->weights.size();
        vector<pair<int,double> > entries(2 + vSize);
        
        entries[0] = make_pair(endToUse, 1.0);
        entries[1] = make_pair(startToUse, -1.0);
        
        assert(endToUse < parent->lp->getNumCols());
        assert(startToUse < parent->lp->getNumCols());
        
        bool allVariablesAreStable = true;
        
        {
            for (int v = 0; v < vSize; ++v) {
                entries[2+v].second = -(currDE->weights[v]);
                if (parent->assertions) assert(entries[2+v].second != -0.0);
                #ifdef STOCHASTICDURATIONS
                int & vToUse = currDE->variables[v].first;
                #else
                int & vToUse = currDE->variables[v];
                #endif
                if (vToUse != -1) {
                    const map<int, int>::iterator bsItr = beforeStep->find(vToUse);
                    #ifndef NDEBUG
                    if (bsItr == beforeStep->end()) {
                        if (vToUse >= RPGBuilder::getPNECount()) {
                            std::cerr << "Internal error: variable -" << *(RPGBuilder::getPNE(vToUse - RPGBuilder::getPNECount())) << " not defined in the LP at a step where it is needed for a duration constraint.\n";
                        } else {
                            std::cerr << "Internal error: variable " << *(RPGBuilder::getPNE(vToUse)) << " not defined in the LP at a step where it is needed for a duration constraint.\n";
                        }
                        assert(bsItr != beforeStep->end());
                    }
                    #endif
                    entries[2+v].first = bsItr->second;
                    allVariablesAreStable = (allVariablesAreStable && stableVariable[vToUse]);
                } else {
                    std::cerr << "Internal error: negative variable " << vToUse << " in duration expression\n";
                    exit(1);
                }
            }
        }

        switch (durationType) {
        case VAL::E_EQUALS: {
            if ((lpDebug & 1) && !vSize) cout << "Simple constant fixed duration: " << currDE->constant << "\n";
                        
            parent->lp->addRow(entries, currDE->constant, currDE->constant);
            if (parent->nameLPElements) {
                int constrIdx = parent->lp->getNumRows() - 1;
                ostringstream namestream;
                namestream << "dur" << startToUse << "fixed" << durationID << ": v" << startToUse << " -> v" << endToUse;
                string asString = namestream.str();
                parent->lp->setRowName(constrIdx, asString);
                durationIsFixed = (durationIsFixed || allVariablesAreStable);
            }
            break;
        }
        case VAL::E_GREATEQ: {
            if ((lpDebug & 1) && !vSize) cout << "Simple constant minimum duration: " << currDE->constant << "\n";
                        
            parent->lp->addRow(entries, currDE->constant, LPinfinity);
            if (parent->nameLPElements) {
                int constrIdx = parent->lp->getNumRows() - 1;
                ostringstream namestream;
                namestream << "dur" << startToUse << "min" << durationID << ": v" << startToUse << " -> v" << endToUse;
                string asString = namestream.str();
                parent->lp->setRowName(constrIdx, asString);
                durationIsFixed = false;
            }
            break;
        }
        case VAL::E_LESSEQ: {            
            
            if ((lpDebug & 1) && !vSize) cout << "Simple constant maximum duration: " << currDE->constant << "\n";
            
            
            parent->lp->addRow(entries, 0.0, currDE->constant);
            if (parent->nameLPElements) {
                int constrIdx = parent->lp->getNumRows() - 1;
                ostringstream namestream;
                namestream << "dur" << startToUse << "max" << durationID << ": v" << startToUse << " -> v" << endToUse;
                string asString = namestream.str();
                parent->lp->setRowName(constrIdx, asString);
                durationIsFixed = false;
            }
            break;
        }
        default:
            assert(false);
        }

    }

};

LPScheduler::LPScheduler(const MinimalState & theState,
                         list<FFEvent> & header,
                         list<FFEvent> & now,
                         const int & justAppliedStep,
                         list<StartEvent> & startEventQueue,
                         ParentData * parentData,
                         map<int, list<list<StartEvent>::iterator > > & compulsaryEnds,
                         const vector<double> * secondMin,
                         const vector<double> * secondMax,
                         list<int> * tilComesBefore,
                         const bool & setObjectiveToMetric) : cd(0)
{

    if (!initialised) initialise();


    static vector<pair<int,double> > emptyEntries(0);

    static const bool optimised = true;

    assertions = true;

    static vector<vector<pair<double, double> > > & actionTSBounds = TemporalAnalysis::getActionTSBounds();

    includeMetricTrackingVariables = setObjectiveToMetric;    
    stableVariable = vector<bool>(numVars, true);

    tsVarCount = header.size() + now.size();

    bool shouldFail = false;
    bool shouldSucceed = false;

    bool paranoia = (Globals::paranoidScheduling || Globals::profileScheduling);

    if (tsVarCount == 0) {
        lp = 0; solved = true; return;
    }

    if (hybridBFLP && parentData) {

        cd = parentData->spawnChildData(startEventQueue, header, now, setObjectiveToMetric, theState.temporalConstraints, justAppliedStep);
        if (!cd || !cd->propagateNewEdges()) {
            if (lpDebug & 1) cout << "STP proved LP unsolvable, skipping LP\n";
            //assert(!cd || cd->checkItContainsAllTheseEdges(theState.temporalConstraints));
            if (paranoia || Globals::profileScheduling) {
                shouldSucceed = false;
                shouldFail = true;
            } else {
                lp = 0; solved = false; return;
            }
        } else if (!setObjectiveToMetric && !cd->doLPSolve()) {
            if (lpDebug & 1) cout << "No need to solve LP - STP is sufficient\n";
            if (paranoia) {
                #ifndef NDEBUG
                if (Globals::paranoidScheduling) {
                    assert(!cd || cd->checkItContainsAllTheseEdges(theState.temporalConstraints));
                }
                #endif
                shouldSucceed = true;
                shouldFail = false;
            } else {
                lp = 0; solved = true; return;
            }
        } else {
            #ifndef NDEBUG
            if (Globals::paranoidScheduling) {
                assert(!cd || cd->checkItContainsAllTheseEdges(theState.temporalConstraints));
            }
            #endif
        }

        if (cd) {                            
            if (!paranoia) {
                cd->distsToLPStamps();
                cd->distsToLPMinStamps();                
            }

            cd->letTheLPSetTimestamps();
        }
    }



    if (paranoia && lpDebug) {
        if (shouldSucceed) cout << "*** Should now succeed ***\n";
        if (shouldFail) cout << "*** Should now fail ***\n";
    }

    planAsAVector.resize(tsVarCount, (FFEvent*) 0);

    /* Number of remaining unvisited ancestors for each step */
    vector<int> fanIn(tsVarCount, 0);

    /* The steps that necessarily follow each step in the plan */
    vector<map<int, bool> > fanOut(tsVarCount);


    /* For each step in the plan, this records a mapping between state
     * variables, and the invariants involving that variable that begun at
     * that time step.  The idea, then, is if we are at a step that follows
     * I, and are changing variable v, we need to check any constraints
     * from this invariantsThisStepStartsOnVariableI[I][v].
     */
    vector<map<int, ConstraintSet > > invariantsThisStepStartsOnVariableI(tsVarCount);

    /* We use this to push forwards which active ancestors a given step has,
     * i.e. actions which have started but not yet finished.  For each of
     * these, we use the previous action starts (and the effects of the current
     * step) to determine which invariants we need to add as constraints
     * over the step's fluent vector.
     */
    vector<set<int> > activeAncestorsOfStep(tsVarCount);

    /* This is used to record whether the corresponding end of a start action
     * at the indexed step has been added to the LP.  If so, its invariants
     * have expired.  This is used in combination with the previous variable.
     */
    vector<bool> correspondEndHasBeenInserted(tsVarCount,false);
    
    /*
     * Step that comes after all future end actions.  -1 if we don't need this constraint
     * (i.e. we aren't imposing a strict total order.  @see TotalOrderTransformer
     */
    const int stepThatMustPrecedeFutureEnds = MinimalState::getTransformer()->stepThatMustPrecedeUnfinishedActions(theState.temporalConstraints);

    map<int, bool> fakeEdgeForThis;
    if (stepThatMustPrecedeFutureEnds != -1) {
        fakeEdgeForThis.insert(make_pair(stepThatMustPrecedeFutureEnds, true));
    }

    list<int> openList;
    int imaginaryEnds = 0;

    int mustVisit = 0;
    {
        int stepID = 0;
        for (int pass = 0; pass < 2; ++pass) {
            list<FFEvent> & currList = (pass ? now : header);

            list<FFEvent>::iterator citr = currList.begin();
            const list<FFEvent>::iterator cend = currList.end();

            for (; citr != cend; ++citr, ++stepID, ++mustVisit) {

                planAsAVector[stepID] = &(*citr);
                citr->lpMaxTimestamp = LPinfinity;
                if (citr->lpMinTimestamp < 0.0) citr->lpMinTimestamp = 0.0;
                //if (citr->time_spec == VAL::E_AT_START && TemporalAnalysis::canSkipToEnd(citr->action->getID())) ++imaginaryEnds;

                const map<int, bool> * const stepsThatComeBeforeThisOne = theState.temporalConstraints->stepsBefore(stepID);

                if (stepsThatComeBeforeThisOne) {
                    fanIn[stepID] = stepsThatComeBeforeThisOne->size();
                    if (lpDebug & 512) {
                        cout << fanIn[stepID] << " recorded steps precede " << stepID << ": [";
                    }
                    map<int, bool>::const_iterator cbItr = stepsThatComeBeforeThisOne->begin();
                    const map<int, bool>::const_iterator cbEnd = stepsThatComeBeforeThisOne->end();
                    for (; cbItr != cbEnd; ++cbItr) {
                        if (lpDebug & 512) cout << " " << cbItr->first;
                        fanOut[cbItr->first].insert(make_pair(stepID, cbItr->second));
                    }
                    if (lpDebug & 512) cout << " ]\n";
                }

                /*if (citr->time_spec == VAL::E_AT_END) {
                    ++fanIn[stepID];
                    fanOut[citr->pairWithStep].insert(make_pair(stepID, false));
                }*/


                if (citr->time_spec == VAL::E_AT_END && !citr->getEffects
                        && stepThatMustPrecedeFutureEnds != -1
                        && citr->pairWithStep != stepThatMustPrecedeFutureEnds) {

                    // For the ends of all actions that have not yet finished - if we're using
                    // a total order, these must come after the step just added

                    if (fanOut[stepThatMustPrecedeFutureEnds].insert(make_pair(stepID, true)).second) {
                        if (lpDebug & 512) {
                            cout << "Additionally, the TO constraint means " << stepThatMustPrecedeFutureEnds << " precedes " << stepID << endl;
                        }

                        ++fanIn[stepID];
                    } else {
                        if (lpDebug & 512) {
                            cout << "The TO constraint means " << stepThatMustPrecedeFutureEnds << " precedes " << stepID << ", as it already was doing\n";
                        }                        
                    }
                    
                }

                if (!fanIn[stepID]) {
                    if (lpDebug & 512) cout << "No steps precede " << stepID << " - adding to open list\n";
                    openList.push_back(stepID);
                }

                if (   citr->time_spec == VAL::E_AT_END
                    && citr->getEffects == false
                    && RPGBuilder::getRPGDEs(citr->action->getID()).back()->fixed.empty()) {

                    ++imaginaryEnds;

                }
            }
        }
    }
    if (lpDebug & 1) cout << "Making lp for " << tsVarCount << " events, " << imaginaryEnds << " future max events, and " << numVars << " variables\n";
    //if (lpDebug & 1) cout << "Making lp for " << tsVarCount << " events and " << numVars << " variables\n";

    lp = getNewSolver();
    lp->addEmptyRealCols(tsVarCount + numVars + imaginaryEnds);

    int nextImaginaryEndVar = tsVarCount + numVars;

    if (!(lpDebug & 4)) lp->hush();

    const int optVar = (justAppliedStep != -1 ? numVars + justAppliedStep : -1);

    if (optVar != -1) lp->setObjCoeff(optVar, 1.0);

    // make the last real action as early as possible

    if (lpDebug & 1) cout << "Objective function to minimise variable " << optVar << "\n";

    previousObjectiveVar = optVar;

    finalNumericVars = vector<FluentTracking>(numVars);

    for (int i = 0; i < numVars; ++i) {
        finalNumericVars[i] = FluentTracking(initialValues[i]);
        
        if (NumericAnalysis::getDominanceConstraints()[i] == NumericAnalysis::E_IRRELEVANT) {
            finalNumericVars[i].statusOfThisFluent = FluentTracking::FS_IGNORE;
        } else if (NumericAnalysis::getDominanceConstraints()[i] == NumericAnalysis::E_METRICTRACKING) {
            if (setObjectiveToMetric) {
                if (NumericAnalysis::getDataOnWhichVariablesHaveOrderIndependentEffects()[i] != NumericAnalysis::E_ORDER_DEPENDENT) {
                    finalNumericVars[i].statusOfThisFluent = FluentTracking::FS_ORDER_INDEPENDENT;
                }
            } else {
                finalNumericVars[i].statusOfThisFluent = FluentTracking::FS_IGNORE;
            }
        }
        
    }

    timestampVars.reserve(tsVarCount);

    vector<EndDetails> imaginaryMinMax(tsVarCount);

    vector<map<int, int> > fluentsAtStep(tsVarCount);

    vector<int> correspondingStart(tsVarCount);

//    endsOfSkippableActions = vector<int>(tsVarCount,-1);

    nameLPElements = (lpDebug || Globals::globalVerbosity & 32);

    for (int i = 0; i < numVars; ++i) {
        const double curr = initialValues[i];
        lp->setColLower(i, curr);
        lp->setColUpper(i, curr);

        if (nameLPElements) {
            ostringstream namestream;
            namestream << *(RPGBuilder::getPNE(i));
            namestream << "[0.000]";
            string asString = namestream.str();
            lp->setColName(i, asString);
        }
    }

    int nextTIL = 0;
//    double TILupbo = TILtimestamps[nextTIL];


    bool boundNow = false;


//    int stepID = 0;
//    int currVar = numVars;
//    map<int, pair<int, double> > activeGradients;
//    map<int, list<Constraint*>* > activeInvariants;

    map<int, int> outstandingCTS;

    list<int> stableNextTime;

    vector<double> unassignedLowerBounds(tsVarCount, 0.0);

    makespanVarMinimum = 0.0;

    timestampToUpdate = 0;
    timestampToUpdatePartner = 0;

    bool partnerIsEarlier = false;

    timestampToUpdateVar = -1;
    timestampToUpdateStep = -1;
    timestampToUpdatePartnerVar = -1;
    timestampToUpdatePartnerStep = -1;
    
    if ((lpDebug & 1) && !tsVarCount) cout << "No actions in plan\n";

    int actuallyVisited = 0;

    while (!openList.empty()) {

        ++actuallyVisited;

        const int stepID = openList.front();
        openList.pop_front();

        const int currVar = numVars + stepID;


        FFEvent & currEvent = *(planAsAVector[stepID]);

        const int actID = (currEvent.action ? currEvent.action->getID() : -1);
        const VAL::time_spec currTS = currEvent.time_spec;
        const int divisionID = currEvent.divisionID;

        if (currVar == optVar) {
            timestampToUpdate = &currEvent;
            timestampToUpdateVar = currVar;
            timestampToUpdateStep = stepID;

            if (actID != -1 && !RPGBuilder::getRPGDEs(actID).empty()) {
                if (currTS == VAL::E_AT_START) {
                    timestampToUpdatePartner = planAsAVector[stepID + 1];
                    timestampToUpdatePartnerStep = stepID + 1;
                    timestampToUpdatePartnerVar = numVars + stepID + 1;
                    partnerIsEarlier = false;
                } else {
                    timestampToUpdatePartner = planAsAVector[stepID - 1];
                    timestampToUpdatePartnerStep = stepID - 1;
                    timestampToUpdatePartnerVar = numVars + stepID - 1;
                    partnerIsEarlier = true;
                }
            }
        }


        const map<int, bool> * const stepsThatComeBeforeThisOne = theState.temporalConstraints->stepsBefore(stepID);

//        const bool implicitEnd = (currTS == VAL::E_AT_START && TemporalAnalysis::canSkipToEnd(actID));


        if (lpDebug & 1) {
            cout << "Adding action ";
            if (actID != -1) {
                cout << *(currEvent.action) << " ";
                if (currTS == VAL::E_AT_START) {
                    cout << "start\n";
                } else if (currTS == VAL::E_AT_END) {
                    if (currEvent.getEffects) {
                        cout << "end\n";
                    } else {
                        cout << "end placeholder\n";
                    }
                } else {
                    cout << "intermediate point " << divisionID << "\n";
                }

            } else {
                cout << " for TIL " << divisionID << "\n";
            }
        }

        if (nameLPElements) {
            ostringstream namestream;
            namestream << stepID << ": ";
            if (actID != -1) {
                namestream << *(currEvent.action) << " ";
                if (currTS == VAL::E_AT_START) {
                    namestream << "S";
                } else if (currTS == VAL::E_AT_END) {
                    namestream << "E";
                } else {
                    namestream << "I" << divisionID;
                }
            } else {
                namestream << "TIL" << divisionID;
            }

            string asString = namestream.str();
            lp->setColName(currVar, asString);
            if (lpDebug & 1) cout << "C" << currVar << " becomes " << asString << "\n";
        }


        if (currEvent.getEffects) {
            list<int>::iterator sntItr = stableNextTime.begin();
            const list<int>::iterator sntEnd = stableNextTime.end();

            for (; sntItr != sntEnd; ++sntItr) {
                if (lpDebug & 1) cout << "--- Variable " << *(RPGBuilder::getPNE(*sntItr)) << " becomes stable ---\n";
                stableVariable[*sntItr] = true;
                //durationalVary.erase(*sntItr);
            }

            stableNextTime.clear();

        }


        if (currTS == VAL::E_AT_START) {
            correspondingStart[stepID] = stepID;            
        } else if (currTS != VAL::E_AT) {
            correspondingStart[stepID] = correspondingStart[currEvent.pairWithStep];
            //correspondEndHasBeenInserted[currEvent.pairWithStep] = true;
        }

        {

            bool printed = false;

            for (int predecessorPass = 0; predecessorPass < 2; ++predecessorPass) {
                const map<int, bool> * currentPredecessors;

                if (predecessorPass) {
                    if (currTS != VAL::E_AT_END || currEvent.getEffects) {
                        // if it's the end of an action in the past, we don't need the extra TO edge
                        break;
                    }
                    if (currEvent.pairWithStep == stepThatMustPrecedeFutureEnds) {
                        break;
                    }
                    currentPredecessors = &(fakeEdgeForThis);
                    if (lpDebug & 1) {
                        if (!fakeEdgeForThis.empty()) {
                            if (!printed) {
                                cout << "Steps before this one, only due to TO: [";
                                printed = true;
                            } else {
                                cout << "] + TO: [";
                            }
                        }
                    }

                } else {
                    currentPredecessors = stepsThatComeBeforeThisOne;
                }

                if (currentPredecessors) {

                    map<int, bool>::const_iterator beforeItr = currentPredecessors->begin();
                    const map<int, bool>::const_iterator beforeEnd = currentPredecessors->end();

                    for (; beforeItr != beforeEnd; ++beforeItr) {
                        if ((lpDebug & 1) && !printed) {
                            cout << "Steps before this one: [";
                            printed = true;
                        }
                        if (lpDebug & 1) cout << " " << beforeItr->first;
                        const int prevVar = beforeItr->first + numVars;

                        if (currTS != VAL::E_AT_END || prevVar != currVar - 1) {

                            static vector<pair<int,double> > entries(2);
                            entries[0].second = 1.0;
                            entries[1].second = -1.0;
                            
                            entries[0].first = currVar;
                            entries[1].first = prevVar;

                            lp->addRow(entries, (beforeItr->second ? EPSILON : 0.0), LPinfinity); // syntax for '>= 0 or EPSILON'

                            if (nameLPElements) {
                                int constrIdx = lp->getNumRows() - 1;
                                ostringstream namestream;
                                if (beforeItr->second) {
                                    namestream << "t" << beforeItr->first << "_l_t" << stepID;
                                } else {
                                    namestream << "t" << beforeItr->first << "_le_t" << stepID;
                                }
                                string asString = namestream.str();
                                lp->setRowName(constrIdx, asString);
                            }
                        } else {
                            if (lpDebug & 1) cout << " <start>";
                        }
                    }
                }
            }


            if (lpDebug & 1 && printed) cout << " ]\n";

        }

        if (lpDebug & 1) {
            if (!activeAncestorsOfStep[stepID].empty()) {
                cout << "Active ancestors:";
                set<int>::const_iterator aaItr = activeAncestorsOfStep[stepID].begin();
                const set<int>::const_iterator aaEnd = activeAncestorsOfStep[stepID].end();
                for (; aaItr != aaEnd; ++aaItr) {
                    if (!correspondEndHasBeenInserted[*aaItr]) cout << " " << *aaItr;
                }
                cout << endl;
            }
        }

        timestampVars[stepID] = currVar;

        {
            double boundForSuccessors = unassignedLowerBounds[stepID];

            if (currTS == VAL::E_AT) {
                const double thisTIL = TILtimestamps[divisionID];

                lp->setColLower(currVar, thisTIL);
                lp->setColUpper(currVar, thisTIL);
                if (lpDebug & 1) cout << "- As it's a TIL, fixing column bounds of " << currVar << " to " << thisTIL << endl;
                nextTIL = divisionID + 1;
//                TILupbo = TILtimestamps[nextTIL];
                if (boundForSuccessors < thisTIL) {
                    boundForSuccessors = thisTIL;
                }

            } else {


                if (optimised) {
                    if (currEvent.lpMinTimestamp != -1.0) {
                        if (boundForSuccessors < currEvent.lpMinTimestamp) {
                            boundForSuccessors = currEvent.lpMinTimestamp;
                        }
                    }
                } 
               
                const double workingLower = actionTSBounds[actID][currTS == VAL::E_AT_START ? 0 : 1].first;
                
                if (boundForSuccessors < workingLower) {
                    boundForSuccessors = workingLower;
                }
                                               
                
                lp->setColLower(currVar, boundForSuccessors);

                /* if (implicitEnd) {
                    lp->setColLower(endsOfSkippableActions[stepID], unassignedLowerBound + EPSILON);
                }*/

                if (optimised) {
                    double workingUpper = actionTSBounds[actID][currTS == VAL::E_AT_START ? 0 : 1].second;
//                    if (TILupbo < DBL_MAX) {
//                        if (workingUpper > TILupbo - EPSILON) workingUpper = TILupbo - EPSILON;
//                    }
                    if (currEvent.lpMaxTimestamp < LPinfinity) {
                        if (workingUpper > currEvent.lpMaxTimestamp) workingUpper = currEvent.lpMaxTimestamp;
                    }
                    if (workingUpper != LPinfinity) {
                        if (workingUpper < boundForSuccessors) {
                            solved = false; return;
                        }
                        lp->setColUpper(currVar, workingUpper);
                    }
                }

            }

            const double plusEpsilon = boundForSuccessors + EPSILON;

            map<int, bool> & succs = fanOut[stepID];

            map<int, bool>::iterator succItr = succs.begin();
            const map<int, bool>::iterator succEnd = succs.end();

            for (; succItr != succEnd; ++succItr) {
                double & toUpdate = unassignedLowerBounds[succItr->first];
                if (succItr->second) {
                    if (toUpdate < plusEpsilon) {
                        toUpdate = plusEpsilon;
                    }
                } else {
                    if (toUpdate < boundForSuccessors) {
                        toUpdate = boundForSuccessors;
                    }
                }
            }

            if (currEvent.time_spec != VAL::E_AT && boundForSuccessors > makespanVarMinimum) {
                makespanVarMinimum = boundForSuccessors;
            }
        }


        if (currTS != VAL::E_AT) {
            addConstraintsForTILMutexes(currVar, pointsThatWouldBeMutexWithOptimisationTILs[actID][currTS == VAL::E_AT_START ? 0 : 1]);
        }


        if (currEvent.getEffects) {

            InterestingMap currInterest;
            CountedConstraintSet activeInvariants;

            collateRelevantVariablesAndInvariants(currInterest, activeInvariants, stepID, currTS, actID,
                                                  activeAncestorsOfStep, correspondEndHasBeenInserted,
                                                  invariantsThisStepStartsOnVariableI);

            map<int, int> atStep;
            map<int, int> & beforeStep = fluentsAtStep[stepID];

            addConstraintsToGetValuesOfVariablesNow(currInterest, stepID, currVar, beforeStep);



            if (optimised && boundNow) {
                boundNow = false;

                if (secondMin || secondMax) {

                    map<int, int>::iterator bsItr = beforeStep.begin();
                    const map<int, int>::iterator bsEnd = beforeStep.end();

                    for (int v = 0; v < numVars; ++v) {

                        if (bsItr != bsEnd && (v == bsItr->first)) {
                            if (secondMin) lp->setColLower(bsItr->second, (*secondMin)[v]);
                            if (secondMax) lp->setColUpper(bsItr->second, (*secondMax)[v]);
                            ++bsItr;
                        } else {
                            const int update = finalNumericVars[v].lastEffectValueVariable;

                            if (update != -1) {
                                if (secondMin) lp->setColLower(update, (*secondMin)[v]);
                                if (secondMax) lp->setColUpper(update, (*secondMax)[v]);
                            }
                        }

                    }

                }
            }

            map<int, int> * constrsOver = &beforeStep;
            /*const int dummyEnd = */ generateEndDetails(currTS, actID, stepID, currEvent, planAsAVector, nextImaginaryEndVar, imaginaryMinMax);

            ConstraintAdder adder(this, currEvent, "inv@t", stepID, beforeStep);

            if (currTS == VAL::E_AT_START || currTS == VAL::E_AT_END) {

                constrsOver = &atStep;

                for_each(activeInvariants.begin(), activeInvariants.end(), adder);


                {

                    list<const Constraint*> & currList = constraints[actID][(currTS == VAL::E_AT_START ? 0 : 2)];

                    adder.changeLabel("pre@t");
                    for_each(currList.begin(), currList.end(), adder);
                    adder.changeLabel("inv@t");

                }


                InterestingMap untouched(currInterest);
                list<int> unstableNextTime;

                list<RPGBuilder::RPGNumericEffect* > & currEffs = instantEffects[actID][(currTS == VAL::E_AT_START ? 0 : 1)];

                adder.supplyEffectData(atStep, untouched, stableNextTime, unstableNextTime);

                for_each(currEffs.begin(), currEffs.end(), adder);


                {
                    list<int>::iterator unstItr = unstableNextTime.begin();
                    const list<int>::iterator unstEnd = unstableNextTime.end();

                    for (; unstItr != unstEnd; ++unstItr) {
                        if (lpDebug & 1) cout << "Variable " << *(RPGBuilder::getPNE(*unstItr)) << " becomes unstable after an instantaneous effect\n";
                        stableVariable[*unstItr] = false;
                    }
                }


                {
                    int colIdx = lp->getNumCols();
                    int constrIdx = lp->getNumRows();
                    InterestingMap::iterator sItr = untouched.begin();
                    const InterestingMap::iterator sItrEnd = untouched.end();

                    for (; sItr != sItrEnd; ++sItr, ++colIdx, ++constrIdx) {
                        
                        if (finalNumericVars[sItr->first].statusOfThisFluent != FluentTracking::FS_NORMAL) {
                            --colIdx;
                            --constrIdx;
                            continue;
                        }
                        
                        
                        #ifndef NDEBUG
                        bool foundCTS = false;
                        if (sItr->second) {
                            list<pair<int, RPGBuilder::LinearEffects::EffectExpression> > & geList = gradientEffects[actID][0];
                            list<pair<int, RPGBuilder::LinearEffects::EffectExpression> >::iterator geItr = geList.begin();
                            const list<pair<int, RPGBuilder::LinearEffects::EffectExpression> >::iterator geEnd = geList.end();

                            for (; geItr != geEnd; ++geItr) {
                                if (geItr->first == sItr->first) {
                                    foundCTS = true;
                                    break;
                                }
                            }
                        }
                        assert(!sItr->second || foundCTS);
                        #endif

                        static vector<pair<int,double> > entries(2);

                        entries[0].second = 1.0;
                        entries[1].second = -1.0;

                        entries[0].first = colIdx;
                        entries[1].first = beforeStep[sItr->first];

                        assert(lp->getNumCols() == colIdx);
                        
                        lp->addCol(emptyEntries, -LPinfinity, LPinfinity, MILPSolver::C_REAL);

                        assert((lp->getNumCols() -1) == colIdx);
                        
                        if (nameLPElements) {
                            ostringstream namestream;
                            namestream << *(RPGBuilder::getPNE(sItr->first));
                            namestream << "a" << stepID;
                            string asString = namestream.str();
                            lp->setColName(colIdx, asString);
                        }


                        lp->addRow(entries, 0.0, 0.0);

                        if (nameLPElements) {
                            ostringstream namestream;
                            namestream << "no-eff@" << stepID;
                            string asString = namestream.str();
                            lp->setRowName(constrIdx, asString);
                        }

                        atStep[sItr->first] = colIdx;
                    }
                }
            }

            if (currTS == VAL::E_AT_START) {
                activeInvariants.insert(constraints[actID][1].begin(), constraints[actID][1].end());
                if (lpDebug & 1024) {
                    cout << COLOUR_light_green << "Recording invariants started at " << stepID << COLOUR_default << endl;
                }
                recordVariablesInvolvedInThisStepsInvariants(constraints[actID][1], invariantsThisStepStartsOnVariableI[stepID]);
                
            } else if (currTS == VAL::E_AT_END) {
                activeInvariants.erase(constraints[actID][1].begin(), constraints[actID][1].end());
                correspondEndHasBeenInserted[currEvent.pairWithStep] = true;               
            }

            adder.applyTo = constrsOver;

            for_each(activeInvariants.begin(), activeInvariants.end(), adder);

            bool durationIsFixed = true;
            
            if (currTS == VAL::E_AT_START && !RPGBuilder::getRPGDEs(actID).empty()) {


                DurationAdder durAdder(this, currEvent, stepID, beforeStep, stableVariable);

                RPGBuilder::RPGDuration* const currDuration = RPGBuilder::getRPGDEs(actID).back();

                if (!currDuration->fixed.empty()) {

                    durAdder.setStartEnd(currVar, imaginaryMinMax[stepID].imaginaryMin, VAL::E_EQUALS);
                    for_each(currDuration->fixed.begin(), currDuration->fixed.end(), durAdder);
                }

                if (!currDuration->min.empty()) {

                    durAdder.setStartEnd(currVar, imaginaryMinMax[stepID].imaginaryMin, VAL::E_GREATEQ);
                    for_each(currDuration->min.begin(), currDuration->min.end(), durAdder);

                }

                if (!currDuration->max.empty()) {

                    durAdder.setStartEnd(currVar, (currDuration->fixed.empty() ? imaginaryMinMax[stepID].imaginaryMax : imaginaryMinMax[stepID].imaginaryMin), VAL::E_LESSEQ);
                    for_each(currDuration->max.begin(), currDuration->max.end(), durAdder);

                }
                
                durationIsFixed = durAdder.durationIsFixed;

            }

            if (currTS == VAL::E_AT_START) {
                list<pair<int, RPGBuilder::LinearEffects::EffectExpression> > & geList = gradientEffects[actID][0];
                list<pair<int, RPGBuilder::LinearEffects::EffectExpression> >::iterator geItr = geList.begin();
                const list<pair<int, RPGBuilder::LinearEffects::EffectExpression> >::iterator geEnd = geList.end();
                for (; geItr != geEnd; ++geItr) {
                    FluentTracking & currTracker = finalNumericVars[geItr->first];

                    if (currTracker.statusOfThisFluent != FluentTracking::FS_NORMAL) {
                        if (lpDebug & 1) {
                            cout << "Ignoring order-independent CTS effect on " << *(RPGBuilder::getPNE(geItr->first)) << endl;
                        }
                        // If the status is FS_IGNORE, we should ignore it;
                        // If the status is FS_ORDER_INDEPENDENT, we can just take
                        // the integral of the change at the end
                        continue;
                    }
                    ++(currTracker.activeGradientCount);
                    currTracker.activeGradient += geItr->second.constant;
                    currTracker.lastEffectTimestampVariable = timestampVars[stepID];
                    currTracker.lastEffectValueVariable = (*constrsOver)[geItr->first];
                    currTracker.postLastEffectValue = NaN;
                    stableVariable[geItr->first] = false;
                    
                    if (!durationIsFixed) {
                        
                        // If an action has a non-constant-valued duration AND a cts effect on a variable, then we need to
                        // mark that that variable is subject to duration-dependent change.  Otherwise, it will be considered
                        // stable once the action has ended; whereas, really, the time it ends at (and hence the
                        // cumulative effect) can vary.
                        if (lpDebug & 1) cout << "Variable " << *(RPGBuilder::getPNE(geItr->first)) << " becomes duration-dependent\n";
                        finalNumericVars[geItr->first].everHadADurationDependentEffect = true;
                    }
                    
                    if (lpDebug & 1) {
                        cout << "Variable " << *(RPGBuilder::getPNE(geItr->first)) << " becomes unstable due to CTS effect; gradient is now " << currTracker.activeGradient << "\n";
                    }
                }

            } else if (currTS == VAL::E_AT_END) {

                list<pair<int, RPGBuilder::LinearEffects::EffectExpression> > & geList = gradientEffects[actID].back();
                list<pair<int, RPGBuilder::LinearEffects::EffectExpression> >::iterator geItr = geList.begin();
                const list<pair<int, RPGBuilder::LinearEffects::EffectExpression> >::iterator geEnd = geList.end();
                for (; geItr != geEnd; ++geItr) {

                    FluentTracking & currTracker = finalNumericVars[geItr->first];
                    
                    if (currTracker.statusOfThisFluent == FluentTracking::FS_NORMAL) {
                        if (!--(currTracker.activeGradientCount)) {
                            currTracker.activeGradient = 0.0;
                            if (!finalNumericVars[geItr->first].everHadADurationDependentEffect) {
                                stableNextTime.push_back(geItr->first);
                                if (lpDebug & 1) cout << "Variable " << *(RPGBuilder::getPNE(geItr->first)) << " becomes stable next time: " << currTracker.activeGradientCount << " active gradients on it, and never a duration-dependent effect\n";
                            }
                        } else {
                            currTracker.activeGradient -= geItr->second.constant;
                        }
                        
                    } else if (currTracker.statusOfThisFluent == FluentTracking::FS_ORDER_INDEPENDENT) {
                        
                        // For order-independent change, we integrate the effect that has just finished -
                        // add gradient * (end - start) to the terms
                        
                        currTracker.orderIndependentValueTerms.insert(make_pair(timestampVars[stepID], 0.0)).first->second += geItr->second.constant;
                        currTracker.orderIndependentValueTerms.insert(make_pair(timestampVars[correspondingStart[stepID]], 0.0)).first->second -= geItr->second.constant;
                        
                        if (lpDebug & 1) {
                            cout << "Noting order-independent CTS effect on " << *(RPGBuilder::getPNE(geItr->first)) << endl;
                            cout << " = " << geItr->second.constant << " * (" << lp->getColName(timestampVars[stepID]);
                            cout << " - " << lp->getColName(timestampVars[correspondingStart[stepID]]) << ")\n";
                                 
                        }
                        
                        
                    }
                }

            }

            {

                InterestingMap::iterator ciItr = currInterest.begin();
                const InterestingMap::iterator ciEnd = currInterest.end();

                for (; ciItr != ciEnd; ++ciItr) {
                    if (ciItr->second) {
                        const int effVar = ciItr->first;
                        FluentTracking & currTracker = finalNumericVars[effVar];

                        if (currTracker.statusOfThisFluent == FluentTracking::FS_NORMAL) {
                            currTracker.lastEffectTimestampVariable = timestampVars[stepID];
                            currTracker.lastEffectValueVariable = (*constrsOver)[effVar];
                            currTracker.postLastEffectValue = NaN;
                        }
                    }
                }

            }

        }




        {

            set<int> ancestorsToPassOn = activeAncestorsOfStep[stepID];
            if (currEvent.time_spec == VAL::E_AT_START) {
                if (!constraints[actID][1].empty()) {
                    ancestorsToPassOn.insert(stepID);
                }
            } else if (currEvent.time_spec == VAL::E_AT_END) {
                ancestorsToPassOn.erase(currEvent.pairWithStep);
            }

            // Finally, handle ordering constraints

            bool tilSwitch = false;

            map<int, bool>::iterator foItr = fanOut[stepID].begin();
            const map<int, bool>::iterator foEnd = fanOut[stepID].end();

            if (foItr == foEnd) {
                if (currEvent.time_spec != VAL::E_AT) endsOfThreads.push_back(timestampVars[stepID]);
            } else {
                for (; foItr != foEnd; ++foItr) {
                    const int succ = foItr->first;
                    if (!(--(fanIn[succ]))) {
                        openList.push_back(succ);
                        if (lpDebug & 512) {
                            cout << "No predecessors remaining for " << succ << " - placing it on the open list\n";
                        }
                    } else {
                        if (lpDebug & 512) {
                            cout << "Now only " << fanIn[succ] << " predecessors for step " << succ << endl;
                        }
                    }

                    // remembering to push forwards which invariants need to be checked at each successor step

                    activeAncestorsOfStep[succ].insert(ancestorsToPassOn.begin(), ancestorsToPassOn.end());

                    if (currEvent.time_spec != VAL::E_AT) {
                        if (planAsAVector[succ]->time_spec == VAL::E_AT) {
                            if (!tilSwitch) {
                                endsOfThreads.push_back(timestampVars[stepID]);
                                tilSwitch = true;
                            }
                        }
                    }
                }
            }
        }

    }

    if (actuallyVisited < mustVisit) {
        if (lpDebug & 2) {
            cout << "Never visited:\n";
            
            {
                int stepID = 0;
                for (int pass = 0; pass < 2; ++pass) {
                    list<FFEvent> & currList = (pass ? now : header);
                    
                    list<FFEvent>::iterator citr = currList.begin();
                    const list<FFEvent>::iterator cend = currList.end();
                    
                    for (; citr != cend; ++citr, ++stepID, ++mustVisit) {
                        
                        if (fanIn[stepID]) {
                            cout << stepID << ": ";
                            if (planAsAVector[stepID]->action) {
                                if (planAsAVector[stepID]->time_spec == VAL::E_AT_START) {
                                    cout << "\t" << *(planAsAVector[stepID]->action) << ", start";
                                } else {
                                    cout << "\t" << *(planAsAVector[stepID]->action) << ", end";
                                }
                            }
                            cout << endl;
                        }
                        
                    }
                }
            }
        
        }
        solved = false;
        return;
    }

    // Now to sort out 'now'.
    // First, find which variables are undergoing continuous change, and make a pair of placeholder variables:
    // - one for the time at which we are going to read their value
    // - one containing the value itself
    //
    // This loop will add constraints for the latter, linking its value to the last thing that changed it, and scheduling it after that
    // After that, we add constraints to the former scheduling it before the ends of open actions who are changing this variable

    // The timestamp of 'now', if doing total order search
    int totalOrderNowTimestamp = -1;

    for (int varID = 0; varID < numVars; ++varID) {
        FluentTracking & currTracker = finalNumericVars[varID];

        // for fluents we are ignoring, or putting straight into the metric, we can skip this
        if (currTracker.statusOfThisFluent != FluentTracking::FS_NORMAL) continue;
        
        // if no gradients are active, 'lastEffectValueVariable' is adequate as it can't have changed since then        
        if (!currTracker.activeGradientCount) continue;

        lp->addCol(emptyEntries, -LPinfinity, LPinfinity, MILPSolver::C_REAL);

        const int nowValueIdx = lp->getNumCols() - 1;

        if (nameLPElements) {
            ostringstream namestream;
            namestream << *(RPGBuilder::getPNE(varID));
            namestream << "-now-v";
            string asString = namestream.str();
            lp->setColName(nowValueIdx, asString);
        }

        int nowTimeIdx;
        if (Globals::totalOrder) {
            
            if (totalOrderNowTimestamp == -1) {
                lp->addCol(emptyEntries, 0.0, LPinfinity, MILPSolver::C_REAL);                
                nowTimeIdx = totalOrderNowTimestamp = lp->getNumCols() - 1;
                
                if (nameLPElements) {
                    lp->setColName(nowTimeIdx, "now-t");
                }
                
                static vector<pair<int,double> > entries(2);
                
                entries[0].first = nowTimeIdx;
                
                assert(timestampToUpdateVar != -1);
                
                entries[1].first = timestampToUpdateVar;
                entries[0].second = 1.0;
                entries[1].second = -1.0;
                
                // 'now' must come no sooner than epsilon after the most recent step
                lp->addRow(entries, EPSILON, LPinfinity);
                
                if (nameLPElements) {
                    const int constrIdx = lp->getNumRows() - 1;
                    lp->setRowName(constrIdx, "last-before-now-t");
                }
                
            } else {            
                nowTimeIdx = totalOrderNowTimestamp;
            }
            
        } else {

            lp->addCol(emptyEntries, 0.0, LPinfinity, MILPSolver::C_REAL);

            nowTimeIdx = lp->getNumCols() - 1;

            if (nameLPElements) {
                ostringstream namestream;
                namestream << *(RPGBuilder::getPNE(varID));
                namestream << "-now-t";
                string asString = namestream.str();
                lp->setColName(nowTimeIdx, asString);
            }

            // now need to encode tnow >= tlast
            // i.e. tnow - tlast >= 0

            {
                static vector<pair<int,double> > entries(2);
                
                entries[0].first = nowTimeIdx;
                entries[1].first = currTracker.lastEffectTimestampVariable;
                entries[0].second = 1.0;
                entries[1].second = -1.0;

                lp->addRow(entries, 0.0, LPinfinity);

                if (nameLPElements) {
                    const int constrIdx = lp->getNumRows() - 1;
                    ostringstream namestream;
                    namestream << *(RPGBuilder::getPNE(varID));
                    namestream << "-last-now-t";
                    string asString = namestream.str();
                    lp->setRowName(constrIdx, asString);
                }
            }
        }
        
        // now need to encode v(now) = v(last) + grad * (tnow - tlast)
        // i.e. v(now) - v(last) - grad * tnow + grad * tlast = 0
        {
            static vector<pair<int,double> > entries(4);
            
            entries[0].first = nowValueIdx;
            entries[1].first = currTracker.lastEffectValueVariable;
            entries[2].first = nowTimeIdx;
            entries[3].first = currTracker.lastEffectTimestampVariable;

            entries[0].second = 1.0;
            entries[1].second = -1.0;
            entries[2].second = -currTracker.activeGradient;
            entries[3].second = currTracker.activeGradient;

            lp->addRow(entries, 0.0, 0.0);

            if (nameLPElements) {
                const int constrIdx = lp->getNumRows() - 1;
                ostringstream namestream;
                namestream << *(RPGBuilder::getPNE(varID));
                namestream << "-now";
                string asString = namestream.str();
                lp->setRowName(constrIdx, asString);
            }
        }

        // this specific now then becomes the latest thing to establish a value for this variable

        currTracker.lastEffectTimestampVariable = nowTimeIdx;
        currTracker.lastEffectValueVariable = nowValueIdx;

    }

    for (int stepID = 0; stepID < tsVarCount; ++stepID) {

        FFEvent & currEvent = *(planAsAVector[stepID]);

        if (currEvent.getEffects) continue; // if we don't get its effects yet, it must be the end of an open action

        const int actID = currEvent.action->getID();

        list<pair<int, RPGBuilder::LinearEffects::EffectExpression> > & geList = gradientEffects[actID][0];

        list<pair<int, RPGBuilder::LinearEffects::EffectExpression> >::iterator geItr = geList.begin();
        const list<pair<int, RPGBuilder::LinearEffects::EffectExpression> >::iterator geEnd = geList.end();

        for (; geItr != geEnd; ++geItr) {
            FluentTracking & currTracker = finalNumericVars[geItr->first];

            // skip for fluents we are ignoring, or adding straight to the objective
            if (currTracker.statusOfThisFluent != FluentTracking::FS_NORMAL) continue;
            
            
            // now need to encode tend >= tnow
            // i.e. tend - tnow >= 0
            {
                static vector<pair<int,double> > entries(2);
                
                entries[0].first = timestampVars[stepID];
                entries[1].first = currTracker.lastEffectTimestampVariable;
                entries[0].second = 1.0;
                entries[1].second = -1.0;

                lp->addRow(entries, 0.0, LPinfinity);

                if (nameLPElements) {
                    const int constrIdx = lp->getNumRows() - 1;
                    ostringstream namestream;
                    namestream << "tnow" << *(RPGBuilder::getPNE(geItr->first)) << " <= t" << stepID;
                    string asString = namestream.str();
                    lp->setRowName(constrIdx, asString);
                }
            }

        }
    }


    { // now add the movable 'now' timestamp



        {
            map<int, list<list<StartEvent>::iterator > >::iterator ceItr = compulsaryEnds.begin();
            const map<int, list<list<StartEvent>::iterator > >::iterator ceEnd = compulsaryEnds.end();

            for (; ceItr != ceEnd; ++ceItr) {
                //const int actID = ceItr->first;
                //list<EndDetails> & destList = openDurationConstraints[actID];
                list<list<StartEvent>::iterator >::iterator matchItr = ceItr->second.begin();
                const list<list<StartEvent>::iterator >::iterator matchEnd = ceItr->second.end();

                for (; matchItr != matchEnd; ++matchItr) {
                    if ((*matchItr)->ignore) continue;
                    //EndDetails & currEndDetails = imaginaryMinMax[(*matchItr)->stepID];

                    /*if (optimised) {
                        const double localMin = (*matchItr)->lpMinTimestamp;
                        const double localMax = (*matchItr)->lpMaxTimestamp;
                        if (localMax < COIN_DBL_MAX) {
                            {
                                double exUp = lp->getColUpper()[currEndDetails.imaginaryMax];
                                if (exUp > localMax) {
                                    lp->setColUpper(currEndDetails.imaginaryMax, localMax);
                                }
                            }
                            {
                                double exUp = lp->getColUpper()[currEndDetails.imaginaryMin];
                                if (exUp > localMax) {
                                    lp->setColUpper(currEndDetails.imaginaryMin, localMax);
                                }
                            }
                        }
                        {
                            double exLow = lp->getColLower()[currEndDetails.imaginaryMax];
                            if (exLow < localMin) {
                                lp->setColLower(currEndDetails.imaginaryMax, localMin);
                            }
                        }
                        {
                            double exLow = lp->getColLower()[currEndDetails.imaginaryMin];
                            if (exLow < localMin) {
                                lp->setColLower(currEndDetails.imaginaryMin, localMin);
                            }
                        }


                    }*/

                    /*{

                        weightScratch[0] = -1.0;
                        weightScratch[1] = 1.0;

                        varScratch[0] = currVar;
                        varScratch[1] = currEndDetails.imaginaryMax;

                        lp->addRow(2, varScratch, weightScratch, 0.0, COIN_DBL_MAX);

                        currEndDetails.first = *matchItr;

                        destList.push_back(currEndDetails);

                                                if (nameLPElements) {
                            int constrIdx = lp->getNumRows() - 1;
                            ostringstream namestream;
                            namestream << "tnow < iEnd" << (*matchItr)->stepID;
                            string asString = namestream.str();
                            lp->setRowName(constrIdx, asString);
                        }

                    }*/

                    /*if (orderOptimised) {
                        set<int>::iterator orderItr = (*matchItr)->getEndComesBefore().begin();
                        const set<int>::iterator orderEnd = (*matchItr)->getEndComesBefore().end();

                        for (; orderItr != orderEnd; ++orderItr) {

                            if (*orderItr == -1) {
                                const int colIdx = imaginaryMinMax[(*matchItr)->stepID].imaginaryMin;
                                if (lp->getColUpper()[colIdx] < TILupbo) lp->setColUpper(colIdx, TILupbo);
                            } else {
                                weightScratch[0] = -1.0;
                                weightScratch[1] = 1.0;

                                varScratch[0] = imaginaryMinMax[(*matchItr)->stepID].imaginaryMin;
                                varScratch[1] = imaginaryMinMax[*orderItr].imaginaryMin;

                                lp->addRow(2, varScratch, weightScratch, 0.001, DBL_MAX);

                                if (nameLPElements) {
                                    int constrIdx = lp->getNumRows() - 1;
                                    ostringstream namestream;
                                    namestream << imaginaryMinMax[(*matchItr)->stepID].imaginaryMin << " ecb " << imaginaryMinMax[*orderItr].imaginaryMin;
                                    string asString = namestream.str();
                                    lp->setRowName(constrIdx, asString);
                                }
                            }
                        }
                    }*/

                    /*if (orderOptimised) {
                        set<int>::iterator orderItr = (*matchItr)->getEndComesAfter().begin();
                        const set<int>::iterator orderEnd = (*matchItr)->getEndComesAfter().end();

                        for (; orderItr != orderEnd; ++orderItr) {
                            weightScratch[0] = 1.0;
                            weightScratch[1] = -1.0;

                            varScratch[0] = imaginaryMinMax[(*matchItr)->stepID].imaginaryMin;
                            varScratch[1] = imaginaryMinMax[*orderItr].imaginaryMin;

                            lp->addRow(2, varScratch, weightScratch, 0.001, DBL_MAX);

                            if (nameLPElements) {
                                int constrIdx = lp->getNumRows() - 1;
                                ostringstream namestream;
                                namestream << imaginaryMinMax[(*matchItr)->stepID].imaginaryMin << " eca " << imaginaryMinMax[*orderItr].imaginaryMin;
                                string asString = namestream.str();
                                lp->setRowName(constrIdx, asString);
                            }

                        }
                    }*/


                    /*if (prevVar != -1) {
                        weightScratch[0] = -1.0;
                        weightScratch[1] = 1.0;

                        varScratch[0] = prevVar;
                        varScratch[1] = currEndDetails.imaginaryMin;

                        lp->addRow(2, varScratch, weightScratch, 0.001, COIN_DBL_MAX);

                                                if (nameLPElements) {
                            int constrIdx = lp->getNumRows() - 1;
                            ostringstream namestream;
                            namestream << "iend" << (*matchItr)->stepID << " tprev";
                            string asString = namestream.str();
                            lp->setRowName(constrIdx, asString);
                        }
                    }*/


                    /*                    RPGBuilder::RPGDuration* currDuration = RPGBuilder::getRPGDEs(actID)[(*matchItr)->divisionsApplied];

                                        if (currDuration->fixed) {
                                            varScratch[0] = currVar;
                                            varScratch[1] = timestampVars[(*matchItr)->stepID];

                                            RPGBuilder::DurationExpr * const currDE = currDuration->fixed;
                                            const int vSize = currDE->weights.size();
                                            for (int v = 0; v < vSize; ++v) {
                                                weightScratch[2+v] = currDE->weights[v];
                                                if (assertions) assert(weightScratch[2+v] != 0.0);
                                                varScratch[2+v] = fluentsAtStep[(*matchItr)->stepID][currDE->variables[v]];
                                            }
                                            weightScratch[0] = -1.0;
                                            weightScratch[1] = 1.0;

                                            const double rTerm = (currDE->constant == 0.0 ? 0.0 : -currDE->constant);
                                            add_constraintex(lp, 2 + vSize, weightScratch, varScratch, GE, rTerm);
                                            ++constrIdx;
                                            destList.push_back(pair<list<StartEvent>::iterator, int>(*matchItr, constrIdx));

                                        } else if (currDuration->max) {
                                            varScratch[0] = currVar;
                                            varScratch[1] = timestampVars[(*matchItr)->stepID];

                                            RPGBuilder::DurationExpr * const currDE = currDuration->max;

                                            const int vSize = currDE->weights.size();
                                            for (int v = 0; v < vSize; ++v) {
                                                weightScratch[2+v] = currDE->weights[v];
                                                if (assertions) assert(weightScratch[2+v] != 0.0);
                                                varScratch[2+v] = fluentsAtStep[(*matchItr)->stepID][currDE->variables[v]];
                                            }
                                            weightScratch[0] = -1.0;
                                            weightScratch[1] = 1.0;

                                            const double rTerm = (currDE->constant == 0.0 ? 0.0 : -currDE->constant);

                                            add_constraintex(lp, 2 + vSize, weightScratch, varScratch, GE, rTerm);
                                            ++constrIdx;
                                            destList.push_back(pair<list<StartEvent>::iterator, int>(*matchItr, constrIdx));
                                        } else {
                                            destList.push_back(pair<list<StartEvent>::iterator, int>(*matchItr, -1));
                                        }
                    */

                }
            }

        }

    }

    if (lpDebug & 1) cout << "LP complete, " << lp->getNumCols() << " columns, " << lp->getNumRows() << " rows\n";

    if (setObjectiveToMetric) {
        if (!scheduleToMetric()) {
            solved = false;
            return;
        }
    }
    


//    if (lpDebug & 2) print_lp(lp);
    if (lpDebug & 4) nicerLPPrint(lp);
    if (lpDebug & 8) checkForZeroRows(lp);

    if (optVar == -1) {
        solved = true;
        return;
    }

    if (lpDebug & 8) {
        lp->writeLp("stateevaluation.lp");
        cout << "About to call solve\n";
    }

    solved = lp->solve(false);
    
    if (lpDebug & 8) {
        if (solved) {
            cout << "Solve called succeeded\n";
        } else {
            cout << "Solve called failed\n";
        }
    }

    if (solved) {
        if (shouldFail) {
            if (cd) cd->printDotFile(cout);
        }
        assert(!shouldFail);

        const int loopLim = planAsAVector.size();
        
        const double * const partialSoln = lp->getPartialSolution(numVars, numVars + loopLim);        

        assert(!cd || !cd->willSetTimestamps());
        
        for (int stepID = 0; stepID < loopLim; ++stepID) {
            FFEvent * const itr = planAsAVector[stepID];
            if (itr->time_spec == VAL::E_AT) {
                itr->lpTimestamp = TILtimestamps[itr->divisionID];
                continue;
            }
            itr->lpTimestamp = partialSoln[timestampVars[stepID] - numVars];
            if (lpDebug & 2) {
                cout << "Timestamp of " << *(itr->action) << " (var " << timestampVars[stepID] << ") ";
                if (itr->time_spec == VAL::E_AT_START) cout << " start = "; else cout << "end = ";
                cout << itr->lpTimestamp << "\n";                
            }
        }

        if (!setObjectiveToMetric) {
            
            if (Globals::paranoidScheduling && cd && !cd->doLPSolve()) {
                cd->distsToLPMinStampsAndCheck(planAsAVector);
            }
            
            if (optimised) {
                if (timestampToUpdate) {
                    pushTimestampToMin();
                }
            }                                        
                                
        
            if (Globals::paranoidScheduling && cd && !cd->doLPSolve()) {
                cd->distsToLPStamps(justAppliedStep);
            }
        }
        
        /*if (lpDebug & 8 && !mutexCols.empty()) {
            cout << "Values of TIL mutex column variables:\n";
            {
                list<int>::const_iterator mcItr = mutexCols.begin();
                const list<int>::const_iterator mcEnd = mutexCols.end();
                for (; mcItr != mcEnd; ++mcItr) {
                    cout << "\t" << lp->getColName(*mcItr) << " = " << lp->getSingleSolutionVariableValue(*mcItr) << endl;
                    assert(lp->isColumnBinary(*mcItr));
                }
            }
            
            cout << "Values of TIL mutex rows:\n";
            {
                list<int>::const_iterator mcItr = mutexRows.begin();
                const list<int>::const_iterator mcEnd = mutexRows.end();
                for (; mcItr != mcEnd; ++mcItr) {
                    printRow(lp, *mcItr, *mcItr+1);
                    cout << "\t" << lp->getRowName(*mcItr) << " = " << lp->getSingleSolutionRowValue(*mcItr) << endl;
                }
            }
        }*/

    } else {
        if (lpDebug) {
            if (shouldSucceed) {
                cout << "According to STP, LP call should have succeeded\n";
                if (cd) cd->printDotFile(cout);
            }
        }
        assert(!shouldSucceed);
    }
};

LPScheduler::~LPScheduler()
{

    delete lp;
    delete cd;
};



const LPScheduler::Constraint* LPScheduler::buildConstraint(RPGBuilder::RPGNumericPrecondition & pre)
{

    if (lpDebug & 4) cout << pre << " with op " << pre.op << " becomes:";

    Constraint toReturn;

    switch (pre.op) {
    case VAL::E_GREATER: {
        toReturn.upper = DBL_MAX;
        toReturn.lower = pre.RHSConstant + 0.0001; // HACK - some small value to fake greater-than using a GE
        if (lpDebug & 4) cout << " >= " << toReturn.lower;
        break;
    }
    case VAL::E_GREATEQ: {
        toReturn.upper = DBL_MAX;
        toReturn.lower = pre.RHSConstant;
        if (lpDebug & 4) cout << " >= " << toReturn.lower;
        break;
    }
    case VAL::E_LESS: {
        toReturn.lower = -DBL_MAX;
        toReturn.upper = pre.RHSConstant - 0.0001; // HACK - some small value to fake less-than using a LE
        if (lpDebug & 4) cout << " <= " << toReturn.upper;
        break;
    }
    case VAL::E_LESSEQ: {
        toReturn.lower = -DBL_MAX;
        toReturn.upper = pre.RHSConstant;
        if (lpDebug & 4) cout << " <= " << toReturn.upper;
        break;
    }
    case VAL::E_EQUALS: {
        toReturn.lower = toReturn.upper = pre.RHSConstant;
        break;
    }
    };

    int v = pre.LHSVariable;
    if (v < numVars) { // simple variable
        toReturn.weights = vector<double>(1);
        toReturn.variables = vector<int>(1);

        toReturn.weights[0] = pre.LHSConstant;
        toReturn.variables[0] = pre.LHSVariable;
        if (lpDebug & 4) cout << "Constraint on simple variable: \n";
    } else {
        v -= numVars;

        if (v < numVars) { // negative variable

            if (lpDebug & 4) cout << "Constraint on negative variable: \n";
            toReturn.weights = vector<double>(1);
            toReturn.variables = vector<int>(1);

            toReturn.weights[0] = -1 * pre.LHSConstant;
            toReturn.variables[0] = pre.LHSVariable;
        } else {
            if (lpDebug & 4) cout << "Constraint on AV: \n";
            
            RPGBuilder::ArtificialVariable & av = RPGBuilder::getArtificialVariable(pre.LHSVariable);
            const int loopLim = av.size;
            toReturn.weights = vector<double>(loopLim);
            toReturn.variables = vector<int>(loopLim);

            for (int s = 0; s < loopLim; ++s) {
                toReturn.weights[s] = av.weights[s];
                int lv = av.fluents[s];
                if (lv >= numVars) {
                    lv -= numVars;
                    toReturn.weights[s] *= -1;
                }
                toReturn.variables[s] = lv;
            }

            if (pre.op == VAL::E_GREATER || pre.op == VAL::E_GREATEQ) {
                toReturn.lower -= av.constant; // constant term from the AV
            } else if (pre.op == VAL::E_EQUALS) {
                toReturn.lower -= av.constant;
                toReturn.upper -= av.constant;
            } else {
                toReturn.upper += av.constant; // constant term from the AV
            }

        }
    }
    if (lpDebug & 4) {
        int Slim = toReturn.weights.size();
        for (int i = 0; i < Slim; ++i) {
            if (i) cout << " + ";
            cout << toReturn.weights[i] << "." << *(RPGBuilder::getPNE(toReturn.variables[i]));
        }

        cout << " in [";

        if (toReturn.lower != -DBL_MAX) {
            cout << toReturn.lower << ",";
        } else {
            cout << "-inf,";
        }

        if (toReturn.upper != DBL_MAX) {
            cout << toReturn.upper << "]\n";
        } else {
            cout << "inf]\n";
        }

    }

    return Constraint::requestConstraint(toReturn);


};

void LPScheduler::initialise()
{

    initialised = true;

    const bool initDebug = false;

    numVars = RPGBuilder::getPNECount();
    const int actCount = RPGBuilder::getFixedDEs().size();

    gradientEffects .resize(actCount);
    instantEffects.resize(actCount);
    constraints.resize(actCount);
    interesting.resize(actCount);
    boringAct.resize(actCount, vector<pair<bool,bool> >(2, pair<bool,bool>(true,true)));
    pointsThatWouldBeMutexWithOptimisationTILs.resize(actCount, vector<vector<double> >(2));
    
    if (initDebug) cout << "Initialising LP lookups for " << actCount << " actions\n";

    for (int a = 0; a < actCount; ++a) {

        if (initDebug) {
            cout << "[" << a << "] ";
            cout.flush();
        }

        if (!RPGBuilder::rogueActions[a]) {

            interesting[a] = vector<InterestingMap>(3);

            RPGBuilder::LinearEffects* const discretisation = RPGBuilder::getLinearDiscretisation()[a];

            if (discretisation) {
                const int divisions = discretisation->divisions;

                bool allMetricTracking = true;
                {
                    vector<list<pair<int, RPGBuilder::LinearEffects::EffectExpression> > > & toFill = gradientEffects[a] = vector<list<pair<int, RPGBuilder::LinearEffects::EffectExpression> > >(divisions);
                    const int varCount = discretisation->vars.size();
                    for (int v = 0; v < varCount; ++v) {
                        const int currVar = (discretisation->vars)[v];
                        if (currVar < 0) continue;
                        interesting[a][0].insertEffect(currVar);
                        interesting[a][1].insertEffect(currVar);
                        interesting[a][2].insertEffect(currVar);
                        for (int d = 0; d < divisions; ++d) {
                            toFill[d].push_back(pair<int, RPGBuilder::LinearEffects::EffectExpression>(currVar, discretisation->effects[d][v]));
                        }
                        if (   NumericAnalysis::getDominanceConstraints()[currVar] != NumericAnalysis::E_METRICTRACKING
                            && NumericAnalysis::getDominanceConstraints()[currVar] != NumericAnalysis::E_IRRELEVANT) {
                            allMetricTracking = false;
                        }
                    }
                }

                boringAct[a][0].first = allMetricTracking;
                boringAct[a][1].first = allMetricTracking;
                
                boringAct[a][0].second = false;
                boringAct[a][1].second = false;


            } else {
                gradientEffects[a] = vector<list<pair<int, RPGBuilder::LinearEffects::EffectExpression> > >(1);
            }

            {

                vector<list<RPGBuilder::RPGNumericEffect* > > & toFill = instantEffects[a] = vector<list<RPGBuilder::RPGNumericEffect* > >(2);

                for (int pass = 0; pass < 2; ++pass) {
                    list<int> & currList = (pass ? RPGBuilder::getEndEffNumerics()[a] : RPGBuilder::getStartEffNumerics()[a]);

                    list<int>::iterator clItr = currList.begin();
                    const list<int>::iterator clEnd = currList.end();

                    const int iPoint = (pass ? 2 : 0);

                    for (; clItr != clEnd; ++clItr) {

                        RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[*clItr];

                        toFill[pass].push_back(&currEff);

                        interesting[a][iPoint].insertEffect(currEff.fluentIndex);

                        if (initDebug) {
                            if (iPoint == 2) {
                                cout << *(RPGBuilder::getInstantiatedOp(a)) << " end writes to " << *(RPGBuilder::getPNE(currEff.fluentIndex)) << "\n";
                            } else {
                                cout << *(RPGBuilder::getInstantiatedOp(a)) << " start writes to " << *(RPGBuilder::getPNE(currEff.fluentIndex)) << "\n";
                            }
                            assert(interesting[a][iPoint].find(currEff.fluentIndex)->second);
                        }

                        for (int s = 0; s < currEff.size; ++s) {
                            int vv = currEff.variables[s];
                            if (vv >= 0) {
                                if (vv >= numVars) vv -= numVars;
                                interesting[a][iPoint].insertPrecondition(vv);
                            } else if (vv <= -2) {
                                boringAct[a][pass].second = false;
                                if (   NumericAnalysis::getDominanceConstraints()[currEff.fluentIndex] != NumericAnalysis::E_IRRELEVANT
                                    && NumericAnalysis::getDominanceConstraints()[currEff.fluentIndex] != NumericAnalysis::E_METRICTRACKING) {
                                    boringAct[a][pass].first = false;
                                }
                            }
                        }
                    }
                }
            }

            {
                vector<list<const Constraint*> > & toFill = constraints[a] = vector<list<const Constraint*> >(3);

                for (int pass = 0; pass < 3; ++pass) {
                    list<int> & currList = (pass ? (pass == 2 ? RPGBuilder::getEndPreNumerics()[a] : RPGBuilder::getInvariantNumerics()[a]) : RPGBuilder::getStartPreNumerics()[a]);
                    
                    
                    /*if (pass == 1) {
                        cout << "In LP: " << *(RPGBuilder::getInstantiatedOp(a)) << " has " << currList.size() << " numeric invariants\n";
                    }*/
                    list<int>::iterator clItr = currList.begin();
                    const list<int>::iterator clEnd = currList.end();

                    for (; clItr != clEnd; ++clItr) {
                        toFill[pass].push_back(buildConstraint(RPGBuilder::getNumericPreTable()[*clItr]));

                        const Constraint * const curr = toFill[pass].back();
                        vector<int>::const_iterator itr = curr->variables.begin();
                        const vector<int>::const_iterator itrEnd = curr->variables.end();

                        for (; itr != itrEnd; ++itr) {
                            if (*itr >= 0) {
                                interesting[a][pass].insertPrecondition(*itr);
                            } else if (*itr <= -2) {
                                boringAct[a][0] = make_pair(false,false);
                                boringAct[a][1] = make_pair(false,false);
                            }
                        }

                    }
                }

            }
            
            if (!RPGBuilder::getRPGDEs(a).empty()) {
                RPGBuilder::RPGDuration* const currDuration = RPGBuilder::getRPGDEs(a).back();
                for (int pass = 0; pass < 3; ++pass) {
                    const list<RPGBuilder::DurationExpr*> & currDurs = (*currDuration)[pass];
                    
                    list<RPGBuilder::DurationExpr*>::const_iterator dItr = currDurs.begin();
                    const list<RPGBuilder::DurationExpr*>::const_iterator dEnd = currDurs.end();
                    
                    for (; dItr != dEnd; ++dItr) {
                        const int vCount = (*dItr)->variables.size();
                        
                        int vv;
                        for (int i = 0; i < vCount; ++i) {
                            vv = (*dItr)->variables[i];
                            if (vv >= 0) {
                                if (vv >= numVars) vv -= numVars;
                                interesting[a][0].insertPrecondition(vv);
                            }
                        }
                    }
                }
            }
            
            
            {
                pointsThatWouldBeMutexWithOptimisationTILs[a].resize(2);
                
                const list<RPGBuilder::ConditionalEffect> & ceffs = RPGBuilder::getActionsToConditionalEffects()[a];

                list<RPGBuilder::ConditionalEffect>::const_iterator ceffItr = ceffs.begin();
                const list<RPGBuilder::ConditionalEffect>::const_iterator ceffEnd = ceffs.end();
                
                for (; ceffItr != ceffEnd; ++ceffItr) {
                    
                    const list<pair<Literal*, VAL::time_spec> > & conds = ceffItr->getPropositionalConditions();
                    
                    list<pair<Literal*, VAL::time_spec> >::const_iterator condItr = conds.begin();
                    const list<pair<Literal*, VAL::time_spec> >::const_iterator condEnd = conds.end();
                    
                    for (; condItr != condEnd; ++condItr) {
                        const list<pair<double,double> > * windows = TemporalAnalysis::factIsVisibleInWindows(condItr->first);
                        
                        if (!windows) {
                            continue;
                        }
                        
                        list<pair<double,double> >::const_iterator wItr = windows->begin();
                        const list<pair<double,double> >::const_iterator wEnd = windows->end();
                        
                        for (; wItr != wEnd; ++wItr) {
                            if (wItr->second == DBL_MAX) break;
                            if (condItr->second == VAL::E_AT_START) {
                                pointsThatWouldBeMutexWithOptimisationTILs[a][0].push_back(wItr->second); // start points cannot coincide with start conditions being deleted
                            } else if (condItr->second == VAL::E_AT_END) {
                                pointsThatWouldBeMutexWithOptimisationTILs[a][1].push_back(wItr->second); // end points cannot coincide with end conditions being deleted
                            }
                        }
                    }
                }
            }

        }


    }

    if (initDebug) cout << "\n";

    {
        list<pair<int, int> > & goals = RPGBuilder::getNumericRPGGoals();
        list<pair<int, int> >::iterator gItr = goals.begin();
        const list<pair<int, int> >::iterator gEnd = goals.end();

        for (; gItr != gEnd; ++gItr) {
            if (gItr->first != -1) goalConstraints.push_back(buildConstraint(RPGBuilder::getNumericPreTable()[gItr->first]));
            if (gItr->second != -1) goalConstraints.push_back(buildConstraint(RPGBuilder::getNumericPreTable()[gItr->second]));
        }
    }


    {
        LiteralSet propositional; // don't care...
        RPGBuilder::getInitialState(propositional, initialValues);
    }

    {

        list<RPGBuilder::FakeTILAction> & TILs = RPGBuilder::getTILs();
        const int tilCount = TILs.size();
        TILtimestamps = vector<double>(tilCount + 1);

        int t = 0;
        list<RPGBuilder::FakeTILAction>::iterator tilItr = TILs.begin();
        const list<RPGBuilder::FakeTILAction>::iterator tilEnd = TILs.end();

        for (; tilItr != tilEnd; ++tilItr, ++t) {
            TILtimestamps[t] = tilItr->duration;
        }

        TILtimestamps[t] = DBL_MAX;

    }


};

void LPScheduler::addConstraintsForTILMutexes(const int & timestampVar, const vector<double> & mutexTimestamps)
{
    
    if (mutexTimestamps.empty()) return;
        
    static const vector<pair<int,double> > emptyEntries;
    
    static vector<pair<int,double> > binaryConstraint(2);
    
    const int mtCount = mutexTimestamps.size();
    
    const pair<double,double> tsBounds = make_pair(lp->getColLower(timestampVar), lp->getColUpper(timestampVar));
    
    for (int mt = 0; mt < mtCount; ++mt) {
        if (mutexTimestamps[mt] < tsBounds.first || mutexTimestamps[mt] > tsBounds.second) {
            // no need to forbid the variable from taking a value which it could not anyway
            continue;
        }
        
        
        // make a variable that takes the value 1 if timestampVar falls after mutexTimestamps[mt],
        // or 0 if it falls before
        
        lp->addCol(emptyEntries, 0, 1, MILPSolver::C_BOOL);
        
        const int beforeOrAfter = lp->getNumCols() - 1;
        
        //mutexCols.push_back(beforeOrAfter);
        
        if (nameLPElements) {
            ostringstream n;
            n << "col" << timestampVar << "neq" << mutexTimestamps[mt];
            const string cname(n.str());
            lp->setColName(beforeOrAfter, cname);
        }
        
        // First, make the setting of this variable force upper and lower bounds
        
        binaryConstraint[0].first = timestampVar;
        binaryConstraint[0].second = 1.0;
        
        binaryConstraint[1].first = beforeOrAfter;
        binaryConstraint[1].second = -N;
        
        lp->addRow(binaryConstraint, -LPinfinity, mutexTimestamps[mt] - SAFE);
        
        //mutexRows.push_back(lp->getNumRows() - 1);
        
        if (nameLPElements) {
            ostringstream n;
            n << "set" << timestampVar << "lt" << mutexTimestamps[mt];
            const string cname(n.str());
            lp->setRowName(lp->getNumRows() - 1, cname);
        }
        
        binaryConstraint[0].first = timestampVar;
        binaryConstraint[0].second = 1.0;
        
        binaryConstraint[1].first = beforeOrAfter;
        binaryConstraint[1].second = -(mutexTimestamps[mt] + SAFE);
        
        lp->addRow(binaryConstraint, 0, LPinfinity);
        
        //mutexRows.push_back(lp->getNumRows() - 1);
        
        if (nameLPElements) {
            ostringstream n;
            n << "set" << timestampVar << "gt" << mutexTimestamps[mt];
            const string cname(n.str());
            lp->setRowName(lp->getNumRows() - 1, cname);
        }
        
        // Second, make the setting of the timestamp force the correct value of this variable
        
        binaryConstraint[0].first = timestampVar;
        binaryConstraint[0].second = -1.0;
        
        binaryConstraint[1].first = beforeOrAfter;
        binaryConstraint[1].second = N;
        
        lp->addRow(binaryConstraint, -(mutexTimestamps[mt] - SAFE), LPinfinity);
        
        //mutexRows.push_back(lp->getNumRows() - 1);
        
        if (nameLPElements) {
            ostringstream n;
            n << "if" << timestampVar << "gt" << mutexTimestamps[mt];
            const string cname(n.str());
            lp->setRowName(lp->getNumRows() - 1, cname);
        }
        
        binaryConstraint[0].first = timestampVar;
        binaryConstraint[0].second = 1.0;
        
        binaryConstraint[1].first = beforeOrAfter;
        binaryConstraint[1].second = -N;
        
        lp->addRow(binaryConstraint, mutexTimestamps[mt] + SAFE - N, LPinfinity);
        
        //mutexRows.push_back(lp->getNumRows() - 1);
        
        if (nameLPElements) {
            ostringstream n;
            n << "if" << timestampVar << "lt" << mutexTimestamps[mt];
            const string cname(n.str());
            lp->setRowName(lp->getNumRows() - 1, cname);
        }
        
    }
}



void LPScheduler::updateStateFluents(vector<double> & min, vector<double> & max, vector<double> & timeAtWhichValueIsDefined)
{

    if (!lp) return;
    if (previousObjectiveVar == -1) {
        return;
    }
    if (timestampToUpdateVar == -1) {
        return;        
    }
        
    assert(solved);

    if (workOutFactLayerZeroBoundsStraightAfterRecentAction) {
        timeAtWhichValueIsDefined.resize(min.size(), -1.0);
    }
    
    map<int,double> knownMinValueOfColumn;
    
    knownMinValueOfColumn.insert(make_pair(timestampToUpdateVar,lp->getColLower(timestampToUpdateVar)));
    
    static const bool optimised = true;
    
    for (int s = 0; s < numVars; ++s) {
        if (   !stableVariable[s]
            && (NumericAnalysis::getDominanceConstraints()[s] != NumericAnalysis::E_METRICTRACKING)
            && (NumericAnalysis::getDominanceConstraints()[s] != NumericAnalysis::E_IRRELEVANT) ) {

            if (lpDebug & 1) cout << "New bounds on " << *(RPGBuilder::getPNE(s)) << ", were [" << min[s] << "," << max[s] << "] now: [";

            if (previousObjectiveVar != -1) lp->setObjCoeff(previousObjectiveVar, 0.0);

            bool nonDDAmethod = false;
            double oldColUpper;
        
            if (workOutFactLayerZeroBoundsStraightAfterRecentAction) {
                if (!finalNumericVars[s].everHadADurationDependentEffect) {
                    const int relevantTSVar = finalNumericVars[s].lastEffectTimestampVariable;
                    oldColUpper = lp->getColUpper(relevantTSVar);
                    nonDDAmethod = true;
                    
                    pair<map<int,double>::iterator,bool> knownMin = knownMinValueOfColumn.insert(make_pair(relevantTSVar,0.0));
                    
                    if (knownMin.second) {
                        previousObjectiveVar = relevantTSVar;
                        lp->setObjCoeff(previousObjectiveVar, 1.0);
                        lp->setMaximiseObjective(false);
                        lp->solve(false);
                        
                        knownMin.first->second = lp->getSingleSolutionVariableValue(previousObjectiveVar);
                        lp->setObjCoeff(previousObjectiveVar, 0.0);
                        
                        if (optimised) {
                            // since we minimised the value of this variable, we can use this as its lower bound from hereon
                            lp->setColLower(previousObjectiveVar, knownMin.first->second);
                        }
                    }
                    
                    timeAtWhichValueIsDefined[s] = knownMin.first->second;
                    
                    // temporary additional constraint to make sure now is as early as it can be
                    lp->setColUpper(relevantTSVar, timeAtWhichValueIsDefined[s]);
                    
                                                                
                }                
            }
            
            
            previousObjectiveVar = finalNumericVars[s].lastEffectValueVariable;
            lp->setObjCoeff(previousObjectiveVar, 1.0);

            lp->setMaximiseObjective(true);
            lp->solve(false);

            const double mv = lp->getSingleSolutionVariableValue(previousObjectiveVar);
            max[s] = mv;

            if (!nonDDAmethod && optimised) {
                lp->setColUpper(previousObjectiveVar, mv);
            }

            lp->setMaximiseObjective(false);
            lp->solve(false);
            
            const double mvTwo = lp->getSingleSolutionVariableValue(previousObjectiveVar);
            min[s] = mvTwo;
            if (!nonDDAmethod && optimised) {
                lp->setColLower(previousObjectiveVar, mvTwo);
            }
            
            if (nonDDAmethod) {
                // restore previous upper bound rather than the tighter one used
                lp->setColUpper(finalNumericVars[s].lastEffectTimestampVariable, oldColUpper);

                if (lpDebug & 1) cout << mvTwo << "," << mv << "] from t=" << timeAtWhichValueIsDefined[s] << " onwards\n";
                                            
            } else {
                if (lpDebug & 1) cout << mvTwo << "," << mv << "]\n";                            
            }

        } else {
            if (lpDebug & 1) cout << "Skipping updating bounds on " << *(RPGBuilder::getPNE(s)) << ", remain at [" << min[s] << "," << max[s] << "]\n";
        }
    }

};

void LPScheduler::extrapolateBoundsAfterRecentAction(const list<StartEvent> * startEventQueue, vector<double> & min, vector<double> & max, const vector<double> & timeAtWhichValueIsDefined)
{
    if (!lp) return;
    if (previousObjectiveVar == -1) {
        return;
    }
    if (timestampToUpdateVar == -1) {
        return;        
    }
    if (timeAtWhichValueIsDefined.empty()) {
        return;
    }
            
    assert(solved);
       
    vector<RPGBuilder::LinearEffects*> & LD = RPGBuilder::getLinearDiscretisation();
    
    list<StartEvent>::const_iterator evItr = startEventQueue->begin();
    const list<StartEvent>::const_iterator evEnd = startEventQueue->end();
    
    for (; evItr != evEnd; ++evItr) {
        
        const RPGBuilder::LinearEffects* currLD = LD[evItr->actID];
        if (!currLD) continue;
                               
        const double multiplier = evItr->maxDuration - evItr->elapsed;
        
        const vector<int> & varList = currLD->vars;
        const vector<RPGBuilder::LinearEffects::EffectExpression> & changeList = currLD->effects[0];
        
        const int effCount = varList.size();
        
        for (int e = 0; e < effCount; ++e) {
            if (timeAtWhichValueIsDefined[varList[e]] < 0.0) {
                // ignore effects upon variables where the -/ method wasn't used anyway
                continue;                
            }
            if (changeList[e].constant > 0.0) {
                max[varList[e]] += changeList[e].constant * multiplier;
                //cout << "Boosted upper bound on " << *(RPGBuilder::getPNE(varList[e])) << endl;
            }
            if (changeList[e].constant < 0.0) {
                min[varList[e]] += changeList[e].constant * multiplier;
                //cout << "Boosted lower bound on " << *(RPGBuilder::getPNE(varList[e])) << endl;
            }
        }        
    }
}


bool LPScheduler::isSolution(const MinimalState & state, list<FFEvent> & header, list<FFEvent> & now)
{
    static const vector<pair<int,double> > emptyEntries;

    if (!lp) {
        static list<pair<int, int> > & numGoals = RPGBuilder::getNumericRPGGoals();

        list<pair<int, int> >::iterator ngItr = numGoals.begin();
        const list<pair<int, int> >::iterator ngEnd = numGoals.end();

        for (; ngItr != ngEnd; ++ngItr) {
            if (ngItr->first != -1) {
                const RPGBuilder::RPGNumericPrecondition & currNP = RPGBuilder::getNumericPreTable()[ngItr->first];
                if (!currNP.isSatisfiedWCalculate(state.secondMin, state.secondMax)) return false;
            }

            if (ngItr->second != -1) {
                const RPGBuilder::RPGNumericPrecondition & currNP = RPGBuilder::getNumericPreTable()[ngItr->second];
                if (!currNP.isSatisfiedWCalculate(state.secondMin, state.secondMax)) return false;
            }

        }

        {
            list<FFEvent>::iterator itr = header.begin();
            list<FFEvent>::iterator endPt = header.end();
            for (; itr != endPt; ++itr) {
                itr->lpTimestamp = itr->lpMinTimestamp;
            }
        }

        {
            list<FFEvent>::iterator itr = now.begin();
            list<FFEvent>::iterator endPt = now.end();
            for (; itr != endPt; ++itr) {
                itr->lpTimestamp = itr->lpMinTimestamp;
            }
        }


        return true;

    }


    if (lpDebug & 1) {
        cout << "Extending model to do goal checking\n";
    }

//    const int rowCount = lp->getNumRows();
//    const int colCount = lp->getNumCols();
//    const int goalCount = goalConstraints.size();

//    lp->resize(rowCount + goalCount, colCount);

    list<const Constraint*>::iterator gItr = goalConstraints.begin();
    const list<const Constraint*>::iterator gEnd = goalConstraints.end();

    for (; gItr != gEnd; ++gItr) {
        const int cSize = (*gItr)->weights.size();

        vector<pair<int,double> > entries;
        entries.reserve(cSize);
        
        double offset = 0.0;
        int v;
        for (int s = 0 ; s < cSize; ++s) {
            v = finalNumericVars[(*gItr)->variables[s]].lastEffectValueVariable;
            if (v != -1) {
                if (lpDebug & 1) {
                    cout << "Value of " << (*gItr)->variables[s] << " is column " << v << endl;
                }
                entries.push_back(make_pair(v, (*gItr)->weights[s]));
            } else {
                if (lpDebug & 1) {
                    cout << "Value of " << (*gItr)->variables[s] << " is constant: " << finalNumericVars[(*gItr)->variables[s]].postLastEffectValue << endl;
                }
                
                offset += ((*gItr)->weights[s] * finalNumericVars[(*gItr)->variables[s]].postLastEffectValue);
            }
        }
        if (entries.empty()) {
            if ((offset < (*gItr)->lower) || (offset > (*gItr)->upper)) {
                if (lpDebug & 1) {
                    cout << "A goal constraint is not in range\n";
                }
                return false;
            } else {
                if (lpDebug & 1) {
                    cout << "Goal constraint evaluates to " << offset << ", which is in range [" << (*gItr)->lower << "..\n";
                }
            }
        } else {
            lp->addRow(entries, (*gItr)->lower - offset, (*gItr)->upper - offset);
        }
    }

    if (previousObjectiveVar != -1) lp->setObjCoeff(previousObjectiveVar, 0.0);

    lp->addCol(emptyEntries, makespanVarMinimum, LPinfinity, MILPSolver::C_REAL);

    const int makespanVar = lp->getNumCols() - 1;

    previousObjectiveVar = makespanVar; // the 1.0 passed at the end of addColumn sets its objective coefficient to 1

    {
        list<int>::iterator comesAfter = endsOfThreads.begin();
        const list<int>::iterator comesAfterEnd = endsOfThreads.end();

        for (; comesAfter != comesAfterEnd; ++comesAfter) {
            static vector<pair<int,double> > entries(2);
            
            entries[0].second = 1.0;
            entries[1].second = -1.0;

            entries[0].first = makespanVar;
            entries[1].first = *comesAfter;

            lp->addRow(entries, 0.0, LPinfinity);

        }
    }

    //cout << "Optimising makespan, lower bound is " << makespanVarMinimum << "\n";

    const bool success = lp->solve(false);

    if (success) {

        if (lpDebug & 1) cout << "Goal reached\n";
//        if (lpDebug & 2) print_solution(lp, timestampVars.back());

        //const double * lpvars = lp->getSolution();

        const double * const partialSoln = lp->getPartialSolution(numVars, numVars + header.size() + now.size()); 
        
        bool headerLoop = true;
        list<FFEvent>::iterator endPt = header.end();
        list<FFEvent>::iterator itr = header.begin();

        if (itr == endPt) {
            headerLoop = false;
            endPt = now.end();
            itr = now.begin();
        }

        //int v = timestampVars.front();

        int v = 0;
        int solStepID = 0;

        while (itr != endPt) {

            if (itr->action) {

                itr->lpTimestamp = partialSoln[v];

                if (lpDebug & 1) {
                    cout << "Var " << v << " (" << lp->getColName(v) << "), paired with " << (itr->pairWithStep) << " = ";
                    cout << partialSoln[v] << "\n";
                }
            }

            ++solStepID;
            ++v;
            ++itr;

            if (itr == endPt && headerLoop) {
                endPt = now.end();
                itr = now.begin();
                headerLoop = false;
            }
        }

    } else {
        if (lpDebug & 1) cout << "Goal not reached\n";
    }

    return solved;


};

bool LPScheduler::addAnyNumericConstraints(const list<pair<int, VAL::time_spec > > & numericConditions,
                                           const int & actStartAt, const int & actEndAt, list<int> & conditionVars)
{
    static const vector<pair<int,double> > emptyEntries(0);
    static const bool debug = (Globals::globalVerbosity & 32);    
    static const int pneCount = RPGBuilder::getPNECount();
    
    list<pair<int, VAL::time_spec > >::const_iterator condItr = numericConditions.begin();
    const list<pair<int, VAL::time_spec > >::const_iterator condEnd = numericConditions.end();
    
    for (; condItr != condEnd; ++condItr) {
        const RPGBuilder::RPGNumericPrecondition & currPre = RPGBuilder::getNumericPreTable()[condItr->first];
        
        /// TODO extend to variables other than ?duration
        
        double threshold = currPre.RHSConstant;
        VAL::comparison_op op = currPre.op;
        
        if (currPre.LHSVariable != -3) {
            if (currPre.LHSVariable < 2 * pneCount) {
                cout << "Ignoring conditional effect dependent on " << currPre << " for now\n";
                return false;
            }
            const RPGBuilder::ArtificialVariable & currAV = RPGBuilder::getArtificialVariable(currPre.LHSVariable);
            
            if (currAV.size != 1 || (currAV.fluents[0] != -3 && currAV.fluents[0] != -19)) {
                cout << "Ignoring conditional effect dependent on " << currPre << " for now\n";
                return false;
            }
            
            threshold = currPre.RHSConstant - currAV.constant;
            
            if (currAV.fluents[0] == -19) {
                if (op == VAL::E_GREATEQ) {
                    op = VAL::E_LESSEQ;
                } else if (op == VAL::E_GREATER) {
                    op = VAL::E_LESS;
                }
                if (threshold != 0.0) {
                    threshold = -threshold;
                }
            }
        }
        
        
        lp->addCol(emptyEntries, 0, 1, MILPSolver::C_BOOL);
        
        const int switchVar = lp->getNumCols() - 1;
        
        if (nameLPElements) {
            ostringstream s;
            s << "dur" << actStartAt;
            if (op == VAL::E_LESS) {
               s << "lt";
            } else if (op == VAL::E_LESSEQ) {
                s << "leq";
            }if (op == VAL::E_GREATER) {
                s << "gt";
            } else if (op == VAL::E_GREATEQ) {
                s << "geq";
            }
            s << threshold;
            string cname = s.str();
            lp->setColName(switchVar, cname);
        }

        if (debug) {
            cout << "Adding switch var " << lp->getColName(switchVar) << " for constraint (?duration ";
            switch (op) {
                case VAL::E_GREATER:
                {
                    cout << ">";
                    break;
                }
                case VAL::E_GREATEQ:
                {
                    cout << ">=";
                    break;
                }
                case VAL::E_LESS:
                {
                    cout << "<";
                    break;
                }
                case VAL::E_LESSEQ:
                {
                    cout << "<=";
                    break;
                }
                default:
                {
                    cout << " - Error, = constraint should have been preprocessed into a <= and a >=\n";
                    exit(1);
                }
            }
            cout << " " << threshold << ")" << endl;                    
        }
        

        
        static vector<pair<int,double> > durConstraint(3);
        
        // first, we construct a constraint that enforces the bound on ?duration if switchVar = 1
        
        durConstraint[0].first = actEndAt;
        durConstraint[0].second = 1.0;
        
        durConstraint[1].first = actStartAt;
        durConstraint[1].second = -1.0;
        
        durConstraint[2].first = switchVar;
        
        switch (op) {
            case VAL::E_GREATER:
            {
                durConstraint[2].second = -(threshold + EPSILON);
                lp->addRow(durConstraint, 0.0, LPinfinity);
                break;
            }
            case VAL::E_GREATEQ:
            {
                durConstraint[2].second = -threshold;
                lp->addRow(durConstraint, 0.0, LPinfinity);
                break;
            }
            case VAL::E_LESS:
            {
                durConstraint[2].second = (N - (threshold - EPSILON));
                lp->addRow(durConstraint, -LPinfinity, N);
                break;
            }
            case VAL::E_LESSEQ:
            {
                durConstraint[2].second = (N - threshold);
                lp->addRow(durConstraint, -LPinfinity, N);
                break;
            }
            default:
            {
                std::cerr << "Internal error: = precondition should have been preprocessed into a >=, <= pair\n";
                exit(1);
            }
            
        }
        if (nameLPElements) {
            ostringstream s;
            s << "sw" << switchVar << "impldur";
            string cname = s.str();
            lp->setRowName(lp->getNumRows() - 1, cname);
        }
        
        // next, we construct a constraint that means switchVar = 1 if the bound on ?duration is met
        
        switch (op) {
            case VAL::E_GREATER:
            case VAL::E_GREATEQ:
            {
                
                durConstraint[0].first = actEndAt;
                durConstraint[0].second = -1.0;
                
                durConstraint[1].first = actStartAt;
                durConstraint[1].second = 1.0;
                
                durConstraint[2].first = switchVar;
                durConstraint[2].second = N;
                
                if (op == VAL::E_GREATER) {
                    lp->addRow(durConstraint, -threshold, LPinfinity);
                } else {
                    lp->addRow(durConstraint, -(threshold - EPSILON), LPinfinity);
                }
                
                break;
            }
            
            case VAL::E_LESS:
            case VAL::E_LESSEQ:
            {
                durConstraint[0].first = actEndAt;
                durConstraint[0].second = 1.0;
                
                durConstraint[1].first = actStartAt;
                durConstraint[1].second = -1.0;
                
                durConstraint[2].first = switchVar;
                durConstraint[2].second = N;
                
                if (op == VAL::E_LESS) {
                    lp->addRow(durConstraint, threshold, LPinfinity);
                } else {
                    lp->addRow(durConstraint, threshold + EPSILON, LPinfinity);
                }
                
                break;
            }
            default:
            {
                std::cerr << "Internal error: = precondition should have been preprocessed into a >=, <= pair\n";
                exit(1);
            }
            
            
        }
        
        if (nameLPElements) {
            ostringstream s;
            s << "durimplsv" << switchVar;
            string cname = s.str();
            lp->setRowName(lp->getNumRows() - 1, cname);
        }
        
        conditionVars.push_back(switchVar);
    }
    
    return true;
}


bool LPScheduler::addAnyTimeWindowConstraints(const list<pair<Literal*, VAL::time_spec > > & propositionalConditions,
                                              const int & actStartAt, const int & actEndAt, list<int> & conditionVars)
{
    
    static const vector<pair<int,double> > emptyEntries(0);
    static const bool debug = (Globals::globalVerbosity & 32);    
    
    list<pair<Literal*, VAL::time_spec > >::const_iterator condItr = propositionalConditions.begin();
    const list<pair<Literal*, VAL::time_spec > >::const_iterator condEnd = propositionalConditions.end();
    
    for (int cep = 0; condItr != condEnd; ++condItr, ++cep) {
        const Literal* const currLit = condItr->first;
    
        const list<pair<double,double> > * windows = TemporalAnalysis::factIsVisibleInWindows(currLit);
        
        if (!windows) {
            if (debug) {
                cout << *currLit << " does not form TIL windows\n";
            }
            return false;
        }

        if (debug) {
            cout << *currLit << " forms TIL windows\n";
        }            
        
        vector<int> boundVars(2,-1);
        
        if (condItr->second != VAL::E_AT_END) {
            if (debug) {
                cout << lp->getColName(actStartAt) << " needs to fall within a window\n";
            }            
            boundVars[0] = actStartAt;
        }        
        if (condItr->second != VAL::E_AT_START) {
            if (debug) {
                cout << lp->getColName(actEndAt) << " needs to fall within a window\n";
            }                        
            boundVars[1] = actEndAt;
        }
        
        list<pair<double,double> >::const_iterator wItr = windows->begin();
        const list<pair<double,double> >::const_iterator wEnd = windows->end();
        
        list<int> windowSwitches;
        
        for (; wItr != wEnd; ++wItr) {
            int startAndEndVar = -1;
            for (int vi = 0; vi < 2; ++vi) {            
                if (boundVars[vi] == -1) continue;
                
                lp->addCol(emptyEntries, 0, 1, MILPSolver::C_BOOL);
                const int switchAB = lp->getNumCols() - 1;
                
                if (nameLPElements) {
                    ostringstream s;
                    s << boundVars[vi]<< "in" << wItr->first << "to";
                    if (wItr->second == DBL_MAX) {
                        s << "inf";
                    } else {
                        s << wItr->second;
                    }
                    string cname = s.str();
                    lp->setColName(switchAB, cname);
                }
                
                static vector<pair<int,double> > binaryConstraint(2);
                
                if (wItr->first == 0.0) {
                    
                    // if switchAB is set to 1, upper bound on boundVars[vi] is wItr->second
                    binaryConstraint[0].second = 1.0;
                    binaryConstraint[0].first = boundVars[vi];                    
                    binaryConstraint[1].second = (N - wItr->second - SAFE);
                    binaryConstraint[1].first = switchAB;                    
                    lp->addRow(binaryConstraint, 0.0, N);
                    
                    // if boundVars[i] < wItr->second, then switchAB must be = 1
                    binaryConstraint[0].second = 1.0;
                    binaryConstraint[0].first = boundVars[vi];                    
                    binaryConstraint[1].second = N;
                    binaryConstraint[1].first = switchAB;                    
                    lp->addRow(binaryConstraint, wItr->second, LPinfinity);
                    
                } else if (wItr->second == DBL_MAX) {

                    // if switchAB is set to 1, lower bound on boundVars[vi] is wItr->first
                    binaryConstraint[0].second = 1;
                    binaryConstraint[0].first = boundVars[vi];                    
                    binaryConstraint[1].second = -(wItr->first + SAFE);
                    binaryConstraint[1].first = switchAB;                    
                    lp->addRow(binaryConstraint, 0.0, LPinfinity);
                    
                    // if boundVars[i] > wItr->first, then switchAB must be = 1
                    binaryConstraint[0].second = -1;
                    binaryConstraint[0].first = boundVars[vi];                    
                    binaryConstraint[1].second = N;
                    binaryConstraint[1].first = switchAB;                    
                    lp->addRow(binaryConstraint, -wItr->first, LPinfinity);
                    
                } else {
                    
                    lp->addCol(emptyEntries, 0, 1, MILPSolver::C_BOOL);
                    const int lb = lp->getNumCols() - 1;
                    
                    if (nameLPElements) {
                        ostringstream s;
                        s << boundVars[vi]<< "bef" << wItr->second;
                        string cname(s.str());
                        lp->setColName(lb, cname);
                    }
                    
                    // if lb is set to 1, upper bound on boundVars[vi] is wItr->second
                    binaryConstraint[0].second = 1;
                    binaryConstraint[0].first = boundVars[vi];                    
                    binaryConstraint[1].second = (N - wItr->second - SAFE);
                    binaryConstraint[1].first = lb;                    
                    lp->addRow(binaryConstraint, 0.0, N);
                    
                    // if boundVars[i] < wItr->second, then lb must be = 1
                    binaryConstraint[0].second = 1;
                    binaryConstraint[0].first = boundVars[vi];                    
                    binaryConstraint[1].second = N;
                    binaryConstraint[1].first = lb;                    
                    lp->addRow(binaryConstraint, wItr->second, LPinfinity);

                    
                    lp->addCol(emptyEntries, 0, 1, MILPSolver::C_BOOL);
                    const int ga = lp->getNumCols() - 1;

                    if (nameLPElements) {
                        ostringstream s;
                        s << boundVars[vi]<< "aft" << wItr->first;
                        string cname(s.str());
                        lp->setColName(ga, cname);
                    }
                    
                    // if ga is set to 1, lower bound on boundVars[vi] is wItr->first
                    binaryConstraint[0].second = 1;
                    binaryConstraint[0].first = boundVars[vi];                    
                    binaryConstraint[1].second = -(wItr->first + SAFE);
                    binaryConstraint[1].first = ga;                    
                    lp->addRow(binaryConstraint, 0.0, LPinfinity);
                    
                    // if boundVars[i] > wItr->first, then ga must be = 1
                    binaryConstraint[0].second = -1;
                    binaryConstraint[0].first = boundVars[vi];                    
                    binaryConstraint[1].second = N;
                    binaryConstraint[1].first = ga;                    
                    lp->addRow(binaryConstraint, -wItr->first, LPinfinity);
                    
                    // if switchAB = 1, lb = 1
                    binaryConstraint[0].second = -1;
                    binaryConstraint[0].first = switchAB;                    
                    binaryConstraint[1].second = 1;
                    binaryConstraint[1].first = lb;                    
                    lp->addRow(binaryConstraint, 0, LPinfinity);

                    // if switchAB = 1, ga = 1
                    binaryConstraint[0].second = -1;
                    binaryConstraint[0].first = switchAB;                    
                    binaryConstraint[1].second = 1;
                    binaryConstraint[1].first = ga;                    
                    lp->addRow(binaryConstraint, 0, LPinfinity);
                                      
                    static vector<pair<int,double> > bothForceAB(3);
                    // if ga =1 and lb = 1, switchAB = 1
                    bothForceAB[0].second = 1;
                    bothForceAB[0].first = switchAB;
                    bothForceAB[1].second = -1;
                    bothForceAB[1].first = ga;
                    bothForceAB[2].second = -1;
                    bothForceAB[2].first = lb;
                    lp->addRow(bothForceAB, -1, LPinfinity);                    
                }
                
                if (startAndEndVar == -1) {
                    startAndEndVar = switchAB;
                } else {
                    const int switchAB2 = startAndEndVar;
                    
                    lp->addCol(emptyEntries, 0, 1, MILPSolver::C_BOOL);
                    const int switchABBoth = lp->getNumCols() - 1;
                    
                    static vector<pair<int,double> > bothForceOverall(3);
                    // If both the start and the end are in, then over all is in
                    bothForceOverall[0].second = 1;
                    bothForceOverall[0].first = switchABBoth;
                    bothForceOverall[1].second = -1;
                    bothForceOverall[1].first = switchAB;
                    bothForceOverall[2].second = -1;
                    bothForceOverall[2].first = switchAB2;
                    lp->addRow(bothForceOverall, -1, LPinfinity);                    
                    
                    static vector<pair<int,double> > binaryConstraint(2);
                    
                    // if overall is in, then the end must be in
                    binaryConstraint[0].second = 1;
                    binaryConstraint[0].first = switchAB;
                    binaryConstraint[1].second = -1;
                    binaryConstraint[1].first = switchABBoth;
                    lp->addRow(binaryConstraint, 0.0, LPinfinity);

                    // if overall is in, then the start must be in
                    binaryConstraint[0].second = 1;
                    binaryConstraint[0].first = switchAB2;
                    binaryConstraint[1].second = -1;
                    binaryConstraint[1].first = switchABBoth;
                    lp->addRow(binaryConstraint, 0.0, LPinfinity);
                    
                }
            }
            
            windowSwitches.push_back(startAndEndVar);
        }
        
        const int wCount = windowSwitches.size();
        if (wCount == 1) {
            conditionVars.push_back(windowSwitches.front());
        } else {
            lp->addCol(emptyEntries, 0, 1, MILPSolver::C_BOOL);
            const int switchOverall = lp->getNumCols() - 1;
            
            if (nameLPElements) {
                ostringstream s;
                s << actStartAt << "c" << cep << "met";
                string cname(s.str());
                lp->setColName(switchOverall, cname);
                
            }
            conditionVars.push_back(switchOverall);
            
            {
                static vector<pair<int,double> > binaryConstraint(2);
                
                list<int>::const_iterator swItr = windowSwitches.begin();
                const list<int>::const_iterator swEnd = windowSwitches.end();
                
                for (; swItr != swEnd; ++swItr) {
                    // if one window's switch is true, the overall switch must be true
                    binaryConstraint[0].second = 1;
                    binaryConstraint[0].first = switchOverall;
                    binaryConstraint[1].second = -1;
                    binaryConstraint[1].first = *swItr;
                    lp->addRow(binaryConstraint, 0.0, LPinfinity);
                }
            }
            
            vector<pair<int,double> > entries(wCount + 1);
            
            {
                list<int>::const_iterator swItr = windowSwitches.begin();
                const list<int>::const_iterator swEnd = windowSwitches.end();
                
                for (int vi = 0; swItr != swEnd; ++swItr, ++vi) {
                    entries[vi].first = *swItr;
                    entries[vi].second = 1.0;
                }
            }
            entries[wCount].first = switchOverall;
            entries[wCount].second = -1.0;
            
            // if the overall switch is 1, then at least one of the window
            // switches must also be 1
            lp->addRow(entries, 0.0, LPinfinity);

        }
    }
    
    return true;
}


bool LPScheduler::scheduleToMetric()
{   
    static const vector<pair<int,double> > emptyEntries(0);
    
    int variableForRecentStep = -1;
    if (previousObjectiveVar != -1) {
        variableForRecentStep = previousObjectiveVar;
        lp->setObjCoeff(previousObjectiveVar, 0.0);
        previousObjectiveVar = -1;
    }

    // First, make sure that in scheduling to the metric, we don't break the goals
    
    list<const Constraint*>::iterator gItr = goalConstraints.begin();
    const list<const Constraint*>::iterator gEnd = goalConstraints.end();
    
    for (; gItr != gEnd; ++gItr) {
        const int cSize = (*gItr)->weights.size();
        
        vector<pair<int,double> > entries(cSize);
        
        for (int s = 0 ; s < cSize; ++s) {
            entries[s].second = (*gItr)->weights[s];
            if (assertions) assert(entries[s].second != 0.0);
            entries[s].first = finalNumericVars[(*gItr)->variables[s]].lastEffectValueVariable;
        }
        lp->addRow(entries, (*gItr)->lower, (*gItr)->upper);
    }
    
    // ... and make a term for makespan, in case it appears in the objective
    // (or, after that, to find a good makespan within the best-quality solutions)
    
    
    lp->addCol(emptyEntries, makespanVarMinimum, LPinfinity, MILPSolver::C_REAL);

    const int variableForMakespan = lp->getNumCols() - 1;

    {
        list<int>::iterator comesAfter = endsOfThreads.begin();
        const list<int>::iterator comesAfterEnd = endsOfThreads.end();

        for (; comesAfter != comesAfterEnd; ++comesAfter) {
            static vector<pair<int,double> > entries(2);
            
            entries[0].second = 1.0;
            entries[1].second = -1.0;

            entries[0].first = variableForMakespan;
            entries[1].first = *comesAfter;

            lp->addRow(entries, 0.0, LPinfinity);

        }
    }

    
    const RPGBuilder::Metric * const metric = RPGBuilder::getMetric();
    
    MILPSolver::Objective newObjective(!metric->minimise);
    
    const int termCount = metric->variables.size();
    
    /* For each metric variable, specifies its weight in the objective function */
    map<int, double> mapVariableToObjectiveWeight;
    
    /* For each metric term, the LP column containing its value. */
    vector<int> columnForTerm(termCount);
    
    if (termCount == 1 && metric->variables.front() < 0) {
        cout << "; Warning: metric is just to optimise makespan, so post-hoc optimisation is redundant unless being used as a partial-order lifter\n";
    }
    
    list<double>::const_iterator wItr = metric->weights.begin();
    list<int>::const_iterator vItr = metric->variables.begin();
    
    for (int t = 0; t < termCount; ++t, ++vItr, ++wItr) {
        const int currVar = *vItr;

        mapVariableToObjectiveWeight.insert(make_pair(currVar, *wItr));
        
        if (currVar < 0) {
            columnForTerm[t] = variableForMakespan;
            if (variableForMakespan != -1) {
                newObjective.getTerm(variableForMakespan).linearCoefficient = *wItr;
            }
            continue;
        }
        
        FluentTracking & varInfo = finalNumericVars[currVar];
        
        if (varInfo.statusOfThisFluent == FluentTracking::FS_IGNORE) continue;
        
        if (varInfo.statusOfThisFluent == FluentTracking::FS_NORMAL) {
            if (lpDebug & 1) {
                cout << *(RPGBuilder::getPNE(currVar)) << " is a normal variable, adding " << *wItr << " times it to the objective\n";
            }
            columnForTerm[t] = varInfo.lastEffectTimestampVariable;

            if (varInfo.lastEffectTimestampVariable != -1) {
                newObjective.getTerm(varInfo.lastEffectTimestampVariable).linearCoefficient = *wItr;
            }                
            
        } else {

            if (lpDebug & 1) {
                cout << *(RPGBuilder::getPNE(currVar)) << " is an order-independent metric variable, adding terms to objective:\n\t";
            }
            
            map<int,double>::const_iterator termItr = varInfo.orderIndependentValueTerms.begin();
            const map<int,double>::const_iterator termEnd = varInfo.orderIndependentValueTerms.end();
            
            for (bool plus=false; termItr != termEnd; ++termItr) {
                newObjective.getTerm(termItr->first).linearCoefficient += (*wItr * termItr->second);
                if (lpDebug & 1) {
                    if (plus) {
                        cout << " + ";
                    }
                    cout << (*wItr * termItr->second) << " * " << lp->getColName(termItr->first);
                    plus = true;
                }
            }
            if (lpDebug & 1) {
                cout << endl;
            }
                
            
        }
    }

    
    const int actCount = planAsAVector.size();
    
    for (int act = 0; act < actCount; ++act) {
        FFEvent * const currEvent = planAsAVector[act];        
        if (currEvent->time_spec == VAL::E_AT) continue;
        if (currEvent->time_spec == VAL::E_AT_END) continue;
        
        const list<RPGBuilder::ConditionalEffect> & condEffs = RPGBuilder::getActionsToConditionalEffects()[currEvent->action->getID()];
        if (condEffs.empty()) continue;
        
        const int actStartAt = timestampVars[act];
        const int actEndAt = (RPGBuilder::getRPGDEs(currEvent->action->getID()).empty() ? -1 : timestampVars[currEvent->pairWithStep]);
        
        list<RPGBuilder::ConditionalEffect>::const_iterator ceItr = condEffs.begin();
        const list<RPGBuilder::ConditionalEffect>::const_iterator ceEnd = condEffs.end();
        
        for (int cep = 0; ceItr != ceEnd; ++ceItr, ++cep) {
            list<int> conditionVars;
            
            
            const bool success = addAnyTimeWindowConstraints(ceItr->getPropositionalConditions(), actStartAt, actEndAt, conditionVars);

            if (!success) continue;
            
            const bool success2 = addAnyNumericConstraints(ceItr->getNumericPreconditions(), actStartAt, actEndAt, conditionVars);
            
            if (!success2) continue;
            
            int colRepresentingAllConstraintsSatisfied = -1;
            
            const int cvCount = conditionVars.size();
            
            if (cvCount == 1) {                
                colRepresentingAllConstraintsSatisfied = conditionVars.front();
            } else if (cvCount > 1) {
                lp->addCol(emptyEntries, 0, 1, MILPSolver::C_BOOL);            
                colRepresentingAllConstraintsSatisfied = lp->getNumCols() - 1;

                if (nameLPElements) {
                    ostringstream s;
                    s << "s" << act << "ce" << cep << "satisfied";
                    string cname(s.str());
                    lp->setColName(colRepresentingAllConstraintsSatisfied, cname);
                }
                
                static vector<pair<int,double> > binaryConstraint(2);
                {
                    list<int>::const_iterator cvItr = conditionVars.begin();
                    const list<int>::const_iterator cvEnd = conditionVars.end();
                    
                    for (; cvItr != cvEnd; ++cvItr) {
                        // if all constraints are considered satisfied, each must be
                        binaryConstraint[0].second = 1;
                        binaryConstraint[0].first  = *cvItr;
                        binaryConstraint[1].second = -1;
                        binaryConstraint[1].first  = colRepresentingAllConstraintsSatisfied;
                        lp->addRow(binaryConstraint, 0.0, LPinfinity);
                    }
                }
                
                vector<pair<int,double> > entries(cvCount + 1);
                                                
                {
                    list<int>::const_iterator cvItr = conditionVars.begin();
                    const list<int>::const_iterator cvEnd = conditionVars.end();
 
                    for (int cvi = 0; cvItr != cvEnd; ++cvItr, ++cvi) {
                        entries[cvi].second = -1;
                        entries[cvi].first = *cvItr;
                    }
                                                
                }

                entries[cvCount].first = colRepresentingAllConstraintsSatisfied;
                entries[cvCount].second = 1.0;
                
                // if all the condition switches are set to 1, the overall switch must be also
            
                lp->addRow(entries, 1-cvCount, LPinfinity);
                                                  
            }
            
            // now, using the variable 'colRepresentingAllConstraintsSatisfied' we need to
            // add the conditional effects
            
             const list<pair<int, VAL::time_spec> > & effs = ceItr->getNumericEffects();
            list<pair<int, VAL::time_spec> >::const_iterator effItr = effs.begin();
            const list<pair<int, VAL::time_spec> >::const_iterator effEnd = effs.end();
            
            /* The sum effect on the objective if all conditions are met. */
            
            MILPSolver::Objective::Coefficient & overallObjectiveWeightOfEffect = newObjective.getTerm(colRepresentingAllConstraintsSatisfied);
            
            for (; effItr != effEnd; ++effItr) {
                const RPGBuilder::RPGNumericEffect & currEff = RPGBuilder::getNumericEff()[effItr->first];
                
                map<int,double>::const_iterator wItr = mapVariableToObjectiveWeight.find(currEff.fluentIndex);
                
                if (wItr == mapVariableToObjectiveWeight.end()) {
                    // has no weight (or a 0 weight) in the :metric
                    continue;
                }
                
                if (currEff.isAssignment) {
                    cout << "Warning: ignoring conditional metric effect " << currEff << ", as for now only increase/decrease metric conditional effects are supported\n";
                    continue;
                }
                
                
                {
                    int s = 0;
                    for (; s < currEff.size; ++s) {
                        if (currEff.variables[s] != -3 && currEff.variables[s] != -19) {
                            break;
                        }
                    }
                    if (s != currEff.size) {
                        cout << "Warning: ignoring conditional metric effect " << currEff << ", as for now the conditional effects can only be in terms of constants or ?duration\n";
                        continue;
                    }
                }
                                                
                overallObjectiveWeightOfEffect.linearCoefficient += wItr->second * currEff.constant;
                
                for (int s = 0; s < currEff.size; ++s) {
                    if (currEff.variables[s] == -3) {
                        overallObjectiveWeightOfEffect.nonLinearCoefficients.insert(make_pair(actEndAt, 0.0)).first->second += (currEff.weights[s] * wItr->second);
                        overallObjectiveWeightOfEffect.nonLinearCoefficients.insert(make_pair(actStartAt, 0.0)).first->second -= (currEff.weights[s] * wItr->second);
                    } else {
                        assert(currEff.variables[s] == -19);
                        overallObjectiveWeightOfEffect.nonLinearCoefficients.insert(make_pair(actEndAt, 0.0)).first->second -= (currEff.weights[s] * wItr->second);
                        overallObjectiveWeightOfEffect.nonLinearCoefficients.insert(make_pair(actStartAt, 0.0)).first->second += (currEff.weights[s] * wItr->second);
                    }
                }
                
                
            }
     
            if (colRepresentingAllConstraintsSatisfied != -1) {
                if (Globals::globalVerbosity & 32) {
                    cout << "Adding term to objective: " << overallObjectiveWeightOfEffect.linearCoefficient << "*" << lp->getColName(colRepresentingAllConstraintsSatisfied) << endl;
                }
                if (!overallObjectiveWeightOfEffect.nonLinearCoefficients.empty()) {
                    if (Globals::globalVerbosity & 32) {
                        cout << "Adding quadratic terms to objective: " << lp->getColName(colRepresentingAllConstraintsSatisfied) << " * (";
                        map<int,double>::const_iterator qtItr = overallObjectiveWeightOfEffect.nonLinearCoefficients.begin();
                        const map<int,double>::const_iterator qtEnd = overallObjectiveWeightOfEffect.nonLinearCoefficients.end();
                        
                        for (bool plus = false; qtItr != qtEnd; ++qtItr, plus = true) {
                            if (plus) {
                                cout << " + ";
                            }
                            if (qtItr->second != 1.0) {
                                cout << qtItr->second << " * ";
                            }
                            cout << lp->getColName(qtItr->first);
                        }
                        cout << ")\n";
                    }
                }
            }
        }
    }
    
    if (Globals::globalVerbosity & 32) {
        MILPSolver::debug = true;
    }
    
    lp->setQuadraticObjective(newObjective);
    if (!lp->quadraticPreSolve()) {
        if (Globals::globalVerbosity & 32) {
            cout << "Could not solve problem with objective set to problem metric\n";
        }
        return false;
    }
    
    lp->setMaximiseObjective(false);
    lp->setObjCoeff(variableForMakespan, 1);

    if (Globals::globalVerbosity & 32) {
        cout << "Set objective to minimise makespan variable " << lp->getColName(variableForMakespan) << endl;
        lp->writeLp("final.lp");
    }
    
    return true;
    // all done - the actual (final) solving is done back in the calling method (the constructor)
    
}

void LPScheduler::pushTimestampToMin()
{
    if (lpDebug & 1) {
        cout << COLOUR_light_red << "Min timestamp of new step is now " << timestampToUpdate->lpTimestamp << ", rather than " << timestampToUpdate->lpMinTimestamp << COLOUR_default << endl;
    }                                                                
    
    if (cd) {
        if (!cd->updateLPMinTimestamp(timestampToUpdate->lpMinTimestamp, timestampToUpdateStep)) {
            std::cerr << "Internal error: the solution found by the LP should not violate the temporal constraints used by BF\n";
            exit(1);
        }
        if (!Globals::profileScheduling) {
            cd->distsToLPMinStamps();
        }
        
        lp->setColLower(timestampToUpdateVar, timestampToUpdate->lpMinTimestamp);
        
        if (timestampToUpdatePartner) {
            lp->setColLower(timestampToUpdatePartnerVar, timestampToUpdatePartner->lpMinTimestamp);
        }
        
    } else {
        timestampToUpdate->lpMinTimestamp = timestampToUpdate->lpTimestamp;
        lp->setColLower(timestampToUpdateVar, timestampToUpdate->lpMinTimestamp);
        
        if (timestampToUpdatePartner) {
            if (timestampToUpdate->time_spec == VAL::E_AT_END) {
                const double newMin = timestampToUpdate->lpMinTimestamp - timestampToUpdate->maxDuration;
                double & oldMin = timestampToUpdatePartner->lpMinTimestamp;
                if (newMin > oldMin) {
                    if (lpDebug & 1) {
                        cout << COLOUR_light_red << "Min timestamp of corresponding start is now " << newMin << " rather than " << oldMin << COLOUR_default << endl;
                    }                            
                    
                    oldMin = newMin;
                    lp->setColLower(timestampToUpdatePartnerVar, newMin);
                }
            } else {
                const double newMin = timestampToUpdate->lpMinTimestamp + timestampToUpdate->minDuration;
                double & oldMin = timestampToUpdatePartner->lpMinTimestamp;
                if (newMin > oldMin) {
                    if (lpDebug & 1) {
                        cout << COLOUR_light_red << "Min timestamp of corresponding end is now " << newMin << " rather than " << oldMin << COLOUR_default << endl;
                    }                            
                    
                    oldMin = newMin;
                    lp->setColLower(timestampToUpdatePartnerVar, newMin);
                }
            }
        }
    }            
}

bool LPScheduler::addRelaxedPlan(list<FFEvent> & header, list<FFEvent> & now, list<pair<double, list<ActionSegment> > > & relaxedPlan)
{

    if (!lp) return true;

    if (RPGBuilder::modifiedRPG) return true;

    static const bool optimised = true;

    map<int, list<EndDetails> > compulsaryEnds(openDurationConstraints);

    bool recalculate = false;

    {
        list<pair<double, list<ActionSegment> > >::iterator rpItr = relaxedPlan.begin();
        const list<pair<double, list<ActionSegment> > >::iterator rpEnd = relaxedPlan.end();

        for (; rpItr != rpEnd; ++rpItr) {
            double offset = 0.0;
            bool changeFlag = false;
            if (rpItr->first > EPSILON) {
                changeFlag = true;
                offset = rpItr->first - EPSILON;
            }

            list<ActionSegment>::iterator asItr = rpItr->second.begin();
            const list<ActionSegment>::iterator asEnd = rpItr->second.end();

            for (; asItr != asEnd; ++asItr) {
                if (asItr->second != VAL::E_AT_START) {
                    const int actID = asItr->first->getID();
                    const int divMatch = (asItr->second == VAL::E_OVER_ALL ? asItr->divisionID : gradientEffects[actID].size() - 1);

                    map<int, list<EndDetails> >::iterator ceItr = compulsaryEnds.find(actID);

                    RPGBuilder::RPGDuration* currDuration = RPGBuilder::getRPGDEs(actID)[divMatch];

                    if (currDuration->min.empty()) {

                        map<int, list<EndDetails> >::iterator ceItr = compulsaryEnds.find(actID);
                        if (ceItr != compulsaryEnds.end()) {

                            list<EndDetails>::iterator matchItr = ceItr->second.begin();
                            const list<EndDetails>::iterator matchEnd = ceItr->second.end();

                            for (; matchItr != matchEnd; ++matchItr) {
                                if (matchItr->first->divisionsApplied == divMatch) {

                                    int * secItr = &(matchItr->lastToMin);

                                    double tmp = lp->getRowLower(*secItr);

                                    if (lpDebug & 1) cout << "Changed RHS of timestamp for " << *(asItr->first) << " from " << tmp << " to " << tmp + offset << "\n";

                                    tmp += offset;
                                    lp->setRowLower(*secItr, tmp);

                                    recalculate = (recalculate || changeFlag);



                                    ceItr->second.erase(matchItr);
                                    if (ceItr->second.empty()) {
                                        compulsaryEnds.erase(ceItr);
                                    }
                                    break;
                                }
                            }
                        }
                    }

                }
            }
        }
    }

    if (recalculate && timestampToUpdateVar != -1) {
        if (lpDebug & 1) cout << "Recalculating timestamps following relaxed plan\n";


        if (previousObjectiveVar != -1) lp->setObjCoeff(previousObjectiveVar, 0.0);

        lp->setObjCoeff(timestampToUpdateVar, 1.0);

        previousObjectiveVar = timestampToUpdateVar;

        const bool success = lp->solve(false);


        if (success) {

            const double * const partialSoln = lp->getPartialSolution(numVars, numVars + now.size() + header.size());

            bool headerLoop = true;
            list<FFEvent>::iterator endPt = header.end();
            list<FFEvent>::iterator itr = header.begin();

            if (itr == endPt) {
                headerLoop = false;
                endPt = now.end();
                itr = now.begin();
            }

            int v = timestampVars.front();
            int solStepID = 0;

            while (itr != endPt) {

                if (itr->action) {
                    itr->lpTimestamp = partialSoln[v-numVars];

                }
                ++solStepID;
                ++v;
                ++itr;

                if (itr == endPt && headerLoop) {
                    endPt = now.end();
                    itr = now.begin();
                    headerLoop = false;
                }
            }

            if (optimised) {
                pushTimestampToMin();
            }
        }

    } else {
        if (lpDebug & 1) cout << "No need to recalculate timestamps following relaxed plan\n";
    }

    return solved;

}

ParentData * LPScheduler::prime(list<FFEvent> & header, const TemporalConstraints * const cons, list<StartEvent> & open, const bool includeMetric)
{

    if (!hybridBFLP) return 0;
    const bool primeDebug = (Globals::globalVerbosity & 4096);

    const int qSize = header.size() + 3;

    ParentData * const toReturn = new ParentData(qSize, &header, 0);

    bool needLP = false;

    list<FFEvent>::iterator hItr = header.begin();
    const list<FFEvent>::iterator hEnd = header.end();

    map<int, pair<double, double> > rememberDurs;

    int i = 0;

    for (; hItr != hEnd; ++hItr, ++i) {
        double m = hItr->lpMinTimestamp;
        if (m != 0.0) m = -m;
        toReturn->setRawDistToFromZero(i, m, hItr->lpMaxTimestamp);

        if (hItr->action) {
            if (RPGBuilder::getRPGDEs(hItr->action->getID()).empty()) {
                toReturn->setNonTemporal(i);
            } else {
                if (hItr->pairWithStep == -1) {
                    rememberDurs[i] = make_pair(hItr->minDuration, hItr->maxDuration);
                } else {
                    toReturn->setPairWith(i, hItr->pairWithStep);
                }
            }
        } else {
            assert(hItr->time_spec == VAL::E_AT);
            toReturn->setTIL(i);
        }
        instantiatedOp * const act = hItr->action;
        if (act) {
            if (hItr->time_spec == VAL::E_AT_START) {
                needLP = needLP || (!isBoring(act->getID(), 0, includeMetric));
            } else {
                needLP = needLP || (!isBoring(act->getID(), 1, includeMetric));
            }
        }
    }

    toReturn->setWhetherNeedsLP(needLP);


//    const double minAtLeast = (i > 0 ? toReturn->getDistToZero()[i] - 0.001 : 0.0);


    if (primeDebug) {
        cout << "Parent nodes 0 to " << i - 1 << ": events [";
        
        for (int p = 0; p < i; ++p) {
            if (p) cout << ",";
            cout << -toReturn->getDistToZero()[p];
        }
        cout << "]\n";
        cout << "Parent node " << i << ": gaps for new actions\n";
    }

    // leave a gap in case a start event (or TIL) needs to go in later
    toReturn->startGapIsStep(i);
    ++i;

    // leave a gap in case an end event needs to go in later
    toReturn->endGapIsStep(i);

    {
        list<FFEvent>::iterator hItr = header.begin();
        const list<FFEvent>::iterator hEnd = header.end();

        for (i = 0; hItr != hEnd; ++hItr, ++i) {
            const map<int, bool> * stepCons = cons->stepsBefore(i);
            if (!stepCons) continue;

            IncomingAndOutgoing & forThisStep = toReturn->makeEdgeListFor(i);

            //toReturn->makeEdgeListFor(i).initialisePredecessors(*stepCons);

            int exclude = -1;

            if (hItr->time_spec == VAL::E_AT_END) {
                exclude = hItr->pairWithStep;
            }

            map<int, bool>::const_iterator ntfItr = stepCons->begin();
            const map<int, bool>::const_iterator ntfEnd = stepCons->end();

            for (; ntfItr != ntfEnd; ++ntfItr) {
                if (ntfItr->first == exclude) continue;

                if (primeDebug) cout << "\t\tStep " << i << " needs to start after step " << ntfItr->first << "\n";
                toReturn->makeEdgeListFor(ntfItr->first).addFollower(i, ntfItr->second);
                forThisStep.addPredecessor(ntfItr->first, ntfItr->second);
            }
        }
    }

    const int mostRecentStep = MinimalState::getTransformer()->stepThatMustPrecedeUnfinishedActions(cons);

    list<StartEvent>::iterator seqItr = open.begin();
    const list<StartEvent>::iterator seqEnd = open.end();

    for (; seqItr != seqEnd; ++seqItr) {
        if (seqItr->ignore) {
            continue;
        }
        const int startWasAt = seqItr->stepID;
        i = toReturn->getPairWith()[startWasAt];

        IncomingAndOutgoing & myEdgeList = toReturn->makeEdgeListFor(i);

        if (mostRecentStep != -1) {
            if (mostRecentStep != startWasAt) {
                if (primeDebug) {
                    cout << "TO: Insisting that an unfinished action at " << i << " must follow " << mostRecentStep << ", the most recent step\n";
                }
                myEdgeList.addPredecessor(mostRecentStep, true);
                toReturn->makeEdgeListFor(mostRecentStep).addFollower(i, true);
            }
        }

        if (!ignoreABedges) {

            {
                set<int>::iterator ecaItr = seqItr->getEndComesAfter().begin();
                const set<int>::iterator ecaEnd = seqItr->getEndComesAfter().end();

                for (; ecaItr != ecaEnd; ++ecaItr) {
                    const int af = *ecaItr;

                    if (af >= 0) {
                        if (primeDebug) cout << "\t\tEnd of " << seqItr->stepID << " comes after end of " << af << ", therefore...\n";
                        const int afw = toReturn->getPairWith()[af];
                        myEdgeList.addPredecessor(afw, true);
                        toReturn->makeEdgeListFor(afw).addFollower(i, true);
                        if (primeDebug) cout << "\t\t" << afw << " before " << i << "\n";
                    }
                }
            }

            {
                set<int>::iterator ecbItr = seqItr->getEndComesBefore().begin();
                const set<int>::iterator ecbEnd = seqItr->getEndComesBefore().end();

                for (; ecbItr != ecbEnd; ++ecbItr) {
                    const int af = *ecbItr;

                    if (af >= 0) {
                        if (primeDebug) cout << "\t\tEnd of " << seqItr->stepID << " comes before end of " << af << ", therefore...\n";
                        const int afw = toReturn->getPairWith()[af];
                        myEdgeList.addFollower(afw, true);
                        toReturn->makeEdgeListFor(afw).addPredecessor(i, true);
                        if (primeDebug) cout << "\t\t" << i << " before " << afw << "\n";

                    }
                }
            }

        }

    }

    if (checkSanity) toReturn->sanityCheck();

    if (Globals::paranoidScheduling) {
        // should, here, be consistent
        const bool noisy = Globals::globalVerbosity & 4096;
        
        const int mSize = header.size() + 1;

        vector<FFEvent*> eventsWithFakes(mSize - 1, (FFEvent*)0);
        
        vector<vector<double> > matrix(mSize, vector<double>(mSize, DBL_MAX));

        for (int m = 0; m < mSize; ++m) {
            matrix[m][m] = 0.0;
        }


        list<FFEvent>::iterator hItr = header.begin();
        const list<FFEvent>::iterator hEnd = header.end();
        
        for (int m = 1; hItr != hEnd; ++hItr, ++m) {
            matrix[m][0] = 0.0;
            eventsWithFakes[m-1] = &(*hItr);
            if (hItr->time_spec == VAL::E_AT_START) {
                const vector<pair<double, double> > & tsBounds = TemporalAnalysis::getActionTSBounds()[hItr->action->getID()];
                const double startMin = tsBounds[0].first;
                const double startMax = tsBounds[0].second;

                matrix[m][0] = -1 * startMin;
                matrix[0][m] = startMax;
                if (noisy) cout << "Edge from " << m - 1 << " to time zero - " << -startMin << " due to earliest RPG start point of action\n";
            } else if (hItr->time_spec == VAL::E_AT_END) {
                const vector<pair<double, double> > & tsBounds = TemporalAnalysis::getActionTSBounds()[hItr->action->getID()];
                const double endMin = tsBounds[1].first;
                const double endMax = tsBounds[1].second;

                matrix[m][0] = -1 * endMin;
                matrix[0][m] = endMax;
                if (noisy) cout << "Edge from " << m - 1 << " to time zero - " << -endMin << " due to earliest RPG end point of action\n";

            }
        }

        for (int m = 1; m < mSize; ++m) {
            if (!eventsWithFakes[m-1]) continue;

            const int & thisPair = eventsWithFakes[m-1]->pairWithStep;
            if (thisPair >= 0) {
                if (eventsWithFakes[m-1]->time_spec == VAL::E_AT_START) {
                    matrix[m][thisPair + 1] = eventsWithFakes[m-1]->maxDuration;
                    if (noisy) cout << "Edge from " << m - 1 << " to " << thisPair << " - " << eventsWithFakes[m-1]->maxDuration << " due to max duration" << endl;
                } else {
                    matrix[m][thisPair + 1] = -1 * eventsWithFakes[m-1]->minDuration;
                    if (noisy) cout << "Edge from " << m - 1 << " to " << thisPair << " - " << -eventsWithFakes[m-1]->minDuration << " due to min duration" << endl;
                }
            } else if (thisPair == -2) {
                const double tilTime = LPScheduler::getTILTimestamp(eventsWithFakes[m-1]->divisionID);
                matrix[0][m] = tilTime;
                matrix[m][0] = -1 * tilTime;
            }

            if (eventsWithFakes[m-1]->lpMinTimestamp != -1.0) {
                const double backEdge = (eventsWithFakes[m-1]->lpMinTimestamp == 0.0 ? 0.0 : -(eventsWithFakes[m-1]->lpMinTimestamp));
                if (backEdge < matrix[m][0]) {
                    matrix[m][0] = backEdge;
                    if (noisy) cout << "Changing edge from " << m - 1 << " to time zero - " << backEdge << " due to known minimum timestamp for action\n";
                } else {
                    if (noisy) cout << "Not changing edge from " << m - 1 << " to time zero due to known minimum timestamp for action\n";
                }
            } else {
                if (noisy) cout << "Not changing edge from " << m - 1 << " to time zero due to known minimum timestamp for action\n";
            }
            
            if (eventsWithFakes[m-1]->lpMaxTimestamp != -1.0) {
                if (eventsWithFakes[m-1]->lpMaxTimestamp < matrix[0][m]) {
                    matrix[0][m] = eventsWithFakes[m-1]->lpMaxTimestamp;
                    if (noisy) cout << "Changing edge from time zero to " << m - 1 << " to " << matrix[0][m] << " due to known maximum timestamp for action\n";
                } else {
                    if (noisy) cout << "Not changing edge from time zero to " << m - 1 << " due to known maximum timestamp for action\n";
                }
            } else {
                if (noisy) cout << "Not changing edge from time zero to " << m - 1 << " due to known maximum timestamp for action\n";
            }

            map<int, IncomingAndOutgoing>::const_iterator eItr = toReturn->getTemporaryEdges().find(m - 1);
            if (eItr != toReturn->getTemporaryEdges().end()) {
                map<int, bool>::const_iterator pItr = eItr->second.mustPrecedeThis().begin();
                const map<int, bool>::const_iterator pEnd = eItr->second.mustPrecedeThis().end();

                for (; pItr != pEnd; ++pItr) {
                    matrix[m][pItr->first + 1] = (pItr->second ? -0.001 : 0);
                    if (noisy) cout << "Edge from " << m - 1 << " to " << pItr->first << " - " << (pItr->second ? -0.001 : 0) << " due to recorded predecessors" << endl;
                }
            }
        }


        int k, i, j;
        double distIK, distKJ;

        for (k = 0; k < mSize; ++k) {
            for (i = 0; i < mSize; ++i) {
                distIK = matrix[i][k];
                if (distIK == DBL_MAX) continue;
                for (j = 0; j < mSize; ++j) {
                    distKJ = matrix[k][j];

                    if (distKJ != DBL_MAX) {
                        double & distIJ = matrix[i][j];
                        if ((distIK + distKJ) - distIJ < -0.00001) {
                            distIJ = distIK + distKJ;
                        }
                    }
                }
            }
        }

        for (int m = 0; m < mSize; ++m) {
            assert(fabs(matrix[m][m]) < 0.000000001);
        }

        if (noisy) {
            cout << "Floyd gives pre-child-generation timestamps of: [";
            for (int m = 1; m < mSize; ++m) {
                if (!eventsWithFakes[m-1]) continue;
                if (m >= 2) cout << ",";
                cout << -(matrix[m][0]);
            }
            cout << "]\n";
        }
    }

    
    return toReturn;
}


ChildData * ParentData::spawnChildData(list<StartEvent> & seq,
                                       list<FFEvent> & header, list<FFEvent> & newActs,
                                       const bool & includeMetric,
                                       const TemporalConstraints * const cons, const int & stepID)
{

    const bool spawnDebug = (Globals::globalVerbosity & 4096);

    const int newActCount = newActs.size();

    assert(newActCount <= 2);

    if (spawnDebug) {
        cout << "Spawning child data for ";
        if (newActCount) {
            if (newActCount == 1) {
                cout << "1 new step\n";
            } else {
                cout << newActCount << " new steps\n";
            }
        } else {
            cout << "Spawning child data by ending the action at step " << stepID << endl;
        }
    }

    if (spawnDebug) {
        list<FFEvent>::iterator hItr = header.begin();
        const list<FFEvent>::iterator hEnd = header.end();

        list<FFEvent>::iterator ppItr = parentPlan->begin();
        const list<FFEvent>::iterator ppEnd = parentPlan->end();

        for (int i = 0; hItr != hEnd; ++hItr, ++i, ++ppItr) {
            assert(ppItr != ppEnd);
            assert(hItr->action == ppItr->action);
            assert(hItr->time_spec == ppItr->time_spec);
            assert(hItr->time_spec != VAL::E_AT || hItr->divisionID == ppItr->divisionID);
            assert(hItr->time_spec != VAL::E_AT || pairWith[i] == -2);
        }
    }

    const int mustComeBeforeOpenEnds = MinimalState::getTransformer()->stepThatMustPrecedeUnfinishedActions(cons);
        
        
    double TILupbo = MinimalState::getTransformer()->latestTimePointOfActionsStartedHere(nextTIL);

    ChildData * toReturn = 0;

    if (startGap > stepID) { // if we're only triggering an existing action within the plan

        if (spawnDebug) cout << "Is an end action\n";
        assert(newActs.empty());

        // in this case, the end was already assigned an event, so we need to nuke the old one

        list<FFEvent>::iterator findIt = header.begin();
        for (int i = 0; i < stepID; ++i, ++findIt) ;

        FFEvent & s = *findIt;

        toReturn = new ChildData(&Q, distFromZero, distToZero, pairWith, eventsWithFakes, temporaryEdges, needsLP);

        toReturn->updateLPNeed(!LPScheduler::isBoring(s.action->getID(), 1, includeMetric));

        const int startLocation = s.pairWithStep;
        const int endLocation = stepID;

        const double endMax = toReturn->getDistFromZero()[endLocation];
        double endMin = toReturn->getDistToZero()[endLocation];
        if (endMin != 0.0) endMin = -endMin;

        s.lpMinTimestamp = endMin;
        s.lpMaxTimestamp = endMax;

        // fortunately, we left gaps in the event sequence
        vector<FFEvent*> & childEvents = toReturn->getEventsWithFakes();

        int i = 0;

        {
            list<FFEvent>::iterator hItr = header.begin();
            const list<FFEvent>::iterator hEnd = header.end();

            for (; hItr != hEnd; ++hItr, ++i) {
                childEvents[i] = &(*hItr);
            }
        }

        assert(i == startGap);

        //toReturn->pairEventWith(startLocation, endLocation);

        IncomingAndOutgoing & edgesForTheEndAction = toReturn->makeEdgeListFor(endLocation);

        map<int, bool> nowAfter(edgesForTheEndAction.mustFollowThis());

        if (mustComeBeforeOpenEnds != -1) {
            assert(mustComeBeforeOpenEnds == endLocation);

            if (spawnDebug) {
                cout << "This step, " << endLocation<< ", must come before all open ends\n";
            }
            
            list<StartEvent>::iterator seqFillItr = seq.begin();
            const list<StartEvent>::iterator seqFillEnd = seq.end();

            for (; seqFillItr != seqFillEnd; ++seqFillItr) {
                if (seqFillItr->stepID == startLocation) continue;
                const int cbf = toReturn->getPairWith()[seqFillItr->stepID];
                map<int, bool>::const_iterator oldEdge = edgesForTheEndAction.mustPrecedeThis().find(cbf);
                if (oldEdge != edgesForTheEndAction.mustPrecedeThis().end() && oldEdge->second) {
                    if (spawnDebug) {
                        cout << "Previously required this action to end strictly after another that hasn't finished yet - STP inconsistent\n";
                    }
                    delete toReturn;
                    toReturn = 0;
                    return toReturn;
                }

                const pair<map<int, bool>::iterator, bool> insPair = nowAfter.insert(make_pair(cbf, true));
                if (insPair.second || !insPair.first->second) {
                    insPair.first->second = true;
                    if (spawnDebug) cout << "Adding epsilon separation edge from " << endLocation << " to some future end event " << cbf << "\n";
                    toReturn->addNewEdge(BFEdge(endLocation, cbf, 0.001, DBL_MAX));
                }
            }
        }

        {// now see if any new edges are in cons that weren't present before

            const map<int, bool> * eventsBeforeEndAction = cons->stepsBefore(endLocation);


            // first, things that used to come before this - any extras?
            if (eventsBeforeEndAction) {
                map<int, bool>::const_iterator oldCbf = edgesForTheEndAction.mustPrecedeThis().begin();
                const map<int, bool>::const_iterator oldCbfEnd = edgesForTheEndAction.mustPrecedeThis().end();

                map<int, bool>::const_iterator newCbf = eventsBeforeEndAction->begin();
                const map<int, bool>::const_iterator newCbfEnd = eventsBeforeEndAction->end();

                while (oldCbf != oldCbfEnd && newCbf != newCbfEnd) {
                    if (newCbf->first < oldCbf->first) {
                        if (newCbf->first != startLocation) {
                            if (spawnDebug) {
                                if (newCbf->second) {
                                    cout << "New 'before' edge: adding epsilon separation edge from " << newCbf->first << " to the end snap-action that has been applied\n";
                                } else {
                                    cout << "New 'before' edge: adding zero separation edge from " << newCbf->first << " to the end snap-action that has been applied\n";
                                }
                            }
                            toReturn->addNewEdge(BFEdge(newCbf->first, endLocation, (newCbf->second ? 0.001 : 0.0), DBL_MAX));
                        }
                        ++newCbf;
                    } else {
                        assert(newCbf->first == oldCbf->first); // or else edges have disappeared....
                        assert(!oldCbf->second || newCbf->second);
                        if (newCbf->first != startLocation) {
                            if (newCbf->second && !oldCbf->second) {
                                if (spawnDebug) {
                                    cout << "Raised cost from zero to epsilon on the separation edge from " << newCbf->first << " to the end snap-action that has been applied\n";
                                }
                                toReturn->addNewEdge(BFEdge(newCbf->first, endLocation, 0.001, DBL_MAX));
                            } else {
                                if (spawnDebug) {
                                    cout << "Already knew there should be an epsilon separation edge from " << newCbf->first << " to the end snap-action that has been applied\n";
                                }                                
                            }
                        }
                        ++oldCbf;
                        ++newCbf;
                    }
                }

                for (; newCbf != newCbfEnd; ++newCbf) {
                    if (newCbf->first != startLocation) {
                        if (spawnDebug) {
                            if (newCbf->second) {
                                cout << "New 'before' edge: adding epsilon separation edge from " << newCbf->first << " to the end snap-action that has been applied\n";
                            } else {
                                cout << "New 'before' edge: adding zero separation edge from " << newCbf->first << " to the end snap-action that has been applied\n";
                            }
                        }
                        toReturn->addNewEdge(BFEdge(newCbf->first, endLocation, (newCbf->second ? 0.001 : 0.0), DBL_MAX));
                    }
                }
            }


            // then, things this has to come before - necessarily must be ends of actions yet to have been applied
            // but, better check...

            if (Globals::paranoidScheduling) {
                const int consSize = cons->size();
                for (int cs = 0; cs < consSize; ++cs) {
                    if (!cons->stepsBefore(cs)) continue;
                    if (!toReturn->getEventsWithFakes()[cs]) {
                        cout << "Warning: event at step " << cs << " is null\n";
                        continue;
                    }
                    if (!toReturn->getEventsWithFakes()[cs]->getEffects) continue;

                    assert(cons->stepsBefore(cs)->find(endLocation) == cons->stepsBefore(cs)->end());
                }
            }

            {
                list<StartEvent>::iterator seqFillItr = seq.begin();
                const list<StartEvent>::iterator seqFillEnd = seq.end();

                for (; seqFillItr != seqFillEnd; ++seqFillItr) {
                    const int cbf = toReturn->getPairWith()[seqFillItr->stepID];

                    const map<int, bool> * const eventsBeforeFutureAction = cons->stepsBefore(cbf);
                    if (!eventsBeforeFutureAction) {
                        continue;
                    }

                    map<int, bool>::const_iterator toThisItr = eventsBeforeFutureAction->find(endLocation);

                    if (toThisItr != eventsBeforeFutureAction->end()) {
                        const pair<map<int, bool>::iterator, bool> insPair = nowAfter.insert(make_pair(cbf, toThisItr->second));

                        if (insPair.second || (toThisItr->second && !insPair.first->second)) {
                            if (toThisItr->second) {
                                insPair.first->second = true;
                            }
                            if (spawnDebug) {
                                if (toThisItr->second) {
                                    cout << "Adding epsilon separation edge to the end snap-action " << cbf << " that has yet to be applied\n";
                                } else {
                                    cout << "Adding zero separation edge to the end snap-action " << cbf << " that has yet to be applied\n";
                                }
                            }
                            toReturn->addNewEdge(BFEdge(endLocation, cbf, (insPair.first->second ? 0.001 : 0.0), DBL_MAX));
                        }
                    }

                }

            }

        }


        return toReturn;

    }

    assert(newActCount);

    FFEvent & s = newActs.front();

    assert(newActs.front().time_spec == VAL::E_AT_START || newActs.front().time_spec == VAL::E_AT);

    if (s.time_spec == VAL::E_AT_START) {
        const bool nonTemporal = RPGBuilder::getRPGDEs(s.action->getID()).empty();

        if (spawnDebug) {
            if (!nonTemporal) {
                cout << "Is a start action - start goes at " << startGap << ", end goes at " << endGap << "\n";
                cout << "Duration in the range [" << s.minDuration << "," << s.maxDuration << "]";

                if (LPScheduler::isBoring(s.action->getID(), 0, includeMetric)) {
                    cout << ", and start is numerically boring\n";
                } else {
                    cout << ", and start is numerically interesting\n";
                }

            } else {
                if (spawnDebug) cout << "Is an instantaneous action, goes at " << startGap << "\n";
                if (LPScheduler::isBoring(s.action->getID(), 0, includeMetric)) {
                    cout << ", and is numerically boring\n";
                } else {
                    cout << ", and is numerically interesting\n";
                }
            }
        }

        toReturn = new ChildData(&Q, distFromZero, distToZero, pairWith, eventsWithFakes, temporaryEdges, needsLP);

        toReturn->updateLPNeed(!LPScheduler::isBoring(s.action->getID(), 0, includeMetric));

        if (spawnDebug) {
            if (toReturn->doLPSolve()) {
                cout << "We need to solve an LP if the STP is solvable\n";
            } else {
                cout << "We don't need the LP so far\n";
            }
        }

        const int endLocation = endGap;
        const int startLocation = startGap;
        // fortunately, we left gaps in the event sequence
        vector<FFEvent*> & childEvents = toReturn->getEventsWithFakes();


        {
            int i = 0;

            list<FFEvent>::iterator hItr = header.begin();
            const list<FFEvent>::iterator hEnd = header.end();

            for (; hItr != hEnd; ++hItr, ++i) {
                childEvents[i] = &(*hItr);
            }
        }

        childEvents[startLocation] = &s;

        if (!nonTemporal) {

            assert(newActCount == 2);

            FFEvent & s2 = newActs.back();
            childEvents[endLocation] = &s2;

            toReturn->pairEventWith(startLocation, endLocation);

        } else {
            toReturn->setNonTemporal(startGap);
        }

        const vector<pair<double, double> > & tsBounds = TemporalAnalysis::getActionTSBounds()[s.action->getID()];

        double startMin = tsBounds[0].first;
        double endMin = tsBounds[1].first;

        double startMax = tsBounds[0].second;
        double endMax = tsBounds[1].second;


        {
            const map<int, bool> * const preceding = cons->stepsBefore(startLocation);
            if (preceding) {
                map<int, bool>::const_iterator pItr = preceding->begin();
                const map<int, bool>::const_iterator pEnd = preceding->end();

                for (; pItr != pEnd; ++pItr) {
                    const double w = -(toReturn->getDistToZero()[pItr->first] - (pItr->second ? 0.001 : 0.0));
                    if (startMin < w) startMin = w;
                }
            }
        }

        if (startMax > TILupbo) startMax = TILupbo;

        if (nonTemporal) {
            if (spawnDebug) cout << "Edges back to previous steps give earliest start position " << startMin << " vs " << tsBounds[0].first << " from RPG\n";

            toReturn->autoMinima[startLocation] = startMin;

            if (startMax < startMin) {
                delete toReturn;
                return 0;
            }
        } else {

            {
                const map<int, bool> * const preceding = cons->stepsBefore(endLocation);
                if (preceding) {
                    map<int, bool>::const_iterator pItr = preceding->begin();
                    const map<int, bool>::const_iterator pEnd = preceding->end();
                    
                    for (; pItr != pEnd; ++pItr) {
                        const double w = -(toReturn->getDistToZero()[pItr->first] - (pItr->second ? 0.001 : 0.0));
                        if (endMin < w) endMin = w;
                    }
                }
            }                        
            
            {
                const double sEndMin = startMin + s.minDuration;
                if (sEndMin > endMin) endMin = sEndMin;
            }

            {
                const double sStartMin = endMin - s.maxDuration;
                if (sStartMin > startMin) startMin = sStartMin;
            }

            if (startMax != DBL_MAX && s.maxDuration != DBL_MAX) {
                const double sEndMax = startMax + s.maxDuration;
                if (sEndMax < endMax) endMax = sEndMax;
            }

            if (endMax != DBL_MAX) {
                const double sStartMax = endMax - s.minDuration;
                if (sStartMax < startMax) startMax = sStartMax;
            }

            if (spawnDebug) {
                cout << "Edges back to previous steps give earliest positions " << startMin << ";" << endMin << " vs " << tsBounds[0].first << ";" << tsBounds[1].first << " from RPG\n";
            }

            toReturn->autoMinima[startLocation] = startMin;
            toReturn->autoMinima[endLocation] = endMin;

            if (startMax < startMin || endMax < endMin) {
                delete toReturn;
                return 0;
            }
        }

        if (startMax < DBL_MAX) {
            toReturn->setDFZ(startLocation, startMax);
        }

        if (startMin > 0.0) {
            toReturn->setDTZ(startLocation, -startMin);
        }

        if (Globals::paranoidScheduling || Globals::profileScheduling) {
            childEvents[startLocation]->lpMinTimestamp = 0.0;
            childEvents[startLocation]->lpMaxTimestamp = DBL_MAX;
        } else {
            childEvents[startLocation]->lpMinTimestamp = startMin;        
            childEvents[startLocation]->lpMaxTimestamp = startMax;            
        }

        if (spawnDebug) cout << "Initially putting at " << startMin;

        if (!nonTemporal) {
            if (endMax < DBL_MAX) {
                toReturn->setDFZ(endLocation, endMax);
            }


            if (endMin > 0.0) {
                toReturn->setDTZ(endLocation, -endMin);
            }


            if (Globals::paranoidScheduling || Globals::profileScheduling) {
                childEvents[endLocation]->lpMinTimestamp = 0.0;
                childEvents[endLocation]->lpMaxTimestamp = DBL_MAX;
            } else {
                childEvents[endLocation]->lpMinTimestamp = endMin;
                childEvents[endLocation]->lpMaxTimestamp = endMax;
            } 
        
            if (spawnDebug) {
                cout << " ; " << endMin << "\n";
            }

        } else {
            if (spawnDebug) cout << "\n";
        }

        const int mustComeBeforeOpenEnds = MinimalState::getTransformer()->stepThatMustPrecedeUnfinishedActions(cons);

        if (mustComeBeforeOpenEnds != -1) {
            assert(mustComeBeforeOpenEnds == startLocation);

            list<StartEvent>::iterator seqFillItr = seq.begin();
            const list<StartEvent>::iterator seqFillEnd = seq.end();

            for (; seqFillItr != seqFillEnd; ++seqFillItr) {
                if (seqFillItr->stepID == startLocation) continue;
                const int cbf = toReturn->getPairWith()[seqFillItr->stepID];

                // The edge is definitely new, as it's from a start (an event that wasn't in before)

                if (spawnDebug) cout << "Adding total-order epsilon separation edge from " << startLocation << " to some future end event " << cbf << "\n";
                toReturn->addNewEdge(BFEdge(startLocation, cbf, 0.001, DBL_MAX));
            }
        } else {
            list<StartEvent>::iterator seqFillItr = seq.begin();
            const list<StartEvent>::iterator seqFillEnd = seq.end();

            for (; seqFillItr != seqFillEnd; ++seqFillItr) {
                if (seqFillItr->stepID == startLocation) continue;
                const int cbf = toReturn->getPairWith()[seqFillItr->stepID];
                const map<int, bool> * beforeFutureStep = cons->stepsBefore(cbf);
                if (!beforeFutureStep) continue;

                map<int, bool>::const_iterator toThisItr = beforeFutureStep->find(startLocation);
                if (toThisItr != beforeFutureStep->end()) {
                    if (spawnDebug) {
                        if (toThisItr->second) {
                            cout << "Adding epsilon separation edge from " << startLocation << " to some future end event " << cbf << ", due to recorded constraints\n";
                        } else {
                            cout << "Adding zero separation edge from " << startLocation << " to some future end event " << cbf << ", due to recorded constraints\n";
                        }
                    }
                    toReturn->addNewEdge(BFEdge(startLocation, cbf, (toThisItr->second ? 0.001 : 0.0), DBL_MAX));
                }
            }
        }

        {
            const map<int, bool> * beforeNewStep = cons->stepsBefore(startLocation);

            if (beforeNewStep) {
                map<int, bool>::const_iterator ecaItr = beforeNewStep->begin();
                const map<int, bool>::const_iterator ecaEnd = beforeNewStep->end();

                for (; ecaItr != ecaEnd; ++ecaItr) {
                    const int af = ecaItr->first;

                    if (spawnDebug) {
                        if (ecaItr->second) {
                            cout << "Recorded constraints: adding edge to denote that the start of the new action comes at least epsilon after step " << af << endl;
                        } else {
                            cout << "Recorded constraints: adding edge to denote that the start of the new action comes after (or alongside) step " << af << endl;
                        }
                    }

                    toReturn->addNewEdge(BFEdge(af, startLocation, (ecaItr->second ? 0.001 : 0.0), DBL_MAX));
                }
            }
        }
        
        if (!nonTemporal) {
            toReturn->makeEdgeListFor(endLocation);

            {
                const map<int, bool> * beforeNewEndStep = cons->stepsBefore(endLocation);

                if (beforeNewEndStep) {
                    map<int, bool>::const_iterator ecaItr = beforeNewEndStep->begin();
                    const map<int, bool>::const_iterator ecaEnd = beforeNewEndStep->end();

                    for (; ecaItr != ecaEnd; ++ecaItr) {
                        const int af = ecaItr->first;
                        if (af == startLocation) continue;

                        if (spawnDebug) {
                            if (ecaItr->second) {
                                cout << "Due to recorded constraints: adding edge to denote that the end of the new action comes at least epsilon after step " << af << endl;
                            } else {
                                cout << "Due to recorded constraints: adding edge to denote that the end of the new action comes after (or alongside) step " << af << endl;
                            }
                        }

                        toReturn->addNewEdge(BFEdge(af, endLocation, (ecaItr->second ? 0.001 : 0.0), DBL_MAX));
                    }
                } else {
                    if (spawnDebug) {
                        cout << "No steps must come before the end just added\n";
                    }
                }
            }
            
            {

                list<StartEvent>::iterator seqFillItr = seq.begin();
                const list<StartEvent>::iterator seqFillEnd = seq.end();

                for (; seqFillItr != seqFillEnd; ++seqFillItr) {
                    if (seqFillItr->stepID == startLocation) continue;

                    const int cbf = toReturn->getPairWith()[seqFillItr->stepID];
                    const map<int, bool> * beforeFutureStep = cons->stepsBefore(cbf);
                    if (!beforeFutureStep) continue;

                    map<int, bool>::const_iterator toThisItr = beforeFutureStep->find(endLocation);
                    if (toThisItr != beforeFutureStep->end()) {
                        if (spawnDebug) {
                            if (toThisItr->second) {
                                cout << "Constraint to future step: adding epsilon separation edge from " << endLocation << " to some future end event " << cbf << "\n";
                            } else {
                                cout << "Constraint to future step: adding zero separation edge from " << endLocation << " to some future end event " << cbf << "\n";
                            }
                        }
                        toReturn->addNewEdge(BFEdge(endLocation, cbf, (toThisItr->second ? 0.001 : 0.0), DBL_MAX));
                    }

                }
            }

            /*
            StartEvent & ev = seq.back();

            if (!ignoreABedges) {


                {
                    set<int>::iterator ecaItr = ev.getEndComesAfter().begin();
                    const set<int>::iterator ecaEnd = ev.getEndComesAfter().end();

                    for (; ecaItr != ecaEnd; ++ecaItr) {
                        const int af = *ecaItr;

                        if (af >= 0) {
                            if (spawnDebug) cout << "Adding edge to denote that the end of the new action comes after the end of that starting at " << af << ", i.e. after node " << toReturn->getPairWith()[af] << "\n";
                            toReturn->addNewEdge(BFEdge(toReturn->getPairWith()[af], endLocation, 0.001, DBL_MAX));
                        }
                    }
                }

                {
                    set<int>::iterator ecbItr = ev.getEndComesAfterPair().begin();
                    const set<int>::iterator ecbEnd = ev.getEndComesAfterPair().end();

                    for (; ecbItr != ecbEnd; ++ecbItr) {
                        const int af = *ecbItr;

                        if (af >= 0) {
                            if (spawnDebug) cout << "Adding edge to denote that the end of the action comes before the end of that starting at " << af << ", i.e. before node " << toReturn->getPairWith()[af] << ", due to eca pair\n";
                            toReturn->addNewEdge(BFEdge(endLocation, toReturn->getPairWith()[af], 0.001, DBL_MAX));
                        }
                    }
                }


                {
                    set<int>::iterator ecbItr = ev.getEndComesBefore().begin();
                    const set<int>::iterator ecbEnd = ev.getEndComesBefore().end();

                    for (; ecbItr != ecbEnd; ++ecbItr) {
                        const int af = *ecbItr;

                        if (af >= 0) {
                            if (spawnDebug) cout << "Adding edge to denote that the end action comes before the end of that starting at node " << af << ", i.e. node " << toReturn->getPairWith()[af] << "\n";
                            toReturn->addNewEdge(BFEdge(endLocation, toReturn->getPairWith()[af], 0.001, DBL_MAX));
                        }
                    }
                }

                int workingFanIn = ev.fanIn;
                if (workingFanIn) {
                    set<int>::iterator ecaItr = ev.getEndComesBeforePair().begin();
                    const set<int>::iterator ecaEnd = ev.getEndComesBeforePair().end();

                    for (; ecaItr != ecaEnd; ++ecaItr) {
                        const int af = *ecaItr;

                        if (af >= 0) {
                            if (spawnDebug) cout << "Adding edge to denote that the end of the action comes after that which started at node " << *ecaItr << ", i.e. " << toReturn->getPairWith()[*ecaItr] << ", due to it being on the back-end of an end-comes-before\n";
                            toReturn->addNewEdge(BFEdge(toReturn->getPairWith()[*ecaItr], endLocation, 0.001, DBL_MAX));
                        }
                    }
                }
            }*/
        }


    } else {
        if (spawnDebug) cout << "Is a TIL\n";

        assert(newActCount == 1);

        FFEvent & s = newActs.front();

        toReturn = new ChildData(&Q, distFromZero, distToZero, pairWith, eventsWithFakes, temporaryEdges, needsLP);

        vector<FFEvent*> & childEvents = toReturn->getEventsWithFakes();


        int i = 0;

        {
            list<FFEvent>::iterator hItr = header.begin();
            const list<FFEvent>::iterator hEnd = header.end();

            for (; hItr != hEnd; ++hItr, ++i) {
                childEvents[i] = &(*hItr);
            }
        }
        ++i;

        childEvents[startGap] = &s;
        toReturn->setTil(startGap);

        const double tilTime = LPScheduler::getTILTimestamp(s.divisionID);

        s.lpMinTimestamp = tilTime;
        s.lpMaxTimestamp = tilTime;

        toReturn->setDFZ(startGap, tilTime);
        if (tilTime != 0.0) {
            toReturn->setDTZ(startGap, -tilTime);
        } else {
            toReturn->setDTZ(startGap, 0.0);
        }

        const int mustComeBeforeOpenEnds = MinimalState::getTransformer()->stepThatMustPrecedeUnfinishedActions(cons);

        if (mustComeBeforeOpenEnds != -1) {
            assert(mustComeBeforeOpenEnds == startGap);

            list<StartEvent>::iterator seqFillItr = seq.begin();
            const list<StartEvent>::iterator seqFillEnd = seq.end();

            for (; seqFillItr != seqFillEnd; ++seqFillItr) {
                const int cbf = toReturn->getPairWith()[seqFillItr->stepID];
                if (spawnDebug) cout << "Adding edge from time zero to a future step " << cbf << " to set its min ts to " << tilTime + 0.001 << "\n";
                toReturn->addNewEdge(BFEdge(-1, cbf, tilTime + 0.001, distFromZero[cbf]));
                
                if (spawnDebug) cout << "Adding epsilon separation edge from " << startGap << " to some future end event " << cbf << "\n";
                toReturn->addNewEdge(BFEdge(startGap, cbf, 0.001, DBL_MAX));

            }
        } else {
            list<StartEvent>::iterator seqFillItr = seq.begin();
            const list<StartEvent>::iterator seqFillEnd = seq.end();

            for (; seqFillItr != seqFillEnd; ++seqFillItr) {
                const int cbf = toReturn->getPairWith()[seqFillItr->stepID];
                const map<int, bool> * beforeFutureStep = cons->stepsBefore(cbf);
                if (!beforeFutureStep) continue;

                if (beforeFutureStep->find(startGap) != beforeFutureStep->end()) {
                    if (spawnDebug) cout << "Adding edge from time zero to a future step " << cbf << " to set its min ts to " << tilTime + 0.001 << "\n";
                    toReturn->addNewEdge(BFEdge(-1, cbf, tilTime + 0.001, distFromZero[cbf]));
                    
                    if (spawnDebug) cout << "Adding epsilon separation edge from " << startGap << " to some future end event " << cbf << "\n";
                    toReturn->addNewEdge(BFEdge(startGap, cbf, 0.001, DBL_MAX));
                    
                }
            }
        }

        const map<int, bool> * beforeNewStep = cons->stepsBefore(startGap);

        if (beforeNewStep) {
            map<int, bool>::const_iterator ecaItr = beforeNewStep->begin();
            const map<int, bool>::const_iterator ecaEnd = beforeNewStep->end();

            for (; ecaItr != ecaEnd; ++ecaItr) {
                const int af = ecaItr->first;

                if (spawnDebug) cout << "TIL comes after step " << af << endl;

                double w = distToZero[af];
                if (w != 0.0) w = -w;
                if (w > tilTime - 0.001) {
                    if (spawnDebug) cout << "- Found a simple cycle: the earliest step " << af << " could go at was " << w << ", i.e. too late for a TIL at " << tilTime << endl;
                    delete toReturn;
                    return 0;
                }

                if (distFromZero[af] > tilTime - 0.001) {
                    if (spawnDebug) cout << "Adding edge from time zero to a preceding step " << af << " to set its max ts to " << tilTime - 0.001 << "\n";
                    toReturn->addNewEdge(BFEdge(-1, af, w, tilTime - 0.001));
                    if (spawnDebug) cout << "Adding epsilon separation edge from step " << af << " to TIL\n";
                    toReturn->addNewEdge(BFEdge(af, startGap, 0.001, DBL_MAX));                                        
                } else {
                    if (spawnDebug) cout << "Latest step " << af << " could come at was " << distFromZero[af] << ", so it is unaffected\n";
                    toReturn->makeEdgeListFor(af).addFollower(startGap, true);
                    toReturn->makeEdgeListFor(startGap).addPredecessor(af, true);
                }
                
                /* else {
                    childEvents[af]->lpMaxTimestamp = tilTime - 0.001;
                }*/

            }
        }

    }

    return toReturn;
};

bool ChildData::propagateNewEdges()
{

    static const bool floydDoubleCheck = Globals::paranoidScheduling;
    static const bool noisy = Globals::globalVerbosity & 4096;

    if (Globals::globalVerbosity & 4096) {
        cout << "Propagating " << newEdges.size() << " edges\n";
        cout << "Before new edges, timestamps of events are: [";
        const int fs = eventsWithFakes.size();
        for (int i = 0; i < fs; ++i) {
            if (!(eventsWithFakes[i])) continue;
            if (i) cout << ",";
            cout << -distToZero[i];
        }
        cout << "]\n";
    }
    if (checkSanity) sanityCheck();

    if (floydDoubleCheck) {
        // should, here, be consistent
        const int mSize = eventsWithFakes.size() + 1;

        vector<vector<double> > matrix(mSize, vector<double>(mSize, DBL_MAX));

        for (int m = 0; m < mSize; ++m) {
            matrix[m][m] = 0.0;
        }

        for (int m = 1; m < mSize; ++m) {
            matrix[m][0] = 0.0;
            if (!eventsWithFakes[m-1]) continue;
            if (eventsWithFakes[m-1]->time_spec == VAL::E_AT_START) {
                const vector<pair<double, double> > & tsBounds = TemporalAnalysis::getActionTSBounds()[eventsWithFakes[m-1]->action->getID()];
                const double startMin = tsBounds[0].first;
                const double startMax = tsBounds[0].second;

                matrix[m][0] = -1 * startMin;
                matrix[0][m] = startMax;
                if (noisy) cout << "Edge from " << m - 1 << " to time zero - " << -startMin << " due to earliest RPG start point of action\n";
            } else if (eventsWithFakes[m-1]->time_spec == VAL::E_AT_END) {
                const vector<pair<double, double> > & tsBounds = TemporalAnalysis::getActionTSBounds()[eventsWithFakes[m-1]->action->getID()];
                const double endMin = tsBounds[1].first;
                const double endMax = tsBounds[1].second;

                matrix[m][0] = -1 * endMin;
                matrix[0][m] = endMax;
                if (noisy) cout << "Edge from " << m - 1 << " to time zero - " << -endMin << " due to earliest RPG end point of action\n";

            }
        }

        for (int m = 1; m < mSize; ++m) {
            if (!eventsWithFakes[m-1]) continue;

            if (pairWith[m-1] >= 0) {
                if (eventsWithFakes[m-1]->time_spec == VAL::E_AT_START) {
                    matrix[m][pairWith[m-1] + 1] = eventsWithFakes[m-1]->maxDuration;
                    if (noisy) cout << "Edge from " << m - 1 << " to " << pairWith[m-1] << " - " << eventsWithFakes[m-1]->maxDuration << " due to max duration" << endl;
                } else {
                    matrix[m][pairWith[m-1] + 1] = -1 * eventsWithFakes[m-1]->minDuration;
                    if (noisy) cout << "Edge from " << m - 1 << " to " << pairWith[m-1] << " - " << -eventsWithFakes[m-1]->minDuration << " due to min duration" << endl;
                }
            } else if (pairWith[m-1] == -2) {
                const double tilTime = LPScheduler::getTILTimestamp(eventsWithFakes[m-1]->divisionID);
                matrix[0][m] = tilTime;
                matrix[m][0] = -1 * tilTime;
            }

            if (eventsWithFakes[m-1]->lpMinTimestamp != -1.0) {
                const double backEdge = (eventsWithFakes[m-1]->lpMinTimestamp == 0.0 ? 0.0 : -(eventsWithFakes[m-1]->lpMinTimestamp));
                if (backEdge < matrix[m][0]) {
                    matrix[m][0] = backEdge;
                    if (noisy) cout << "Changing edge from " << m - 1 << " to time zero - " << backEdge << " due to known minimum timestamp for action\n";
                } else {
                    if (noisy) cout << "Not changing edge from " << m - 1 << " to time zero due to known minimum timestamp for action\n";
                }
            } else {
                if (noisy) cout << "Not changing edge from " << m - 1 << " to time zero due to known minimum timestamp for action\n";
            }
            
            if (eventsWithFakes[m-1]->lpMaxTimestamp != -1.0) {
                if (eventsWithFakes[m-1]->lpMaxTimestamp < matrix[0][m]) {
                    matrix[0][m] = eventsWithFakes[m-1]->lpMaxTimestamp;
                    if (noisy) cout << "Changing edge from time zero to " << m - 1 << " to " << matrix[0][m] << " due to known maximum timestamp for action\n";
                } else {
                    if (noisy) cout << "Not changing edge from time zero to " << m - 1 << " due to known maximum timestamp for action\n";
                }
            } else {
                if (noisy) cout << "Not changing edge from time zero to " << m - 1 << " due to known maximum timestamp for action\n";
            }

            map<int, IncomingAndOutgoing>::const_iterator eItr = temporaryEdges.find(m - 1);
            if (eItr != temporaryEdges.end()) {
                map<int, bool>::const_iterator pItr = eItr->second.mustPrecedeThis().begin();
                const map<int, bool>::const_iterator pEnd = eItr->second.mustPrecedeThis().end();

                for (; pItr != pEnd; ++pItr) {
                    matrix[m][pItr->first + 1] = (pItr->second ? -0.001 : 0);
                    if (noisy) cout << "Edge from " << m - 1 << " to " << pItr->first << " - " << (pItr->second ? -0.001 : 0) << " due to recorded predecessors" << endl;
                }
            }
        }

        {
            map<int, double>::const_iterator aItr = autoMinima.begin();
            const map<int, double>::const_iterator aEnd = autoMinima.end();

            for (; aItr != aEnd; ++aItr) {
                matrix[aItr->first + 1][0] = -aItr->second;
                if (noisy) cout << "Edge from " << aItr->first << " to time zero, weight " << -aItr->second << " due to it having to follow after actions with known timestamps\n";
            }
        }

        int k, i, j;
        double distIK, distKJ;

        for (k = 0; k < mSize; ++k) {
            for (i = 0; i < mSize; ++i) {
                distIK = matrix[i][k];
                if (distIK == DBL_MAX) continue;
                for (j = 0; j < mSize; ++j) {
                    distKJ = matrix[k][j];

                    if (distKJ != DBL_MAX) {
                        double & distIJ = matrix[i][j];
                        if ((distIK + distKJ) - distIJ < -0.00001) {
                            distIJ = distIK + distKJ;
                        }
                    }
                }
            }
        }

        for (int m = 0; m < mSize; ++m) {
            assert(fabs(matrix[m][m]) < 0.000000001);
        }

        if (noisy) {
            cout << "Floyd gives pre-expansion timestamps of: [";
            for (int m = 1; m < mSize; ++m) {
                if (!eventsWithFakes[m-1]) continue;
                if (m >= 2) cout << ",";
                cout << -(matrix[m][0]);
            }
            cout << "]\n";
        }
    }

    list<BFEdge>::iterator neItr = newEdges.begin();
    const list<BFEdge>::iterator neEnd = newEdges.end();

    for (; neItr != neEnd; ++neItr) {
        const bool pResult = Propagation(*Q, *neItr, distFromZero, distToZero, pairWith, eventsWithFakes, temporaryEdges);

        if (noisy) {
            cout << "After that edge, timestamps of events are: [";
            const int fs = eventsWithFakes.size();
            for (int i = 0; i < fs; ++i) {
                if (!(eventsWithFakes[i])) continue;
                if (i) cout << ",";
                cout << -distToZero[i];
            }
            cout << "]\n";
                    
        }
        
        if (floydDoubleCheck) {

            const int mSize = eventsWithFakes.size() + 1;

            vector<vector<double> > matrix(mSize, vector<double>(mSize, DBL_MAX));
            vector<vector<int> > paths(mSize, vector<int>(mSize, -1));

            for (int m = 0; m < mSize; ++m) {
                matrix[m][m] = 0.0;
                paths[m][m] = m;
            }

            for (int m = 1; m < mSize; ++m) {
                matrix[m][0] = 0.0;
                paths[m][0] = 0;
                paths[0][m] = m;
                if (!eventsWithFakes[m-1]) continue;
                if (eventsWithFakes[m-1]->time_spec == VAL::E_AT_START) {
                    const vector<pair<double, double> > & tsBounds = TemporalAnalysis::getActionTSBounds()[eventsWithFakes[m-1]->action->getID()];
                    const double startMin = tsBounds[0].first;
                    const double startMax = tsBounds[0].second;

                    matrix[m][0] = -1 * startMin;
                    matrix[0][m] = startMax;
                    
                    if (noisy) cout << "Edge from " << m - 1 << " to time zero - " << -startMin << " due to earliest RPG start point of action\n";
                } else if (eventsWithFakes[m-1]->time_spec == VAL::E_AT_END) {
                    const vector<pair<double, double> > & tsBounds = TemporalAnalysis::getActionTSBounds()[eventsWithFakes[m-1]->action->getID()];
                    const double endMin = tsBounds[1].first;
                    const double endMax = tsBounds[1].second;

                    matrix[m][0] = -1 * endMin;
                    matrix[0][m] = endMax;
                    if (noisy) cout << "Edge from " << m - 1 << " to time zero - " << -endMin << " due to earliest RPG end point of action\n";

                }
            }

            for (int m = 1; m < mSize; ++m) {
                if (!eventsWithFakes[m-1]) continue;

                if (pairWith[m-1] >= 0) {
                    if (eventsWithFakes[m-1]->time_spec == VAL::E_AT_START) {
                        matrix[m][pairWith[m-1] + 1] = eventsWithFakes[m-1]->maxDuration;
                        paths[m][pairWith[m-1] + 1] = pairWith[m-1] + 1;
                        if (noisy) cout << "Edge from " << m - 1 << " to " << pairWith[m-1] << " - " << eventsWithFakes[m-1]->maxDuration << " due to max duration" << endl;
                    } else {
                        matrix[m][pairWith[m-1] + 1] = -1 * eventsWithFakes[m-1]->minDuration;
                        paths[m][pairWith[m-1] + 1] = pairWith[m-1] + 1;
                        if (noisy) cout << "Edge from " << m - 1 << " to " << pairWith[m-1] << " - " << -eventsWithFakes[m-1]->minDuration << " due to min duration" << endl;
                    }
                } else if (pairWith[m-1] == -2) {
                    const double tilTime = LPScheduler::getTILTimestamp(eventsWithFakes[m-1]->divisionID);
                    matrix[0][m] = tilTime;
                    paths[0][m] = m;
                    matrix[m][0] = -1 * tilTime;
                    paths[m][0] = 0;
                }

                if (eventsWithFakes[m-1]->lpMinTimestamp != -1.0) {
                    const double backEdge = (eventsWithFakes[m-1]->lpMinTimestamp == 0.0 ? 0.0 : -(eventsWithFakes[m-1]->lpMinTimestamp));
                    if (backEdge < matrix[m][0]) {
                        matrix[m][0] = backEdge;
                        paths[m][0] = 0;
                        if (noisy) cout << "Changing edge from " << m - 1 << " to time zero - " << backEdge << " due to known minimum timestamp for action\n";
                    } else {
                        if (noisy) cout << "Not changing edge from " << m - 1 << " to time zero due to known minimum timestamp for action\n";
                    }
                } else {
                    if (noisy) cout << "Not changing edge from " << m - 1 << " to time zero due to known minimum timestamp for action\n";
                }
            
                if (eventsWithFakes[m-1]->lpMaxTimestamp != -1.0) {
                    if (eventsWithFakes[m-1]->lpMaxTimestamp < matrix[0][m]) {
                        matrix[0][m] = eventsWithFakes[m-1]->lpMaxTimestamp;
                        paths[0][m] = m;
                        if (noisy) cout << "Changing edge from time zero to " << m - 1 << " to " << matrix[0][m] << " due to known maximum timestamp for action\n";
                    } else {
                        if (noisy) cout << "Not changing edge from time zero to " << m - 1 << " due to known maximum timestamp for action\n";
                    }
                } else {
                    if (noisy) cout << "Not changing edge from time zero to " << m - 1 << " due to known maximum timestamp for action\n";
                }

                map<int, IncomingAndOutgoing>::const_iterator eItr = temporaryEdges.find(m - 1);
                if (eItr != temporaryEdges.end()) {
                    map<int, bool>::const_iterator pItr = eItr->second.mustPrecedeThis().begin();
                    const map<int, bool>::const_iterator pEnd = eItr->second.mustPrecedeThis().end();

                    for (; pItr != pEnd; ++pItr) {
                        matrix[m][pItr->first + 1] = (pItr->second ? -0.001 : 0);
                        paths[m][pItr->first + 1] = pItr->first + 1;
                        if (noisy) cout << "Edge from " << m - 1 << " to " << pItr->first << " - " << (pItr->second ? -0.001 : 0) << " due to recorded predecessors" << endl;
                    }
                }
            }

            {
                map<int, double>::const_iterator aItr = autoMinima.begin();
                const map<int, double>::const_iterator aEnd = autoMinima.end();

                for (; aItr != aEnd; ++aItr) {
                    matrix[aItr->first + 1][0] = -aItr->second;
                    paths[aItr->first + 1][0] = 0;
                    if (noisy) cout << "Edge from " << aItr->first << " to time zero, weight " << -aItr->second << " due to it having to follow after actions with known timestamps\n";
                }
            }

            int k, i, j;
            double distIK, distKJ;

            for (k = 0; k < mSize; ++k) {
                for (i = 0; i < mSize; ++i) {
                    distIK = matrix[i][k];
                    if (distIK == DBL_MAX) continue;
                    for (j = 0; j < mSize; ++j) {
                        distKJ = matrix[k][j];

                        if (distKJ != DBL_MAX) {
                            double & distIJ = matrix[i][j];
                            if ((distIK + distKJ) - distIJ < -0.00001) {
                                /*if (noisy) {
                                    cout << "Path from " << i-1 << " to " << j-1 << " is shorter via " << k-1 << endl;
                                    if (distIJ == DBL_MAX) {
                                        cout << "Was infinite, now " << distIK + distKJ << endl;
                                    } else {
                                        cout << "Was " << distIJ << ", now " << distIK + distKJ << endl;
                                    }
                                }*/
                                
                                distIJ = distIK + distKJ;
                                paths[i][j] = k;
                            }
                        }
                    }
                }
            }

            bool isOkay = true;

            for (int m = 0; m < mSize; ++m) {
                if (matrix[m][m] < 0.0) {
                    isOkay = false;
                }
            }

            if (isOkay) {
                bool matchup = true;
                for (int m = 1; m < mSize; ++m) {
                    if (!eventsWithFakes[m-1]) continue;
                    if (noisy) {
                        cout << (-1 * matrix[m][0]) << " vs " << -distToZero[m-1] << " - step " << m - 1 << " (" << eventsWithFakes[m-1]->lpMinTimestamp << ")" <<  endl;
                        if (fabs(fabs(distToZero[m-1]) - fabs(matrix[m][0])) >= 0.00001) {
                            cout << " - Path from " << (m-1) << " to time zero: ";
                            int previous = m;
                            int goTo = paths[m][0];        
                            cout << " to " << goTo - 1 << ", ";
                            while (goTo != previous) {
                                cout << matrix[previous][goTo];
                                previous = goTo;
                                goTo = paths[goTo][0];         
                                cout << "; to " << goTo - 1 << ", ";
                            }
                            cout << endl;
                        }
                    }
                    matchup = matchup && (fabs(fabs(distToZero[m-1]) - fabs(matrix[m][0])) < 0.00001);
                }
                assert(matchup);
                assert(pResult);
            } else {
                assert(!pResult);
            }

        }

        if (!pResult) {
            return false;
        }
    }

    newEdges.clear();

    if (checkSanity) sanityCheck();

    return true;
}

bool ChildData::updateLPMinTimestamp(const double & d, const int & stepID)
{
    double w = distToZero[stepID];
    if (w != 0) w = -w;
    if (d <= w) {
        if (Globals::globalVerbosity & 4096) {
            cout << COLOUR_light_red << "Post LP, not changing minimum timestamp of node " << stepID << ": is still " << w << COLOUR_default << endl;
        }
        return true;
    }

    if (Globals::globalVerbosity & 4096) {
        cout << COLOUR_light_red << "Post LP, setting minimum timestamp of node " << stepID << " to " << d << " rather than " << w << COLOUR_default << endl;
    }
    addNewEdge(BFEdge(-1, stepID, d, distFromZero[stepID]));
    return propagateNewEdges();
}



};
