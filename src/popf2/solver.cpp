#include "solver.h"
#include "globals.h"

#include <iostream>
#include <limits>
#include <cassert>
#include <cstdlib>
#include <sstream>

using std::make_pair;
using std::numeric_limits;
using std::cout;
using std::endl;
using std::ostringstream;

bool MILPSolver::debug = false;

bool branchOnBinaryVariable(MILPSolver * const lp, const vector<MILPSolver::Objective::const_iterator> & branchOver,
                            const int & branchCount, const int & var, const map<int,double> & coeffs,
                            vector<bool> & solution, const bool & maximise, double & bestSolutionCost)
{
    static const bool debug = MILPSolver::debug;
    
    if (var == branchCount) {
        
        if (maximise) {
            if (lp->getObjValue() > bestSolutionCost) {
                bestSolutionCost = lp->getObjValue();
                if (debug) {
                    ostringstream fn;
                    fn << "quadratic";
                    for (int bv = 0; bv < branchCount; ++bv) {
                        if (bv) cout << ",";
                        
                        if (solution[bv]) {
                            cout << "1";   
                            fn << "1";
                            assert(lp->getColLower(branchOver[bv]->first) == 1); 
                        } else {
                            cout << "0";
                            fn << "0";
                            assert(lp->getColUpper(branchOver[bv]->first) == 0);
                        }
                        
                    }
                    cout << "=" << bestSolutionCost << " - new best\n";
                    fn << ".lp";
                    string cname = fn.str();
                    lp->writeLp(cname);
                    lp->writeLp("bestquadratic.lp");
                }
                return true;
            } else {
                return false;
            }
        } else {
            if (lp->getObjValue() < bestSolutionCost) {
                bestSolutionCost = lp->getObjValue();
                if (debug) {
                    ostringstream fn;
                    fn << "quadratic";
                    for (int bv = 0; bv < branchCount; ++bv) {
                        if (bv) cout << ",";
                        
                        if (solution[bv]) {
                            cout << "1";   
                            fn << "1";
                            assert(lp->getColLower(branchOver[bv]->first) == 1); 
                        } else {
                            cout << "0";
                            fn << "0";
                            assert(lp->getColUpper(branchOver[bv]->first) == 0);
                        }
                        
                    }
                    cout << "=" << bestSolutionCost << " - new best\n";
                    fn << ".lp";
                    string cname = fn.str();
                    lp->writeLp(cname);
                    lp->writeLp("bestquadratic.lp");
                }
                return true;
            } else {
                return false;
            }
        }
    }
        
    double solnValue = bestSolutionCost;
    
    const int tryFirst = lp->getSingleSolutionVariableValue(branchOver[var]->first);
    
    for (int pass = 0; pass < 2; ++pass) {
        const int currValue = (pass ? 1 - tryFirst : tryFirst);        
        //std::cout << string(" ", var) << currValue << std::endl;
        vector<bool> childSolution(solution);
        
        if (currValue == 1) {
            // if the variable takes the value 1, then we need to add the quadratic terms associated with it
            
            childSolution[var] = true;
            lp->setColLower(branchOver[var]->first, 1);
            
            map<int,double> childCoeffs(coeffs);
            
            map<int,double>::iterator insItr = childCoeffs.begin();
            
            map<int,double>::const_iterator quadraticTerms = branchOver[var]->second.nonLinearCoefficients.begin();
            const map<int,double>::const_iterator quadraticEnd = branchOver[var]->second.nonLinearCoefficients.end();
            
            for (; quadraticTerms != quadraticEnd; ++quadraticTerms) {
                insItr = childCoeffs.insert(insItr, make_pair(quadraticTerms->first, 0.0));
                insItr->second += quadraticTerms->second;
                lp->setObjCoeff(insItr->first, insItr->second);
            }
            
            if (lp->solve(true)) { // if the LP can be solved
                const bool wasBetter = branchOnBinaryVariable(lp, branchOver, branchCount, var + 1, childCoeffs, childSolution, maximise, solnValue);
                if (wasBetter) {                   
                    solution = childSolution;
                }
            }
            
            // now revert objective function changes
            
            map<int,double>::const_iterator oldCoeffsItr = coeffs.begin();
            const map<int,double>::const_iterator oldCoeffsEnd = coeffs.end();
            
            map<int,double>::const_iterator newCoeffsItr = childCoeffs.begin();
            const map<int,double>::const_iterator newCoeffsEnd = childCoeffs.end();
            
            while (oldCoeffsItr != oldCoeffsEnd) {
                assert(newCoeffsItr != newCoeffsEnd); // we should never have lost coefficients - at worse, they will be zeroed
                
                if (newCoeffsItr->first < oldCoeffsItr->first) {
                    // re-zero any terms which previously had no coefficient 
                    lp->setObjCoeff(newCoeffsItr->first, 0.0);
                    ++newCoeffsItr;
                } else {
                    // reset previous coefficient
                    lp->setObjCoeff(oldCoeffsItr->first, oldCoeffsItr->second);
                    ++newCoeffsItr;
                    ++oldCoeffsItr;
                }
            }
            
            // re-zero any remaining terms which previously had no coefficient
            for (; newCoeffsItr != newCoeffsEnd; ++newCoeffsItr) {
                lp->setObjCoeff(newCoeffsItr->first, 0.0);
            }
            
            
            lp->setColLower(branchOver[var]->first, 0);
        } else {
            
            // simple case - variable takes the value 0, so the quadratic terms associated with it are 0 too
            childSolution[var] = false;
            lp->setColUpper(branchOver[var]->first, 0);
            
            if (lp->solve(true)) { // if the LP can be solved
                const bool wasBetter = branchOnBinaryVariable(lp, branchOver, branchCount, var + 1, coeffs, childSolution, maximise, solnValue);
                if (wasBetter) {                   
                    solution = childSolution;
                }
            }
            
            lp->setColUpper(branchOver[var]->first, 1);
        }
    }
    
    if (maximise) {
        if (solnValue > bestSolutionCost) {
            bestSolutionCost = solnValue;
            return true;
        }
    } else {
        if (solnValue < bestSolutionCost) {
            bestSolutionCost = solnValue;
            return true;
        }
    }
    
    return false;
}

bool MILPSolver::quadraticPreSolve()
{
    clearObjective();
    
    setMaximiseObjective(quadraticObjective.maximise());
    
    vector<Objective::const_iterator> branchOver;
    
    branchOver.reserve(quadraticObjective.size());
    
    map<int,double> coeffs;
    
    Objective::const_iterator tItr = quadraticObjective.begin();
    const Objective::const_iterator tEnd = quadraticObjective.end();
    
    for (; tItr != tEnd; ++tItr) {
        
        // First, note any linear coefficients - these don't need to be branched over
        if (tItr->second.linearCoefficient != 0.0) {
            setObjCoeff(tItr->first, tItr->second.linearCoefficient);
            coeffs.insert(make_pair(tItr->first, 0.0)).first->second += tItr->second.linearCoefficient;
        }
        
        if (!tItr->second.nonLinearCoefficients.empty()) {
            // we have a quadratic term - this algorithm insists that tItr->first
            // is a binary variable, so we must check this is the case
            
            
            if ( !isColumnBinary(tItr->first)) {
                std::cerr << "Warning: unsupported quadratic terms in MIQCP objective - must be of the form <binary var> * <var> * <weight>,\n";
                std::cerr << "but column \"" << getColName(tItr->first) << "\" (" << tItr->first << ") is not binary\n";
                std::cerr << "This suggests a bug in the code building the objective for scheduling the plan according to the task metric\n";
                return false;
            }
            
            branchOver.push_back(tItr);
        }
    }
    
    const int branchCount = branchOver.size();
    
    const bool solvableAtAll = solve(false);
    
    if (!solvableAtAll) {
        return false;
    }

    double bestSolutionQuality;

    if (branchCount) {
            
        vector<bool> solution(branchCount);
        bestSolutionQuality = (quadraticObjective.maximise() ? -1 : 1) * numeric_limits<double>::max();
        if (!branchOnBinaryVariable(this, branchOver, branchCount, 0, coeffs, solution, quadraticObjective.maximise(), bestSolutionQuality )) {
            // no solution could be found when any combination of the extra objective terms was introduced
            return false;
        }
        
        // finally, fix the best combination of binary variables, and the solution quality
        
        for (int bv = 0; bv < branchCount; ++bv) {
            if (solution[bv]) {
                setColLower(branchOver[bv]->first, 1);
                
                map<int,double>::iterator insItr = coeffs.begin();
                
                map<int,double>::const_iterator quadraticTerms = branchOver[bv]->second.nonLinearCoefficients.begin();
                const map<int,double>::const_iterator quadraticEnd = branchOver[bv]->second.nonLinearCoefficients.end();
                
                for (; quadraticTerms != quadraticEnd; ++quadraticTerms) {
                    insItr = coeffs.insert(insItr, make_pair(quadraticTerms->first, 0.0));
                    insItr->second += quadraticTerms->second;
                }
                
            } else {
                setColUpper(branchOver[bv]->first, 0);
            }
        }

    } else {
        bestSolutionQuality = getObjValue();
        if (MILPSolver::debug) {
            writeLp("bestquadratic.lp");
        }
    }
        
    clearObjective();
    
    vector<pair<int,double> > entries;
    entries.insert(entries.end(), coeffs.begin(), coeffs.end());
    
    if (quadraticObjective.maximise()) {
        addRow(entries, bestSolutionQuality - 0.001, getInfinity());
    } else {
        addRow(entries, -getInfinity(), bestSolutionQuality + 0.001);
    }
    
    if (MILPSolver::debug) {
        cout << "Bound on solution quality according to task metric: " << bestSolutionQuality << endl;
        writeLp("quadratic.lp");
    }
    
    return true;
    
}
