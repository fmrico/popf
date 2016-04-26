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

#ifndef SOLVERCLP_H
#define SOLVERCLP_H


#include <set>
#include <map>

using std::set;
using std::map;

#include "solver.h"

struct IloEnv;
struct IloModel;
struct IloNumVarArray;
struct IloObjective;
struct IloCplex;

class MILPSolverCPX : public MILPSolver {

    private:
        struct Constraints;
        
        IloEnv * env;
        int * envUsers;
        
        IloModel * model;
        IloNumVarArray * modelvar;
        Constraints * modelcon;
        IloObjective * obj;
        IloCplex * cplex;
        double * solArray;
        
        int colCount;
        int rowCount;

        map<int,bool> integervars;
        map<int, map<int,double> > coeffs;
        
        MILPSolverCPX(MILPSolverCPX & c);
        
    public:
        MILPSolverCPX();
        virtual ~MILPSolverCPX();
        
        virtual MILPSolver * clone();
        
        virtual double getInfinity();
        
        virtual int getNumCols();
        virtual int getNumRows();
        
        virtual void setColName(const int & var, const string & asString);
        virtual string getColName(const int & var);
        virtual void setRowName(const int & cons, const string & asString);
        virtual string getRowName(const int & cons);
        
        //virtual void setInteger(const int & var);
        
        virtual double getColUpper(const int & var);
        virtual void setColUpper(const int & var, const double & b);
        virtual double getColLower(const int & var);
        virtual void setColLower(const int & var, const double & b);
        virtual void setColBounds(const int & var, const double & lb, const double & ub);
        
        virtual bool isColumnInteger(const int & col);
        virtual bool isColumnBinary(const int & col);
        
        virtual double getRowUpper(const int & var);
        virtual void setRowUpper(const int & c, const double & b);
        virtual double getRowLower(const int & var);
        virtual void setRowLower(const int & c, const double & b);
        
        virtual void addCol(const vector<pair<int,double> > & entries, const double & lb, const double & ub, const ColumnType & type);
        virtual void addRow(const vector<pair<int,double> > & entries, const double & lb, const double & ub);
        virtual void setMaximiseObjective(const bool & maxim);
                
        virtual void clearObjective();
        virtual void setObjective(double * const entries);
        virtual void setObjCoeff(const int & var, const double & w);
        virtual void writeLp(const string & filename);
        virtual bool solve(const bool & skipPresolve); 
        
        virtual const double * getSolution();
        virtual const double * getSolutionRows();
        virtual const double * getPartialSolution(const int & from, const int & to);
        virtual double getSingleSolutionVariableValue(const int & col);
        virtual double getObjValue();
        
        virtual bool supportsQuadratic() const;
        
        virtual void getRow(const int & i, vector<pair<int,double> > & entries);
        
        virtual void hush();

};

#endif
