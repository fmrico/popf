#ifndef SOLVERCLP_H
#define SOLVERCLP_H

#include "solver.h"

#include "OsiSolverInterface.hpp"
#include "CbcModel.hpp"

#include "OsiClpSolverInterface.hpp"


class MILPSolverCLP : public MILPSolver {

    private:
        OsiClpSolverInterface * lp;
        CbcModel * milp;
        ClpSolve * solvectl;
        
        bool solvedYet;
        bool hasIntegerVariables;        

        static double * scratchW;
        static int * scratchI;
        static int scratchSize;
        static void transferToScratch(const vector<pair<int,double> > & entries);
        
        MILPSolverCLP(const MILPSolverCLP & c);
        
    public:
        MILPSolverCLP();
        virtual ~MILPSolverCLP();
        
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
                        
        virtual bool isColumnInteger(const int & var);
        
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
        virtual double getObjValue();
        
        virtual void getRow(const int & i, vector<pair<int,double> > & entries);
        
        virtual bool supportsQuadratic() const;
        
        virtual void hush();

};

#endif
