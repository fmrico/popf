#ifndef SOLVER_H
#define SOLVER_H

#include <string>
#include <vector>
#include <utility>
#include <list>
#include <map>
using std::string;
using std::vector;
using std::pair;
using std::list;
using std::map;

extern void readParams(char * argv[], const int & a);

class MILPSolver {


    public:

        static bool debug;
        
        class Objective {
          
        public:
            struct Coefficient {
                double linearCoefficient;
                map<int,double> nonLinearCoefficients;
                
                Coefficient()
                : linearCoefficient(0.0)
                {
                }
                
                
            };
            
        protected:
            map<int, Coefficient> terms;
            bool _maximise;
        public:
            
            typedef map<int, Coefficient>::const_iterator const_iterator;
            
            Objective(const bool maxObj=false)
                : _maximise(maxObj) {
            }
                        
            void setMaximise(const bool & m) {
                _maximise = m;
            }
            
            inline const bool & maximise() const {
                return _maximise;
            }
            
            Coefficient & getTerm(const int & v) {
                return terms.insert(std::make_pair(v, Coefficient())).first->second;
            }
            
            const_iterator begin() const {
                return terms.begin();
            }
            
            const_iterator end() const {
                return terms.end();
            }
            
            size_t size() const {
                return terms.size();
            }
        };
        
        enum ColumnType {
            C_REAL = 1,
            C_INT = 2,
            C_BOOL = 3
        };
        
    protected:
        
        MILPSolver() {
        }
                        
        Objective quadraticObjective;
        
    public:
        
        virtual ~MILPSolver() {            
        }
        
        virtual MILPSolver * clone() = 0;
        
        virtual double getInfinity() = 0;
        
        
        /// Functions for the rows (constraints) of the model
        
        virtual void addRow(const vector<pair<int,double> > & entries, const double & lb, const double & ub) = 0;                
        
        virtual void setRowName(const int & cons, const string & asString) = 0;        
        virtual string getRowName(const int & cons) = 0;        
        
        virtual double getRowUpper(const int & var) = 0;
        virtual void setRowUpper(const int & c, const double & b) = 0;
        virtual double getRowLower(const int & var) = 0;
        virtual void setRowLower(const int & c, const double & b) = 0;
        
        virtual int getNumRows() = 0;
        
        /// Functions for the columns (variables) of the model
        
        virtual void addCol(const vector<pair<int,double> > & entries, const double & lb, const double & ub, const ColumnType & type) = 0;
        
        /**
         * Ascertain whether the given column is an integer.
         *
         * @param c  A column index
         * @return <code>true</code> if column <code>c</code> is an integer.
         */
        virtual bool isColumnInteger(const int & c) = 0;
        
        /**
        * Ascertain whether the given column is binary (i.e. a [0,1] integer).
        *
        * @param c  A column index
        * @return <code>true</code> if column <code>c</code> is binary.
        */        
        virtual bool isColumnBinary(const int & c) {
            return (isColumnInteger(c) && (getColLower(c) == 0) && (getColUpper(c) == 1));
        }
        
        virtual void setColName(const int & var, const string & asString) = 0;
        virtual string getColName(const int & var) = 0;
        
        virtual double getColUpper(const int & var) = 0;
        virtual void setColUpper(const int & var, const double & b) = 0;
        virtual double getColLower(const int & var) = 0;
        virtual void setColLower(const int & var, const double & b) = 0;
        virtual void setColBounds(const int & var, const double & lb, const double & ub) = 0;

        virtual int getNumCols() = 0;
        
        
        /**
        *  Add empty columns to the LP, for real-valued variables.  Default bounds are
        *  <code>0</code> to <code>getInfinity()</code>.
        *
        *  @param n  The number of columns to add
        */ 
        virtual void addEmptyRealCols(const int & n) {
            static const vector<pair<int,double> > emptyEntries;
            
            for (int i = 0; i < n; ++i) {
                addCol(emptyEntries, 0.0, getInfinity(), C_REAL);
            }
        }
        
        
        /// Objective function calls
        
        
        /**
         *  Specify that the calls to <code>solve()</code> should maximise the given objective,
         *  rather than minimising.
         *
         *  @param maxim  If <code>true</code>, the objective will be maximised.
         */
        virtual void setMaximiseObjective(const bool & maxim) = 0;
               
        
        /**
         *  Set the objective function according to the vector of weights given.
         *
         *  @param entries  An array, size equal to <code>getNumCols()</code> containing the
         *                  linear coefficient for each column, to use in the objective.
         */
        virtual void setObjective(double * const entries) = 0;
        
        /**
         *  Specify a (possibly quadratic) objective to use when solving the LP.
         *
         *  @param o  The objective function to use
         */
        virtual void setQuadraticObjective(const Objective & o) {
            quadraticObjective = o;
        }
        
        /**
         *  Set the linear coefficient in the objective function of the given column.
         *
         *  @param var  The column whose coefficient is to be changed
         *  @param w    The coefficient for column <code>var</code>
         */
        virtual void setObjCoeff(const int & var, const double & w) = 0;
        
        /**
         *  Clear the objective coefficients (set each to 0).
         */        
        virtual void clearObjective() = 0;

        
        ///  Solving, and accessing solutions
        
        virtual bool solve(const bool & skipPresolve) = 0;
        
        /**
         *  Find the best combination of the settings to the binary variables
         *  in the quadratic objective passed to
         *  <code>setQuadraticObjective()</code>.  The default implementation
         *  branches over the settings to the integer objective terms, fixing
         *  the bounds on the variables to the best solution found.  Note
         *  that after calling this a call to <code>solve()</code> is still
         *  to be able to access the best solution.
         *
         *  @return <code>true</code> if the MIQCP could be solved.
         *
         */
        virtual bool quadraticPreSolve();
        
        virtual const double * getSolution() = 0;
        
        /**
         *  Obtain the solution values of a subset of the columns in the LP.
         *  @param from  The start column index of the range to return (inclusive)
         *  @param to    The end column index of the range to return (exclusive)
         *
         *  @return A pointer to an array, where index 0 contains the value of the variable <code>from</code> etc.
         */
        virtual const double * getPartialSolution(const int & from, const int & to) = 0;
        
        /**
         *  Obtain the solution values of the specified variable.
         *
         *  @param col  The index of the variable (column)
         *  @return  The value that variable takes in the most recent solution to the LP
         */
        virtual double getSingleSolutionVariableValue(const int & col) {
            return getSolution()[col];
        }
        
        /**
         *  Obtain the value of the row in the solution.
         *
         *  @return  An array containing <code>getNumRows()</code> values, one for each row.
         */
        virtual const double * getSolutionRows() = 0;
        
        /**
         *  Obtain the value of the specified row in the solution.
         *  @param row  A row index
         *  @return  The value it takes in the solution
         */
        virtual double getSingleSolutionRowValue(const int & row) {
            return getSolutionRows()[row];
        }
        
        /** Return the computed value of the objective function. */
        virtual double getObjValue() = 0;
        
        /// Miscellaneous functions
        
        virtual void writeLp(const string & filename) = 0;
        
        /**
         *  Get a row from the LP, returning the coefficients it contains.
         *
         *  @param i  The index of the row to get
         *  @param entries  A reference to a vector of <code>int</code>,<code>double</code> pairs which,
         *                  after calling the method, will contain the column indices and coefficients
         *                  of row <code>i</code> of the LP.
         */
        virtual void getRow(const int & i, vector<pair<int,double> > & entries) = 0;

        /** Suppress all output from the solver */
        virtual void hush() = 0;
                
        
};

extern MILPSolver * getNewSolver();
extern const double LPinfinity;

#endif

