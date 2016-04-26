#include "solver-cpx.h"

#include <sstream>

#define IL_STD 1

#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

bool provided = false;

vector<pair<int, int> > intParams;
vector<pair<int, int> > boolParams;
vector<pair<int,double> > numParams;

void addIntParam(const int & id, char * const val)
{
    
    istringstream cnv(val);
    
    int i;
    
    #ifndef NDEBUG
    assert(cnv >> i);
    #else
    cnv >> i;
    #endif
    
    intParams.push_back(make_pair(id, i));
    
}

void addBoolParam(const int & id, char * const val)
{
    
    if (string(val) == "yes") {
        boolParams.push_back(make_pair(id,1));
    } else {
        assert(string(val) == "no");
        boolParams.push_back(make_pair(id,0));
    }
        
}

void addNumParam(const int & id, char * const val)
{
    
    istringstream cnv(val);
    
    double i;
    
    #ifndef NDEBUG
    assert(cnv >> i);
    #else
    cnv >> i;
    #endif
    
    numParams.push_back(make_pair(id, i));
    
}


void readParams(char * argv[], const int & a)
{
    provided = true;
    
    for (int i = 0; i < a; i+=2) {
        argv[i] = &(argv[i][1]);
        
        const string argvi(argv[i]);
        
        if (argvi == "advance") {
            addIntParam(CPX_PARAM_ADVIND, argv[i+1]);
            continue;
        }

        if (argvi == "barrier_algorithm") {
            addIntParam(CPX_PARAM_BARALG, argv[i+1]);
            continue;
        }

        if (argvi == "barrier_colnonzeros") {
            addIntParam(CPX_PARAM_BARCOLNZ, argv[i+1]);
            continue;
        }

        if (argvi == "barrier_convergetol") {
            addNumParam(CPX_PARAM_BAREPCOMP, argv[i+1]);
            continue;
        }

        if (argvi == "barrier_crossover") {
            addIntParam(CPX_PARAM_BARCROSSALG, argv[i+1]);
            continue;
        }

        if (argvi == "barrier_limits_corrections") {
            addIntParam(CPX_PARAM_BARMAXCOR, argv[i+1]);
            continue;
        }

        if (argvi == "barrier_limits_growth") {
            addNumParam(CPX_PARAM_BARGROWTH, argv[i+1]);
            continue;
        }

        if (argvi == "barrier_ordering") {
            addIntParam(CPX_PARAM_BARORDER, argv[i+1]);
            continue;
        }

        if (argvi == "barrier_qcpconvergetol") {
            addNumParam(CPX_PARAM_BARQCPEPCOMP, argv[i+1]);
            continue;
        }

        if (argvi == "barrier_startalg") {
            addIntParam(CPX_PARAM_BARSTARTALG, argv[i+1]);
            continue;
        }

        if (argvi == "emphasis_memory") {
            addBoolParam(CPX_PARAM_MEMORYEMPHASIS, argv[i+1]);
            continue;
        }

        if (argvi == "emphasis_mip") {
            addIntParam(CPX_PARAM_MIPEMPHASIS, argv[i+1]);
            continue;
        }

        if (argvi == "emphasis_numerical") {
            addBoolParam(CPX_PARAM_NUMERICALEMPHASIS, argv[i+1]);
            continue;
        }

        if (argvi == "feasopt_mode") {
            addIntParam(CPX_PARAM_FEASOPTMODE, argv[i+1]);
            continue;
        }

        if (argvi == "feasopt_tolerance") {
            addNumParam(CPX_PARAM_EPRELAX, argv[i+1]);
            continue;
        }

        if (argvi == "lpmethod") {
            addIntParam(CPX_PARAM_LPMETHOD, argv[i+1]);
            continue;
        }

        if (argvi == "mip_cuts_cliques ") {
            addIntParam(CPX_PARAM_CLIQUES, argv[i+1]);
            continue;
        }

        if (argvi == "mip_cuts_covers") {
            addIntParam(CPX_PARAM_COVERS, argv[i+1]);
            continue;
        }

        if (argvi == "mip_cuts_disjunctive") {
            addIntParam(CPX_PARAM_DISJCUTS, argv[i+1]);
            continue;
        }

        if (argvi == "mip_cuts_flowcovers") {
            addIntParam(CPX_PARAM_FLOWCOVERS, argv[i+1]);
            continue;
        }

        if (argvi == "mip_cuts_gomory") {
            addIntParam(CPX_PARAM_FRACCUTS, argv[i+1]);
            continue;
        }

        if (argvi == "mip_cuts_gubcovers") {
            addIntParam(CPX_PARAM_GUBCOVERS, argv[i+1]);
            continue;
        }

        if (argvi == "mip_cuts_implied") {
            addIntParam(CPX_PARAM_IMPLBD, argv[i+1]);
            continue;
        }
        
        if (argvi == "mip_cuts_mcfcut") {
            addIntParam(CPX_PARAM_MCFCUTS, argv[i+1]);
            continue;
        }
                
        if (argvi == "mip_cuts_mircut") {
            addIntParam(CPX_PARAM_MIRCUTS, argv[i+1]);
            continue;
        }

        if (argvi == "mip_cuts_pathcut") {
            addIntParam(CPX_PARAM_FLOWPATHS, argv[i+1]);
            continue;
        }
        
        if (argvi == "mip_cuts_zerohalfcut") {
            addIntParam(CPX_PARAM_ZEROHALFCUTS, argv[i+1]);
            continue;
        }

        if (argvi == "mip_limits_aggforcut") {
            addIntParam(CPX_PARAM_AGGCUTLIM, argv[i+1]);
            continue;
        }

        if (argvi == "mip_limits_cutpasses") {
            addIntParam(CPX_PARAM_CUTPASS, argv[i+1]);
            continue;
        }

        if (argvi == "mip_limits_cutsfactor") {
            addNumParam(CPX_PARAM_CUTSFACTOR, argv[i+1]);
            continue;
        }

        if (argvi == "mip_limits_gomorycand") {
            addIntParam(CPX_PARAM_FRACCAND, argv[i+1]);
            continue;
        }

        if (argvi == "mip_limits_gomorypass") {
            addIntParam(CPX_PARAM_FRACPASS, argv[i+1]);
            continue;
        }

        if (argvi == "mip_limits_polishtime") {
            addNumParam(CPX_PARAM_POLISHTIME, argv[i+1]);
            continue;
        }

        if (argvi == "mip_limit_probetime") {
            addNumParam(CPX_PARAM_PROBETIME, argv[i+1]);
            continue;
        }

        if (argvi == "mip_limits_repairtries") {
            addIntParam(CPX_PARAM_REPAIRTRIES, argv[i+1]);
            continue;
        }

        if (argvi == "mip_limits_strongcand") {
            addIntParam(CPX_PARAM_STRONGCANDLIM, argv[i+1]);
            continue;
        }

        if (argvi == "mip_limits_strongit") {
            addIntParam(CPX_PARAM_STRONGITLIM, argv[i+1]);
            continue;
        }

        if (argvi == "mip_limits_submipnodelim") {
            addIntParam(CPX_PARAM_SUBMIPNODELIM, argv[i+1]);
            continue;
        }

        if (argvi == "mip_ordertype") {
            addIntParam(CPX_PARAM_MIPORDTYPE, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_backtrack") {
            addNumParam(CPX_PARAM_BTTOL, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_bbinterval") {
            addIntParam(CPX_PARAM_BBINTERVAL, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_branch") {
            addIntParam(CPX_PARAM_BRDIR, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_dive") {
            addIntParam(CPX_PARAM_DIVETYPE, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_file") {
            addIntParam(CPX_PARAM_NODEFILEIND, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_fpheur") {
            addIntParam(CPX_PARAM_FPHEUR, argv[i+1]);
            continue;
        }
        
        if (argvi == "mip_strategy_heuristicfreq") {
            addIntParam(CPX_PARAM_HEURFREQ, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_lbheur") {
            addBoolParam(CPX_PARAM_LBHEUR, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_nodeselect") {
            addIntParam(CPX_PARAM_NODESEL, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_order") {
            addBoolParam(CPX_PARAM_MIPORDIND, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_presolvenode") {
            addIntParam(CPX_PARAM_PRESLVND, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_probe") {
            addIntParam(CPX_PARAM_PROBE, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_rinsheur") {
            addIntParam(CPX_PARAM_RINSHEUR, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_search") {
            addIntParam(CPX_PARAM_MIPSEARCH, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_startalgorithm") {
            addIntParam(CPX_PARAM_STARTALG, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_subalgorithm") {
            addIntParam(CPX_PARAM_SUBALG, argv[i+1]);
            continue;
        }

        if (argvi == "mip_strategy_variableselect") {
            addIntParam(CPX_PARAM_VARSEL, argv[i+1]);
            continue;
        }

        if (argvi == "network_netfind") {
            addIntParam(CPX_PARAM_NETFIND, argv[i+1]);
            continue;
        }

        if (argvi == "network_pricing") {
            addIntParam(CPX_PARAM_NETPPRIIND, argv[i+1]);
            continue;
        }

        if (argvi == "perturbation_constant") {
            addNumParam(CPX_PARAM_EPPER, argv[i+1]);
        }

        if (argvi == "preprocessing_aggregator") {
            addIntParam(CPX_PARAM_AGGIND, argv[i+1]);
            continue;
        }

        if (argvi == "preprocessing_boundstrength") {
            addIntParam(CPX_PARAM_BNDSTRENIND, argv[i+1]);
            continue;
        }

        if (argvi == "preprocessing_coeffreduce") {
            addIntParam(CPX_PARAM_COEREDIND, argv[i+1]);
            continue;
        }

        if (argvi == "preprocessing_dependency") {
            addIntParam(CPX_PARAM_DEPIND, argv[i+1]);
            continue;
        }

        if (argvi == "preprocessing_dual") {
            addIntParam(CPX_PARAM_PREDUAL, argv[i+1]);
            continue;
        }

        if (argvi == "preprocessing_fill") {
            addIntParam(CPX_PARAM_AGGFILL, argv[i+1]);
            continue;
        }

        if (argvi == "preprocessing_linear") {
            addBoolParam(CPX_PARAM_PRELINEAR, argv[i+1]);
            continue;
        }

        if (argvi == "preprocessing_numpass") {
            addIntParam(CPX_PARAM_PREPASS, argv[i+1]);
            continue;
        }

        if (argvi == "preprocessing_presolve") {
            addIntParam(CPX_PARAM_PREIND, argv[i+1]);
            continue;
        }

        if (argvi == "preprocessing_qpmakepsd") {
            addBoolParam(CPX_PARAM_QPMAKEPSDIND, argv[i+1]);
            continue;
        }

        if (argvi == "preprocessing_reduce") {
            addIntParam(CPX_PARAM_REDUCE, argv[i+1]);
            continue;
        }

        if (argvi == "preprocessing_relax") {
            addIntParam(CPX_PARAM_RELAXPREIND, argv[i+1]);
            continue;
        }

        if (argvi == "preprocessing_repeatpresolve") {
            addIntParam(CPX_PARAM_REPEATPRESOLVE, argv[i+1]);
            continue;
        }

        if (argvi == "preprocessing_symmetry") {
            addIntParam(CPX_PARAM_SYMMETRY, argv[i+1]);
            continue;
        }

        if (argvi == "qpmethod") {
            addIntParam(CPX_PARAM_QPMETHOD, argv[i+1]);
            continue;
        }

        if (argvi == "read_scale") {
            addIntParam(CPX_PARAM_SCAIND, argv[i+1]);
            continue;
        }

        if (argvi == "sifting_algorithm") {
            addIntParam(CPX_PARAM_SIFTALG, argv[i+1]);
            continue;
        }

        if (argvi == "simplex_crash") {
            addIntParam(CPX_PARAM_CRAIND, argv[i+1]);
            continue;
        }

        if (argvi == "simplex_dgradient") {
            addIntParam(CPX_PARAM_DPRIIND, argv[i+1]);
            continue;
        }

        if (argvi == "simplex_limits_perturbation") {
            addIntParam(CPX_PARAM_PERLIM, argv[i+1]);
            continue;
        }

        if (argvi == "simplex_limits_singularity") {
            addIntParam(CPX_PARAM_SINGLIM, argv[i+1]);
            continue;
        }

        if (argvi == "simplex_perturbation") {
            addBoolParam(CPX_PARAM_PERIND, argv[i+1]);
            ++i;
            continue;
        }
        
        if (argvi == "simplex_perturbation_switch") {
            addBoolParam(CPX_PARAM_PERIND, argv[i+1]);
            continue;
        }

        if (argvi == "simplex_pgradient") {
            addIntParam(CPX_PARAM_PPRIIND, argv[i+1]);
            continue;
        }

        if (argvi == "simplex_pricing") {
            addIntParam(CPX_PARAM_PRICELIM, argv[i+1]);
            continue;
        }

        if (argvi == "simplex_refactor") {
            addIntParam(CPX_PARAM_REINV, argv[i+1]);
            continue;
        }

        if (argvi == "simplex_tolerances_feasibility") {
            addNumParam(CPX_PARAM_EPINT, argv[i+1]);
            continue;
        }

        if (argvi == "simplex_tolerances_markowitz") {
            addNumParam(CPX_PARAM_EPMRK, argv[i+1]);
            continue;
        }

        if (argvi == "simplex_tolerances_optimality") {
            addNumParam(CPX_PARAM_EPOPT, argv[i+1]);
            continue;
        }

    }
}

#include "solver-cpx-paramils.cpp"

struct MILPSolverCPX::Constraints {
      
    map<int, IloRange> data;
};

MILPSolverCPX::MILPSolverCPX()
{
    env = new IloEnv;
    envUsers = new int;    
    (*envUsers) = 1;
    
    model = new IloModel(*env);
    
    modelvar = new IloNumVarArray(*env);
    modelcon = new Constraints();
    
    obj = new IloObjective(*env);
    obj->setSense(IloObjective::Minimize);
    
    model->add(*obj);
    
    colCount = 0;
    rowCount = 0;
    
    cplex = new IloCplex(*model);
    
    setILS(cplex);
    solArray = 0;
}

MILPSolverCPX::MILPSolverCPX(MILPSolverCPX & c)
{
    static const bool debug = false;
    
    if (debug) {
        cout << "Copying...\n";
        ostringstream copyfn;
        copyfn << "copyconstructor" << *(c.envUsers) << ".lp";
        const string copyfns = copyfn.str();
        c.writeLp(copyfns.c_str());
    }
        
    env = c.env;
    envUsers = c.envUsers;
    ++(*envUsers);
    
    model = new IloModel(*env);
    
    modelvar = new IloNumVarArray(*env);
    modelcon = new Constraints();   
    
    obj = new IloObjective(*env);
    obj->setSense(IloObjective::Minimize);
    
    model->add(*obj);

    
    colCount = c.colCount;
    rowCount = c.rowCount;

    integervars = c.integervars;   

    map<int,bool>::const_iterator iItr = integervars.begin();
    const map<int,bool>::const_iterator iEnd = integervars.end();
        
    for (int ci = 0; ci < colCount; ++ci) {
        const double lb = (*(c.modelvar))[ci].getLB();
        const double ub = (*(c.modelvar))[ci].getUB();
        
        if (iItr != iEnd && iItr->first == ci) {
            modelvar->add(IloNumVar(*env, lb, ub, (iItr->second ? ILOBOOL : ILOINT)));
            //cout << "Column " << ci << " in the copy is an integer\n";
            ++iItr;
        } else {        
            modelvar->add(IloNumVar(*env, lb, ub));
        }
        const char * oldName = (*(c.modelvar))[ci].getName();
        (*modelvar)[ci].setName(oldName);
       
    }
    
    coeffs = c.coeffs;
    
    map<int,map<int,double> >::const_iterator coItr = coeffs.begin();
    const map<int,map<int,double> >::const_iterator coEnd = coeffs.end();
    
    map<int,IloRange>::iterator dItr = c.modelcon->data.begin();
    const map<int,IloRange>::iterator dEnd = c.modelcon->data.end();
    
    for (int r = 0; dItr != dEnd; ++r, ++dItr) {
        const double lb = dItr->second.getLB();
        const double ub = dItr->second.getUB();
        
        IloRange & newRange = modelcon->data[r] = IloAdd(*model, IloRange(*env, lb,ub));
        
       
        if (dItr->second.getName()) {
            newRange.setName(dItr->second.getName());
        }
        
        if (coItr == coEnd) continue;
        if (r < coItr->first) continue;
        
        map<int,double>::const_iterator fItr = coItr->second.begin();
        const map<int,double>::const_iterator fEnd = coItr->second.end();
        
        for (; fItr != fEnd; ++fItr) {
            newRange.setLinearCoef((*modelvar)[fItr->first], fItr->second);
        }
        ++coItr;
    }
    
    
    
    cplex = new IloCplex(*model);
    setILS(cplex);
    
    solArray = 0;
    
    if (debug) {
        ostringstream copyfn;
        copyfn << "aftercopyconstructor" << *(c.envUsers) << ".lp";
        const string copyfns = copyfn.str();
        
        writeLp(copyfns.c_str());
    }
}


MILPSolverCPX::~MILPSolverCPX()
{
    delete cplex;
    delete obj;
    delete modelcon;
    delete modelvar;
    delete model;
    
    if (!(--(*envUsers))) {
        env->end();
        delete env;
        delete envUsers;
    }
    
    delete [] solArray;
}

MILPSolver * MILPSolverCPX::clone()
{
    return new MILPSolverCPX(*this);
}

double MILPSolverCPX::getInfinity()
{    
    return IloInfinity;
}

int MILPSolverCPX::getNumCols()
{
    return colCount;
}

int MILPSolverCPX::getNumRows()
{
    return rowCount;
}

void MILPSolverCPX::setColName(const int & var, const string & asString)
{
    string nospaces = asString;
        
    const int len = asString.length();
    
    for (int l = 0; l < len; ++l) {
        if (asString[l] != ' ' && asString[l] != '(' && asString[l] != ')' && asString[l] != '-') {
        } else {
            nospaces[l] = '_';
        }
    }
    
    //std::ostringstream nn;
    //nn << "c" << var;
    
    (*modelvar)[var].setName(nospaces.c_str());
    //(*modelvar)[var].setName(nn.str().c_str());
}

string MILPSolverCPX::getColName(const int & var)
{
    const char * n = (*modelvar)[var].getName();
    if (n) {
        return string(n);
    } else {
        std::ostringstream nn;
        nn << "c" << var;
        return nn.str();
    }
}

void MILPSolverCPX::setRowName(const int & cons, const string & asString)
{
    string nospaces = asString;
    
    const int len = asString.length();
    
    for (int l = 0; l < len; ++l) {
        if (asString[l] != ' ' && asString[l] != '(' && asString[l] != ')' && asString[l] != '-') {
        } else {
            nospaces[l] = '_';
        }
    }
    
    modelcon->data[cons].setName(nospaces.c_str());
}

string MILPSolverCPX::getRowName(const int & cons)
{
    const char * n = modelcon->data[cons].getName();
    if (n) {
        return string(n);
    } else {
        std::ostringstream nn;
        nn << "row" << cons;
        return nn.str();
    }
}
/*
void MILPSolverCPX::setInteger(const int & var)
{
    model->add(IloConversion(*env, (*modelvar)[var], ILOINT));
    integervars.insert(var);
}*/

double MILPSolverCPX::getColUpper(const int & var)
{
    return (*modelvar)[var].getUB();
}

void MILPSolverCPX::setColUpper(const int & var, const double & b)
{
    (*modelvar)[var].setUB(b);    
}

double MILPSolverCPX::getColLower(const int & var)
{
    return (*modelvar)[var].getLB();
}

void MILPSolverCPX::setColLower(const int & var, const double & b)
{
    (*modelvar)[var].setLB(b);    
}

void MILPSolverCPX::setColBounds(const int & var, const double & lb, const double & ub)
{
    (*modelvar)[var].setBounds(lb,ub);
}


void MILPSolverCPX::setRowUpper(const int & r, const double & b)
{
    (modelcon->data)[r].setUB(b);
}

void MILPSolverCPX::setRowLower(const int & r, const double & b)
{
    (modelcon->data)[r].setLB(b);
}

double MILPSolverCPX::getRowUpper(const int & r)
{
    return (modelcon->data)[r].getUB();
}

double MILPSolverCPX::getRowLower(const int & r)
{
    return (modelcon->data)[r].getLB();
}

bool MILPSolverCPX::isColumnInteger(const int & col)
{
    return (integervars.find(col) != integervars.end());
}

bool MILPSolverCPX::isColumnBinary(const int & col)
{
    map<int,bool>::const_iterator cItr = integervars.find(col);
    if (cItr == integervars.end()) return false;
    
    return (cItr->second);
}


void MILPSolverCPX::addCol(const vector<pair<int,double> > & entries, const double & lb, const double & ub, const ColumnType & type)
{
    static const bool debug = false;
    
    if (debug) {
        cout << "Adding column to LP: ";
    }
    
    if (type == C_BOOL) {
        (*modelvar).add(IloNumVar(*env, lb, ub, ILOBOOL));
        //cout << "Adding column " << colCount << " as binary\n";
        integervars.insert(make_pair(colCount, true));
    } else if (type == C_INT) {
        (*modelvar).add(IloNumVar(*env, lb, ub, ILOINT));
        //cout << "Adding column " << colCount << " as an integer\n";
        integervars.insert(make_pair(colCount, false));
    } else {
        (*modelvar).add(IloNumVar(*env, lb, ub));
    }
        
    const int entCount = entries.size();
    for (int i = 0; i < entCount; ++i) {
        IloRange & target = modelcon->data[entries[i].first];
        target.setLinearCoef((*modelvar)[colCount], entries[i].second);
        coeffs[entries[i].first][colCount] = entries[i].second;
        if (debug) {
            if (i) cout << " + ";
            cout << entries[i].second << "*" << getRowName(entries[i].first);
        }

    }
    
            
    if (debug) {
        if (lb == -IloInfinity) {
            cout << " <= " << ub << "\n";
        } else if (ub == IloInfinity) {
            cout << " >= " << lb << "\n";
        } else if (ub == lb) {
            cout << " == " << ub << "\n";
        } else {
            cout << " in [" << lb << "," << ub << "]\n";            
        }
    }
                
    ++colCount;
}

void MILPSolverCPX::addRow(const vector<pair<int,double> > & entries, const double & lb, const double & ub)
{
    static const bool debug = false;
    
    if (debug) cout << "Adding row to LP: ";
    
    IloRange & newRange = modelcon->data[rowCount] = IloAdd(*model, IloRange(*env, lb,ub));
    
    map<int,double> & dest = coeffs[rowCount];
    
    const int entCount = entries.size();
    for (int i = 0; i < entCount; ++i) {
        newRange.setLinearCoef((*modelvar)[entries[i].first], entries[i].second);
        dest[entries[i].first] = entries[i].second;
        
        if (debug) {
            if (i) cout << " + ";
            cout << entries[i].second << "*" << entries[i].first;
        }
       
    }
    
    if (debug) {
        if (lb == -IloInfinity) {
            cout << " <= " << ub << "\n";
        } else if (ub == IloInfinity) {
            cout << " >= " << lb << "\n";
        } else if (ub == lb) {
            cout << " == " << ub << "\n";
        } else {
            cout << " in [" << lb << "," << ub << "]\n";            
        }
    }
    ++rowCount;    
    
}

void MILPSolverCPX::setMaximiseObjective(const bool & maxim)
{
    if (maxim) {
        obj->setSense(IloObjective::Maximize);
    } else {
        obj->setSense(IloObjective::Minimize);
    }
}


void MILPSolverCPX::setObjective(double * const entries)
{    
    for (int i = 0; i < colCount; ++i) {
        obj->setLinearCoef((*modelvar)[i], entries[i]);
    }
}


void MILPSolverCPX::setObjCoeff(const int & var, const double & w)
{
    obj->setLinearCoef((*modelvar)[var], w);
}

void MILPSolverCPX::clearObjective()
{
    const IloObjective::Sense direction = obj->getSense();
    
    model->remove(*obj);
    delete obj;
    
    obj = new IloObjective(*env);
    obj->setSense(direction);
    
    model->add(*obj);
}


void MILPSolverCPX::writeLp(const string & filename)
{
    cplex->exportModel(filename.c_str());
}

bool MILPSolverCPX::solve(const bool & skipPresolve)
{
    if (skipPresolve) {
        cplex->setParam(IloCplex::PreInd, 0);
    } else {
        cplex->setParam(IloCplex::PreInd, 1);
    }
    
    //cout << "Solving problem with " << cplex->getNcols() << " columns and " << cplex->getNrows() << " rows\n";
    bool toReturn = false;
    try {
        toReturn = cplex->solve() == IloTrue;
    }
    catch (IloCplex::Exception e) {
        cerr << "Error calling CPLEX: ";
        e.print(cerr);
        exit(1);        
    }
 
    return toReturn;
}

const double * MILPSolverCPX::getSolution()
{
    delete [] solArray;
    solArray = new double[colCount];
    
    for (int i = 0; i < colCount; ++i) {
        try {
            IloNum v = cplex->getValue((*modelvar)[i]);
            solArray[i] = v;
        }
        catch (IloAlgorithm::NotExtractedException e) {
            solArray[i] = (*modelvar)[i].getLB();
        }
    }
    
    return solArray;
}

const double * MILPSolverCPX::getSolutionRows()
{
    delete [] solArray;
    solArray = new double[rowCount];
    
    for (int i = 0; i < rowCount; ++i) {
        try {
            IloNum v = cplex->getValue((modelcon->data)[i]);
            solArray[i] = v;
        }
        catch (IloAlgorithm::NotExtractedException e) {
            solArray[i] = (modelcon->data)[i].getLB();
        }
    }
        
    return solArray;
}

const double * MILPSolverCPX::getPartialSolution(const int & from, const int & to)
{
    delete [] solArray;
    
    if (from == to) {
        solArray = 0;       
    } else {
        solArray = new double[to - from];
        
        for (int i = from; i < to; ++i) {
            try {
                IloNum v = cplex->getValue((*modelvar)[i]);
                solArray[i-from] = v;
            }
            catch (IloAlgorithm::NotExtractedException e) {
                solArray[i-from] = (*modelvar)[i].getLB();
            }
        }
    }
        
    return solArray;
}
    
double MILPSolverCPX::getSingleSolutionVariableValue(const int & col)
{
    try {
        IloNum v = cplex->getValue((*modelvar)[col]);
        return v;
    }
    catch (IloAlgorithm::NotExtractedException e) {
        return (*modelvar)[col].getLB();
    }
}

double MILPSolverCPX::getObjValue()
{
    return cplex->getObjValue();
}

void MILPSolverCPX::getRow(const int & i, vector<pair<int,double> > & entries)
{
    const int colCount = getNumCols();
    
    IloRange & currRow = (modelcon->data)[i];
    
    IloNumExprArg ne = currRow.getExpr();
    
    for(IloExpr::LinearIterator itr = static_cast<IloExpr>(ne).getLinearIterator(); itr.ok(); ++itr) {
        const int colID = itr.getVar().getId();
        for (int c = 0; c < colCount; ++c) {
            if ((*modelvar)[c].getId() == colID) {
                entries.push_back(make_pair(c, itr.getCoef()));
                break;
            }
        }        
    }    
    
}

bool MILPSolverCPX::supportsQuadratic() const
{
    return true;
}


void MILPSolverCPX::hush()
{
    env->setOut(env->getNullStream());
    env->setWarning(env->getNullStream());
    env->setError(env->getNullStream());
    cplex->setOut(env->getNullStream());
    cplex->setWarning(env->getNullStream());
    cplex->setError(env->getNullStream());
}

        
MILPSolver * getNewSolver()
{
    MILPSolver * const toReturn = new MILPSolverCPX();
    return toReturn;
}

const double LPinfinity = IloInfinity;

