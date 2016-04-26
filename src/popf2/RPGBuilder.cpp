#include "RPGBuilder.h"

#include "globals.h"

#include "ptree.h"
#include <FlexLexer.h>
#include "instantiation.h"
#include "SimpleEval.h"
#include "DebugWriteController.h"
#include "typecheck.h"
#include "TIM.h"

#include "FuncAnalysis.h"

//#include "graphconstruct.h"
//#include "PartialPlan.h"

#include "FFSolver.h"

#include <assert.h>

#include <algorithm>
//#include "MILPRPG.h"

#include <sstream>
#include <fstream>

#include "numericanalysis.h"
#include "temporalanalysis.h"

#ifdef STOCHASTICDURATIONS
#include "StochasticDurations.h"
#endif

using namespace TIM;
using namespace Inst;
using namespace VAL;

using std::cerr;
using std::ostringstream;
using std::endl;
using std::ifstream;

namespace Planner
{

whereAreWe WhereAreWeNow = PARSE_UNKNOWN;

ostream & operator << (ostream & o, const whereAreWe & w)
{
    switch (w) {
    case(PARSE_PRECONDITION):
        o << "As a precondition";
        break;
    case(PARSE_EFFECT):
        o << "As an effect";
        break;
    case(PARSE_DURATION):
        o << "As a duration constraint";
        break;
    case(PARSE_GOAL):
        o << "In the problem goals";
        break;
    case(PARSE_INITIAL):
        o << "In the initial state";
        break;
    case(PARSE_CONDITIONALEFFECT):
        o << "In a conditional effect";
        break;
    case(PARSE_CONTINUOUSEFFECT):
        o << "In a continuous effect";
        break;
    case(PARSE_METRIC):
        o << "In the problem metric";
        break;
    case(PARSE_CONSTRAINTS) :
        o << "In the problem constraints/preferences";
        break;
    default:
        o << "At some point";
        break;
    };
    return o;
};

int ActionSegment::tilLimit = 0;


bool RPGBuilder::modifiedRPG = true;
bool RPGBuilder::fullFFHelpfulActions = false;
bool RPGBuilder::sortedExpansion = false;
bool RPGBuilder::noSelfOverlaps = false;
bool RPGBuilder::doTemporalAnalysis = true;
bool RPGBuilder::doSkipAnalysis = true;

RPGHeuristic * RPGBuilder::globalHeuristic = 0;

class GoalNumericCollector : public VisitController
{

private:

    list<RPGBuilder::NumericPrecondition> * numericGoals;
    list<double> * numericGoalDeadlines;
    list<Literal*> * literalGoals;
    list<double> * literalGoalDeadlines;

    list<RPGBuilder::NumericPrecondition> * numToUse;
    list<double> * numDToUse;
    list<Literal*> * litToUse;
    list<double> * litDToUse;

    map<string, int> & prefNameToID;

    VAL::TypeChecker * tc;
    bool adding;
    const VAL::operator_ * op;
    FastEnvironment * fe;

    bool inpres;
    bool checkpos;
    bool inConstraints;
    bool inPreference;

    double currentDeadline;
    
public:

    list<RPGBuilder::Constraint> builtPreferences;
    list<RPGBuilder::Constraint> builtConstraints;

    GoalNumericCollector(list<RPGBuilder::NumericPrecondition> * ng, list<double> * ngD,
                         list<Literal*> * g, list<double> * gD,
                         map<string, int> & p, const VAL::operator_ * o, FastEnvironment * f, VAL::TypeChecker * t = 0) :
            numericGoals(ng), numericGoalDeadlines(ngD), literalGoals(g), literalGoalDeadlines(gD),
            numToUse(numericGoals), numDToUse(numericGoalDeadlines), litToUse(literalGoals), litDToUse(literalGoalDeadlines),
            prefNameToID(p), tc(t), adding(true), op(o), fe(f),
            inpres(true), checkpos(true), inConstraints(false), inPreference(false) {
            
        #ifdef STOCHASTICDURATIONS
        currentDeadline = solutionDeadlineTime;
        #else
        currentDeadline = DBL_MAX;
        #endif
    };


    virtual void visit_comparison(comparison * c) {
        assert(!inPreference || numericGoals != numToUse);
        numToUse->push_back(RPGBuilder::NumericPrecondition(c->getOp(), const_cast<VAL::expression*>(c->getLHS()), const_cast<VAL::expression*>(c->getRHS()), fe, tc));
        if (numDToUse) {
            numDToUse->push_back(currentDeadline);
        }
    };

    virtual void visit_simple_goal(simple_goal * p) {
        assert(!inPreference || literalGoals != litToUse);
        if (VAL::current_analysis->pred_tab.symbol_probe("=") == EPS(p->getProp()->head)->getParent()) {
            Literal tmp(p->getProp(), fe);

            validateLiteral(&tmp);

            VAL::LiteralParameterIterator<VAL::parameter_symbol_list::iterator> tmpBegin = tmp.begin();

            VAL::parameter_symbol * const a = *tmpBegin;
            ++tmpBegin;
            VAL::parameter_symbol * const b = *tmpBegin;

            if (a != b) {
                litToUse->push_back(0);
            }
            return;

        };

        Literal l(p->getProp(), fe);
        validateLiteral(&l);

        Literal* const lookup = instantiatedOp::findLiteral(&l);
        if (lookup) {
            litToUse->push_back(lookup);
            if (litDToUse) {
                litDToUse->push_back(currentDeadline);
            }
        } else {
            cout << "A problem has been encountered, and the problem has been deemed unsolvable\n";
            cout << "--------------------------------------------------------------------------\n";
            cout << "The goal fact:\n" << l << "\n\n";
            cout << "...cannot be found either in the initial state, as an add effect of an\n";
            cout << " action, or as a timed initial literal.  As such, the problem has been deemed\n";
            cout << "unsolvable.\n";
            exit(0);
        }
    };

    virtual void visit_qfied_goal(qfied_goal * p) {
        if (p->getQuantifier() == E_EXISTS) postmortem_noADL();

        vector<vector<VAL::const_symbol*>::const_iterator> vals(p->getVars()->size());
        vector<vector<VAL::const_symbol*>::const_iterator> starts(p->getVars()->size());
        vector<vector<VAL::const_symbol*>::const_iterator> ends(p->getVars()->size());
        vector<VAL::var_symbol *> vars(p->getVars()->size());
        fe->extend(vars.size());
        int i = 0;
        int c = 1;
        for (var_symbol_list::const_iterator pi = p->getVars()->begin();
                pi != p->getVars()->end(); ++pi, ++i) {
            if (instantiatedOp::getValues().find((*pi)->type) == instantiatedOp::getValues().end()) {
                instantiatedOp::getValues()[(*pi)->type] = tc->range(*pi);
            };
            vals[i] = starts[i] = instantiatedOp::getValues()[(*pi)->type].begin();
            ends[i] = instantiatedOp::getValues()[(*pi)->type].end();
            if (ends[i] == starts[i]) {
                return;
            }
            (*fe)[(*pi)] = *(vals[i]);
            vars[i] = *pi;
            c *= instantiatedOp::getValues()[(*pi)->type].size();
        };

        --i;
        while (vals[i] != ends[i]) {
// This is inefficient because it creates a copy of the environment even if the copy is never used.
// In practice, this should not be a problem because a quantified effect presumably uses the variables
// it quantifies.
            FastEnvironment * ecpy = fe;
            fe = fe->copy();
            p->getGoal()->visit(this);
            fe = ecpy;

            int x = 0;
            ++vals[0];
            if (vals[0] != ends[0])(*fe)[vars[0]] = *(vals[0]);
            while (x < i && vals[x] == ends[x]) {
                vals[x] = starts[x];
                (*fe)[vars[x]] = *(vals[x]);
                ++x;
                ++vals[x];
                if (vals[x] != ends[x])(*fe)[vars[x]] = *(vals[x]);
            };
        };

    };
    virtual void visit_conj_goal(conj_goal * p) {
        p->getGoals()->visit(this);
    };
    virtual void visit_disj_goal(disj_goal * p) {
        postmortem_noADL();
    };

    virtual void visit_imply_goal(imply_goal * p) {
        postmortem_noADL();
    };
    virtual void visit_neg_goal(neg_goal * p) {
        postmortem_noADL();
    };
    virtual void visit_preference(preference * p) {
        inPreference = true;
        builtPreferences.push_back(RPGBuilder::Constraint(p->getName()));

        p->getGoal()->visit(this);
        inPreference = false;

    };
    virtual void visit_simple_effect(simple_effect * p) {

    };
    virtual void visit_constraint_goal(constraint_goal *cg) {
//  if (!inPreference) {
//      postmortem_noConstraints();
//  }


        list<RPGBuilder::Constraint> & removeFrom = (inPreference ? builtPreferences : builtConstraints);

        if (!inPreference) {
            #ifdef STOCHASTICDURATIONS
            if (cg->getCons() == VAL::E_WITHIN) {
                const double prevDeadline = currentDeadline;
                currentDeadline = cg->getDeadline();
                cg->getRequirement()->visit(this);
                currentDeadline = prevDeadline;
                return;
            }
            #endif
            builtConstraints.push_back(RPGBuilder::Constraint());
        }

        removeFrom.back().cons = cg->getCons();
        removeFrom.back().deadline = cg->getDeadline();
        removeFrom.back().from = cg->getFrom();

        bool removed = false;

        if (inPreference) {
            if (removeFrom.back().cons != E_ALWAYS && removeFrom.back().cons != E_SOMETIME && removeFrom.back().cons != E_ATMOSTONCE) {

                string & prefName = removeFrom.back().name;
                postmortem_noConstraints(true, prefName.c_str());

                if (prefName != "anonymous") prefNameToID.insert(make_pair(prefName, -1));

                removeFrom.pop_back();
                removed = true;
            }
        } else {
            if (removeFrom.back().cons != E_ALWAYS) {

                postmortem_noConstraints(false);
                removeFrom.pop_back();
                removed = true;
            }
        }

        if (!removed) {

            if (cg->getRequirement()) {
                numToUse = &(removeFrom.back().goalNum);
                litToUse = &(removeFrom.back().goal);
                
                numDToUse = 0;
                litDToUse = 0;

                cg->getRequirement()->visit(this);
            };
            if (cg->getTrigger()) {
                numToUse = &(removeFrom.back().triggerNum);
                litToUse = &(removeFrom.back().trigger);
                
                numDToUse = 0;
                litDToUse = 0;
                
                cg->getTrigger()->visit(this);
            };
        }

        numToUse = numericGoals;
        litToUse = literalGoals;

    };


    virtual void visit_problem(VAL::problem * p) {

        WhereAreWeNow = PARSE_GOAL;
        inpres = false;

        numToUse = numericGoals;
        litToUse = literalGoals;
        if (p->the_goal) {
            p->the_goal->visit(this);
        }
        WhereAreWeNow = PARSE_UNKNOWN;
        if (p->constraints) {
            WhereAreWeNow = PARSE_CONSTRAINTS;
            inConstraints = true;
            p->constraints->visit(this);
            inConstraints = false;
            WhereAreWeNow = PARSE_UNKNOWN;
        }
        numToUse = numericGoals;
        litToUse = literalGoals;

    };


};


void postmortem_noNestedWhens()
{

    cerr << "A problem has been encountered, and the planner has to terminate.\n";
    cerr << "-----------------------------------------------------------------\n";
    cerr << "Unfortunately, at present, the planner does not supported nested (when (...\n";
    cerr << "conditional effects, but they are present in the problem you have provided.\n\n";
    cerr << "To use this planner with your problem, you will have to reformulate it to\n";
    cerr << "avoid these.  In the case of nested whens, one can rewrite:\n\n";
    cerr << "(when (x)\n";
    cerr << "         (and  (when (y) ...\n";
    cerr << "         (and  (when (z) ...\n";
    cerr << "\nas:\n";
    cerr << "(when (and (x) (y))\n";
    cerr << "           ...       )\n";
    cerr << "(when (and (x) (z))\n";
    cerr << "           ...       )\n\n";
    cerr << "Alternatively, dive into the source code yourself or contact the authors,\n";
    cerr << "who may be able to help.  Apologies, and best of luck with your task.\n";

    exit(0);
};

void postmortem_noADL()
{

    cerr << "A problem has been encountered, and the planner has to terminate.\n";
    cerr << "-----------------------------------------------------------------\n";
    cerr << "Unfortunately, at present, the planner does not fully support ADL\n";
    cerr << "unless in the rules for derived predicates.  Only two aspects of\n";
    cerr << "ADL can be used in action definitions:\n";
    cerr << "- forall conditions, containing a simple conjunct of propositional and\n";
    cerr << "  numeric facts;\n";
    cerr << "- Conditional (when... ) effects, and then only with numeric conditions\n";
    cerr << "  and numeric consequences on values which do not appear in the\n";
    cerr << "  preconditions of actions.\n\n";
    cerr << "To use this planner with your problem, you will have to reformulate it to\n";
    cerr << "avoid ADL.  Alternatively, if you have a particularly compelling case\n";
    cerr << "for them, please contact the authors to discuss it with them, who may be able to\n";
    cerr << "extend the planner to meet your needs.\n";

    exit(0);
};

void postmortem_noConstraints(const bool unsupportedPref, const char * n)
{

    if (unsupportedPref) {
        cerr << "Warning - Unsupported Preference Type\n";
        cerr << "-------------------------------------\n";
        cerr << "Unfortunately, at present, the planner does not fully support\n";
        cerr << "PDDL 3 preferences - only 'always', 'atsometime' and 'atmostonce'\nare supported.\n\n";
        cerr << "The planner will continue, but will quietly ignore the preference\nnamed " << n << ".\n\n";
    } else {
        static bool warned = false;
        if (!warned) {
            cerr << "Warning - Unsupported Trajectory Constraint Type\n";
            cerr << "------------------------------------------------\n";
            cerr << "Unfortunately, at present, the planner does not fully support\n";
            cerr << "PDDL 3 constraints - only 'always' is supported.\n\n";
            cerr << "The planner will continue, but will quietly ignore one or more constraints - as\nsuch, the plan it produces may be invalid.\n\n";
            warned = true;
        }

    }
};

void postmortem_isViolatedNotExist(const string & n)
{
    cerr << "A problem has been encountered, and the planner has to terminate.\n";
    cerr << "-----------------------------------------------------------------\n";
    cerr << "An error has been encountered in your metric expression.  The preference:\n\n";
    cerr << "\t" << n << "\n\n";
    cerr << "... does not exist, but the metric contains the expression:\n\n";
    cerr << "\t(is-violated " << n << ")\n\n";
    exit(0);
};

void postmortem_fatalConstraint(const string & whichOne)
{

    cerr << "A problem has been encountered, and the planner has to terminate.\n";
    cerr << "-----------------------------------------------------------------\n";
    cerr << "An error has been encountered in your domain.  The term:\n\n";
    cerr << whichOne;
    cerr << "...appearing as part of a constraint is always false, so no\n";
    cerr << "valid solution plan can ever be found.\n";
    exit(0);

}

void postmortem_twoSimulataneousNumericEffectsOnTheSameVariable(const string & actName, const string & varName)
{
    cerr << "A problem has been encountered, and the planner has to terminate.\n";
    cerr << "-----------------------------------------------------------------\n";
    cerr << "An error has been encountered in the domain.  In the action:\n\n";
    cerr << "\t" << actName << "\n\n";
    cerr << "... there are two simultaneous effects on " << varName << ".\n";
    exit(0);
}

void postmortem_nonLinearCTS(const string & actName, const string & worksOutAs)
{
    cerr << "A problem has been encountered, and the planner has to terminate.\n";
    cerr << "-----------------------------------------------------------------\n";
    cerr << "Unfortunately, the planner only supports continuous effects where the gradient\n";
    cerr << "is state-independent, i.e. evaluates to a single value.  In the action:\n\n";
    cerr << "\t" << actName << "\n\n";
    cerr << "... a continuous effect was encountered that amounts to:\n";
    cerr << "\t" << worksOutAs << "\n\n";
    cerr << "To use this planner with your problem, you will have to reformulate it to\n";
    cerr << "avoid these.  Alternatively, if you have an interesting application for them,\n";
    cerr << "please contact the authors to discuss it with them, who may be able to\n";
    cerr << "extend the planner to meet your needs.\n";

    exit(0);
};

void postmortem_noQuadratic(const string & theOp)
{

    cerr << "A problem has been encountered, and the planner has to terminate.\n";
    cerr << "-----------------------------------------------------------------\n";
    cerr << "Unfortunately, the planner does not supported non-linear numeric conditions,\n";
    cerr << "effects, or duration constraints, but one or more of these is present in\n";
    cerr << "the problem you have provided.  Specifically, the sub-expression:\n\n";
    cerr << "\t" << theOp << "\n\n";
    cerr << "... was encountered.  To use this planner with your problem, you will have\n";
    cerr << "to reformulate it to avoid these.\n";

    exit(0);
};

void postmortem_noTimeSpecifierOnAPropPrecondition(const string & actname, const string & effect)
{
    cerr << "A problem has been encountered with your domain/problem file.\n";
    cerr << "-------------------------------------------------------------\n";
    cerr << "Unfortunately, a bug has been encountered in your domain and problem file,\n";
    cerr << "and the planner has to terminate.  The durative action:\n\n";
    cerr << "\t" << actname << "\n";
    cerr << "has a propositional precondition:\n\n";
    cerr << "\t" << effect << "\n\n";
    cerr << "As it's a durative action, you need to give a time specifier - either\n";
    cerr << "(at start ...), (over all...) or (at end ...) - to indicate when the fact is\n";
    cerr << "to hold during the execution of the action.  For instance:\n";
    cerr << "\t (at start " << effect << ")\n";

    exit(0);
};


void postmortem_fixedAndNotTimeSpecifiers(const string & actname, const bool & multipleEquals)
{
    cerr << "A problem has been encountered with your domain/problem file.\n";
    cerr << "-------------------------------------------------------------\n";
    cerr << "Unfortunately, a bug has been encountered in your domain and problem file,\n";
    cerr << "and the planner has to terminate.  The durative action:\n\n";
    cerr << "\t" << actname << "\n";
    if (multipleEquals) {
        cerr << "has multiple constraints of the form (= ?duration ...).  Actions with fixed durations\n";
        cerr << "can only have one constraint governing their duration.\n";
    } else {
        cerr << "has both duration constraints of the form (= ?duration ...), and those specifying\n";
        cerr << "maximum and/or minimum values.\n";
    }

    exit(0);
};

void postmortem_noTimeSpecifierOnAPropEffect(const string & actname, const string & effect)
{
    cerr << "A problem has been encountered with your domain/problem file.\n";
    cerr << "-------------------------------------------------------------\n";
    cerr << "Unfortunately, a bug has been encountered in your domain and problem file,\n";
    cerr << "and the planner has to terminate.  The durative action:\n\n";
    cerr << "\t" << actname << "\n";
    cerr << "has a propositional effect:\n\n";
    cerr << "\t" << effect << "\n\n";
    cerr << "As it's a durative action, you need to give a time specifier - either\n";
    cerr << "(at start ...) or (at end ...) - to indicate when the effect is to\n";
    cerr << "occur when the action is executed, for instance:\n";
    cerr << "\t (at start " << effect << ")\n";

    exit(0);
};

void postmortem_noTimeSpecifierOnInstantNumericEffect(const string & actname, const string & effect, const string & suggested, const bool & isAssign)
{
    cerr << "A problem has been encountered with your domain/problem file.\n";
    cerr << "-------------------------------------------------------------\n";
    cerr << "Unfortunately, a bug has been encountered in your domain and problem file,\n";
    cerr << "and the planner has to terminate.  The durative action:\n\n";
    cerr << "\t" << actname << "\n\n";
    cerr << "has a numeric effect:\n\n";
    cerr << "\t" << effect << "\n\n";
    cerr << "As it's a durative action, and the effect is not continuous across\n";
    cerr << "the duration of the action (due to #t not being present), you may wish\n";
    cerr << "do one of two things:\n\n";
    cerr << " (i) Give a time specifier - either (at start ...) or (at end ...) - to\n";
    cerr << "     indicate when the effect is to occur when the action is executed, e.g:\n\n";
    cerr << "\t (at start " << effect << ")\n\n";

    if (isAssign) {

        cerr << "(ii) If the effect is meant to be continuous, first note that continuous\n";
        cerr << "     effects cannot be assignments - they have to be increase or decrease\n";
        cerr << "     effects.  Once you have resolved this, include #t where desired.\n";
        cerr << "     For instance, you may have meant:\n";

        cerr << "\t" << suggested << "\n";


    } else {

        cerr << "(ii) If the effect is meant to be continuous, rewrite it to signify\n";
        cerr << "     this by including #t where desired.  For instance, you may have meant:\n\n";
        cerr << "\t" << suggested << "\n";

    }
    exit(0);
};

void postmortem_wrongNumberOfFluentArguments(const string & actname, const bool & haveActName, const whereAreWe & w, const string & predname, const string & lit, const int & givenArgs, const set<int> & realargs)
{
    cerr << "A problem has been encountered with your domain/problem file.\n";
    cerr << "-------------------------------------------------------------\n";
    cerr << "Unfortunately, a bug has been encountered in your domain and problem file,\n";
    cerr << "and the planner has to terminate.  " << w << " ";
    if (haveActName) cerr << "within the action:\n\n\t" << actname << "\n\n";
    cerr << "the invalid functional value:\n\n";
    cerr << "\t" << lit << "\n\n";
    cerr << "is defined. '" << predname << "' cannot take " << givenArgs;
    if (givenArgs != 1) {
        cerr << " arguments";
    } else {
        cerr << " argument";
    }
    cerr << ", it can only take ";
    if (realargs.size() == 1) {
        const int ra = *(realargs.begin());
        cerr << ra << ".\n";

    } else {
        cerr << "either ";
        set<int>::iterator raItr = realargs.begin();
        const set<int>::iterator raEnd = realargs.end();
        int lastPrint = -1;
        for (; raItr != raEnd; ++raItr) {
            if (lastPrint != -1) cerr << lastPrint << ", ";
            lastPrint = *raItr;
        }
        cerr << "or " << lastPrint << ".\n";
    }
    exit(0);

}

void postmortem_wrongNumberOfPredicateArguments(const string & actname, const bool & haveActName, const whereAreWe & w, const string & predname, const string & lit, const int & givenargs, const set<int> & realargs)
{
    cerr << "A problem has been encountered with your domain/problem file.\n";
    cerr << "-------------------------------------------------------------\n";
    cerr << "Unfortunately, a bug has been encountered in your domain and problem file,\n";
    cerr << "and the planner has to terminate.   " << w << " ";
    if (haveActName) cerr << "within the action:\n\n\t" << actname << "\n";
    cerr << "the invalid proposition:\n\n";
    cerr << "\t" << lit << "\n\n";
    cerr << "is defined. '" << predname << "' cannot take " << givenargs;
    if (givenargs != 1) {
        cerr << " arguments";
    } else {
        cerr << " argument";
    }
    cerr << ", it can only take ";
    if (realargs.size() == 1) {
        const int ra = *(realargs.begin());
        cerr << ra << ".\n";

    } else {
        cerr << "either ";
        set<int>::iterator raItr = realargs.begin();
        const set<int>::iterator raEnd = realargs.end();
        int lastPrint = -1;
        for (; raItr != raEnd; ++raItr) {
            if (lastPrint != -1) cerr << lastPrint << ", ";
            lastPrint = *raItr;
        }
        cerr << "or " << lastPrint << ".\n";
    }

    exit(0);
};


void postmortem_mathsError(const string & description, const string & help, const whereAreWe & w)
{
    cerr << "A maths error has been encountered in your domain/problem file.\n";
    cerr << "---------------------------------------------------------------\n";
    cerr << "Unfortunately, a maths error has been encountered in domain/problem,\n";
    cerr << "and the planner has to terminate.   " << w << " a\n";
    cerr << description << " occurred.\n";
    cerr << help;
    exit(0);
}



class ExpressionPrinter: public VisitController
{

private:

    VAL::TypeChecker * tc;
    VAL::FastEnvironment * fe;
    ostream & o;
public:

    ExpressionPrinter(ostream & oIn, VAL::FastEnvironment * f, VAL::TypeChecker * t = 0) :
            tc(t), fe(f), o(oIn) {};

    void printFormula(VAL::expression * e) {
        e->visit(this);
    }

    void visit_plus_expression(const plus_expression * s) {
        o << "(+ ";
        s->getLHS()->visit(this);
        o << " ";
        s->getRHS()->visit(this);
        o << ")";
    }

    void visit_minus_expression(const minus_expression * s) {
        o << "(- ";
        s->getLHS()->visit(this);
        o << " ";
        s->getRHS()->visit(this);
        o << ")";
    }
    void visit_mul_expression(const mul_expression * s) {
        o << "(* ";
        s->getLHS()->visit(this);
        o << " ";
        s->getRHS()->visit(this);
        o << ")";
    }
    void visit_div_expression(const div_expression * s) {
        o << "(/ ";
        s->getLHS()->visit(this);
        o << " ";
        s->getRHS()->visit(this);
        o << ")";
    }

    void visit_uminus_expression(const uminus_expression * s) {
        o << "(- ";
        s->getExpr()->visit(this);
        o << ")";
    }
    void visit_int_expression(const int_expression * s) {
        o << s->double_value();
    }
    void visit_float_expression(const float_expression * s) {
        o << s->double_value();
    };

    void visit_special_val_expr(const special_val_expr * v) {
        if (v->getKind() == E_HASHT) {
            o << "#t";
        } else if (v->getKind() == E_DURATION_VAR) {
            o << "?duration";
        } else if (v->getKind() == E_TOTAL_TIME) {
            o << "total-time";
        }
    };


    void visit_func_term(const func_term * s) {
        PNE * const lookupPNE = new PNE(s, fe);
        //cout << "Looking up " << *lookupPNE << "\n";
        o << *lookupPNE;
    };

    void visit_violation_term(const violation_term * s) {
        o << "(is-violated " << s->getName() << ")";
    };

};


class TimedPrecEffCollector : public VisitController
{
private:

    instantiatedOp* thisIOP;

    VAL::TypeChecker * tc;
    bool adding;
    const VAL::operator_ * op;
    FastEnvironment * fe;

    bool inpres;
    bool checkpos;
    bool debug;
    bool visitingWhen;

    bool visitingDuration;

    RPGBuilder::NoDuplicatePair addToListPositive;
    RPGBuilder::NoDuplicatePair addToListNegative;
    list<RPGBuilder::NumericPrecondition> * addToListNumeric;

    RPGBuilder::NoDuplicatePair addEffToList;
    RPGBuilder::NoDuplicatePair delEffToList;
    
    /** @brief Where to record numeric effects in <code>visit_assignment</code>.
     * 
     * The pointers are updated according to whether the effect is at the start/end
     * of an action.  The list refers to where the numeric effects themselves are
     * to be stored, and the set contains the indices of the numeric variables which
     * already have effects defined upon them at the given time (to allow actions
     * with self-mutex effects to be discarded).
     */
    pair<list<RPGBuilder::NumericEffect>*, set<int>* > addEffToListNumeric;
    

    list<vector<RPGBuilder::NoDuplicatePair > > literalAddTos;
    list<vector<RPGBuilder::NoDuplicatePair > > literalNegativeAddTos;
    list<vector<list<RPGBuilder::NumericPrecondition>* > > numericAddTos;


public:

    list<RPGBuilder::ProtoConditionalEffect*> condEffs;

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
    set<int> startNumericEffsOnVar;

    list<Literal*> endAddEff;
    LiteralSet endAddEffSet;
    list<Literal*> endDelEff;
    LiteralSet endDelEffSet;
    list<RPGBuilder::NumericEffect> endNumericEff;
    set<int> endNumericEffsOnVar;

    list<RPGBuilder::NumericPrecondition *> fixedDurationExpression;
    list<RPGBuilder::NumericPrecondition *> minDurationExpression;
    list<RPGBuilder::NumericPrecondition *> maxDurationExpression;

    bool isDurative;

    static map<string, set<int> > litParamCounts;
    static map<string, set<int> > pneParamCounts;
    static bool initParamCounts;

    static instantiatedOp* toBlame;

    TimedPrecEffCollector(instantiatedOp* currIOP, const VAL::operator_ * o, FastEnvironment * f, VAL::TypeChecker * t = 0) :
            thisIOP(currIOP), tc(t), adding(true), op(o), fe(f), inpres(true), checkpos(true), visitingWhen(false), visitingDuration(false),
            addToListNumeric(0), isDurative(false) {
        debug = (Globals::globalVerbosity & 16);
        addEffToListNumeric.first = 0;
        addEffToListNumeric.second = 0;
        if (!initParamCounts) {
            doInit();
        }
    };

    static void doInit() {
        if (current_analysis->the_domain->predicates) {
            for (pred_decl_list::const_iterator os = current_analysis->the_domain->predicates->begin();
                    os != current_analysis->the_domain->predicates->end(); ++os) {
                litParamCounts[(*os)->getPred()->getName()].insert((*os)->getArgs()->size());
            }
        }
        if (current_analysis->the_domain->functions) {
            for (func_decl_list::const_iterator os = current_analysis->the_domain->functions->begin();
                    os != current_analysis->the_domain->functions->end(); ++os) {
                pneParamCounts[(*os)->getFunction()->getName()].insert((*os)->getArgs()->size());
            }
        }
        initParamCounts = true;
    }

    virtual void visit_simple_goal(simple_goal * p) {

        RPGBuilder::NoDuplicatePair & addToList = (adding ? addToListPositive : addToListNegative);
        if (!addToList) {
            Literal l(p->getProp(), fe);
            string actionname;
            string effectdescription;

            {
                ostringstream o;
                o << *thisIOP;
                actionname = o.str();
            }

            {
                ostringstream o;
                if (!adding) o << "(not ";
                o << l;
                if (!adding) o << ")";
                effectdescription = o.str();
            }
            postmortem_noTimeSpecifierOnAPropPrecondition(actionname, effectdescription);

        }

        if (VAL::current_analysis->pred_tab.symbol_probe("=") == EPS(p->getProp()->head)->getParent()) {
            Literal tmp(p->getProp(), fe);

            VAL::LiteralParameterIterator<VAL::parameter_symbol_list::iterator> tmpBegin = tmp.begin();

            VAL::parameter_symbol * const a = *(tmpBegin);
            ++tmpBegin;
            VAL::parameter_symbol * const b = *(tmpBegin);

            if (adding) {
                if (a != b) {
                    addToList.push_back((Literal*) 0);
                }
            } else {
                if (a == b) {
                    addToList.push_back((Literal*) 0);
                }
            }

            return;
        };


        Literal * l = new Literal(p->getProp(), fe);
        validateLiteral(l);

        if (debug) {
            if (adding) {
                cout << "- Looking up " << *l << "\n";
            } else {
                cout << "- Looking up ¬" << *l << "\n";
            }
        }
        Literal * const addLit = instantiatedOp::findLiteral(l);
        if (!addLit) {
            if (adding) {
                addToList.push_back(addLit);
                if (debug) cout << "\t\tNull\n";
            } else {
                addToList.push_back(addLit);
                if (debug) cout << "\t\tNull, but don't care - is a negative pre\n";
            }
        } else {
            addToList.push_back(addLit);                
        }
        delete l;
        //cout << "Got " << *(addToList->back()) << " with ID " << addToList->back()->getID() << "\n";
        //assert(addToList->back()->getID() >= 0);
    };
    virtual void visit_comparison(comparison * c) {
        if (visitingDuration) {
            list<RPGBuilder::NumericPrecondition*> * assignTo = 0;

            if (c->getOp() == E_EQUALS) {
                if (!fixedDurationExpression.empty()) {
                    ostringstream o;
                    o << *(toBlame);
                    const string actname = o.str();
                    postmortem_fixedAndNotTimeSpecifiers(actname, true);
                }
                assignTo = &fixedDurationExpression;
                if (!minDurationExpression.empty() || !maxDurationExpression.empty()) {
                    ostringstream o;
                    o << *(toBlame);
                    const string actname = o.str();
                    postmortem_fixedAndNotTimeSpecifiers(actname, false);
                }
            } else if (c->getOp() == E_GREATER || c->getOp() == E_GREATEQ) {
//              cout << "### Found a minimum duration for the action\n";
                assignTo = &minDurationExpression;
                if (!fixedDurationExpression.empty()) {
                    ostringstream o;
                    o << *(toBlame);
                    const string actname = o.str();
                    postmortem_fixedAndNotTimeSpecifiers(actname, false);
                }
            } else if (c->getOp() == E_LESS || c->getOp() == E_LESSEQ) {
                assignTo = &maxDurationExpression;
                if (!fixedDurationExpression.empty()) {
                    ostringstream o;
                    o << *(toBlame);
                    const string actname = o.str();
                    postmortem_fixedAndNotTimeSpecifiers(actname, false);
                }

//              cout << "### Found a maximum duration for the action\n";
            }

            if (assignTo == &fixedDurationExpression) {
//              cout << "### Found a fixed duration for the action\n";
            }

            //assert(!(*assignTo)); // double durations should never ever be defined!
            RPGBuilder::NumericPrecondition * const newPre = new RPGBuilder::NumericPrecondition(c->getOp(), const_cast<VAL::expression*>(c->getLHS()), const_cast<VAL::expression*>(c->getRHS()), fe, tc, adding);
            if (newPre->valid) {
                assignTo->push_back(newPre);
                if (debug) {
                    cout << "\tDuration: " << *newPre << endl;
                }
            } else {
                delete newPre;
                assignTo->push_back(0);
            }

        } else {
            addToListNumeric->push_back(RPGBuilder::NumericPrecondition(c->getOp(), const_cast<VAL::expression*>(c->getLHS()), const_cast<VAL::expression*>(c->getRHS()), fe, tc, adding));
        }
    };

    virtual void visit_qfied_goal(qfied_goal * p) {
        if (p->getQuantifier() == E_EXISTS) postmortem_noADL();

        vector<vector<VAL::const_symbol*>::const_iterator> vals(p->getVars()->size());
        vector<vector<VAL::const_symbol*>::const_iterator> starts(p->getVars()->size());
        vector<vector<VAL::const_symbol*>::const_iterator> ends(p->getVars()->size());
        vector<VAL::var_symbol *> vars(p->getVars()->size());
        fe->extend(vars.size());
        int i = 0;
        int c = 1;
        for (var_symbol_list::const_iterator pi = p->getVars()->begin();
                pi != p->getVars()->end(); ++pi, ++i) {
            if (instantiatedOp::getValues().find((*pi)->type) == instantiatedOp::getValues().end()) {
                instantiatedOp::getValues()[(*pi)->type] = tc->range(*pi);
            };
            vals[i] = starts[i] = instantiatedOp::getValues()[(*pi)->type].begin();
            ends[i] = instantiatedOp::getValues()[(*pi)->type].end();
            if (ends[i] == starts[i]) {
                return;
            }
            (*fe)[(*pi)] = *(vals[i]);
            vars[i] = *pi;
            c *= instantiatedOp::getValues()[(*pi)->type].size();
        };

        --i;
        while (vals[i] != ends[i]) {
// This is inefficient because it creates a copy of the environment even if the copy is never used.
// In practice, this should not be a problem because a quantified effect presumably uses the variables
// it quantifies.
            FastEnvironment * ecpy = fe;
            fe = fe->copy();
            p->getGoal()->visit(this);
            fe = ecpy;

            int x = 0;
            ++vals[0];
            if (vals[0] != ends[0])(*fe)[vars[0]] = *(vals[0]);
            while (x < i && vals[x] == ends[x]) {
                vals[x] = starts[x];
                (*fe)[vars[x]] = *(vals[x]);
                ++x;
                ++vals[x];
                if (vals[x] != ends[x])(*fe)[vars[x]] = *(vals[x]);
            };
        };

    };
    virtual void visit_disj_goal_internal(const goal_list * gl) {
        assert(adding);

        const RPGBuilder::NoDuplicatePair oldAddToListPositive = addToListPositive;
        const RPGBuilder::NoDuplicatePair oldAddToListNegative = addToListNegative;
        list<RPGBuilder::NumericPrecondition> * const oldAddToListNumeric = addToListNumeric;

        list<Literal*> conds;
        LiteralSet condsSet;
        list<Literal*> negativeConds;
        LiteralSet negativeCondsSet;
        list<RPGBuilder::NumericPrecondition> numericConds;

        addToListPositive = RPGBuilder::NoDuplicatePair(&conds, &condsSet);
        addToListNegative = RPGBuilder::NoDuplicatePair(&negativeConds, &negativeCondsSet);
        addToListNumeric = &numericConds;

        bool tautology = false;

        gl->visit(this);

        {
            list<Literal*>::iterator cItr = conds.begin();
            const list<Literal*>::iterator cEnd = conds.end();

            while (cItr != cEnd) {
                if (*cItr) {
                    if (EPS((*cItr)->getHead())->appearsStatic()) {
                        if (EPS((*cItr)->getHead())->getInitials()->get(fe, (*cItr)->getProp())) {
                            tautology = true;
                            break;
                        } else {
                            const list<Literal*>::iterator cDel = cItr++;
                            conds.erase(cItr);
                        }
                    } else {
                        ++cItr;
                    }
                } else {
                    const list<Literal*>::iterator cDel = cItr++;
                    conds.erase(cItr);
                }
            }
        }

        if (!tautology) {
            list<Literal*>::iterator cItr = negativeConds.begin();
            const list<Literal*>::iterator cEnd = negativeConds.end();

            while (cItr != cEnd) {
                if (!(*cItr)) {
                    tautology = true;
                    break;
                }
                if (EPS((*cItr)->getHead())->appearsStatic()) {
                    if (!(EPS((*cItr)->getHead())->getInitials()->get(fe, (*cItr)->getProp()))) {
                        tautology = true;
                        break;
                    }
                }
                ++cItr;
            }
        }

        if (!tautology) {
            if (conds.size() + negativeConds.size() + numericConds.size() > 1) postmortem_noADL();
        }

        addToListPositive = oldAddToListPositive;
        addToListNegative = oldAddToListNegative;
        addToListNumeric = oldAddToListNumeric;

        addToListPositive.insert(conds.begin(), conds.end());
        addToListNegative.insert(negativeConds.begin(), negativeConds.end());
        addToListNumeric->insert(addToListNumeric->end(), numericConds.begin(), numericConds.end());
    }


    virtual void visit_conj_goal(conj_goal * p) {
        if (adding) {
            p->getGoals()->visit(this);
            return;
        }

        adding = true;
        visit_disj_goal_internal(p->getGoals());
        adding = false;
    };
    virtual void visit_disj_goal(disj_goal * p) {
        if (adding) {
            visit_disj_goal_internal(p->getGoals());
        } else {
            adding = false;
            p->getGoals()->visit(this);
            adding = true;
            return;
        }
    };

    virtual void visit_timed_goal(timed_goal * p) {

        const RPGBuilder::NoDuplicatePair oldAddToListPositive = addToListPositive;
        const RPGBuilder::NoDuplicatePair oldAddToListNegative = addToListNegative;
        list<RPGBuilder::NumericPrecondition> * const oldAddToListNumeric = addToListNumeric;

        switch (p->getTime()) {
        case VAL::E_AT_START:
            if (debug) cout << "\tAt start\n";
            addToListPositive = literalAddTos.back()[0];
            addToListNegative = literalNegativeAddTos.back()[0];
            addToListNumeric = numericAddTos.back()[0];
            break;
        case VAL::E_AT_END:
            if (debug) cout << "\tAt end\n";
            addToListPositive = literalAddTos.back()[2];
            addToListNegative = literalNegativeAddTos.back()[2];
            addToListNumeric = numericAddTos.back()[2];
            break;
        case VAL::E_OVER_ALL:
            if (debug) cout << "\tOver all\n";
            addToListPositive = literalAddTos.back()[1];
            addToListNegative = literalNegativeAddTos.back()[1];
            addToListNumeric = numericAddTos.back()[1];
            break;
        default:
            cout << "Error, unsupported precondition time specification in action schema (not start, end, or over all)\n";
            exit(0);
            break;
        };
        p->getGoal()->visit(this);
        /*      if (visitingWhen) {
                    addToList = &(condEffs.back()->startPrec);
                    addToListNumeric = &(condEffs.back()->startPrecNumeric);
                } else {
                    addToList = &startPrec;
                    addToListNumeric = &startPrecNumeric;
                }*/
        addToListPositive = oldAddToListPositive;
        addToListNegative = oldAddToListNegative;
        addToListNumeric = oldAddToListNumeric;
    };

    virtual void visit_imply_goal(imply_goal * g) {

        if (!adding) {
            adding = true;
            g->getAntecedent()->visit(this);
            adding = false;
            g->getConsequent()->visit(this);
        }

        const RPGBuilder::NoDuplicatePair oldAddToListPositive = addToListPositive;
        const RPGBuilder::NoDuplicatePair oldAddToListNegative = addToListNegative;
        list<RPGBuilder::NumericPrecondition> * const oldAddToListNumeric = addToListNumeric;

        list<Literal*> conds;
        LiteralSet condsSet;
        list<Literal*> negativeConds;
        LiteralSet negativeCondsSet;
        list<RPGBuilder::NumericPrecondition> numericConds;

        addToListPositive = RPGBuilder::NoDuplicatePair(&conds, &condsSet);
        addToListNegative = RPGBuilder::NoDuplicatePair(&negativeConds, &negativeCondsSet);
        addToListNumeric = &numericConds;

        literalAddTos.push_back(vector<RPGBuilder::NoDuplicatePair>(3, addToListPositive));
        literalNegativeAddTos.push_back(vector<RPGBuilder::NoDuplicatePair>(3, addToListNegative));
        numericAddTos.push_back(vector<list<RPGBuilder::NumericPrecondition> * >(3, &numericConds));

        adding = false;
        g->getAntecedent()->visit(this);
        adding = true;
        list<Literal*>::iterator cItr = negativeConds.begin();
        const list<Literal*>::iterator cEnd = negativeConds.end();

        for (; cItr != cEnd; ++cItr) {
            Literal* const currLit = *cItr;
            if (currLit) {
                if (EPS(currLit->getHead())->appearsStatic()) {
                    if (!(EPS(currLit->getHead())->getInitials()->get(fe, currLit->getProp()))) {
                        literalAddTos.pop_back();
                        literalNegativeAddTos.pop_back();
                        numericAddTos.pop_back();
                        addToListPositive = oldAddToListPositive;
                        addToListNegative = oldAddToListNegative;
                        addToListNumeric = oldAddToListNumeric;
                        return;
                    }
                } else {
                    break;
                }
            } else {
                literalAddTos.pop_back();
                literalNegativeAddTos.pop_back();
                numericAddTos.pop_back();
                addToListPositive = oldAddToListPositive;
                addToListNegative = oldAddToListNegative;
                addToListNumeric = oldAddToListNumeric;
                return;
            }
        }
        if (cItr != cEnd || !numericConds.empty()) postmortem_noADL();

        literalAddTos.pop_back();
        numericAddTos.pop_back();
        addToListPositive = oldAddToListPositive;
        addToListNegative = oldAddToListNegative;
        addToListNumeric = oldAddToListNumeric;
        g->getConsequent()->visit(this);
    };

    virtual void visit_forall_effect(forall_effect * p) {
        vector<vector<VAL::const_symbol*>::const_iterator> vals(p->getVarsList()->size());
        vector<vector<VAL::const_symbol*>::const_iterator> starts(p->getVarsList()->size());
        vector<vector<VAL::const_symbol*>::const_iterator> ends(p->getVarsList()->size());
        vector<VAL::var_symbol *> vars(p->getVarsList()->size());
        fe->extend(vars.size());
        int i = 0;
        int c = 1;
        for (var_symbol_list::const_iterator pi = p->getVarsList()->begin();
                pi != p->getVarsList()->end(); ++pi, ++i) {
            if (instantiatedOp::getValues().find((*pi)->type) == instantiatedOp::getValues().end()) {
                instantiatedOp::getValues()[(*pi)->type] = tc->range(*pi);
            };
            vals[i] = starts[i] = instantiatedOp::getValues()[(*pi)->type].begin();
            ends[i] = instantiatedOp::getValues()[(*pi)->type].end();
            if (ends[i] == starts[i]) return;
            (*fe)[(*pi)] = *(vals[i]);
            vars[i] = *pi;
            c *= instantiatedOp::getValues()[(*pi)->type].size();
        };

        --i;
        while (vals[i] != ends[i]) {
            FastEnvironment * ecpy = fe;
            fe = fe->copy();
            p->getEffects()->visit(this);
            fe = ecpy;

            int x = 0;
            ++vals[0];
            if (vals[0] != ends[0])(*fe)[vars[0]] = *(vals[0]);
            while (x < i && vals[x] == ends[x]) {
                vals[x] = starts[x];
                (*fe)[vars[x]] = *(vals[x]);
                ++x;
                ++vals[x];
                if (vals[x] != ends[x])(*fe)[vars[x]] = *(vals[x]);
            };
        };

    };

    virtual void visit_cond_effect(cond_effect * p) {
        if (visitingWhen) {
            postmortem_noNestedWhens();
        }

        condEffs.push_back(new RPGBuilder::ProtoConditionalEffect());
        visitingWhen = true;

        literalAddTos.push_back(vector<RPGBuilder::NoDuplicatePair>(3));
        literalNegativeAddTos.push_back(vector<RPGBuilder::NoDuplicatePair>(3));
        numericAddTos.push_back(vector<list<RPGBuilder::NumericPrecondition> * >(3));

        literalAddTos.back()[0] = RPGBuilder::NoDuplicatePair(&(condEffs.back()->startPrec), &(condEffs.back()->startPrecSet));
        literalAddTos.back()[1] = RPGBuilder::NoDuplicatePair(&(condEffs.back()->inv), &(condEffs.back()->invSet));
        literalAddTos.back()[2] = RPGBuilder::NoDuplicatePair(&(condEffs.back()->endPrec), &(condEffs.back()->endPrecSet));

        literalNegativeAddTos.back()[0] = RPGBuilder::NoDuplicatePair(&(condEffs.back()->startNegPrec), &(condEffs.back()->startNegPrecSet));
        literalNegativeAddTos.back()[1] = RPGBuilder::NoDuplicatePair(&(condEffs.back()->negInv), &(condEffs.back()->negInvSet));
        literalNegativeAddTos.back()[2] = RPGBuilder::NoDuplicatePair(&(condEffs.back()->endNegPrec), &(condEffs.back()->endNegPrecSet));

        numericAddTos.back()[0] = &(condEffs.back()->startPrecNumeric);
        numericAddTos.back()[1] = &(condEffs.back()->invNumeric);
        numericAddTos.back()[2] = &(condEffs.back()->endPrecNumeric);

        const RPGBuilder::NoDuplicatePair oldAddToListPositive = addToListPositive;
        const RPGBuilder::NoDuplicatePair oldAddToListNegative = addToListNegative;
        list<RPGBuilder::NumericPrecondition> * const oldAddToListNumeric = addToListNumeric;
        const RPGBuilder::NoDuplicatePair oldAddEffToList = addEffToList;
        const RPGBuilder::NoDuplicatePair oldDelEffToList = delEffToList;
        const pair<list<RPGBuilder::NumericEffect>*,set<int>*> oldAddEffToListNumeric = addEffToListNumeric;

        if (!isDurative) {
            addToListPositive = literalAddTos.back()[0];
            addToListNegative = literalNegativeAddTos.back()[0];
            addToListNumeric = &(condEffs.back()->startPrecNumeric);
            addEffToList = RPGBuilder::NoDuplicatePair(&(condEffs.back()->startAddEff), &(condEffs.back()->startAddEffSet));
            delEffToList = RPGBuilder::NoDuplicatePair(&(condEffs.back()->startDelEff), &(condEffs.back()->startDelEffSet));
            addEffToListNumeric = make_pair(&(condEffs.back()->startNumericEff), &(condEffs.back()->startNumericEffsOnVar));
        } else {
            addToListPositive = RPGBuilder::NoDuplicatePair();
            addToListNegative = RPGBuilder::NoDuplicatePair();
            addToListNumeric = 0;
            addEffToList = RPGBuilder::NoDuplicatePair();
            delEffToList = RPGBuilder::NoDuplicatePair();
            addEffToListNumeric.first = 0;
            addEffToListNumeric.second = 0;
        }

        p->getCondition()->visit(this);
        p->getEffects()->visit(this);

        addToListPositive = oldAddToListPositive;
        addToListNegative = oldAddToListNegative;
        addToListNumeric = oldAddToListNumeric;
        addEffToList = oldAddEffToList;
        delEffToList = oldDelEffToList;
        addEffToListNumeric = oldAddEffToListNumeric;


        visitingWhen = false;

        literalAddTos.pop_back();
        numericAddTos.pop_back();

    };
    virtual void visit_neg_goal(neg_goal * p) {

        const bool oldAdding = adding;
        adding = !oldAdding;
        p->getGoal()->visit(this);
        adding = oldAdding;

    };
    virtual void visit_preference(preference * p) {
        p->getGoal()->visit(this);
    };
    virtual void visit_constraint_goal(constraint_goal *cg) {
        if (cg->getRequirement()) {
            cg->getRequirement()->visit(this);
        };
        if (cg->getTrigger()) {
            cg->getTrigger()->visit(this);
        };
    };

    virtual void visit_timed_effect(timed_effect * p) {
        const RPGBuilder::NoDuplicatePair oldAdd = addEffToList;
        const RPGBuilder::NoDuplicatePair oldDel = delEffToList;
        const pair<list<RPGBuilder::NumericEffect>*, set<int>* > oldNum = addEffToListNumeric;
        switch (p->ts) {
        case VAL::E_AT_END: {
            if (debug) cout << "\tAt end\n";
            if (visitingWhen) {
                addEffToList = RPGBuilder::NoDuplicatePair(&(condEffs.back()->endAddEff), &(condEffs.back()->endAddEffSet));
                delEffToList = RPGBuilder::NoDuplicatePair(&(condEffs.back()->endDelEff), &(condEffs.back()->endDelEffSet));
                addEffToListNumeric.first = &(condEffs.back()->endNumericEff);
                addEffToListNumeric.second = &(condEffs.back()->endNumericEffsOnVar);
            } else {
                addEffToList = RPGBuilder::NoDuplicatePair(&endAddEff, &endAddEffSet);
                delEffToList = RPGBuilder::NoDuplicatePair(&endDelEff, &endDelEffSet);
                addEffToListNumeric.first = &endNumericEff;
                addEffToListNumeric.second = &endNumericEffsOnVar;
            }
        }
        break;

        case VAL::E_AT_START: {
            if (debug) cout << "\tAt start\n";

            if (visitingWhen) {
                addEffToList = RPGBuilder::NoDuplicatePair(&(condEffs.back()->startAddEff), &(condEffs.back()->startAddEffSet));
                delEffToList = RPGBuilder::NoDuplicatePair(&(condEffs.back()->startDelEff), &(condEffs.back()->startDelEffSet));
                addEffToListNumeric.first = &(condEffs.back()->startNumericEff);
                addEffToListNumeric.second = &(condEffs.back()->startNumericEffsOnVar);
            } else {
                addEffToList = RPGBuilder::NoDuplicatePair(&startAddEff, &startAddEffSet);
                delEffToList = RPGBuilder::NoDuplicatePair(&startDelEff, &startDelEffSet);
                addEffToListNumeric.first = &startNumericEff;
                addEffToListNumeric.second = &startNumericEffsOnVar;
            }
        
            break;
            }
        case VAL::E_CONTINUOUS: {
            break;
        }
        default: {
            cout << "Error, unsupported effect time specification " << p->ts << " in action schema " << *toBlame << " (not start or end, or continuous)\n";
            exit(0);
        }
        };
        p->effs->visit(this);

        addEffToList = oldAdd;
        delEffToList = oldDel;
        addEffToListNumeric = oldNum;
        
        if (debug) {
            cout << "Effect visited; reverting to numeric effect list and set at " << addEffToListNumeric.first << " and " << addEffToListNumeric.second << endl;
        }
    };

    virtual void visit_simple_effect(simple_effect * p) {

        Literal * l = new Literal(p->prop, fe);

        validateLiteral(l);

        if (adding) {

            if (!addEffToList) {
                string actionname;
                string effectdescription;

                {
                    ostringstream o;
                    o << *thisIOP;
                    actionname = o.str();
                }

                {
                    ostringstream o;
                    o << *l;
                    effectdescription = o.str();
                }
                postmortem_noTimeSpecifierOnAPropEffect(actionname, effectdescription);
            }
            addEffToList.push_back(instantiatedOp::findLiteral(l));
            if (debug) {
                if (addEffToList.back()) {
                    cout << "\t\t" << *(addEffToList.back()) << "\n";
                } else {
                    cout << "\t\tNull\n";
                }
            }

            delete l;
        } else {

            if (!delEffToList) {
                string actionname;
                string effectdescription;

                {
                    ostringstream o;
                    o << *thisIOP;
                    actionname = o.str();
                }

                {
                    ostringstream o;
                    o << "(not ";
                    o << *l;
                    o << ")";
                    effectdescription = o.str();
                }
                postmortem_noTimeSpecifierOnAPropEffect(actionname, effectdescription);
            }

            Literal* const realised = instantiatedOp::findLiteral(l);
            
            if (debug && !realised) {
                cout << "\t\tnot a fact that was never added: " << *l << endl;
            }
            if (realised) {
                delEffToList.push_back(realised);
                if (debug) {
                    if (delEffToList.back()) {
                        cout << "\t\tnot " << *(delEffToList.back()) << "\n";
                    } else {
                        cout << "\t\tnot Null\n";
                    }
                }

                delete l;
            }
        }
    };

    virtual void visit_effect_lists(effect_lists * p) {
        p->add_effects.pc_list<simple_effect*>::visit(this);
        p->forall_effects.pc_list<forall_effect*>::visit(this);
        p->cond_effects.pc_list<cond_effect*>::visit(this);
        p->timed_effects.pc_list<timed_effect*>::visit(this);
        const bool whatwas = adding;
        adding = !adding;
        p->del_effects.pc_list<simple_effect*>::visit(this);
        adding = whatwas;
        p->assign_effects.pc_list<assignment*>::visit(this);
    };


    virtual void visit_operator_(VAL::operator_ * p) {
        if (debug) cout << "Going through preconditions\n";
        adding = true;
        inpres = true;
        checkpos = true;
        WhereAreWeNow = PARSE_PRECONDITION;
        if (p->precondition) p->precondition->visit(this);
        inpres = false;
        if (debug) cout << "Going through effects\n";
        adding = true;
        WhereAreWeNow = PARSE_EFFECT;
        p->effects->visit(this);
        WhereAreWeNow = PARSE_UNKNOWN;
    };
    virtual void visit_action(VAL::action * p) {
        toBlame = thisIOP;
        isDurative = false;
        addToListPositive = RPGBuilder::NoDuplicatePair(&startPrec, &startPrecSet);
        addToListNegative = RPGBuilder::NoDuplicatePair(&startNegPrec, &startNegPrecSet);
        addToListNumeric = &startPrecNumeric;
        addEffToList = RPGBuilder::NoDuplicatePair(&startAddEff, &startAddEffSet);
        delEffToList = RPGBuilder::NoDuplicatePair(&startDelEff, &startDelEffSet);
        addEffToListNumeric.first = &startNumericEff;
        addEffToListNumeric.second = &startNumericEffsOnVar;

        literalAddTos.push_back(vector<RPGBuilder::NoDuplicatePair>(3));
        literalNegativeAddTos.push_back(vector<RPGBuilder::NoDuplicatePair>(3));
        numericAddTos.push_back(vector<list<RPGBuilder::NumericPrecondition> * >(3, (list<RPGBuilder::NumericPrecondition>*) 0));

        literalAddTos.back()[0] = addToListPositive;
        literalNegativeAddTos.back()[0] = addToListNegative;
        numericAddTos.back()[0] = addToListNumeric;

        visit_operator_(p); //static_cast<VAL::operator_*>(p));
        toBlame = 0;
    };
    virtual void visit_durative_action(VAL::durative_action * p) {
        toBlame = thisIOP;
        isDurative = true;

        literalAddTos.push_back(vector<RPGBuilder::NoDuplicatePair>(3));
        literalNegativeAddTos.push_back(vector<RPGBuilder::NoDuplicatePair>(3));
        numericAddTos.push_back(vector<list<RPGBuilder::NumericPrecondition> * >(3));

        literalAddTos.back()[0] = RPGBuilder::NoDuplicatePair(&startPrec, &startPrecSet);
        literalAddTos.back()[1] = RPGBuilder::NoDuplicatePair(&inv, &invSet);
        literalAddTos.back()[2] = RPGBuilder::NoDuplicatePair(&endPrec, &endPrecSet);

        literalNegativeAddTos.back()[0] = RPGBuilder::NoDuplicatePair(&startNegPrec, &startNegPrecSet);
        literalNegativeAddTos.back()[1] = RPGBuilder::NoDuplicatePair(&negInv, &negInvSet);
        literalNegativeAddTos.back()[2] = RPGBuilder::NoDuplicatePair(&endNegPrec, &endNegPrecSet);

        numericAddTos.back()[0] = &startPrecNumeric;
        numericAddTos.back()[1] = &invNumeric;
        numericAddTos.back()[2] = &endPrecNumeric;

        visit_operator_(p); //static_cast<VAL::operator_*>(p));
        visitingDuration = true;
        
        if (debug) cout << "Going through duration\n";
        WhereAreWeNow = PARSE_DURATION;
        p->dur_constraint->visit(this);
        WhereAreWeNow = PARSE_UNKNOWN;
        visitingDuration = false;
        toBlame = 0;

    };
    virtual void visit_process(VAL::process * p) {
        visit_operator_(p);
    };
    virtual void visit_event(VAL::event * p) {
        visit_operator_(p);
    };
    virtual void visit_problem(VAL::problem * p) {
        p->initial_state->visit(this);
        inpres = false;
        if (p->the_goal) {
            p->the_goal->visit(this);
        }
    };

    virtual void visit_assignment(assignment * a) {
        PNE p(a->getFTerm(), fe);
        validatePNE(&p);
        PNE * pne = instantiatedOp::getPNE(&p);
        if (!addEffToListNumeric.first) {
            startNumericEff.push_back(RPGBuilder::NumericEffect(a->getOp(), pne->getStateID(), const_cast<VAL::expression*>(a->getExpr()), fe, tc));
            assert(startNumericEff.back().fluentIndex < instantiatedOp::howManyNonStaticPNEs());
            assert(startNumericEff.back().fluentIndex >= 0);
            
            
            bool isCTS = false;
            list<RPGBuilder::Operand>::iterator fItr = startNumericEff.back().formula.begin();
            const list<RPGBuilder::Operand>::iterator fEnd = startNumericEff.back().formula.end();

            for (; fItr != fEnd; ++fItr) {
                if (fItr->numericOp == RPGBuilder::NE_FLUENT && (fItr->fluentValue == -2 || fItr->fluentValue == -18)) {
                    isCTS = true;
                    break;
                }
            }

            if (!isCTS) {
                string actionname;
                string effectdescription;
                string suggested;

                bool isAssign = false;

                {
                    ostringstream o;
                    o << *thisIOP;
                    actionname = o.str();
                }

                {
                    ostringstream o;
                    ostringstream o2;
                    if (a->getOp() == E_INCREASE) {
                        o << "(increase ";
                        o2 << "(increase ";
                    } else if (a->getOp() == E_DECREASE) {
                        o << "(decrease ";
                        o2 << "(decrease ";
                    } else {
                        isAssign = true;
                        o << "(assign ";
                        o2 << "(increase ";
                    }

                    o << *pne << " ";
                    o2 << *pne << " (* #t ";

                    {
                        ExpressionPrinter p(o, fe, tc);
                        p.printFormula(const_cast<VAL::expression*>(a->getExpr()));
                    }
                    {
                        ExpressionPrinter p(o2, fe, tc);
                        p.printFormula(const_cast<VAL::expression*>(a->getExpr()));
                    }

                    o << ")";
                    o2 << "))";

                    effectdescription = o.str();
                    suggested = o2.str();
                }
                postmortem_noTimeSpecifierOnInstantNumericEffect(actionname, effectdescription, suggested, isAssign);

            }

        } else {
            if (debug) {
                cout << "Using set at " << (addEffToListNumeric.second) << " to check for self-mutex effects\n";
            }
            if (!addEffToListNumeric.second->insert(pne->getStateID()).second) {
                string actionname;
                string varname;
                
                {
                    ostringstream o;
                    o << *thisIOP;
                    actionname = o.str();
                }
                
                {
                    ostringstream o;
                    o << *pne;
                    varname = o.str();
                }
                
                postmortem_twoSimulataneousNumericEffectsOnTheSameVariable(actionname, varname);
            }
            addEffToListNumeric.first->push_back(RPGBuilder::NumericEffect(a->getOp(), pne->getStateID(), const_cast<VAL::expression*>(a->getExpr()), fe, tc));
            assert(addEffToListNumeric.first->back().fluentIndex < instantiatedOp::howManyNonStaticPNEs());
            assert(addEffToListNumeric.first->back().fluentIndex >= 0);
        }
    };

};

instantiatedOp* TimedPrecEffCollector::toBlame = 0;
map<string, set<int> > TimedPrecEffCollector::litParamCounts;
map<string, set<int> > TimedPrecEffCollector::pneParamCounts;
bool TimedPrecEffCollector::initParamCounts = false;

void validatePNE(PNE * c)
{
    const set<int> & expectedArgs = TimedPrecEffCollector::pneParamCounts[c->getHead()->getName()];
    const int givenArgs = c->getFunc()->getArgs()->size();
    if (expectedArgs.find(givenArgs) == expectedArgs.end()) {
        string actionname;
        string predname;
        string lit;
        bool actLabel = false;

        if (TimedPrecEffCollector::toBlame) {
            ostringstream o;
            o << *(TimedPrecEffCollector::toBlame);
            actionname = o.str();
            actLabel = true;
        }

        {
            ostringstream o;
            o << c->getFunc()->getFunction()->getName();
            predname = o.str();
        }

        {
            ostringstream o;
            o << *c;
            lit = o.str();
        }

        postmortem_wrongNumberOfFluentArguments(actionname, actLabel, WhereAreWeNow, predname, lit, givenArgs, expectedArgs);
    }

};

void validateLiteral(Literal * l)
{

    const set<int> & expectedArgs = TimedPrecEffCollector::litParamCounts[l->getProp()->head->getName()];
    const int givenArgs = l->getProp()->args->size();
    if (expectedArgs.find(givenArgs) == expectedArgs.end()) {
        string actionname;
        string predname;
        string lit;

        bool actLabel = false;

        if (TimedPrecEffCollector::toBlame) {
            ostringstream o;
            o << *(TimedPrecEffCollector::toBlame);
            actionname = o.str();
            actLabel = true;
        }

        {
            ostringstream o;
            o << l->getProp()->head->getName();
            predname = o.str();
        }

        {
            ostringstream o;
            o << *l;
            lit = o.str();
        }

        postmortem_wrongNumberOfPredicateArguments(actionname, actLabel, WhereAreWeNow, predname, lit, givenArgs, expectedArgs);
    }
}


class InitialStateCollector : public VisitController
{

private:
    VAL::TypeChecker * tc;
    bool adding;
    const VAL::operator_ * op;
    FastEnvironment * fe;

    bool inpres;
    bool checkpos;
    int assignTo;
    bool TIL;

    list<double> workingValues;

public:

    LiteralSet initialState;
    vector<double> initialFluents;

    LiteralSet tilAddSet;
    LiteralSet tilDeleteSet;
    map<double, RPGBuilder::FakeTILAction> timedInitialLiterals;


    InitialStateCollector(const VAL::operator_ * o, FastEnvironment * f, VAL::TypeChecker * t = 0)
        : tc(t), adding(true), op(o), fe(f), inpres(true), checkpos(true), assignTo(-1), TIL(false) {
                
        const int ifSize = instantiatedOp::howManyNonStaticPNEs();
        initialFluents.resize(ifSize, 0.0);
    }

    virtual void visit_simple_goal(simple_goal * p) {

    };
    virtual void visit_qfied_goal(qfied_goal * p) {
        p->getGoal()->visit(this);
    };
    virtual void visit_conj_goal(conj_goal * p) {
        p->getGoals()->visit(this);
    };
    virtual void visit_disj_goal(disj_goal * p) {
        p->getGoals()->visit(this);
    };
    virtual void visit_timed_goal(timed_goal * p) {
        p->getGoal()->visit(this);
    };
    virtual void visit_imply_goal(imply_goal * p) {
        p->getAntecedent()->visit(this);
        p->getConsequent()->visit(this);
    };
    virtual void visit_neg_goal(neg_goal * p) {
        bool oldcheck = checkpos;
        checkpos = !checkpos;
        p->getGoal()->visit(this);
        checkpos = oldcheck;
    };
    virtual void visit_preference(preference * p) {
        p->getGoal()->visit(this);
    };

    virtual void visit_simple_effect(simple_effect * p) {
        if (TIL) {
            if (adding) {
                Literal * l = new Literal(p->prop, fe);
                validateLiteral(l);
                tilAddSet.insert(instantiatedOp::findLiteral(l));
                delete l;
            } else {
                Literal * l = new Literal(p->prop, fe);
                validateLiteral(l);
                tilDeleteSet.insert(instantiatedOp::findLiteral(l));
                delete l;
            }
        } else {

            if (adding) {
                Literal l(p->prop, fe);
                validateLiteral(&l);

                Literal * lookup = instantiatedOp::findLiteral(&l);

                if (lookup) {
                    if (lookup->getStateID() >= 0) {
                        initialState.insert(lookup);
                    }
                } else {
                    cout << "Internal error: cannot locate initial state fact " << l << " in proposition look-up table\n";
                    exit(1);
                }

            }
        }
    };
    virtual void visit_constraint_goal(constraint_goal *cg) {
        if (cg->getRequirement()) {
            cg->getRequirement()->visit(this);
        };
        if (cg->getTrigger()) {
            cg->getTrigger()->visit(this);
        };
    };

    virtual void visit_forall_effect(forall_effect * p) {
    };
    virtual void visit_cond_effect(cond_effect * p) {
        p->getCondition()->visit(this);
        p->getEffects()->visit(this);
    };
    virtual void visit_timed_effect(timed_effect * p) {
        p->effs->visit(this);
    };
    virtual void visit_timed_initial_literal(timed_initial_literal * p) {
        const double time_stamp = p->time_stamp;
        TIL = true;
        p->effs->visit(this);
        const map<double, RPGBuilder::FakeTILAction>::iterator lookup = timedInitialLiterals.find(time_stamp);
        if (lookup == timedInitialLiterals.end()) {
            timedInitialLiterals.insert(pair<double, RPGBuilder::FakeTILAction>(time_stamp, RPGBuilder::FakeTILAction(time_stamp, tilAddSet, tilDeleteSet)));
        } else {
            lookup->second.mergeIn(tilAddSet, tilDeleteSet);
        }
        tilAddSet.clear();
        tilDeleteSet.clear();
        TIL = false;
    };
    virtual void visit_effect_lists(effect_lists * p) {
        p->add_effects.pc_list<simple_effect*>::visit(this);
        p->forall_effects.pc_list<forall_effect*>::visit(this);
        p->cond_effects.pc_list<cond_effect*>::visit(this);
        p->timed_effects.pc_list<timed_effect*>::visit(this);
        bool whatwas = adding;
        adding = !adding;
        p->del_effects.pc_list<simple_effect*>::visit(this);
        adding = whatwas;
        p->assign_effects.pc_list<assignment*>::visit(this);
    };
    virtual void visit_operator_(VAL::operator_ * p) {
        inpres = true;
        checkpos = true;
        p->precondition->visit(this);
        inpres = false;

        adding = true;
        p->effects->visit(this);
    };
    virtual void visit_action(VAL::action * p) {
        visit_operator_(p); //static_cast<VAL::operator_*>(p));
    };
    virtual void visit_durative_action(VAL::durative_action * p) {
        visit_operator_(p); //static_cast<VAL::operator_*>(p));
    };
    virtual void visit_process(VAL::process * p) {
        visit_operator_(p);
    };
    virtual void visit_event(VAL::event * p) {
        visit_operator_(p);
    };
    virtual void visit_problem(VAL::problem * p) {
        TimedPrecEffCollector::toBlame = 0;
        WhereAreWeNow = PARSE_INITIAL;
        p->initial_state->visit(this);
        WhereAreWeNow = PARSE_UNKNOWN;
        //inpres = false;
        //p->the_goal->visit(this);
    };

    virtual void visit_assignment(assignment * a) {
        const func_term * ft = a->getFTerm();
        PNE * const pne = new PNE(ft, fe);
        validatePNE(pne);
        PNE * const realPNE = instantiatedOp::getPNE(pne);
        delete pne;
        if (realPNE->getStateID() >= 0) {
            assignTo = realPNE->getStateID();
            a->getExpr()->visit(this);
            initialFluents[assignTo] = workingValues.back();
            workingValues.pop_back();
            assignTo = -1;
        }

    };

    virtual void visit_plus_expression(plus_expression * e) {
        e->getLHS()->visit(this);
        e->getRHS()->visit(this);
        const double r = workingValues.back(); workingValues.pop_back();
        const double l = workingValues.back(); workingValues.pop_back();
        workingValues.push_back(r + l);
    };

    virtual void visit_minus_expression(minus_expression * e) {
        e->getLHS()->visit(this);
        e->getRHS()->visit(this);
        const double r = workingValues.back(); workingValues.pop_back();
        const double l = workingValues.back(); workingValues.pop_back();
        workingValues.push_back(l - r);
    };

    virtual void visit_mul_expression(mul_expression * e) {
        e->getLHS()->visit(this);
        e->getRHS()->visit(this);
        const double r = workingValues.back(); workingValues.pop_back();
        const double l = workingValues.back(); workingValues.pop_back();
        workingValues.push_back(r * l);
    };

    virtual void visit_div_expression(div_expression * e) {
        e->getLHS()->visit(this);
        e->getRHS()->visit(this);
        const double r = workingValues.back(); workingValues.pop_back();
        const double l = workingValues.back(); workingValues.pop_back();

        if (r == 0.0) {
            postmortem_mathsError("division by zero error", "", WhereAreWeNow);
        }

        workingValues.push_back(l / r);
    };
    virtual void visit_uminus_expression(uminus_expression * e) {
        e->getExpr()->visit(this);
        const double r = workingValues.back(); workingValues.pop_back();
        workingValues.push_back(-r);
    };

    virtual void visit_int_expression(int_expression * e) {
        //assert(assignTo != -1);
        //initialFluents[assignTo] = e->double_value();
        workingValues.push_back(e->double_value());
    };

    virtual void visit_float_expression(float_expression * e) {
        //assert(assignTo != -1);
        //initialFluents[assignTo] = e->double_value();
        workingValues.push_back(e->double_value());
    };

    void visit_special_val_expr(const special_val_expr * v) {
        postmortem_mathsError("#t, ?duration or total-time",
                              "Special values, such as these, cannot be used to define initial fluent values.\n", WhereAreWeNow);
    };


    void visit_func_term(const func_term * s) {
        PNE * const lookupPNE = new PNE(s, fe);
        validatePNE(lookupPNE);
        //cout << "Looking up " << *lookupPNE << "\n";
        PNE * realPNE = instantiatedOp::findPNE(lookupPNE);


        if (!realPNE) {
            ostringstream o;
            o << "reference to an undefined value '" << *lookupPNE << "'.";
            string toPass = o.str();
            postmortem_mathsError(toPass, "", WhereAreWeNow);
        } else {
//          cout << "'" << realPNE->getHead()->getName() << "'\n";
            if (realPNE->getHead()->getName() == "fake-duration") {
                postmortem_mathsError("?duration",
                                      "Special values, such as this, cannot be used to define initial fluent values.\n",
                                      WhereAreWeNow);
            } else {
                pair<bool, double> isSet = EFT(realPNE->getHead())->getInitial(realPNE->begin(), realPNE->end());
                if (isSet.first) {
                    workingValues.push_back(isSet.second);
                } else {
                    ostringstream o;
                    o << "reference to '" << *realPNE << "', which has no known initial-state value";
                    string toPass = o.str();
                    postmortem_mathsError(toPass,
                                          "Values within the formulae to define initial fluent values must only refer to\nconstant values.\n", WhereAreWeNow);
                }
            }

        }
        delete lookupPNE;
    };

};
bool RPGBuilder::RPGdebug = false;
bool RPGBuilder::problemIsNotTemporal = true;

// ### Search for this to find where the static members are ###

vector<list<pair<int, VAL::time_spec> > > RPGBuilder::preconditionsToActions;
vector<list<pair<int, VAL::time_spec> > > RPGBuilder::negativePreconditionsToActions;
list<pair<int, VAL::time_spec> > RPGBuilder::preconditionlessActions;
list<pair<int, VAL::time_spec> > RPGBuilder::onlyNumericPreconditionActions;

vector<list<RPGBuilder::ProtoConditionalEffect*> > RPGBuilder::actionsToRawConditionalEffects;


vector<list<Literal*> > RPGBuilder::actionsToStartPreconditions;
vector<list<Literal*> > RPGBuilder::actionsToInvariants;
vector<list<Literal*> > RPGBuilder::actionsToEndPreconditions;
vector<LiteralSet> RPGBuilder::actionsToEndOneShots;

vector<list<Literal*> > RPGBuilder::actionsToStartNegativePreconditions;
vector<list<Literal*> > RPGBuilder::actionsToNegativeInvariants;
vector<list<Literal*> > RPGBuilder::actionsToEndNegativePreconditions;

vector<list<Literal*> > RPGBuilder::actionsToStartEffects;
vector<list<Literal*> > RPGBuilder::actionsToStartNegativeEffects;
vector<list<Literal*> > RPGBuilder::actionsToEndEffects;
vector<list<Literal*> > RPGBuilder::actionsToEndNegativeEffects;

vector<list<pair<int, VAL::time_spec> > > RPGBuilder::effectsToActions;
vector<list<pair<int, VAL::time_spec> > > RPGBuilder::negativeEffectsToActions;

vector<vector<RPGBuilder::RPGDuration*> > RPGBuilder::rpgDurationExpressions;


vector<RPGBuilder::LinearEffects*> RPGBuilder::linearDiscretisation;

vector<list<RPGBuilder::NumericPrecondition*> > RPGBuilder::fixedDurationExpressions;
vector<list<RPGBuilder::NumericPrecondition*> > RPGBuilder::minDurationExpressions;
vector<list<RPGBuilder::NumericPrecondition*> > RPGBuilder::maxDurationExpressions;

vector<double> RPGBuilder::actionsToMinDurations;
vector<double> RPGBuilder::actionsToMaxDurations;
vector<double> RPGBuilder::nonTemporalDuration;

vector<list<RPGBuilder::NumericPrecondition> > RPGBuilder::actionsToStartNumericPreconditions;
vector<list<RPGBuilder::NumericPrecondition> > RPGBuilder::actionsToNumericInvariants;
vector<list<RPGBuilder::NumericPrecondition> > RPGBuilder::actionsToEndNumericPreconditions;

vector<list<RPGBuilder::NumericEffect> > RPGBuilder::actionsToStartNumericEffects;
vector<list<RPGBuilder::NumericEffect> > RPGBuilder::actionsToEndNumericEffects;


vector<int> RPGBuilder::initialUnsatisfiedStartPreconditions;
vector<int> RPGBuilder::initialUnsatisfiedInvariants;
vector<int> RPGBuilder::initialUnsatisfiedEndPreconditions;

vector<double> RPGBuilder::achievedInLayer;
vector<double> RPGBuilder::achievedInLayerReset;
vector<pair<int, VAL::time_spec> > RPGBuilder::achievedBy;
vector<pair<int, VAL::time_spec> > RPGBuilder::achievedByReset;

vector<double> RPGBuilder::negativeAchievedInLayer;
vector<double> RPGBuilder::negativeAchievedInLayerReset;
vector<pair<int, VAL::time_spec> > RPGBuilder::negativeAchievedBy;
vector<pair<int, VAL::time_spec> > RPGBuilder::negativeAchievedByReset;

vector<double> RPGBuilder::numericAchievedInLayer;
vector<double> RPGBuilder::numericAchievedInLayerReset;
vector<ActionFluentModification*> RPGBuilder::numericAchievedBy;
vector<ActionFluentModification*> RPGBuilder::numericAchievedByReset;


vector<Literal*> RPGBuilder::literals;

vector<vector<Literal*> > RPGBuilder::propositionGroups;
vector<int> RPGBuilder::literalToPropositionGroupID;
bool RPGBuilder::readPropositionGroups = false;


vector<instantiatedOp*> RPGBuilder::instantiatedOps;
vector<bool> RPGBuilder::realRogueActions;
const vector<bool> & RPGBuilder::rogueActions = RPGBuilder::realRogueActions;

vector<PNE*> RPGBuilder::pnes;
vector<pair<bool, bool> > RPGBuilder::staticLiterals;

vector<RPGBuilder::RPGNumericPrecondition> RPGBuilder::rpgNumericPreconditions;
vector<list<pair<int, VAL::time_spec> > > RPGBuilder::rpgNumericPreconditionsToActions;

vector<RPGBuilder::RPGNumericEffect> RPGBuilder::rpgNumericEffects;
vector<list<pair<int, VAL::time_spec> > > RPGBuilder::rpgNumericEffectsToActions;

vector<list<int> > RPGBuilder::actionsToRPGNumericStartEffects;
vector<list<int> > RPGBuilder::actionsToRPGNumericEndEffects;


vector<RPGBuilder::ArtificialVariable> RPGBuilder::rpgArtificialVariables;
vector<list<int> > RPGBuilder::rpgArtificialVariablesToPreconditions;
vector<list<int> > RPGBuilder::rpgPositiveVariablesToPreconditions;
vector<list<int> > RPGBuilder::rpgNegativeVariablesToPreconditions;

vector<list<int> > RPGBuilder::actionsToRPGNumericStartPreconditions;
vector<list<int> > RPGBuilder::actionsToRPGNumericInvariants;
vector<list<int> > RPGBuilder::actionsToRPGNumericEndPreconditions;
vector<list<int> > RPGBuilder::actionsToProcessedStartRPGNumericPreconditions;

vector<list<int> > RPGBuilder::rpgVariableDependencies;

RPGBuilder::Metric * RPGBuilder::theMetric = 0;
set<int> RPGBuilder::metricVars;
    

list<Literal*> RPGBuilder::literalGoals;
list<double> RPGBuilder::literalGoalDeadlines;
list<RPGBuilder::NumericPrecondition> RPGBuilder::numericGoals;
list<double> RPGBuilder::numericGoalDeadlines;
list<pair<int, int> > RPGBuilder::numericRPGGoals;
list<double> RPGBuilder::rpgNumericGoalDeadlines;

vector<RPGBuilder::Constraint> RPGBuilder::preferences;
map<string, int> RPGBuilder::prefNameToID;

vector<RPGBuilder::Constraint> RPGBuilder::constraints;

vector<list<RPGBuilder::ConditionalEffect> > RPGBuilder::actionsToConditionalEffects;

vector<int> RPGBuilder::initialUnsatisfiedNumericStartPreconditions;
vector<int> RPGBuilder::initialUnsatisfiedNumericInvariants;
vector<int> RPGBuilder::initialUnsatisfiedNumericEndPreconditions;


vector<list<pair<int, VAL::time_spec> > > RPGBuilder::processedPreconditionsToActions;
vector<list<pair<int, VAL::time_spec> > > RPGBuilder::processedNegativePreconditionsToActions;
vector<list<Literal*> > RPGBuilder::actionsToProcessedStartPreconditions;
vector<list<Literal*> > RPGBuilder::actionsToProcessedStartNegativePreconditions;
vector<int> RPGBuilder::initialUnsatisfiedProcessedStartPreconditions;

vector<list<pair<int, VAL::time_spec> > > RPGBuilder::processedRPGNumericPreconditionsToActions;
vector<list<RPGBuilder::NumericPrecondition> > RPGBuilder::actionsToProcessedStartNumericPreconditions;
vector<int> RPGBuilder::initialUnsatisfiedProcessedStartNumericPreconditions;

vector<list<int> > RPGBuilder::mentionedInFluentInvariants;

list<RPGBuilder::FakeTILAction> RPGBuilder::timedInitialLiterals;
vector<RPGBuilder::FakeTILAction*> RPGBuilder::timedInitialLiteralsVector;
list<RPGBuilder::FakeTILAction> RPGBuilder::optimisationTimedInitialLiterals;
vector<RPGBuilder::FakeTILAction*> RPGBuilder::optimisationTimedInitialLiteralsVector;
vector<RPGBuilder::FakeTILAction*> RPGBuilder::allTimedInitialLiteralsVector;

map<int, set<int> > RPGBuilder::tilsThatAddFact;
map<int, set<int> > RPGBuilder::tilsThatDeleteFact;
vector<RPGBuilder::KShotFormula*> RPGBuilder::kShotFormulae;
vector<bool> RPGBuilder::selfMutexes;
vector<bool> RPGBuilder::oneShotLiterals;

vector<double> RPGBuilder::maxNeeded;
map<int, int> RPGBuilder::uninterestingnessCriteria;
set<int> RPGBuilder::concurrentRedundantActions;

vector<list<int> > RPGBuilder::realVariablesToRPGEffects;

void deleteAndEmpty(list<RPGBuilder::NumericPrecondition*> & l)
{

    list<RPGBuilder::NumericPrecondition*>::iterator delItr = l.begin();
    const list<RPGBuilder::NumericPrecondition*>::iterator delEnd = l.end();

    for (; delItr != delEnd; ++delItr) {
        delete *delItr;
    }
    l.clear();
};


bool checkIfRogue(TimedPrecEffCollector & c)
{

    const bool rogueDebug = false;

    for (int pass = 0; pass < 3; ++pass) {
        
        list<RPGBuilder::NumericPrecondition*> & durList = (pass ? (pass == 2 ? c.maxDurationExpression : c.minDurationExpression)
                                                                 : c.fixedDurationExpression);
                                                                 
        list<RPGBuilder::NumericPrecondition*>::const_iterator dItr = durList.begin();
        const list<RPGBuilder::NumericPrecondition*>::const_iterator dEnd = durList.end();
        
        for (; dItr != dEnd; ++dItr) {
            if (!(*dItr)) {
                return true;
            }
            assert((*dItr)->valid);
        }
    }

    {

        list<Literal*>::iterator llItr = c.startPrec.begin();
        const list<Literal*>::iterator llEnd = c.startPrec.end();

        for (; llItr != llEnd; ++llItr) {
            if (!(*llItr)) {
                if (rogueDebug) cout << "Start Precondition " << *llItr << " doesn't exist\n";
                return true;
            }
        }

    }

    {

        list<Literal*>::iterator llItr = c.inv.begin();
        const list<Literal*>::iterator llEnd = c.inv.end();

        for (; llItr != llEnd; ++llItr) {
            if (!(*llItr)) {
                if (rogueDebug) cout << "Invariant " << *llItr << " doesn't exist\n";
                return true;
            }
        }

    }

    {

        list<Literal*>::iterator llItr = c.endPrec.begin();
        const list<Literal*>::iterator llEnd = c.endPrec.end();

        for (; llItr != llEnd; ++llItr) {
            if (!(*llItr)) {
                if (rogueDebug) cout << "End Precondition " << *llItr << " doesn't exist\n";
                return true;
            }
        }

    }

    {

        list<Literal*>::iterator llItr = c.startAddEff.begin();
        const list<Literal*>::iterator llEnd = c.startAddEff.end();

        for (; llItr != llEnd; ++llItr) {
            if (!(*llItr)) {
                if (rogueDebug) cout << "Start Add Effect " << *llItr << " doesn't exist\n";
                return true;
            }

        }
    }

    {

        list<Literal*>::iterator llItr = c.startDelEff.begin();
        const list<Literal*>::iterator llEnd = c.startDelEff.end();

        for (; llItr != llEnd; ++llItr) {
            if (!(*llItr)) {
                if (rogueDebug) cout << "Start Delete Effect " << *llItr << " doesn't exist\n";
                return true;
            }

        }
    }


    {

        list<Literal*>::iterator llItr = c.endAddEff.begin();
        const list<Literal*>::iterator llEnd = c.endAddEff.end();

        for (; llItr != llEnd; ++llItr) {
            if (!(*llItr)) {
                if (rogueDebug) cout << "End Add Effect " << *llItr << " doesn't exist\n";
                return true;
            }

        }
    }

    {

        list<Literal*>::iterator llItr = c.endDelEff.begin();
        const list<Literal*>::iterator llEnd = c.endDelEff.end();

        for (; llItr != llEnd; ++llItr) {
            if (!(*llItr)) {
                if (rogueDebug) cout << "End Del Effect " << *llItr << " doesn't exist\n";
                return true;
            }

        }
    }

    return false;
}

void RPGBuilder::initialise()
{
    RPGdebug = (Globals::globalVerbosity & 16);
    SimpleEvaluator::setInitialState();
    for (operator_list::const_iterator os = current_analysis->the_domain->ops->begin();
            os != current_analysis->the_domain->ops->end(); ++os) {
        if (RPGdebug) cout << (*os)->name->getName() << "\n";
        instantiatedOp::instantiate(*os, current_analysis->the_problem, *theTC);
        if (RPGdebug) cout << instantiatedOp::howMany() << " so far\n";
    };
    if (RPGdebug && Globals::globalVerbosity & 65536) cout << instantiatedOp::howMany() << "\n";
    if (RPGdebug && Globals::globalVerbosity & 65536) instantiatedOp::writeAll(cout);


    {
        int fpass = 1;
        int numBefore;
        do {
            if (RPGdebug) cout << "\nCollecting and filter, pass " << fpass << "\n";
            instantiatedOp::createAllLiterals(current_analysis->the_problem, theTC);
            if (RPGdebug && Globals::globalVerbosity & 65536) instantiatedOp::writeAllLiterals(cout);

            numBefore = instantiatedOp::howMany();
            if (RPGdebug) {
                cout << "\tNumber of operators before filtering: " << instantiatedOp::howMany() << "\n";
            }
            instantiatedOp::filterOps(theTC);

            if (RPGdebug) {
                cout << "\tNumber of operators after filtering: " << instantiatedOp::howMany() << "\n";
            }
            ++fpass;
        } while (instantiatedOp::howMany() < numBefore);
    }
    if (RPGdebug && Globals::globalVerbosity & 65536) instantiatedOp::writeAllPNEs(cout);

    
    #ifdef ENABLE_DEBUGGING_HOOKS
    Globals::markThatActionsInPlanHaveToBeKept();
    #endif
    
    instantiatedOp::assignStateIDsToNonStaticLiteralsAndPNEs();
    
    if (RPGdebug) cout << "\nCaching action-literal dependencies\n";

    const int operatorCount = instantiatedOp::howMany();
    const int literalCount = instantiatedOp::howManyNonStaticLiterals();
    const int pneCount = instantiatedOp::howManyNonStaticPNEs();

    actionsToStartEffects = vector<list<Literal*> >(operatorCount);
    actionsToStartNegativeEffects = vector<list<Literal*> >(operatorCount);
    actionsToEndEffects = vector<list<Literal*> >(operatorCount);
    actionsToEndNegativeEffects = vector<list<Literal*> >(operatorCount);

    actionsToStartPreconditions = vector<list<Literal*> >(operatorCount);
    actionsToInvariants = vector<list<Literal*> >(operatorCount);
    actionsToEndPreconditions = vector<list<Literal*> >(operatorCount);

    actionsToStartNegativePreconditions = vector<list<Literal*> >(operatorCount);
    actionsToNegativeInvariants = vector<list<Literal*> >(operatorCount);
    actionsToEndNegativePreconditions = vector<list<Literal*> >(operatorCount);

    actionsToProcessedStartPreconditions = vector<list<Literal*> >(operatorCount);
    actionsToProcessedStartNegativePreconditions = vector<list<Literal*> >(operatorCount);

    actionsToStartNumericEffects = vector<list<NumericEffect> >(operatorCount);
    actionsToEndNumericEffects = vector<list<NumericEffect> >(operatorCount);

    actionsToStartNumericPreconditions = vector<list<NumericPrecondition> >(operatorCount);
    actionsToNumericInvariants = vector<list<NumericPrecondition> >(operatorCount);
    actionsToEndNumericPreconditions = vector<list<NumericPrecondition> >(operatorCount);

    actionsToRawConditionalEffects = vector<list<ProtoConditionalEffect*> >(operatorCount);

//  actionsToPositiveNumericEffects = vector<list<SimpleNumericEffect> >(operatorCount);
//  actionsToNegativeNumericEffects = vector<list<pair<int, double> > >(operatorCount);
    //actionsToNumericPreconditions   = vector<list<pair<int, double> > >(operatorCount);

    preconditionsToActions = vector<list<pair<int, VAL::time_spec> > >(literalCount);
    negativePreconditionsToActions = vector<list<pair<int, VAL::time_spec> > >(literalCount);
    processedNegativePreconditionsToActions = vector<list<pair<int, VAL::time_spec> > >(literalCount);
    processedPreconditionsToActions = vector<list<pair<int, VAL::time_spec> > >(literalCount);

    effectsToActions = vector<list<pair<int, VAL::time_spec> > >(literalCount);
    negativeEffectsToActions = vector<list<pair<int, VAL::time_spec> > >(literalCount);
//  positiveNumericEffectsToActions = vector<list<pair<int, double> > >(pneCount);
//  negativeNumericEffectsToActions = vector<list<pair<int, double> > >(pneCount);
//  numericPreconditionsToActions   = vector<list<pair<int, double> > >(pneCount);

    initialUnsatisfiedStartPreconditions = vector<int>(operatorCount);
    initialUnsatisfiedInvariants = vector<int>(operatorCount);
    initialUnsatisfiedEndPreconditions = vector<int>(operatorCount);

    initialUnsatisfiedProcessedStartPreconditions = vector<int>(operatorCount);

    achievedInLayer = vector<double>(literalCount);
    achievedInLayerReset = vector<double>(literalCount, -1.0);
    achievedBy = vector<pair<int, VAL::time_spec> >(literalCount);
    achievedByReset = vector<pair<int, VAL::time_spec> >(literalCount, pair<int, VAL::time_spec>(-1, VAL::E_AT_START));

    negativeAchievedInLayer = vector<double>(literalCount);
    negativeAchievedInLayerReset = vector<double>(literalCount, -1.0);
    negativeAchievedBy = vector<pair<int, VAL::time_spec> >(literalCount);
    negativeAchievedByReset = vector<pair<int, VAL::time_spec> >(literalCount, pair<int, VAL::time_spec>(-1, VAL::E_AT_START));


    linearDiscretisation = vector<LinearEffects*>(operatorCount);

//  increasedInLayer = vector<int>(pneCount);
//  increasedBy = vector<pair<int, double> >(pneCount);
//  increasedReset = vector<pair<int, double> >(pneCount);

    literals = vector<Literal*>(literalCount);
    instantiatedOps = vector<instantiatedOp*>(operatorCount);
    realRogueActions = vector<bool>(operatorCount);
    pnes = vector<PNE*>(pneCount);

    TimedPrecEffCollector::doInit(); // for robustness checking - set which predicate names are legal, and how many parameters they have

    {
        InitialStateCollector c(0, 0, theTC);
        current_analysis->the_problem->visit(&c);

        const int tilCount = c.timedInitialLiterals.size();

        timedInitialLiteralsVector = vector<FakeTILAction*>(tilCount);

        map<double, FakeTILAction>::iterator tilItr = c.timedInitialLiterals.begin();
        const map<double, FakeTILAction>::iterator tilEnd = c.timedInitialLiterals.end();

        for (int i = 0; tilItr != tilEnd; ++tilItr, ++i) {

            timedInitialLiterals.push_back(tilItr->second);
            FakeTILAction * const currFake = timedInitialLiteralsVector[i] = &(timedInitialLiterals.back());

            {
                list<Literal*> & effList = currFake->addEffects;

                list<Literal*>::iterator elItr = effList.begin();
                const list<Literal*>::iterator elEnd = effList.end();

                for (; elItr != elEnd; ++elItr) {
                    effectsToActions[(*elItr)->getStateID()].push_back(make_pair(i, VAL::E_AT));
                }
            }
            {
                list<Literal*> & effList = currFake->delEffects;

                list<Literal*>::iterator elItr = effList.begin();
                const list<Literal*>::iterator elEnd = effList.end();

                for (; elItr != elEnd; ++elItr) {
                    negativeEffectsToActions[(*elItr)->getStateID()].push_back(make_pair(i, VAL::E_AT));
                }
            }
        }

        FFEvent::tilLimit = ActionSegment::tilLimit = tilCount - 1;

    }

    actionsToMinDurations = vector<double>(operatorCount);
    actionsToMaxDurations = vector<double>(operatorCount);

//  for (int i = 0; i < pneCount; ++i) increasedReset[i] = pair<int, double>(-1, 0.0);

    {
        cout << "Number of literals: " << literals.size() << endl;
        LiteralStore::iterator lsItr = instantiatedOp::literalsBegin();
        const LiteralStore::iterator lsEnd = instantiatedOp::literalsEnd();

        int i;
        for (; lsItr != lsEnd; ++lsItr) {
            i = (*lsItr)->getStateID();
            if (i != -1) {
                literals[i] = *lsItr;
                //cout << "Literal " << i << " - " << *(*lsItr) << " with global ID " << (*lsItr)->getGlobalID() << "\n";
            }
            
            
        }
    }

    const bool PNEdebug = (Globals::globalVerbosity & 16);

    {
        if (PNEdebug) cout << "PNEs in RPG instantiation:\n";
        
        PNEStore::iterator pneItr = instantiatedOp::pnesBegin();
        const PNEStore::iterator pneEnd = instantiatedOp::pnesEnd();
        int sID;
        for (; pneItr != pneEnd; ++pneItr) {
            sID = (*pneItr)->getStateID();
            if (sID != -1) {
                pnes[sID] = *pneItr;
            }
            if (PNEdebug) cout << *(*pneItr) << " with state ID " << sID << " and global ID " << (*pneItr)->getGlobalID() << "\n";
        }
        if (PNEdebug) cout << "PNEs in operators:\n";
    }


    OpStore::iterator opsItr = instantiatedOp::opsBegin();
    const OpStore::iterator opsEnd = instantiatedOp::opsEnd();

    fixedDurationExpressions = vector<list<NumericPrecondition*> >(operatorCount);
    minDurationExpressions = vector<list<NumericPrecondition*> >(operatorCount);
    maxDurationExpressions = vector<list<NumericPrecondition*> >(operatorCount);

    const int percentageAt = operatorCount / 10;

    cout << "Constructing lookup tables:";
    cout.flush();

    for (; opsItr != opsEnd; ++opsItr) {
        instantiatedOp * const currOp = *opsItr;

        const int operatorID = currOp->getID();

        if (percentageAt) {
            if (!((operatorID + 1) % percentageAt)) {
                cout << " [" << ((operatorID + 1) / percentageAt) << "0%]";
                cout.flush();
            }
        }

        instantiatedOps[operatorID] = currOp;

        realRogueActions[operatorID] = false;

        TimedPrecEffCollector c(currOp, 0, currOp->getEnv(), theTC);
        currOp->forOp()->visit(&c);

        realRogueActions[operatorID] = checkIfRogue(c);

        const bool rogueDebug = false;

        if (RPGdebug) cout << "Operator " << operatorID << " - " << *currOp << "\n";

        if (rogueActions[operatorID]) {
            if (RPGdebug) cout << "Rogue action, skipping";
            deleteAndEmpty(c.fixedDurationExpression);
            deleteAndEmpty(c.minDurationExpression);
            deleteAndEmpty(c.maxDurationExpression);
            linearDiscretisation[operatorID] = 0;
            if (rogueDebug) cout << "Operator " << operatorID << " is a rogue\n";
            #ifdef ENABLE_DEBUGGING_HOOKS
            Globals::eliminatedAction(operatorID, "Had one or more preconditions that were not instantiated");
            #endif
        } else {

            actionsToRawConditionalEffects[operatorID] = c.condEffs;

            initialUnsatisfiedStartPreconditions[operatorID] = c.startPrec.size();
            initialUnsatisfiedInvariants[operatorID] = c.inv.size();
            initialUnsatisfiedEndPreconditions[operatorID] = c.endPrec.size();

            {

                {
                    list<Literal*> & currEffectsList = actionsToStartEffects[operatorID];

                    list<Literal*>::iterator effItr = c.startAddEff.begin();
                    const list<Literal*>::iterator effEnd = c.startAddEff.end();

                    if (RPGdebug) cout << "Operator " << operatorID << " start adds:";

                    for (; effItr != effEnd; ++effItr) {
                        const int effID = (*effItr)->getStateID();
                        assert(effID >= 0);
                        if (RPGdebug) cout << " " << *(*effItr) << " (" << effID << ")";
                        currEffectsList.push_back(*effItr);
                        effectsToActions[effID].push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
                    }
                    if (RPGdebug) cout << "\n";

                }

                {
                    list<Literal*> & currEffectsList = actionsToStartNegativeEffects[operatorID];

                    list<Literal*>::iterator effItr = c.startDelEff.begin();
                    const list<Literal*>::iterator effEnd = c.startDelEff.end();

                    if (RPGdebug) cout << "Operator " << operatorID << " start deletes:";

                    for (; effItr != effEnd; ++effItr) {
                        const int effID = (*effItr)->getStateID();
                        assert(effID >= 0);
                        if (RPGdebug) cout << " " << *(*effItr) << " (" << effID << ")";
                        currEffectsList.push_back(*effItr);
                        negativeEffectsToActions[effID].push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
                    }
                    if (RPGdebug) cout << "\n";

                }

                {
                    list<Literal*> & currEffectsList = actionsToEndEffects[operatorID];

                    list<Literal*>::iterator effItr = c.endAddEff.begin();
                    const list<Literal*>::iterator effEnd = c.endAddEff.end();

                    if (RPGdebug) cout << "Operator " << operatorID << " end adds:";

                    for (; effItr != effEnd; ++effItr) {
                        const int effID = (*effItr)->getStateID();
                        assert(effID >= 0);
                        if (RPGdebug) cout << " " << *(*effItr) << " (" << effID << ")";
                        currEffectsList.push_back(*effItr);
                        effectsToActions[effID].push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
                    }
                    if (RPGdebug) cout << "\n";

                }

                {
                    list<Literal*> & currEffectsList = actionsToEndNegativeEffects[operatorID];

                    list<Literal*>::iterator effItr = c.endDelEff.begin();
                    const list<Literal*>::iterator effEnd = c.endDelEff.end();

                    if (RPGdebug) cout << "Operator " << operatorID << " end deletes:";

                    for (; effItr != effEnd; ++effItr) {
                        const int effID = (*effItr)->getStateID();
                        assert(effID >= 0);
                        if (RPGdebug) cout << " " << *(*effItr) << " (" << effID << ")";
                        currEffectsList.push_back(*effItr);
                        negativeEffectsToActions[effID].push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
                    }
                    if (RPGdebug) cout << "\n";

                }


                {



                    actionsToStartNumericEffects[operatorID] = c.startNumericEff;

                    if (RPGdebug) {
                        cout << "Operator " << operatorID << " start numeric effects:\n";
                        list<NumericEffect>::iterator effItr = c.startNumericEff.begin();
                        const list<NumericEffect>::iterator effEnd = c.startNumericEff.end();
                        for (; effItr != effEnd; ++effItr) {
                            cout << "\t";
                            effItr->display(cout);
                            cout << "\n";
                        }
                    }
                    actionsToEndNumericEffects[operatorID] = c.endNumericEff;
                    if (RPGdebug) {
                        cout << "Operator " << operatorID << " end numeric effects:\n";
                        list<NumericEffect>::iterator effItr = c.endNumericEff.begin();
                        const list<NumericEffect>::iterator effEnd = c.endNumericEff.end();
                        for (; effItr != effEnd; ++effItr) {
                            cout << "\t";
                            effItr->display(cout);
                            cout << "\n";
                        }
                    }

                }



            }

            {


                if (RPGdebug) cout << "Operator requires at start:";


                {

                    list<Literal*> & currPreconditionsList = actionsToStartPreconditions[operatorID];

                    list<Literal*>::iterator precItr = c.startPrec.begin();
                    const list<Literal*>::iterator precEnd = c.startPrec.end();

                    for (; precItr != precEnd; ++precItr) {
                        const int precID = (*precItr)->getStateID();
                        if (precID >= 0) {
                            if (RPGdebug) cout << " " << *(*precItr) << " (" << precID << ")";
                            currPreconditionsList.push_back(*precItr);
                            preconditionsToActions[precID].push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
                        }
                    }

                    initialUnsatisfiedStartPreconditions[operatorID] = currPreconditionsList.size();

                }


                {

                    list<Literal*> & currPreconditionsList = actionsToStartNegativePreconditions[operatorID];

                    list<Literal*>::iterator precItr = c.startNegPrec.begin();
                    const list<Literal*>::iterator precEnd = c.startNegPrec.end();

                    for (; precItr != precEnd; ++precItr) {
                        const int precID = (*precItr)->getStateID();
                        if (precID >= 0) {
                            if (RPGdebug) cout << " ¬" << *(*precItr) << " (" << precID << ")";
                            currPreconditionsList.push_back(*precItr);
                            negativePreconditionsToActions[precID].push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
                        }

                    }

                    if (RPGdebug) {
                        if (currPreconditionsList.empty()) {
                            cout << " no negative facts";
                        }
                        cout << endl;
                    }

                }


                if (RPGdebug) cout << "Operator requires as an invariant:";

                {
                    list<Literal*> & currPreconditionsList = actionsToInvariants[operatorID];

                    list<Literal*>::iterator precItr = c.inv.begin();
                    const list<Literal*>::iterator precEnd = c.inv.end();

                    for (; precItr != precEnd; ++precItr) {
                        const int precID = (*precItr)->getStateID();
                        if (precID >= 0) {
                            if (RPGdebug) cout << " " << *(*precItr) << " (" << precID << ")";
                            currPreconditionsList.push_back(*precItr);
                            preconditionsToActions[precID].push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_OVER_ALL));
                        }
                    }

                    initialUnsatisfiedInvariants[operatorID] = currPreconditionsList.size();
                }


                {
                    list<Literal*> & currPreconditionsList = actionsToNegativeInvariants[operatorID];

                    list<Literal*>::iterator precItr = c.negInv.begin();
                    const list<Literal*>::iterator precEnd = c.negInv.end();

                    for (; precItr != precEnd; ++precItr) {
                        const int precID = (*precItr)->getStateID();
                        if (precID >= 0) {
                            if (RPGdebug) cout << " ¬" << *(*precItr) << " (" << precID << ")";
                            currPreconditionsList.push_back(*precItr);
                            negativePreconditionsToActions[precID].push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_OVER_ALL));
                        }

                    }

                    if (RPGdebug) {
                        if (currPreconditionsList.empty()) {
                            cout << " no negative facts";
                        }
                        cout  << endl;
                    }
                }

                if (RPGdebug) cout << "Operator requires at end:";

                {
                    list<Literal*> & currPreconditionsList = actionsToEndPreconditions[operatorID];

                    list<Literal*>::iterator precItr = c.endPrec.begin();
                    const list<Literal*>::iterator precEnd = c.endPrec.end();

                    for (; precItr != precEnd; ++precItr) {
                        const int precID = (*precItr)->getStateID();
                        if (precID >= 0) {
                            if (RPGdebug) cout << " " << *(*precItr) << " (" << precID << ")";
                            currPreconditionsList.push_back(*precItr);
                            preconditionsToActions[precID].push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
                        }

                    }

                    initialUnsatisfiedEndPreconditions[operatorID] = currPreconditionsList.size();

                }

                {
                    list<Literal*> & currPreconditionsList = actionsToEndNegativePreconditions[operatorID];

                    list<Literal*>::iterator precItr = c.endNegPrec.begin();
                    const list<Literal*>::iterator precEnd = c.endNegPrec.end();

                    for (; precItr != precEnd; ++precItr) {
                        const int precID = (*precItr)->getStateID();
                        if (precID >= 0) {
                            if (RPGdebug) cout << " ¬" << *(*precItr) << " (" << precID << ")";
                            currPreconditionsList.push_back(*precItr);
                            negativePreconditionsToActions[precID].push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
                        }

                    }

                    if (RPGdebug) {
                        if (currPreconditionsList.empty()) {
                            cout << " no negative facts";
                        }
                        cout << "\n";
                    }

                    if (currPreconditionsList.empty() && actionsToEndPreconditions[operatorID].empty()) {
                        if (RPGdebug) cout << "Operator is preconditionless at the end\n";
                        preconditionlessActions.push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
                    }
                }

                {
                    actionsToStartNumericPreconditions[operatorID].insert(actionsToStartNumericPreconditions[operatorID].end(), c.startPrecNumeric.begin(), c.startPrecNumeric.end());
                    actionsToNumericInvariants[operatorID].insert(actionsToNumericInvariants[operatorID].end(), c.invNumeric.begin(), c.invNumeric.end());
                    actionsToEndNumericPreconditions[operatorID].insert(actionsToEndNumericPreconditions[operatorID].end(), c.endPrecNumeric.begin(), c.endPrecNumeric.end());
                }
            }

            /*actionsToStartPreconditions[operatorID] = c.startPrec;
            actionsToInvariants[operatorID] = c.inv;
            actionsToEndPreconditions[operatorID] = c.endPrec;

            actionsToStartEffects[operatorID] = c.startAddEff;
            actionsToStartNegativeEffects[operatorID] = c.startDelEff;
            actionsToEndEffects[operatorID] = c.endAddEff;
            actionsToEndNegativeEffects[operatorID] = c.endDelEff;*/

            if (RPGdebug) {
                cout << "Start negative effects:\n";
                {
                    list<Literal*> & pList = actionsToStartNegativeEffects[operatorID];
                    list<Literal*>::iterator llItr = pList.begin();
                    const list<Literal*>::iterator llEnd = pList.end();

                    for (; llItr != llEnd; ++llItr) {
                        cout << "\t" << *(*llItr) << " (" << (*llItr)->getStateID() << ")\n";
                    }

                }
            }

            /* now do the tRPG processing:
             - collapse invariants into start actions
               - filter out preconditions satisfied by start effects
             - recount initial unsatisfied
             - rebuild literal -> precondition map, for new starts and old ends
            */

            for (int pass = 0; pass < 2; ++pass) {


                list<Literal*> & newStartPrecs = (pass
                                                  ? actionsToProcessedStartNegativePreconditions[operatorID] = actionsToStartNegativePreconditions[operatorID]
                                                          : actionsToProcessedStartPreconditions[operatorID] = actionsToStartPreconditions[operatorID]);

                LiteralSet oldStartEffects;
                {
                    list<Literal*>::iterator effItr = (pass ? c.startDelEff.begin() : c.startAddEff.begin());
                    const list<Literal*>::iterator effEnd = (pass ? c.startDelEff.end() : c.startAddEff.end());

                    for (; effItr != effEnd; ++effItr) {
                        oldStartEffects.insert(*effItr);
                    }
                }

                {

                    const LiteralSet::iterator notFound = oldStartEffects.end();

                    list<Literal*>::iterator precItr = (pass ? c.negInv.begin() : c.inv.begin());
                    const list<Literal*>::iterator precEnd = (pass ? c.negInv.end() : c.inv.end());

                    for (; precItr != precEnd; ++precItr) {
                        if ((*precItr)->getStateID() >= 0) {
                            if (oldStartEffects.find(*precItr) == notFound) {
                                if (RPGdebug) {
                                    if (pass) {
                                        cout << "Negative invariant " << *(*precItr) << " is not met by start effects\n";
                                    } else {
                                        cout << "Invariant " << *(*precItr) << " is not met by start effects\n";
                                    }
                                }
                                newStartPrecs.push_back(*precItr);
                            }
                        }
                    }

                }

                if (RPGdebug) {
                    if (pass) {
                        cout << "Processed start negative preconditions:\n";
                    } else {
                        cout << "Processed start preconditions:\n";
                    }
                    {

                        list<Literal*>::iterator llItr = newStartPrecs.begin();
                        const list<Literal*>::iterator llEnd = newStartPrecs.end();

                        for (; llItr != llEnd; ++llItr) {
                            if (pass) {
                                cout << "\t¬(";
                            } else {
                                cout << "\t(";
                            }
                            cout << *(*llItr) << ") " << (*llItr)->getStateID() << ")\n";
                        }

                    }
                }

                if (!pass) {
                    initialUnsatisfiedProcessedStartPreconditions[operatorID] = newStartPrecs.size();
                } else {
                    if (newStartPrecs.empty() && actionsToProcessedStartPreconditions[operatorID].empty()) {
                        preconditionlessActions.push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
                    }
                }

                {

                    list<Literal*> & currPreconditionsList = newStartPrecs;

                    list<Literal*>::iterator precItr = currPreconditionsList.begin();
                    const list<Literal*>::iterator precEnd = currPreconditionsList.end();

                    for (; precItr != precEnd; ++precItr) {
                        const int precID = (*precItr)->getStateID();
                        assert(precID >= 0);
                        if (pass) {
                            processedNegativePreconditionsToActions[precID].push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
                        } else {
                            processedPreconditionsToActions[precID].push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_START));
                        }
                    }

                }

                {

                    list<Literal*> & currPreconditionsList = (pass ? actionsToEndNegativePreconditions[operatorID] : actionsToEndPreconditions[operatorID]);

                    list<Literal*>::iterator precItr = currPreconditionsList.begin();
                    const list<Literal*>::iterator precEnd = currPreconditionsList.end();

                    for (; precItr != precEnd; ++precItr) {
                        const int precID = (*precItr)->getStateID();
                        assert(precID >= 0);
                        if (pass) {
                            processedNegativePreconditionsToActions[precID].push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
                        } else {
                            processedPreconditionsToActions[precID].push_back(pair<int, VAL::time_spec>(operatorID, VAL::E_AT_END));
                        }
                    }

                }

            }


            linearDiscretisation[operatorID] = 0;

        }

        fixedDurationExpressions[operatorID] = c.fixedDurationExpression;
        maxDurationExpressions[operatorID] = c.maxDurationExpression;
        minDurationExpressions[operatorID] = c.minDurationExpression;

        //      if (fixedDurationExpressions[operatorID]) cout << "### Stored fixed DE\n";


    }

    cout << "\n";

    {
        GoalNumericCollector c(&numericGoals, &numericGoalDeadlines, &literalGoals, &literalGoalDeadlines, prefNameToID, 0, 0, theTC);
        current_analysis->the_problem->visit(&c);

        preferences.reserve(c.builtPreferences.size());
        preferences.insert(preferences.end(), c.builtPreferences.begin(), c.builtPreferences.end());

        const int prefCount = preferences.size();
        for (int i = 0; i < prefCount; ++i) {
            prefNameToID.insert(make_pair(preferences[i].name, i));
        }

        constraints.reserve(c.builtConstraints.size());
        constraints.insert(constraints.end(), c.builtConstraints.begin(), c.builtConstraints.end());
    }


    oneShotInferForTILs(); // this is done here, as it adds extra invariants to actions

    if (doTemporalAnalysis) {
        TemporalAnalysis::processTILDeadlines();
        TemporalAnalysis::findGoalDeadlines(literalGoals, literalGoalDeadlines);

    } else {
        TemporalAnalysis::dummyDeadlineAnalysis();
    }

    findStaticLiterals();

    if (RPGBuilder::readPropositionGroups) {
        RPGBuilder::readPropositionGroupsFile();
    }
        
        
    
    postFilterUnreachableActions();

    pruneStaticPreconditions();

    buildThePropositionalBitOfConditionalEffects();

    buildRPGNumericPreconditions();
    buildRPGNumericEffects();
    handleNumericInvariants();

    {
        list<pair<int, VAL::time_spec> >::iterator plaItr = preconditionlessActions.begin();
        const list<pair<int, VAL::time_spec> >::iterator plaEnd = preconditionlessActions.end();

        while (plaItr != plaEnd) {
            const bool destroy = (plaItr->second == VAL::E_AT_START && initialUnsatisfiedProcessedStartNumericPreconditions[plaItr->first])
                                 || (plaItr->second == VAL::E_AT_END && initialUnsatisfiedNumericEndPreconditions[plaItr->first]);

            if (destroy) {
                onlyNumericPreconditionActions.push_back(*plaItr);
                const list<pair<int, VAL::time_spec> >::iterator delItr = plaItr;
                ++plaItr;
                preconditionlessActions.erase(delItr);
            } else {
                ++plaItr;
            }

        }

    }


    buildMetric(current_analysis->the_problem->metric);
    

    buildDurations(fixedDurationExpressions, minDurationExpressions, maxDurationExpressions);
    findSelfMutexes();
    doSomeUsefulMetricRPGInference();
    NumericAnalysis::findDominanceConstraintsAndMetricTrackingVariables();
    NumericAnalysis::findWhichVariablesHaveOrderIndependentEffects();
    #ifdef POPF3ANALYSIS
    NumericAnalysis::findWhichVariablesAreOnlyInAtStarts();
    NumericAnalysis::findGoalNumericUsageLimits();    
    #endif
    
    NumericAnalysis::findMaximumGradients();
    
    checkConditionalNumericEffectsAreOnlyOnMetricTrackingVariables();
    
    removePointlessEffects();
    
    separateOptimisationTILs();
    findUninterestingnessCriteria();
    findConcurrentRedundantActions();

    if (doTemporalAnalysis) {
        TemporalAnalysis::findActionTimestampLowerBounds();
        postFilterIrrelevantActions();
    } else {
        globalHeuristic = generateRPGHeuristic();
    }
    
    TemporalAnalysis::findCompressionSafeActions();

    #ifdef ENABLE_DEBUGGING_HOOKS
    if (Globals::planFilename) {
        cout << "Final check for whether actions have been erroneously pruned\n";
        for (int i = 0; i < operatorCount; ++i) {
            if (realRogueActions[i]) {
                Globals::eliminatedAction(i, "Noted at the end of preprocessing");
            }
        }
    }
    #endif
};




void RPGBuilder::getInitialState(LiteralSet & initialState, vector<double> & initialFluents)
{

    static LiteralSet isCache;
    static vector<double> ifCache;
    static bool cached = false;

    if (!cached) {

        InitialStateCollector c(0, 0, theTC);
        current_analysis->the_problem->visit(&c);

        isCache = c.initialState;
        ifCache = c.initialFluents;

        cached = true;
    }

    initialState = isCache;
    initialFluents = ifCache;


    assert(initialFluents.size() == instantiatedOp::howManyNonStaticPNEs());

};

void RPGBuilder::getNonStaticInitialState(LiteralSet & initialState, vector<double> & initialFluents)
{

    static LiteralSet isCache;
    static vector<double> ifCache;
    static bool cached = false;

    if (!cached) {

        getInitialState(isCache, ifCache);

        LiteralSet::iterator lsItr = isCache.begin();
        const LiteralSet::iterator lsEnd = isCache.end();

        while (lsItr != lsEnd) {
            if (isStatic(*lsItr).first) {
                const LiteralSet::iterator lsDel = lsItr++;
                isCache.erase(lsDel);
            } else {
                ++lsItr;
            }
        }

        cached = true;
    }

    initialState = isCache;
    initialFluents = ifCache;



};


bool RPGBuilder::stepNeedsToHaveFinished(const ActionSegment & act, const MinimalState & theState, set<int> & toBeNonMutex)
{

    const bool debug = false;

    const int actID = (act.first ? act.first->getID() : -1);

    list<Literal*> * willDelete = 0;
    list<Literal*> * willAdd = 0;

    list<Literal*> * needs = 0;
    list<Literal*> * negativeNeeds = 0;

    unsigned int invariantExemption = INT_MAX;

    if (actID == -1) {

        FakeTILAction * const tilItr = timedInitialLiteralsVector[act.divisionID];
        willDelete = &(tilItr->delEffects);
        willAdd = &(tilItr->addEffects);

    } else {

        if (act.second == VAL::E_AT_START) {
            willDelete = &(RPGBuilder::actionsToStartNegativeEffects[actID]);
            willAdd = &(RPGBuilder::actionsToStartEffects[actID]);
            needs = &(RPGBuilder::actionsToProcessedStartPreconditions[actID]);
            negativeNeeds = &(RPGBuilder::actionsToProcessedStartNegativePreconditions[actID]);
        } else {
            willDelete = &(RPGBuilder::actionsToEndNegativeEffects[actID]);
            willAdd = &(RPGBuilder::actionsToEndEffects[actID]);
            needs = &(RPGBuilder::actionsToEndPreconditions[actID]);
            negativeNeeds = &(RPGBuilder::actionsToEndNegativePreconditions[actID]);
            const map<int, set<int> >::const_iterator saItr = theState.startedActions.find(actID);
            if (saItr == theState.startedActions.end()) {
                if (debug) cout << "Action has not started - cannot be ended\n";
                return false;
            }

            invariantExemption = *(saItr->second.begin());
        }

    }

    #ifdef TOTALORDERSTATES
    
    if (willDelete) {

        list<Literal*>::iterator fItr = willDelete->begin();
        const list<Literal*>::iterator fEnd = willDelete->end();

        for (; fItr != fEnd; ++fItr) {
            const map<int,int>::const_iterator invItr = theState.invariants.find((*fItr)->getStateID());
            
            if (invItr != theState.invariants.end()) {
                if (debug) cout << "Cannot skip past invariants on " << *(*fItr) << " owned by executing non-compression-safe actions\n";
                return false;
            }
            
            const StateBFacts::const_iterator csInvItr = theState.firstAnnotations.find((*fItr)->getStateID());
            
            if (csInvItr != theState.firstAnnotations.end()) {
                // must then come after what needs that fact
                toBeNonMutex.insert(csInvItr->second.second.begin(),csInvItr->second.second.end());
            }
        }
    }
    
    if (needs) {
        
        list<Literal*>::iterator fItr = needs->begin();
        const list<Literal*>::iterator fEnd = needs->end();
        
        for (; fItr != fEnd; ++fItr) {
            const StateFacts::const_iterator invItr = theState.first.find((*fItr)->getStateID());
            if (invItr != theState.first.end()) {
                const StateBFacts::const_iterator invBItr = theState.firstAnnotations.find((*fItr)->getStateID());
                
                if (invBItr != theState.firstAnnotations.end()) {
                    toBeNonMutex.insert(invBItr->second.first.begin(), invBItr->second.first.end());
                }

                // fact is true, carry on
                
                continue;
            }
            if (debug) cout << "Would need an invariant " << *(*fItr) << ", which is not currently true\n";
            
            return false;
            
        }
        
    }
    
    #else
    
    const StateFacts::const_iterator stateEnd = theState.first.end();
    const StateFacts::const_iterator retiredStateEnd = theState.retired.end();

    if (willDelete) {

        list<Literal*>::iterator fItr = willDelete->begin();
        const list<Literal*>::iterator fEnd = willDelete->end();

        for (; fItr != fEnd; ++fItr) {
            const map<int, PropositionAnnotation>::const_iterator invItr = theState.first.find((*fItr)->getStateID());
            if (invItr != stateEnd) {
                map<StepAndBeforeOrAfter, bool>::const_iterator dfItr = invItr->second.deletableFrom.begin();
                const map<StepAndBeforeOrAfter, bool>::const_iterator dfEnd = invItr->second.deletableFrom.end();

                for (; dfItr != dfEnd; ++dfItr) {
                    if (act.second == VAL::E_AT_END && dfItr->first.stepID == invariantExemption) continue;
                    if (dfItr->second == UNSAFETOSKIP) {
                        if (debug) cout << "Cannot skip past invariants on " << *(*fItr) << " owned by step " << dfItr->first.stepID << endl;
                        return false;
                    }
                    toBeNonMutex.insert(dfItr->first.stepID);
                }
            }
        }
    }

    if (willAdd) {

        list<Literal*>::iterator fItr = willAdd->begin();
        const list<Literal*>::iterator fEnd = willAdd->end();

        for (; fItr != fEnd; ++fItr) {
            const map<int, PropositionAnnotation>::const_iterator invItr = theState.retired.find((*fItr)->getStateID());
            if (invItr != retiredStateEnd) {
                map<StepAndBeforeOrAfter, bool>::const_iterator dfItr = invItr->second.addableFrom.begin();
                const map<StepAndBeforeOrAfter, bool>::const_iterator dfEnd = invItr->second.addableFrom.end();

                for (; dfItr != dfEnd; ++dfItr) {
                    if (act.second == VAL::E_AT_END && dfItr->first.stepID == invariantExemption) continue;
                    if (dfItr->second == UNSAFETOSKIP) {
                        if (debug) cout << "Cannot skip past negative invariants on " << *(*fItr) << " owned by step " << dfItr->first.stepID << endl;
                        return false;
                    }
                    toBeNonMutex.insert(dfItr->first.stepID);
                }
            }
        }
    }
    
    if (needs) {

        list<Literal*>::iterator fItr = needs->begin();
        const list<Literal*>::iterator fEnd = needs->end();

        for (; fItr != fEnd; ++fItr) {
            const map<int, PropositionAnnotation>::const_iterator invItr = theState.first.find((*fItr)->getStateID());
            if (invItr == stateEnd) {
                if (debug) cout << "Would need an invariant " << *(*fItr) << ", which is not currently true\n";
                return false;
            }
            const int added = invItr->second.availableFrom.stepID;
            if (added != -1) {
                toBeNonMutex.insert(added);
            }
        }

    }

    if (negativeNeeds) {

        list<Literal*>::iterator fItr = negativeNeeds->begin();
        const list<Literal*>::iterator fEnd = negativeNeeds->end();

        for (; fItr != fEnd; ++fItr) {
            {
                const map<int, PropositionAnnotation>::const_iterator invItr = theState.first.find((*fItr)->getStateID());
                if (invItr != stateEnd) {
                    if (debug) cout << "Need ¬" << *(*fItr) << " but it is currently true\n";
                    return false;
                }
            }
            const map<int, PropositionAnnotation>::const_iterator invItr = theState.retired.find((*fItr)->getStateID());

            if (invItr != retiredStateEnd) {
                const int deleted = invItr->second.negativeAvailableFrom.stepID;
                if (deleted != -1) {
                    toBeNonMutex.insert(deleted);
                }
            }
        }

    }
    
    #endif



    return true;
}

double RPGBuilder::getOpMinDuration(instantiatedOp* op, const int & i)
{
    return getOpMinDuration(op->getID(), i);
};

double RPGBuilder::getOpMinDuration(const int & op, const int & i)
{

    if (i == -1) return actionsToMinDurations[op];

//  if (linearDiscretisation[op]) {
//      return linearDiscretisation[op]->durations[i];
//  } else {
    return actionsToMinDurations[op];
//  }
};

double RPGBuilder::getOpMaxDuration(instantiatedOp* op, const int & i)
{
    return getOpMaxDuration(op->getID(), i);

};

double RPGBuilder::getOpMaxDuration(const int & op, const int & i)
{

    if (i == -1) return actionsToMaxDurations[op];

//  if (linearDiscretisation[op]) {
//      return linearDiscretisation[op]->durations[i];
//  } else {
    return actionsToMaxDurations[op];
//  }

};

#ifdef STOCHASTICDURATIONS
double getValue(const vector<double> & fluents, const pair<int, PNE*> & v) {
    if (v.first != -1) {
        return fluents[v.first];
    }
    assert(EFT(v.second->getHead())->isStatic());
    return EFT(v.second->getHead())->getInitial(v.second->begin(), v.second->end()).second;
}
#else
double getValue(const vector<double> & fluents, const int & v) {
    return fluents[v];
}
#endif

double RPGBuilder::DurationExpr::minOf(const vector<double> & minFluents, const vector<double> & maxFluents)
{

    double toReturn = constant;
    const int lim = weights.size();

    for (int i = 0; i < lim; ++i) {
        const double & currW = weights[i];      
        if (currW < 0.0) {            
            toReturn += currW * getValue(maxFluents,variables[i]);
        } else {
            toReturn += currW * getValue(minFluents,variables[i]);
        }
    }

    return toReturn;

};

double RPGBuilder::DurationExpr::maxOf(const vector<double> & minFluents, const vector<double> & maxFluents)
{

    static const bool debug = false;

    if (debug) {
        cout << "Calculating max of the duration: " << constant;
    }

    double toReturn = constant;
    const int lim = weights.size();

    for (int i = 0; i < lim; ++i) {
        const double & currW = weights[i];
        if (currW < 0.0) {
            if (debug) {
                cout << " - " << -currW << "x" << getValue(minFluents, variables[i]);
            }
            toReturn += currW * getValue(minFluents, variables[i]);
        } else {
            if (debug) {
                cout << " + " << currW << "x" << getValue(maxFluents, variables[i]);
            }
            toReturn += currW * getValue(maxFluents, variables[i]);
        }
    }

    if (debug) {
        cout << " = " << toReturn << endl;
    }

    return toReturn;
};


pair<double, double> RPGBuilder::getOpDuration(instantiatedOp* op, const int & div, const vector<double> & minFluents, const vector<double> & maxFluents)
{
    return getOpDuration(op->getID(), div, minFluents, maxFluents);
};

pair<double, double> RPGBuilder::getOpDuration(const int & a, const int & div, const vector<double> & minFluents, const vector<double> & maxFluents)
{
    assert(!rogueActions[a]);
    if (rpgDurationExpressions[a].empty()) {
        return pair<double, double>(0.001, 0.001);
    }

    bool maxSet = false;
    pair<double, double> toReturn(0.001, 1000000000.0);

    for (int pass = 0; pass < 3; ++pass) {
        list<DurationExpr *> * const currDE =
            (pass ? (pass == 2 ? &(rpgDurationExpressions[a][div]->max) : &(rpgDurationExpressions[a][div]->min))
                     : &(rpgDurationExpressions[a][div]->fixed)
                    );

        list<DurationExpr *>::iterator mdItr = currDE->begin();
        const list<DurationExpr *>::iterator mdEnd = currDE->end();

        for (; mdItr != mdEnd; ++mdItr) {
            if (pass != 2) {
                const double newMin = (*mdItr)->minOf(minFluents, maxFluents);
                if (newMin > toReturn.first) {
                    toReturn.first = newMin;
                }
            }
            if (pass != 1) {
                const double newMax = (*mdItr)->maxOf(minFluents, maxFluents);
                if (!maxSet) {
                    maxSet = true;
                    toReturn.second = newMax;
                } else if (newMax < toReturn.second) {
                    toReturn.second = newMax;
                }
            }
        }
    }

    return toReturn;
};


void RPGBuilder::getEffects(instantiatedOp* op, const bool & start, list<Literal*> & add, list<Literal*> & del, list<NumericEffect> & numeric)
{

    const int actID = op->getID();

    //cout << "getting effects for action " << actID << "\n";

    if (start) {

        {
            list<Literal*> & pList = actionsToStartEffects[actID];
            add.clear(); add.insert(add.end(), pList.begin(), pList.end());
        }
        {
            list<Literal*> & pList = actionsToStartNegativeEffects[actID];
            del.clear(); del.insert(del.end(), pList.begin(), pList.end());
        }
        {
            list<NumericEffect> & nList = actionsToStartNumericEffects[actID];
            numeric.clear(); numeric.insert(numeric.end(), nList.begin(), nList.end());
        }

    } else {

        {
            list<Literal*> & pList = actionsToEndEffects[actID];
            add.clear(); add.insert(add.end(), pList.begin(), pList.end());
        }
        {
            list<Literal*> & pList = actionsToEndNegativeEffects[actID];
            del.clear(); del.insert(del.end(), pList.begin(), pList.end());
        }
        {
            list<NumericEffect> & nList = actionsToEndNumericEffects[actID];
            numeric.clear(); numeric.insert(numeric.end(), nList.begin(), nList.end());
        }

    }

    /*const VAL::time_spec toMatch = (start ? VAL::E_AT_START : E_AT_END);

    {
        VAL::pc_list<VAL::timed_effect *>::iterator sEffItr = op->forOp()->effects->timed_effects.begin();
        const VAL::pc_list<VAL::timed_effect *>::iterator sEffEnd = op->forOp()->effects->timed_effects.end();

        for (; sEffItr != sEffEnd; ++sEffItr) {
            if ((*sEffItr)->ts == toMatch) {

                for (int pass = 0; pass < 2; ++pass) {
                    VAL::pc_list<VAL::simple_effect *> & listToUse = (pass ? (*sEffItr)->effs->del_effects : (*sEffItr)->effs->add_effects);
                    list<Literal*> & listToAddTo = (pass ? del : add);

                    VAL::pc_list<VAL::simple_effect *>::iterator lItr = listToUse.begin();
                    const VAL::pc_list<VAL::simple_effect *>::iterator lEnd = listToUse.end();

                    for (; lItr != lEnd; ++lItr) {
                        Literal l((*lItr)->prop,op->getEnv());
                        listToAddTo.push_back(instantiatedOp::findLiteral(&l));
                    }

                }
            }
        }

    }

    if (start) { // for instantaneous actions, just have a start action which has everything
        {
            list<int>::iterator effItr = actionsToEffects[actID].begin();
            const list<int>::iterator effEnd = actionsToEffects[actID].end();

            for (; effItr != effEnd; ++effItr) add.push_back(literals[*effItr]);
        }
        {
            list<int>::iterator effItr = actionsToNegativeEffects[actID].begin();
            const list<int>::iterator effEnd = actionsToNegativeEffects[actID].end();

            for (; effItr != effEnd; ++effItr) del.push_back(literals[*effItr]);
        }
        {
            numeric.insert(numeric.end(), actionsToNumericEffects[actID].begin(), actionsToNumericEffects[actID].end());
        }
    }*/

};

void RPGBuilder::getPrecInv(instantiatedOp* op, const bool & start, list<Literal*> & precs, list<Literal*> & inv, list<NumericPrecondition> & numericPrec, list<NumericPrecondition> & numericInv)
{

    //TimedPrecCollector c(0,op->getEnv(),theTC);
    //op->forOp()->visit(&c);

    //cout << "Looking up PrecInv for op " << op->getID() << " - " << *op << "\n";

    const int opIndex = op->getID();

    list<Literal*> & invList = actionsToInvariants[opIndex];

    inv.clear(); inv.insert(inv.end(), invList.begin(), invList.end());
    {
        list<NumericPrecondition> & ninv = actionsToNumericInvariants[opIndex];
        numericInv.clear(); numericInv.insert(numericInv.end(), ninv.begin(), ninv.end());
    }
    precs.clear();
    numericPrec.clear();
    if (start) {
        list<Literal*> & pList = actionsToStartPreconditions[opIndex];


        precs.insert(precs.end(), pList.begin(), pList.end());

        list<NumericPrecondition> & npList = actionsToStartNumericPreconditions[opIndex];

        numericPrec.insert(numericPrec.end(), npList.begin(), npList.end());
    } else {
        list<Literal*> & pList = actionsToEndPreconditions[opIndex];
        precs.insert(precs.end(), pList.begin(), pList.end());

        list<NumericPrecondition> & npList = actionsToEndNumericPreconditions[opIndex];

        numericPrec.insert(numericPrec.end(), npList.begin(), npList.end());
    }



};
/*
void RPGBuilder::getCollapsedAction(instantiatedOp* op, list<Literal*> & pre, list<Literal*> & add, list<Literal*> & del, list<NumericPrecondition> & numericPre, list<NumericEffect> & numericEff) {


    const int actID = op->getID();
    {
        list<int>::iterator effItr = actionsToEffects[actID].begin();
        const list<int>::iterator effEnd = actionsToEffects[actID].end();

        for (; effItr != effEnd; ++effItr) add.push_back(literals[*effItr]);
    }

    {
        list<int>::iterator effItr = actionsToNegativeEffects[actID].begin();
        const list<int>::iterator effEnd = actionsToNegativeEffects[actID].end();

        for (; effItr != effEnd; ++effItr) del.push_back(literals[*effItr]);
    }

    {
        list<int>::iterator effItr = actionsToPreconditions[actID].begin();
        const list<int>::iterator effEnd = actionsToPreconditions[actID].end();

        for (; effItr != effEnd; ++effItr) pre.push_back(literals[*effItr]);
    }
    {
        numericPre.insert(numericPre.end(), actionsToNumericPreconditions[actID].begin(), actionsToNumericPreconditions[actID].end());
    }
    {
        numericEff.insert(numericEff.end(), actionsToNumericEffects[actID].begin(), actionsToNumericEffects[actID].end());
    }

};
*/

void RPGBuilder::buildThePropositionalBitOfConditionalEffects()
{

    const int opCount = instantiatedOps.size();
    actionsToConditionalEffects = vector<list<ConditionalEffect> >(opCount);

    for (int i = 0; i < opCount; ++i) {
        list<ProtoConditionalEffect*>::iterator ceItr = actionsToRawConditionalEffects[i].begin();
        const list<ProtoConditionalEffect*>::iterator ceEnd = actionsToRawConditionalEffects[i].end();

        for (; ceItr != ceEnd; ++ceItr) {

            actionsToConditionalEffects[i].push_back(ConditionalEffect());

            ProtoConditionalEffect * const currRaw = *ceItr;
            ConditionalEffect & currCE = actionsToConditionalEffects[i].back();

            for (int pass = 0; pass < 3; ++pass) {
                list<Literal*>::const_iterator clItr;
                list<Literal*>::const_iterator clEnd;
                VAL::time_spec currTS;

                switch (pass) {
                case 0: {
                    clItr = currRaw->startPrec.begin();
                    clEnd = currRaw->startPrec.end();
                    currTS = VAL::E_AT_START;
                    break;
                }
                case 1: {
                    clItr = currRaw->inv.begin();
                    clEnd = currRaw->inv.end();
                    currTS = VAL::E_OVER_ALL;
                    break;
                }
                case 2: {
                    clItr = currRaw->endPrec.begin();
                    clEnd = currRaw->endPrec.end();
                    currTS = VAL::E_AT_END;
                    break;
                }
                default:
                {
                    cerr << "Time specifier on condition effects have to be one of either 'at start', 'over all' or 'at end'\n";
                    exit(1);
                }
                }

                for (; clItr != clEnd; ++clItr) {

                    // For now, we can only handle propositions governed by TILs, for metric optimisation purposes

                    list<pair<int, VAL::time_spec> > & addedBy = effectsToActions[(*clItr)->getStateID()];

                    list<pair<int, VAL::time_spec> >::const_iterator aItr = addedBy.begin();
                    const list<pair<int, VAL::time_spec> >::const_iterator aEnd = addedBy.end();

                    for (; aItr != aEnd; ++aItr) {
                        if (aItr->second != VAL::E_AT) { // if the achiever is not a TIL
                            postmortem_noADL();
                        }
                    }

                    currCE.addCondition(*clItr, currTS);
                }
            }

            // Furthermore, for now, we can't have propositional effects conditionally - only
            // effects on metric-tracking variables

            {
                list<Literal*> & currList = currRaw->startAddEff;
                const VAL::time_spec currTS = VAL::E_AT_START;
                list<Literal*>::iterator clItr = currList.begin();
                const list<Literal*>::iterator clEnd = currList.end();

                for (; clItr != clEnd; ++clItr) {
                    postmortem_noADL();
                    currCE.addAddEffect(*clItr, currTS);
                }
            }

            {
                list<Literal*> & currList = currRaw->endAddEff;
                const VAL::time_spec currTS = VAL::E_AT_END;
                list<Literal*>::iterator clItr = currList.begin();
                const list<Literal*>::iterator clEnd = currList.end();

                for (; clItr != clEnd; ++clItr) {
                    postmortem_noADL();
                    currCE.addAddEffect(*clItr, currTS);
                }
            }

            {
                list<Literal*> & currList = currRaw->startDelEff;
                const VAL::time_spec currTS = VAL::E_AT_START;
                list<Literal*>::iterator clItr = currList.begin();
                const list<Literal*>::iterator clEnd = currList.end();

                for (; clItr != clEnd; ++clItr) {
                    postmortem_noADL();
                    currCE.addDeleteEffect(*clItr, currTS);
                }
            }

            {
                list<Literal*> & currList = currRaw->endDelEff;
                const VAL::time_spec currTS = VAL::E_AT_END;
                list<Literal*>::iterator clItr = currList.begin();
                const list<Literal*>::iterator clEnd = currList.end();

                for (; clItr != clEnd; ++clItr) {
                    postmortem_noADL();
                    currCE.addDeleteEffect(*clItr, currTS);
                }
            }

        }
    }

};

void RPGBuilder::buildDurations(vector<list<NumericPrecondition*> > & fixedDurations, vector<list<NumericPrecondition*> > & minDurations, vector<list<NumericPrecondition*> > & maxDurations)
{

    const bool durDebug = (Globals::globalVerbosity & 32);

    const int lim = fixedDurations.size();

    rpgDurationExpressions = vector<vector<RPGDuration*> >(lim);
    nonTemporalDuration.resize(lim, 0.001);
    
    if (durDebug) {
        cout << "Number of actions potentially needing durations: " << lim << endl;
    }
    
    for (int i = 0; i < lim; ++i) {

        if (!rogueActions[i]) {

            if (durDebug) {
                cout << "Considering durations of " << *(getInstantiatedOp(i)) << ":";
                cout.flush();
            }
            
            bool durationConflict = false;

            rpgDurationExpressions[i] = vector<RPGDuration*>(1);

            static list<NumericPrecondition*>* allDurations[3];

            allDurations[0] = &(fixedDurations[i]);
            allDurations[1] = &(minDurations[i]);
            allDurations[2] = &(maxDurations[i]);

            bool durative = (!allDurations[0]->empty() || !allDurations[1]->empty() || !allDurations[2]->empty());
            
            if (durative) {
                pair<double, bool> evalDurMax(1000000000.0, false);
                pair<double, bool> evalDurMin(0.000, false);

                for (int pass = 0; pass < 3; ++pass) {

                    if (durDebug) {
                        cout << " [" << pass << "]";
                        cout.flush();
                    }
                    list<NumericPrecondition*>::iterator dlItr = allDurations[pass]->begin();
                    const list<NumericPrecondition*>::iterator dlEnd = allDurations[pass]->end();

                    for (; dlItr != dlEnd; ++dlItr) {
                        pair<double, bool> newEval((*dlItr)->constRHS());
                        if (newEval.second) {

                            if (!pass) { // special case for equals constraints

                                if (newEval.first > evalDurMax.first
                                        || newEval.first < evalDurMin.first) {
                
                                    #ifdef ENABLE_DEBUGGING_HOOKS
                                    {
                                        ostringstream s;
                                        s << "Fixed duration of " << newEval.first << " is not acceptable";
                                        Globals::eliminatedAction(i, s.str().c_str());
                                    } 
                                    #endif

                                    durationConflict = true;
                                    break;
                                }
                            }

                            if (pass != 1 && newEval.first < 0.0) {
                                // if the maximum or fixed duration is negative, action
                                // can never be applied
                                #ifdef ENABLE_DEBUGGING_HOOKS
                                {
                                    ostringstream s;
                                    if (pass == 0) {                                        
                                        s << "Duration of action was fixed to " << newEval.first;
                                    } else if (pass == 2) {
                                        s << "Maximum duration of action was negative: " << newEval.first;
                                    }
                                    
                                    Globals::eliminatedAction(i, s.str().c_str());
                                } 
                                #endif
                                durationConflict = true;
                                break;
                            }

                            if (pass != 1) { // if it's a fixed or max constraint
                                if (evalDurMax.second) {
                                    if (newEval.first < evalDurMax.first) evalDurMax.first = newEval.first;
                                } else {
                                    evalDurMax = newEval;
                                }
                            }

                            if (pass != 2) { // if it's a fixed or min constraint
                                if (evalDurMin.second) {
                                    if (newEval.first > evalDurMin.first) evalDurMin.first = newEval.first;
                                } else {
                                    evalDurMin = newEval;
                                }

                            }
                        }
                    }
                }

                if (evalDurMin.first == 0.0) {
                    // For now we only support 0-duration durative actions if their duration is fixed, i.e. min=max=0.0
                    // As such, we impose a lower bound of 0.001 on the duration of any action non-fixed-duration action
                    if (evalDurMax.first != 0.0) {
                        if (evalDurMax.first < 0.001) {
                            durationConflict = true;
                            #ifdef ENABLE_DEBUGGING_HOOKS
                            Globals::eliminatedAction(i, "Unless an action's duration is fixed to 0.0, it must be at least 0.001 long");
                            #endif
                        } else {
                            evalDurMin.first = 0.001;
                        }
                    }
                }
                if (evalDurMax.first == 0.0) {
                    if (evalDurMin.first > 0.0) {
                        #ifdef ENABLE_DEBUGGING_HOOKS
                        Globals::eliminatedAction(i, "Unless an action's duration is fixed to 0.0, it must be at least 0.001 long");
                        #endif
                        durationConflict = true;
                    }
                }

                if (!durationConflict && (evalDurMin.first == 0.0 || evalDurMax.first == 0.0)) {
                    string diagnosis;                    
                    if (!actionsToInvariants[i].empty()) {
                        diagnosis = "* Propositional over all conditions\n";
                        durationConflict = true;
                    }
                    if (!actionsToNegativeInvariants[i].empty()) {
                        diagnosis += "* Negative propositional over all conditions\n";
                        durationConflict = true;
                    }
                    if (!actionsToNumericInvariants[i].empty()) {
                        diagnosis += "* Numeric over all conditions\n";
                        durationConflict = true;
                    }
                    if (!actionsToEndPreconditions[i].empty()) {
                        diagnosis += "* Propositional at end conditions\n";
                        durationConflict = true;
                    }
                    if (!actionsToEndNegativePreconditions[i].empty()) {
                        diagnosis += "* Negative at end conditions\n";
                        durationConflict = true;
                    }
                    if (!actionsToEndNumericPreconditions[i].empty()) {
                        diagnosis += "* Numeric at end conditions\n";
                        durationConflict = true;
                    }
                    if (durationConflict) {
                        static bool issuedWarning = false;
                        static bool issuedSecondWarning = false;
                        
                        if (!issuedWarning) {
                            cout << "== Warning ==\n\n";
                            cout << "The action " << *(instantiatedOps[i]) << " has ";
                            
                            if (evalDurMin.first == 0.0 && evalDurMax.first == 0.0) {
                                cout << "a fixed duration of zero";
                            } else if (evalDurMin.first == 0.0) {
                                cout << "a minimum duration of zero";
                            } else {
                                cout << "a maximum duration of zero";
                            }
                            cout << ",\nbut also has:\n\n";
                            cout << diagnosis;
                            cout << "\nIn this case, correct handling of the action is unclear (e.g. at what point\n";
                            cout << "must the over all/at end conditions hold if there is no gap between the start\n";
                            cout << "and end of the action).  If the intention is for the action to be truly\n";
                            cout << "instantaneous, use a PDDL (:action rather than a (:durative-action.  For now,\n";
                            cout << "however, the action has been discarded.\n\n";
                            issuedWarning = true;
                        } else if (!issuedSecondWarning) {
                            cout << "Other actions to have been discarded due to zero-durations include\n";
                            cout << *(instantiatedOps[i]) << endl;
                            issuedSecondWarning = true;
                        }
                    }
                }    

                if (durationConflict) {
                    pruneIrrelevant(i);
                } else {
                    if (evalDurMin.first != 0.0 && evalDurMax.first != 0.0) {                        
                        if (evalDurMin.second) {
                            actionsToMinDurations[i] = evalDurMin.first;
                        } else {
                            actionsToMinDurations[i] = 0.001;
                        }

                        if (evalDurMax.second) {
                            actionsToMaxDurations[i] = evalDurMax.first;
                        } else {
                            actionsToMaxDurations[i] = 1000000000.0;
                        }

                        list<DurationExpr *> fixedExpr = buildDEList(*(allDurations[0]));
                        list<DurationExpr *> minExpr = buildDEList(*(allDurations[1]));
                        list<DurationExpr *> maxExpr = buildDEList(*(allDurations[2]));

                        rpgDurationExpressions[i][0] = new RPGDuration(fixedExpr, minExpr, maxExpr);

                        problemIsNotTemporal = false;
                    } else {
                        durative = false;
                        
                        if (durDebug) {
                            cout << "Duration of " << *getInstantiatedOp(i) << " is 0, making action non-temporal\n";
                        }
                        assert(actionsToInvariants[i].empty());
                        assert(actionsToNegativeInvariants[i].empty());
                        assert(actionsToNumericInvariants[i].empty());
                        
                        assert(actionsToEndPreconditions[i].empty());
                        assert(actionsToEndNegativePreconditions[i].empty());
                        assert(actionsToEndNumericPreconditions[i].empty());

                        for (int pol = 0; pol < 2; ++pol) {
                            list<Literal*> & currList = (pol ? actionsToEndNegativeEffects[i] : actionsToEndEffects[i]);
                            list<Literal*> & destList = (pol ? actionsToStartNegativeEffects[i] : actionsToStartEffects[i]);
                            list<Literal*>::const_iterator eItr = currList.begin();
                            const list<Literal*>::const_iterator eEnd = currList.end();
                            
                            for (; eItr != eEnd; ++eItr) {
                                list<Literal*>::iterator oItr = destList.begin();
                                const list<Literal*>::iterator oEnd = destList.end();
                                
                                for (; oItr != oEnd; ++oItr) {
                                    if (*oItr == *eItr) break;
                                }
                                
                                if (oItr == oEnd) {
                                    destList.push_back(*eItr);
                                    if (pol) {
                                        negativeEffectsToActions[(*eItr)->getStateID()].push_back(make_pair(i, VAL::E_AT_START));
                                    } else {
                                        effectsToActions[(*eItr)->getStateID()].push_back(make_pair(i, VAL::E_AT_START));
                                    }
                                }
                                if (pol) {
                                    negativeEffectsToActions[(*eItr)->getStateID()].remove(make_pair(i, VAL::E_AT_END));
                                } else {
                                    effectsToActions[(*eItr)->getStateID()].remove(make_pair(i, VAL::E_AT_END));
                                }
                            }
                            currList.clear();
                        }
                        
                        {                            
                            list<int> & currList = actionsToRPGNumericEndEffects[i];
                            list<int> & destList = actionsToRPGNumericStartEffects[i];
                                                        
                            list<int>::const_iterator eItr = currList.begin();
                            const list<int>::const_iterator eEnd = currList.end();
                            
                            for (; eItr != eEnd; ++eItr) {
                                list<int>::iterator oItr = destList.begin();
                                const list<int>::iterator oEnd = destList.end();
                                
                                for (; oItr != oEnd; ++oItr) {
                                    if (*oItr == *eItr) break;
                                }
                                
                                if (oItr == oEnd) {
                                    destList.push_back(*eItr);
                                    rpgNumericEffectsToActions[*eItr].push_back(make_pair(i, VAL::E_AT_START));
                                }
                                rpgNumericEffectsToActions[*eItr].remove(make_pair(i, VAL::E_AT_END));
                            }
                            
                            currList.clear();
                        }
                        
                        nonTemporalDuration[i] = 0.0;
                    }
                }
                
            }
            
            if (!durative) {
                actionsToMinDurations[i] = 0.001;
                actionsToMaxDurations[i] = 0.001;

                rpgDurationExpressions[i] = vector<RPGDuration*>();
            }
            if (durDebug) {
                cout << endl;
            }
            
        }        
    }

}

RPGBuilder::LinearEffects * RPGBuilder::buildLE(const int & i)
{

    const bool localDebug = false;
    list<RPGBuilder::NumericEffect> & effList = actionsToStartNumericEffects[i];

    LinearEffects * toReturn = 0;

    list<RPGBuilder::NumericEffect>::iterator elItr = effList.begin();
    const list<RPGBuilder::NumericEffect>::iterator elEnd = effList.end();

    while (elItr != elEnd) {
        bool isCTS = false;
        list<Operand>::iterator fItr = elItr->formula.begin();
        const list<Operand>::iterator fEnd = elItr->formula.end();

        for (; fItr != fEnd; ++fItr) {
            if (fItr->numericOp == NE_FLUENT && (fItr->fluentValue == -2 || fItr->fluentValue == -18)) {
                isCTS = true;
                break;
            }
        }
        if (isCTS) {
            if (!toReturn) {
                if (localDebug) cout << "Found CTS effects on action " << *(instantiatedOps[i]) << "\n";
                toReturn = new LinearEffects();
//              toReturn->durations = vector<double>(1);
                toReturn->effects = vector<vector<LinearEffects::EffectExpression> >(1);
                toReturn->divisions = 1;
//              list<NumericPrecondition*> & currFixedExpr = fixedDurationExpressions[i];
//              assert(!currFixedExpr.empty());
//              assert(minDurationExpressions[i].empty());
//              assert(maxDurationExpressions[i].empty());

//              pair<double,bool> evalDur(currFixedExpr.front()->constRHS());
//              assert(evalDur.second);
//              toReturn->totalDuration = evalDur.first;
//              toReturn->durations[0] = evalDur.first;
//              if (localDebug) cout << "Duration fixed to " << toReturn->durations[0] << "\n";
            }

            pair<list<double>, list<int> > weightedSum;

            WhereAreWeNow = PARSE_CONTINUOUSEFFECT;
            makeWeightedSum(elItr->formula, weightedSum);
            WhereAreWeNow = PARSE_UNKNOWN;

            if (!weightedSum.second.empty()) {

                if (weightedSum.second.size() == 1 && weightedSum.second.front() == -1 && weightedSum.first.front() == 0.0) {
                    if (localDebug) cout << "\tIncreases " << *(pnes[elItr->fluentIndex]) << " at rate 0 - ignoring\n";
                } else {

                    if (weightedSum.second.size() != 1 || !(weightedSum.second.front() == -2 || weightedSum.second.front() == -18)) {

                        string actName;
                        {
                            ostringstream o;
                            o << *(getInstantiatedOp(i));
                            actName = o.str();
                        }

                        string worksOutAs;

                        {
                            ostringstream o;

                            o << "increase " << *(pnes[elItr->fluentIndex]) << " (";

                            list<double>::iterator wItr = weightedSum.first.begin();
                            const list<double>::iterator wEnd = weightedSum.first.end();

                            list<int>::iterator vItr = weightedSum.second.begin();
                            for (int idx = 0; wItr != wEnd; ++vItr, ++wItr, ++idx) {
                                if (*vItr == -1) {
                                    if (*wItr != 0.0) {
                                        if (*wItr < 0.0) {
                                            if (idx) {
                                                o << " - ";
                                                o << -(*wItr);
                                            } else {
                                                o << *wItr;
                                            }
                                        } else {
                                            if (idx) o << " + ";
                                            o << *wItr;
                                        }
                                    } else {
                                        --idx;
                                    }
                                } else {
                                    if (*wItr == 0.0) {
                                        --idx;
                                    } else  if (*wItr == 1.0) {
                                        if (idx) o << " + ";
                                        if (*vItr >= 0) {
                                            o << *(getPNE(*vItr));
                                        } else if (*vItr == -2) {
                                            o << "#t";
                                        } else if (*vItr == -3) {
                                            o << "?duration";
                                        }
                                    } else if (*wItr == -1.0) {
                                        if (idx) {
                                            o << " - ";
                                        } else {
                                            o << "-";
                                        }
                                        if (*vItr >= 0) {
                                            o << *(getPNE(*vItr));
                                        } else if (*vItr == -2) {
                                            o << "#t";
                                        } else if (*vItr == -3) {
                                            o << "?duration";
                                        }
                                    } else if (*wItr >= 0.0) {
                                        if (idx) o << " + ";
                                        o << "(" << (*wItr) << " * ";
                                        if (*vItr >= 0) {
                                            o << *(getPNE(*vItr));
                                        } else if (*vItr == -2) {
                                            o << "#t";
                                        } else if (*vItr == -3) {
                                            o << "?duration";
                                        }

                                        o << ")";
                                    } else {
                                        o << " -";
                                        o << "(" << -(*wItr) << " * ";
                                        if (*vItr >= 0) {
                                            o << *(getPNE(*vItr));
                                        } else if (*vItr == -2) {
                                            o << "#t";
                                        } else if (*vItr == -3) {
                                            o << "?duration";
                                        }
                                        o << ")";
                                    }
                                }
                            }
                            o << ")";
                            worksOutAs = o.str();
                        }


                        postmortem_nonLinearCTS(actName, worksOutAs);

                    };
                    double currGradient = weightedSum.first.front();
                    if (weightedSum.second.front() == -18 && currGradient != 0.0) {
                        currGradient = -currGradient;
                    }
                    if (elItr->op == VAL::E_DECREASE && currGradient != 0.0) {
                        currGradient = -currGradient;
                    }
                    toReturn->vars.push_back(elItr->fluentIndex);
                    toReturn->effects[0].push_back(currGradient);
                    if (localDebug) cout << "\tIncreases " << *(pnes[elItr->fluentIndex]) << " at rate " << currGradient << "\n";
                }
            } else {
                if (localDebug) cout << "\tIncreases " << *(pnes[elItr->fluentIndex]) << " at rate 0 - ignoring\n";
            }
            list<RPGBuilder::NumericEffect>::iterator elDel = elItr;
            ++elItr;
            effList.erase(elDel);
        } else {
            ++elItr;
        }
    }

    return toReturn;
};
list<RPGBuilder::DurationExpr *> RPGBuilder::buildDEList(list<RPGBuilder::NumericPrecondition *> & d)
{
    list<RPGBuilder::DurationExpr *> toReturn;
    
    list<RPGBuilder::NumericPrecondition *>::iterator lItr = d.begin();
    const list<RPGBuilder::NumericPrecondition *>::iterator lEnd = d.end();
    
    for (; lItr != lEnd; ++lItr) {
        toReturn.push_back(buildDE(*lItr));
    }
    
    return toReturn;
};


RPGBuilder::DurationExpr * RPGBuilder::buildDE(RPGBuilder::NumericPrecondition * expr)
{

    static const bool localDebug = false;
    if (localDebug) cout << "Building duration expression for " << *expr << "\n";

    #ifdef STOCHASTICDURATIONS
    pair<list<double>, list<pair<int, PNE*> > > result;
    #else
    pair<list<double>, list<int> > result;
    #endif

    WhereAreWeNow = PARSE_DURATION;
    makeDurationWeightedSum(expr->RHSformula, result);
    WhereAreWeNow = PARSE_DURATION;

    int rSize = result.first.size();

    if (localDebug) cout << "Final expression has " << rSize << " terms\n";
    DurationExpr * toReturn = new DurationExpr();

    toReturn->weights.reserve(rSize);
    toReturn->variables.reserve(rSize);

    list<double>::iterator wItr = result.first.begin();
    const list<double>::iterator wEnd = result.first.end();

    #ifdef STOCHASTICDURATIONS
    list<pair<int, PNE*> >::iterator vItr = result.second.begin();
    #else
    list<int>::iterator vItr = result.second.begin();
    #endif

    for (; wItr != wEnd; ++wItr, ++vItr) {

        #ifdef STOCHASTICDURATIONS
        if (!vItr->second && vItr->first == -1) {
            if (localDebug) {
                cout << "- Constant term " << *wItr << endl;
            }
            toReturn->constant = *wItr;
        } else {
            if (localDebug) {
                cout << "- Variable term";
                if (vItr->second) {
                    cout << ", stochastic, " << *(vItr->second) << endl;
                } else {
                    cout << ", normal, " << vItr->first << endl;
                }
            }
            
            toReturn->weights.push_back(*wItr);
            toReturn->variables.push_back(*vItr);
        }
        #else
        if (*vItr == -1) {
            toReturn->constant = *wItr;
        } else {
            toReturn->weights.push_back(*wItr);
            toReturn->variables.push_back(*vItr);
        }
        #endif
    }


//  const int s = toReturn->weights.size();
//  cout << "Stored " << s << " non constant terms\n";
//  for (int i = 0; i < s; ++i) {
//      if (i) {
//          cout << " + ";
//      }
//      cout << toReturn->weights[i] << "*" << *(RPGBuilder::getPNE(toReturn->variables[i]));
//  }

//  if (toReturn->constant != 0.0){
//      cout << " + " << toReturn->constant;
//  }

//  cout << "\n";

    toReturn->op = expr->op;

    return toReturn;

}

pair<double, double> RPGBuilder::RPGNumericEffect::applyEffectMinMax(const vector<double> & minFluents, const vector<double> & maxFluents, const double & minDur, const double & maxDur)
{

    static const int varCount = minFluents.size();

    pair<double, double> toReturn(constant, constant);

    if (!isAssignment) {
        toReturn.first += minFluents[fluentIndex];
        toReturn.second += maxFluents[fluentIndex];
    }

    for (int i = 0; i < size; ++i) {
        int vi = variables[i];
        if (vi < 0) {
            if (weights[i] < 0.0) {
                toReturn.first  += weights[i] * maxDur;
                toReturn.second += weights[i] * minDur;
            } else {
                toReturn.first  += weights[i] * minDur;
                toReturn.second += weights[i] * maxDur;
            }
        } else if (vi > varCount) {

            vi -= varCount;
            if (weights[i] < 0.0) {
                toReturn.first  -= weights[i] * minFluents[vi];
                toReturn.second -= weights[i] * maxFluents[vi];
            } else {
                toReturn.first  -= weights[i] * maxFluents[vi];
                toReturn.second -= weights[i] * minFluents[vi];
            }

        } else {

            if (weights[i] < 0.0) {
                toReturn.first  += weights[i] * maxFluents[vi];
                toReturn.second += weights[i] * minFluents[vi];
            } else {
                toReturn.first  += weights[i] * minFluents[vi];
                toReturn.second += weights[i] * maxFluents[vi];
            }
        }

    }
    return toReturn;


};

void RPGBuilder::readPropositionGroupsFile()
{

    ifstream current_in_stream("proposition.groups");
    if (!current_in_stream.good()) {
        cerr << "Warning: proposition.groups could not be opened, planning anyway\n";
        readPropositionGroups = false;
        return;
    }
    
    map<string,Literal*> literalNameToPtr;
    
    LiteralStore::iterator lsItr = instantiatedOp::literalsBegin();
    const LiteralStore::iterator lsEnd = instantiatedOp::literalsEnd();
    
    for (; lsItr != lsEnd; ++lsItr) {
        
        if ((*lsItr)->getStateID() == -1) {
            // ignore static facts
            continue;
        }
        
        ostringstream s;
        s << *(*lsItr);        
        string asString = s.str();
        literalNameToPtr[asString] = *lsItr;
    }
    
    vector<Literal*> prototypeGroup;        
    
    while (!current_in_stream.eof()) {
        string next;
        std::getline(current_in_stream, next);
        
        if (next.empty()) {
            continue;
        }
        
        if (next.find("var") == 0) {
            if (prototypeGroup.size() > 1) {
                propositionGroups.push_back(prototypeGroup);                
            }
            prototypeGroup.clear();
        } else {
            const map<string,Literal*>::const_iterator fItr = literalNameToPtr.find(next);
            if (fItr == literalNameToPtr.end()) {
                cout << "Warning: propositions.groups refers to " << next << ", but a literal with that name can't be found\n";
            } else {
                prototypeGroup.push_back(fItr->second);
            }
        }
    }
    
    if (prototypeGroup.size() > 1) {
        propositionGroups.push_back(prototypeGroup);
    }
        
        
    if (propositionGroups.empty()) {
        readPropositionGroups = false;
        return;
    }
    
    literalToPropositionGroupID.resize(instantiatedOp::howManyNonStaticLiterals(), -1);
    
    const int gCount = propositionGroups.size();
    for (int g = 0; g < gCount; ++g) {
        const int gSize = propositionGroups[g].size();
        for (int i = 0; i < gSize; ++i) {
            literalToPropositionGroupID[propositionGroups[g][i]->getStateID()] = g;
        }
    }
}


};
