#include "RPGBuilder.h"
#include "globals.h"
#include "temporalanalysis.h"

#include <sstream>

using std::ostringstream;

#include "typecheck.h"
#include "TIM.h"
#include "FuncAnalysis.h"

using namespace TIM;
using namespace Inst;
using namespace VAL;

using std::endl;

namespace Planner
{


class ExpressionBuilder: public VisitController
{

private:

    list<RPGBuilder::Operand> & formula;
    VAL::TypeChecker * tc;
    VAL::FastEnvironment * fe;
    bool valid;
    bool debug;
public:

    ExpressionBuilder(list<RPGBuilder::Operand> & formulaIn, VAL::FastEnvironment * f, VAL::TypeChecker * t = 0) :
            formula(formulaIn), tc(t), fe(f), debug(Globals::globalVerbosity & 16) {};

    bool buildFormula(VAL::expression * e) {
        if (debug) cout << "Building numeric expression\n";
        valid = true;
        e->visit(this);
        return valid;
    }

    void visit_plus_expression(const plus_expression * s) {
        if (debug) cout << "+ term\n";
        s->getLHS()->visit(this);
        s->getRHS()->visit(this);
        formula.push_back(RPGBuilder::Operand(RPGBuilder::NE_ADD));
    }

    void visit_minus_expression(const minus_expression * s) {
        if (debug) cout << "- term\n";
        s->getLHS()->visit(this);
        s->getRHS()->visit(this);
        formula.push_back(RPGBuilder::Operand(RPGBuilder::NE_SUBTRACT));
    }
    void visit_mul_expression(const mul_expression * s) {
        if (debug) cout << "* term\n";
        s->getLHS()->visit(this);
        s->getRHS()->visit(this);
        formula.push_back(RPGBuilder::Operand(RPGBuilder::NE_MULTIPLY));
    }
    void visit_div_expression(const div_expression * s) {
        if (debug) cout << "/ term\n";
        s->getLHS()->visit(this);
        s->getRHS()->visit(this);
        formula.push_back(RPGBuilder::Operand(RPGBuilder::NE_DIVIDE));
    }

    void visit_uminus_expression(const uminus_expression * s) {
        if (debug) cout << "0- term\n";
        formula.push_back(RPGBuilder::Operand((double) 0.0));
        s->getExpr()->visit(this);
        formula.push_back(RPGBuilder::Operand(RPGBuilder::NE_SUBTRACT));
    }
    void visit_int_expression(const int_expression * s) {
        if (debug) cout << "int term " << s->double_value() << endl;
        formula.push_back(RPGBuilder::Operand((double) s->double_value()));
    }
    void visit_float_expression(const float_expression * s) {
        if (debug) cout << "float term " << s->double_value() << endl;
        formula.push_back(RPGBuilder::Operand((double) s->double_value()));
    };

    void visit_special_val_expr(const special_val_expr * v) {
        if (v->getKind() == E_HASHT) {
            if (debug) {
                cout << "#t term\n";
            }
            formula.push_back(RPGBuilder::Operand((int) - 2));
        } else if (v->getKind() == E_DURATION_VAR) {
            if (debug) {
                cout << "?duration term\n";
            }            
            formula.push_back(RPGBuilder::Operand((int) - 3));
        } else if (v->getKind() == E_TOTAL_TIME) {
            if (debug) {
                cout << "total-time term\n";
            }            
            formula.push_back(RPGBuilder::Operand((int) - 4));
        } else {
            cout << "Error parsing expression: unsupported task constant " << *v << " found\n";
        }
    };


    void visit_func_term(const func_term * s) {
        PNE lookupPNE(s, fe);
        //cout << "Looking up " << lookupPNE << "\n";
        PNE * realPNE = instantiatedOp::findPNE(&lookupPNE);

        if (!realPNE) {
            if (debug) {
                cout << "PNE " << lookupPNE << " did not exist\n";
            }
            formula.push_back(RPGBuilder::Operand((double) 0.0));
            valid = false;
        } else {
            if (realPNE->getHead()->getName() == "fake-duration") {
                cout << "Detected fake-duration in condition, replaced with ?duration\n";
                formula.push_back(RPGBuilder::Operand((int) - 3));
            } else {
                #ifdef STOCHASTICDURATIONS
                if (   realPNE->getHead()->getName().find("exponential") == 0
                    || realPNE->getHead()->getName().find("stochastic-") == 0) {
                    if (debug) {
                        cout << "Stochastic duration PNE " << *realPNE << ", ID " << realPNE->getGlobalID() << std::endl;
                    }                                                            
                    formula.push_back(realPNE);
                }
                else
                #endif
                if (realPNE->getStateID() == -1) {
                    if (debug) {
                        cout << "PNE " << *realPNE << ", with static value " << EFT(realPNE->getHead())->getInitial(realPNE->begin(), realPNE->end()).second << std::endl; 
                    }                    
                    formula.push_back(RPGBuilder::Operand(EFT(realPNE->getHead())->getInitial(realPNE->begin(), realPNE->end()).second));
                } else {
                    if (debug) {
                        cout << "PNE " << *realPNE << ", ID " << realPNE->getStateID() << std::endl;
                    }                                        
                    formula.push_back(RPGBuilder::Operand((int) realPNE->getStateID()));
                }
            }
        }
    };

    void visit_violation_term(const violation_term * s) {
        formula.push_back(RPGBuilder::Operand(s->getName()));
    };

};

double RPGBuilder::calculateRHS(const list<Operand> & formula, vector<double> & fluents)
{

    list<double> RHS;

    list<Operand>::const_iterator fItr = formula.begin();
    const list<Operand>::const_iterator fEnd = formula.end();

    for (; fItr != fEnd; ++fItr) {
        const Operand & currOperand = *fItr;
        const math_op currMathOp = currOperand.numericOp;
        switch (currMathOp) {
        case RPGBuilder::NE_ADD: {
            const double oldFront = RHS.front();
            RHS.pop_front();
            RHS.front() += oldFront;
        }
        break;
        case RPGBuilder::NE_SUBTRACT: {
            const double oldFront = RHS.front();
            RHS.pop_front();
            RHS.front() -= oldFront;
        }
        break;
        case RPGBuilder::NE_MULTIPLY: {
            const double oldFront = RHS.front();
            RHS.pop_front();
            RHS.front() *= oldFront;
        }
        break;
        case RPGBuilder::NE_DIVIDE: {
            const double oldFront = RHS.front();
            RHS.pop_front();
            RHS.front() /= oldFront;
        }
        break;
        case RPGBuilder::NE_CONSTANT:
            RHS.push_front(currOperand.constantValue);
            break;
        case RPGBuilder::NE_FLUENT:
            RHS.push_front(fluents[currOperand.fluentValue]);
            break;
        #ifdef STOCHASTICDURATIONS            
        case RPGBuilder::NE_STOCHASTIC_DURATION_TERM: {                    
            assert(currOperand.durationVar);
            assert(EFT(currOperand.durationVar->getHead())->isStatic());
            RHS.push_front(EFT(currOperand.durationVar->getHead())->getInitial(currOperand.durationVar->begin(), currOperand.durationVar->end()).second);
            break;
        }
        #endif
        default:
            // this should never happen
            assert(false);
        }
    }

    return RHS.front();

};

pair<double, bool> RPGBuilder::constRHS(const list<Operand> & formula)
{

    list<double> RHS;

    assert(!formula.empty());
    
    list<Operand>::const_iterator fItr = formula.begin();
    const list<Operand>::const_iterator fEnd = formula.end();

    for (; fItr != fEnd; ++fItr) {
        const Operand & currOperand = *fItr;
        const math_op currMathOp = currOperand.numericOp;
        switch (currMathOp) {
        case RPGBuilder::NE_ADD: {
            const double oldFront = RHS.front();
            RHS.pop_front();
            RHS.front() += oldFront;
        }
        break;
        case RPGBuilder::NE_SUBTRACT: {
            const double oldFront = RHS.front();
            RHS.pop_front();
            RHS.front() -= oldFront;
        }
        break;
        case RPGBuilder::NE_MULTIPLY: {
            const double oldFront = RHS.front();
            RHS.pop_front();
            RHS.front() *= oldFront;
        }
        break;
        case RPGBuilder::NE_DIVIDE: {
            const double oldFront = RHS.front();
            RHS.pop_front();
            RHS.front() /= oldFront;
        }
        break;
        case RPGBuilder::NE_CONSTANT: {
            RHS.push_front(currOperand.constantValue);
            break;
        }
        #ifdef STOCHASTICDURATIONS            
        case RPGBuilder::NE_STOCHASTIC_DURATION_TERM: {                    
            assert(currOperand.durationVar);
            if (EFT(currOperand.durationVar->getHead())->isStatic()) {
                RHS.push_front(EFT(currOperand.durationVar->getHead())->getInitial(currOperand.durationVar->begin(), currOperand.durationVar->end()).second);
                break;
            } else {
                return pair<double, bool>(0.0, false);
                break;
            }
        }
        #endif
        case RPGBuilder::NE_FLUENT: {
            //cout << "Duration expression is non-constant\n";
            return pair<double, bool>(0.0, false);            
            break;
        }
        default:
            // this should never happen
            assert(false);
        }
    }

    assert(!RHS.empty());
    
    //cout << "Duration expression is constant" << std::endl;    
    //cout << "Value " << RHS.front() << std::endl;
    
    return pair<double, bool>(RHS.front(), true);

};

double RPGBuilder::ArtificialVariable::evaluateWCalculate(const vector<double> & fluentTable, const int & pneCount)
{
    //cout << "Evaluating AV " << ID << " of size " << size << "\n";
    double toReturn = constant;
    //cout << "Evaluating AV of size " << size << "\n";
    for (int i = 0; i < size; ++i) {
        int var = fluents[i];
        if (var < 0) return std::numeric_limits<double>::signaling_NaN();
        double w = weights[i];
        if (var >= pneCount) {
            var -= pneCount;
            if (w != 0.0) w = 0.0 - w;
        }
        toReturn += w * fluentTable[var];
    }
    return toReturn;
};


double RPGBuilder::ArtificialVariable::evaluateWCalculate(const vector<double> & minFluentTable, const vector<double> & maxFluentTable, const int & pneCount) const
{
    //cout << "Evaluating AV " << ID << " of size " << size << "\n";
    double toReturn = constant;
    //cout << "Evaluating AV of size " << size << "\n";
    for (int i = 0; i < size; ++i) {
        int var = fluents[i];
        if (var < 0) return std::numeric_limits<double>::signaling_NaN();
        double w = weights[i];
        if (var >= pneCount) {
            var -= pneCount;
            if (w != 0.0) w = 0.0 - w;
            toReturn += w * minFluentTable[var];
        } else {
            toReturn += w * maxFluentTable[var];
        }
    }
    return toReturn;
};


bool RPGBuilder::RPGNumericPrecondition::isSatisfiedWCalculate(const vector<double> & maxFluents) const
{

    const int pneCount = RPGBuilder::getPNECount();
    int var = LHSVariable;
    if (var < 0) {
        return false;
    }
    //cout << "Precondition based on variable " << var << "\n";
    if (var < pneCount) {
        //cout << "Precondition based on positive variable " << var << "\n";
        if (op == VAL::E_GREATER) {
            return (maxFluents[var] > RHSConstant);
        } else {
            return (maxFluents[var] >= RHSConstant);
        }
    }
    var -= pneCount;
    if (var < pneCount) {
        //cout << "Precondition based on negative variable " << var << "\n";
        const double localVal = (maxFluents[var] != 0.0 ? 0.0 - maxFluents[var] : 0.0);
        if (op == VAL::E_GREATER) {
            return (localVal > RHSConstant);
        } else {
            return (localVal >= RHSConstant);
        }
    }
    var += pneCount;
    //cout << "Precondition based on artificial variable " << var << "\n";
    ArtificialVariable & av = RPGBuilder::getArtificialVariable(var);
    const double localVal = av.evaluateWCalculate(maxFluents, pneCount);
    if (localVal == std::numeric_limits<double>::signaling_NaN()) return false;
    if (op == VAL::E_GREATER) {
        return (localVal > RHSConstant);
    } else {
        return (localVal >= RHSConstant);
    }

};

bool RPGBuilder::RPGNumericPrecondition::isSatisfiedWCalculate(const vector<double> & minFluents, const vector<double> & maxFluents) const
{

    const int pneCount = RPGBuilder::getPNECount();
    int var = LHSVariable;
    if (var < 0) {
        return false;
    }
    //cout << "Precondition based on variable " << var << "\n";
    if (var < pneCount) {
        //cout << "Precondition based on positive variable " << var << "\n";
        if (op == VAL::E_GREATER) {
            return (maxFluents[var] > RHSConstant);
        } else {
            return (maxFluents[var] >= RHSConstant);
        }
    }
    var -= pneCount;
    if (var < pneCount) {
        //cout << "Precondition based on negative variable " << var << "\n";
        const double localVal = (minFluents[var] != 0.0 ? 0.0 - minFluents[var] : 0.0);
        if (op == VAL::E_GREATER) {
            return (localVal > RHSConstant);
        } else {
            return (localVal >= RHSConstant);
        }
    }
    var += pneCount;
    //cout << "Precondition based on artificial variable " << var << "\n";
    ArtificialVariable & av = RPGBuilder::getArtificialVariable(var);
    const double localVal = av.evaluateWCalculate(minFluents, maxFluents, pneCount);
    if (localVal == std::numeric_limits<double>::signaling_NaN()) return false;
    if (op == VAL::E_GREATER) {
        return (localVal > RHSConstant);
    } else {
        return (localVal >= RHSConstant);
    }

};



RPGBuilder::NumericEffect::NumericEffect(const VAL::assign_op & opIn, const int & fIn, VAL::expression * formulaIn, VAL::FastEnvironment * f, VAL::TypeChecker * t) :
        fluentIndex(fIn), op(opIn)
{
    ExpressionBuilder builder(formula, f, t);
    builder.buildFormula(formulaIn);
};

RPGBuilder::NumericPrecondition::NumericPrecondition(const VAL::comparison_op & opIn, VAL::expression * LHSformulaIn, VAL::expression * RHSformulaIn, VAL::FastEnvironment * f, VAL::TypeChecker * t, const bool p) :
        op(opIn), valid(true), polarity(p)
{
    {
        ExpressionBuilder builder(LHSformula, f, t);
        valid = builder.buildFormula(LHSformulaIn);
    }
    {
        ExpressionBuilder builder(RHSformula, f, t);
        valid = (valid && builder.buildFormula(RHSformulaIn));
    }

};

double RPGBuilder::NumericEffect::applyEffect(vector<double> & fluents) const
{

    const double RHS = calculateRHS(formula, fluents);

    switch (op) {
    case VAL::E_ASSIGN:
        return RHS;
        break;
    case VAL::E_INCREASE:
        return (fluents[fluentIndex] + RHS);
        break;
    case VAL::E_DECREASE:
        return (fluents[fluentIndex] - RHS);
        break;
    case VAL::E_SCALE_UP:
        return (fluents[fluentIndex] * RHS);
        break;
    case VAL::E_SCALE_DOWN:
        return (fluents[fluentIndex] / RHS);
        break;
    default:
        // this should never happen
        assert(false);
    }


};

bool RPGBuilder::NumericPrecondition::isSatisfied(vector<double> & fluents) const
{

    const double LHS = calculateRHS(LHSformula, fluents);
    const double RHS = calculateRHS(RHSformula, fluents);

    switch (op) {
    case VAL::E_GREATER:
        return (LHS > RHS);
        break;
    case VAL::E_GREATEQ:
        return (LHS >= RHS);
        break;
    case VAL::E_LESS:
        return (LHS < RHS);
        break;
    case VAL::E_LESSEQ:
        return (LHS <= RHS);
        break;
    case VAL::E_EQUALS:
        return (LHS == RHS);
        break;
    }

    assert(false); // this should never happen
    return false;

};

double RPGBuilder::NumericPrecondition::evaluateRHS(vector<double> & fluentTable) const
{
    return calculateRHS(RHSformula, fluentTable);
}

pair<double, bool> RPGBuilder::NumericPrecondition::constRHS() const
{
    return RPGBuilder::constRHS(RHSformula);
}

void RPGBuilder::NumericEffect::display(ostream & o) const
{

    o << *(RPGBuilder::getPNE(fluentIndex)) << " ";
    switch (op) {

    case VAL::E_ASSIGN:
        o << "= ";
        break;
    case VAL::E_INCREASE:
        o << "+= ";
        break;
    case VAL::E_DECREASE:
        o << "-= ";
        break;
    case VAL::E_SCALE_UP:
        o << "*= ";
        break;
    case VAL::E_SCALE_DOWN:
        o << "/= ";
        break;
    default:
        break;
    };
    {
        list<Operand>::const_iterator opItr = formula.begin();
        const list<Operand>::const_iterator opEnd = formula.end();
        o << "(";
        for (; opItr != opEnd; ++opItr) {
            const Operand & currOperand = *opItr;
            const math_op currMathOp = currOperand.numericOp;
            switch (currMathOp) {
            case RPGBuilder::NE_ADD: {
                o << " +";
            }
            break;
            case RPGBuilder::NE_SUBTRACT: {
                o << " -";
            }
            break;
            case RPGBuilder::NE_MULTIPLY: {
                o << " *";
            }
            break;
            case RPGBuilder::NE_DIVIDE: {
                o << " /";
            }
            break;
            case RPGBuilder::NE_CONSTANT: {
                o << " " << currOperand.constantValue;
            }
            break;
            case RPGBuilder::NE_FLUENT: {
                if (currOperand.fluentValue < 0) {
                    o << " <special>";
                } else {
                    o << " " << *(RPGBuilder::getPNE(currOperand.fluentValue));
                }
            }
            break;
            default:
                // this should never happen
                assert(false);
            }
        }
        o << ")";
    }

};

void RPGBuilder::NumericPrecondition::display(ostream & o) const
{

    {
        list<Operand>::const_iterator opItr = LHSformula.begin();
        const list<Operand>::const_iterator opEnd = LHSformula.end();
        o << "(";
        for (; opItr != opEnd; ++opItr) {
            const Operand & currOperand = *opItr;
            const math_op currMathOp = currOperand.numericOp;
            switch (currMathOp) {
            case RPGBuilder::NE_ADD: {
                o << " +";
            }
            break;
            case RPGBuilder::NE_SUBTRACT: {
                o << " -";
            }
            break;
            case RPGBuilder::NE_MULTIPLY: {
                o << " *";
            }
            break;
            case RPGBuilder::NE_DIVIDE: {
                o << " /";
            }
            break;
            case RPGBuilder::NE_CONSTANT: {
                o << " " << currOperand.constantValue;
            }
            break;
            case RPGBuilder::NE_FLUENT: {
                if (currOperand.fluentValue < 0) {
                    o << " <special>";
                } else {
                    o << " " << *(RPGBuilder::getPNE(currOperand.fluentValue));
                }
            }
            break;
            #ifdef STOCHASTICDURATIONS            
            case RPGBuilder::NE_STOCHASTIC_DURATION_TERM: {                    
                assert(currOperand.durationVar);
                o << " " << *(currOperand.durationVar);
                break;
            }
            #endif
            default:
                // this should never happen
                assert(false);
            }
        }
        o << ")";
    }

    switch (op) {

    case VAL::E_GREATER:
        o << " > ";
        break;
    case VAL::E_GREATEQ:
        o << " >= ";
        break;
    case VAL::E_LESS:
        o << " < ";
        break;
    case VAL::E_LESSEQ:
        o << " <= ";
        break;
    case VAL::E_EQUALS:
        o << " = ";
        break;
    };
    {
        list<Operand>::const_iterator opItr = RHSformula.begin();
        const list<Operand>::const_iterator opEnd = RHSformula.end();
        o << "(";
        for (; opItr != opEnd; ++opItr) {
            const Operand & currOperand = *opItr;
            const math_op currMathOp = currOperand.numericOp;
            switch (currMathOp) {
            case RPGBuilder::NE_ADD: {
                o << " +";
            }
            break;
            case RPGBuilder::NE_SUBTRACT: {
                o << " -";
            }
            break;
            case RPGBuilder::NE_MULTIPLY: {
                o << " *";
            }
            break;
            case RPGBuilder::NE_DIVIDE: {
                o << " /";
            }
            break;
            case RPGBuilder::NE_CONSTANT: {
                o << " " << currOperand.constantValue;
            }
            break;
            case RPGBuilder::NE_FLUENT: {
                if (currOperand.fluentValue == -1) {
                    o << " <special>";
                } else {
                    o << " " << *(RPGBuilder::getPNE(currOperand.fluentValue));
                }
            }
            break;
            #ifdef STOCHASTICDURATIONS            
            case RPGBuilder::NE_STOCHASTIC_DURATION_TERM: {                    
                assert(currOperand.durationVar);
                o << " " << *(currOperand.durationVar);
                break;
            }
            #endif
            default:
                // this should never happen
                assert(false);
            }
        }
        o << ")";
    }

};

ostream & operator <<(ostream & o, const RPGBuilder::NumericPrecondition & p)
{
    p.display(o);
    return o;
};

ostream & operator <<(ostream & o, const RPGBuilder::NumericEffect & p)
{
    p.display(o);
    return o;
};

ostream & operator <<(ostream & o, const RPGBuilder::RPGNumericPrecondition & p)
{
    p.display(o);
    return o;
};

ostream & operator <<(ostream & o, const RPGBuilder::ArtificialVariable & p)
{
    p.display(o);
    return o;
};

ostream & operator <<(ostream & o, const RPGBuilder::RPGNumericEffect & p)
{
    p.display(o);
    return o;
};

bool RPGBuilder::ArtificialVariable::operator <(const RPGBuilder::ArtificialVariable & v) const
{

    if (size < v.size) return true;
    if (size > v.size) return false;

    for (int i = 0; i < size; ++i) {
        const double w1 = weights[i];
        const double w2 = v.weights[i];
        if (w1 < w2) return true;
        if (w1 > w2) return false;
    }

    for (int i = 0; i < size; ++i) {
        const int w1 = fluents[i];
        const int w2 = v.fluents[i];
        if (w1 < w2) return true;
        if (w1 > w2) return false;
    }

    if (constant < v.constant) return true;

    return false;
};

bool RPGBuilder::RPGNumericPrecondition::operator <(const RPGBuilder::RPGNumericPrecondition & r) const
{

    if (LHSVariable < r.LHSVariable) return true;
    if (LHSVariable > r.LHSVariable) return false;

    if (LHSConstant < r.LHSConstant) return true;
    if (LHSConstant > r.LHSConstant) return false;

    if (op < r.op) return true;
    if (op > r.op) return false;

    if (RHSVariable < r.RHSVariable) return true;
    if (RHSVariable > r.RHSVariable) return false;

    if (RHSConstant < r.RHSConstant) return true;

    return false;

};

bool RPGBuilder::RPGNumericEffect::operator <(const RPGNumericEffect & e) const
{
    if (fluentIndex < e.fluentIndex) return true;
    if (fluentIndex > e.fluentIndex) return false;

    if (!isAssignment && e.isAssignment) return true;
    if (isAssignment && !e.isAssignment) return false;

    if (size < e.size) return false;
    if (size > e.size) return true;

    if (constant < e.constant) return true;
    if (constant > e.constant) return false;

    for (int i = 0; i < size; ++i) {

        if (variables[i] < e.variables[i]) return true;
        if (variables[i] > e.variables[i]) return false;

        if (weights[i] < e.weights[i]) return true;
        if (weights[i] > e.weights[i]) return false;

    }

    return false;


};

void RPGBuilder::ArtificialVariable::display(ostream & o) const
{


    o << "av of size " << size << ", id " << ID << " (";
    const int lim = RPGBuilder::getPNECount();

    for (int i = 0; i < size; ++i) {
        if (i) o << " + ";
        if (weights[i] != 1.0) {
            o << weights[i] << "*";
        }
        const int v = fluents[i];

        if (v < 0) {
            if (v == -3) {
                o << "?duration";
            } else if (v == -19) {
                o << "-?duration";
            } else {
                o << "<special?>";
            }
        } else if (v < lim) {
            o << *(RPGBuilder::getPNE(v));
        } else {
            o << "-1*" << *(RPGBuilder::getPNE(v - lim));
        }
    }
    if (constant != 0.0) {
        if (size) o << " + ";
        o << constant;
    }

    o << ")";
}

void RPGBuilder::RPGNumericPrecondition::display(ostream & o) const
{

    const int lim = RPGBuilder::getPNECount();

    if (LHSVariable < 0) {
        if (LHSVariable == -1) {
            o << LHSConstant;
        } else if (LHSVariable == -3) {
            o << "?duration";
        } else if (LHSVariable == -19) {
            o << "?duration";
        } else {
            o << "<special?>";
        }
    } else {
        if (LHSVariable < lim) {
            if (LHSConstant != 1.0) o << LHSConstant << "*";
            o << *(RPGBuilder::getPNE(LHSVariable));
        } else if (LHSVariable < (2 * lim)) {
            if (LHSConstant != 1.0) o << LHSConstant << "*";
            o << "-1*" << *(RPGBuilder::getPNE(LHSVariable - lim));
        } else {
            o << RPGBuilder::getArtificialVariable(LHSVariable);
        }
    }
    
    if (op == VAL::E_GREATER) {
        o << " > ";
    } else if (op == VAL::E_GREATEQ) {
        o << " >= ";
    } else {
        assert(false);
    }

    if (RHSVariable < 0) {
        if (RHSVariable == -1) {
            o << RHSConstant;
        } else if (RHSVariable == -3) {
            o << "?duration";
        } else if (RHSVariable == -19) {
            o << "?duration";
        } else {
            o << "<special?>";
        }
    } else {
        if (RHSVariable < lim) {
            if (RHSConstant != 1.0) o << RHSConstant << "*";
            o << *(RPGBuilder::getPNE(RHSVariable));
        } else if (RHSVariable < (2 * lim)) {
            if (RHSConstant != 1.0) o << RHSConstant << "*";
            o << "-1*" << *(RPGBuilder::getPNE(RHSVariable - lim));
        } else {
            o << RPGBuilder::getArtificialVariable(RHSVariable);
        }
    }

    o << " [lv=" << LHSVariable << ",lc=" << LHSConstant << ",rv=" << RHSVariable << ",rc=" << RHSConstant << "]";

};

void RPGBuilder::RPGNumericEffect::display(ostream & o) const
{
    static const int lim = RPGBuilder::getPNECount();
    o << "(";

    o << *(RPGBuilder::getPNE(fluentIndex));
    if (isAssignment) {
        o << " =";
    } else {
        o << " +=";
    }
    int t = 0;
    if (constant != 0.0 || size == 0) {
        o << " " << constant;
        ++t;
    }
    for (int i = 0; i < size; ++i, ++t) {
        if (t) o << " + ";
        if (weights[i] != 1.0) {
            o << weights[i] << "*";
        }
        const int v = variables[i];

        if (v == -3) {
            o << "?duration";
        } else if (v == -19) {
            o << "-?duration";
        } else if (v == -2) {
            o << "#t";
        } else if (v == -18) {
            o << "-#t";
        } else if (v < lim) {
            o << *(RPGBuilder::getPNE(v));
        } else {
            o << "-1*" << *(RPGBuilder::getPNE(v - lim));
        }
    }

    o << ")";
}


void RPGBuilder::simplify(pair<list<double>, list<int> > & p)
{



    list<double>::iterator fItr = p.first.begin();
    const list<double>::iterator fEnd = p.first.end();
    list<double>::iterator constTerm = fEnd;

    list<int>::iterator sItr = p.second.begin();
    //const list<double>::iterator sEnd = p.second.end();

    while (fItr != fEnd) {

        if (*sItr >= 0 || *sItr <= -2) {
            ++sItr;
            ++fItr;
        } else {
            if (constTerm == fEnd) {
                constTerm = fItr;
                ++fItr;
                ++sItr;
            } else {
                *constTerm += *fItr;
                list<double>::iterator fErase = fItr;
                list<int>::iterator sErase = sItr;

                ++fItr;
                ++sItr;
                p.first.erase(fErase);
                p.second.erase(sErase);
            }
        }

    }

}

#ifdef STOCHASTICDURATIONS
void RPGBuilder::simplify(pair<list<double>, list<pair<int,PNE*> > > & p)
{
    
    
    
    list<double>::iterator fItr = p.first.begin();
    const list<double>::iterator fEnd = p.first.end();
    list<double>::iterator constTerm = fEnd;
    
    list<pair<int,PNE*> >::iterator sItr = p.second.begin();
    //const list<double>::iterator sEnd = p.second.end();
    
    while (fItr != fEnd) {
        
        if (sItr->second || sItr->first >= 0 || sItr->first <= -2) {
            ++sItr;
            ++fItr;
        } else {
            if (constTerm == fEnd) {
                constTerm = fItr;
                ++fItr;
                ++sItr;
            } else {
                *constTerm += *fItr;
                list<double>::iterator fErase = fItr;
                list<pair<int,PNE*> >::iterator sErase = sItr;
                
                ++fItr;
                ++sItr;
                p.first.erase(fErase);
                p.second.erase(sErase);
            }
        }
        
    }
    
}
#endif


struct InvData {

    typedef RPGBuilder::RPGNumericPrecondition RPGNumericPrecondition;
    typedef RPGBuilder::ArtificialVariable ArtificialVariable;

    struct LTAVPointer {

        bool operator()(const RPGBuilder::ArtificialVariable * const a, const RPGBuilder::ArtificialVariable * const b) {
            return ((*a) < (*b));
        };

    };

    struct LTRNPPointer {

        bool operator()(const RPGBuilder::RPGNumericPrecondition * const a, const RPGBuilder::RPGNumericPrecondition * const b) {
            return ((*a) < (*b));
        };

    };


    set<ArtificialVariable*, LTAVPointer> avReuse;
    set<const RPGNumericPrecondition*, LTRNPPointer> preReuse;

    list<ArtificialVariable*> newAVs;
    list<RPGNumericPrecondition*> newPres;

    bool avReuseInit;
    bool preReuseInit;
    int baseNextAVIndex;
    int nextAVIndex;
    int baseNextPreIndex;
    int nextPreIndex;

    vector<ArtificialVariable> & rpgArtificialVariables;
    vector<RPGNumericPrecondition> & rpgNumericPreconditions;

    InvData(vector<ArtificialVariable> & avs, vector<RPGNumericPrecondition> & pres) : avReuseInit(false), preReuseInit(false), baseNextAVIndex(-1), nextAVIndex(-1), baseNextPreIndex(-1), nextPreIndex(-1), rpgArtificialVariables(avs), rpgNumericPreconditions(pres) {};

    pair<ArtificialVariable*, bool> insertAV(ArtificialVariable* candidate) {
        if (!avReuseInit) {
            const int loopLim = rpgArtificialVariables.size();
            for (int s = 0; s < loopLim; ++s) {
                avReuse.insert(&(rpgArtificialVariables[s]));
                if (rpgArtificialVariables[s].ID > nextAVIndex) nextAVIndex = rpgArtificialVariables[s].ID;
            }
            ++nextAVIndex;
            baseNextAVIndex = nextAVIndex;
            avReuseInit = true;
        }

        pair<set<ArtificialVariable*, LTAVPointer>::iterator, bool> avrItr = avReuse.insert(candidate);

        if (avrItr.second) {
            candidate->ID = nextAVIndex;
            ++nextAVIndex;
            newAVs.push_back(candidate);
            return pair<ArtificialVariable*, bool>(candidate, true);
        }

        delete candidate;

        return pair<ArtificialVariable*, bool>(*(avrItr.first), false);
    };

    pair<const RPGNumericPrecondition*, bool> insertPre(RPGNumericPrecondition* const candidate) {
        if (!preReuseInit) {
            const int loopLim = rpgNumericPreconditions.size();
            for (int s = 0; s < loopLim; ++s) {
                preReuse.insert(&(rpgNumericPreconditions[s]));
                if (rpgNumericPreconditions[s].ID > nextPreIndex) nextPreIndex = rpgNumericPreconditions[s].ID;
            }
            ++nextPreIndex;
            baseNextPreIndex = nextPreIndex;
            preReuseInit = true;
        }

        pair<set<const RPGNumericPrecondition*, LTRNPPointer>::iterator, bool> avrItr = preReuse.insert(candidate);

        if (avrItr.second) {
            candidate->ID = nextPreIndex;
            ++nextPreIndex;
            newPres.push_back(candidate);
            return pair<const RPGNumericPrecondition*, bool>(candidate, true);
        }

        delete candidate;

        return pair<const RPGNumericPrecondition*, bool>(*(avrItr.first), false);
    };

    void erase(const RPGNumericPrecondition * const pre) {
        assert(pre->ID == (nextPreIndex - 1));
        preReuse.erase(pre);
        delete pre;
        newPres.pop_back();
        --nextPreIndex;
    };

    void erase(ArtificialVariable * av) {
        assert(av->ID == (nextAVIndex - 1));
        avReuse.erase(av);
        delete av;
        newAVs.pop_back();
        --nextAVIndex;
    };

    int anyNewPres() {
        return (nextPreIndex - baseNextPreIndex);
    }

    int anyNewAVs() {
        return (nextAVIndex - baseNextAVIndex);
    }


};

bool RPGBuilder::pushInvariantBackThroughStartEffects(const RPGBuilder::RPGNumericPrecondition & pre,
                                                      list<int> & startEffs, RPGBuilder::LinearEffects* ctsEffs,
                                                      InvData & commonData,
                                                      pair<const RPGBuilder::RPGNumericPrecondition *, bool> & preResult, pair<RPGBuilder::ArtificialVariable *, bool> & avResult)
{

    static const bool debug = false;
    
    static const int pneCount = pnes.size();
    map<int, double> lhs;
    double rhs = pre.RHSConstant;
    bool unchanged = true;

    if (debug) {
        cout << "Considering invariant " << pre << endl;
    }
    
    
    if (pre.LHSVariable < pneCount) {
        lhs.insert(make_pair(pre.LHSVariable, pre.LHSConstant));
    } else if (pre.LHSVariable < (2 * pneCount)) {
        lhs.insert(make_pair(pre.LHSVariable - pneCount, -pre.LHSConstant));
    } else {
        ArtificialVariable & currAV = getArtificialVariable(pre.LHSVariable);
        rhs -= currAV.constant;
        for (int s = 0; s < currAV.size; ++s) {
            if (currAV.fluents[s] < pneCount) {
                lhs.insert(make_pair(currAV.fluents[s], currAV.weights[s]));
            } else {
                lhs.insert(make_pair(currAV.fluents[s] - pneCount, -currAV.weights[s]));
            }
        }
    }

//  const map<int, double> lhsBefore = lhs;
//  const double rhsBefore = rhs;


    map<int, pair<map<int, double>, double> > mapOn;

    list<int>::iterator effItr = startEffs.begin();
    const list<int>::iterator effEnd = startEffs.end();

    for (; effItr != effEnd; ++effItr) {
        RPGNumericEffect & currEff = rpgNumericEffects[*effItr];

        map<int, double>::iterator lookup = lhs.find(currEff.fluentIndex);
        if (lookup != lhs.end()) {
            if (debug) {
                cout << "- Relevant start effect: " << currEff << endl;
            }
            map<int, double> localMap;
            double localConst = currEff.constant * lookup->second;
            {
                if (!currEff.isAssignment) localMap.insert(make_pair(currEff.fluentIndex, lookup->second));
                const int loopLim = currEff.weights.size();

                for (int s = 0; s < loopLim; ++s) {
                    if (currEff.variables[s] < pneCount) {
                        localMap.insert(make_pair(currEff.variables[s], currEff.weights[s] * lookup->second));
                    } else {
                        localMap.insert(make_pair(currEff.variables[s] - pneCount, -currEff.weights[s] * lookup->second));
                    }
                }

            }

            mapOn[currEff.fluentIndex] = make_pair(localMap, localConst);

        }
    }

    if (ctsEffs) {
        const int ctsVarCount = ctsEffs->vars.size();
        
        for (int v = 0; v < ctsVarCount; ++v) {
            map<int, double>::iterator lookup = lhs.find(ctsEffs->vars[v]);
            if (lookup != lhs.end()) {
                if (debug) {
                    cout << "- Relevant CTS effect: increase " << *(getPNE(ctsEffs->vars[v])) << " * #t " << ctsEffs->effects[0][v].constant << endl;
                }
                
                map<int, pair<map<int, double>, double> >::iterator moItr = mapOn.find(ctsEffs->vars[v]);
                if (moItr == mapOn.end()) {
                    map<int, double> localMap;
                    localMap.insert(make_pair(ctsEffs->vars[v], lookup->second));
                    moItr = mapOn.insert(make_pair(ctsEffs->vars[v], make_pair(localMap, 0.0))).first;
                }
                moItr->second.second += EPSILON * ctsEffs->effects[0][v].constant * lookup->second;
                
                
                unchanged = false;
                lhs.erase(lookup);
            }
        }
    }

    map<int, pair<map<int, double>, double> >::iterator moItr = mapOn.begin();
    const map<int, pair<map<int, double>, double> >::iterator moEnd = mapOn.end();

    for (; moItr != moEnd; ++moItr) {
        
        if (debug) {
            cout << "Rewriting due to term on " << *(getPNE(moItr->first)) << endl;
        }
        
        rhs -= moItr->second.second;

        if (debug && moItr->second.second != 0.0) {
            cout << "* RHS is now " << rhs;
            if (moItr->second.second > 0.0) {
                cout << " - smaller than it was\n";
            } else {
                cout << " - bigger than it was\n";
            }
        }
        
        map<int, double>::iterator mergeItr = moItr->second.first.begin();
        const map<int, double>::iterator mergeEnd = moItr->second.first.end();

        for (; mergeItr != mergeEnd; ++mergeItr) {
            const map<int, double>::iterator dest = lhs.insert(pair<int, double>(mergeItr->first, 0.0)).first;
            dest->second += mergeItr->second;
            if (debug) {
                cout << "* Changes coeffienct on " << *(getPNE(mergeItr->first)) << " to " << mergeItr->second << endl;
            }
            if (fabs(dest->second < 0.00000000001)) lhs.erase(dest);
        }
    }

    if (fabs(rhs) < 0.00000000001) rhs = 0.0;

    if (lhs.empty()) { //variable-less precondition
        preResult = pair<RPGNumericPrecondition*, bool>(0, false);
        avResult = pair<ArtificialVariable*, bool>(0, false);
        if (pre.op == E_GREATER) {
            return (0.0 > rhs);
        } else {
            return (0.0 >= rhs);
        }
    }

    if (unchanged) {
        if (debug) {
            cout << "Invariant is unchanged: copy as-is" << endl;
        }
        preResult = pair<const RPGNumericPrecondition*, bool>(&pre, false);
        avResult = pair<ArtificialVariable*, bool>(0, false);
        return true;
    }

    const int lhsSize = lhs.size();

    if (lhsSize == 1 && lhs.begin()->second > 0.0) {
        avResult = pair<ArtificialVariable*, bool>(0, false);
        RPGNumericPrecondition * const candidate = new RPGNumericPrecondition();
        candidate->LHSConstant = lhs.begin()->second;
        candidate->LHSVariable = lhs.begin()->first;
        candidate->op = pre.op;
        candidate->RHSConstant = rhs;
        preResult = commonData.insertPre(candidate);
        return true;
    }

    RPGNumericPrecondition * const candidate = new RPGNumericPrecondition();
    candidate->LHSConstant = 1.0;
    candidate->op = pre.op;
    candidate->RHSConstant = 0;


    ArtificialVariable * const candidateAV = new ArtificialVariable();

    candidateAV->size = lhsSize;
    candidateAV->weights.reserve(lhsSize);
    candidateAV->fluents.reserve(lhsSize);
    candidateAV->constant = -rhs;

    map<int, double>::iterator lhsItr = lhs.begin();
    const map<int, double>::iterator lhsEnd = lhs.end();

    for (; lhsItr != lhsEnd; ++lhsItr) {
        if (lhsItr->second >= 0.0) {
            candidateAV->weights.push_back(lhsItr->second);
            candidateAV->fluents.push_back(lhsItr->first);
        } else {
            candidateAV->weights.push_back(-lhsItr->second);
            candidateAV->fluents.push_back(lhsItr->first + pneCount);
        }
    }

    avResult = commonData.insertAV(candidateAV);

    candidate->LHSVariable = avResult.first->ID;

    preResult = commonData.insertPre(candidate);

    return true;

};

void RPGBuilder::handleNumericInvariants()
{

    InvData commonData(rpgArtificialVariables, rpgNumericPreconditions);


    static const int pneCount = pnes.size();
    const int opCount = instantiatedOps.size();
    const int rpgNumPrecCount = rpgNumericPreconditionsToActions.size();

    processedRPGNumericPreconditionsToActions = vector<list<pair<int, VAL::time_spec> > >(rpgNumPrecCount);

    {

        for (int i = 0; i < rpgNumPrecCount; ++i) {
            list<pair<int, VAL::time_spec> > & copyTo = processedRPGNumericPreconditionsToActions[i] = rpgNumericPreconditionsToActions[i];

            list<pair<int, VAL::time_spec> >::iterator ctItr = copyTo.begin();
            const list<pair<int, VAL::time_spec> >::iterator ctEnd = copyTo.end();

            while (ctItr != ctEnd) {
                if (ctItr->second == VAL::E_OVER_ALL) {
                    list<pair<int, VAL::time_spec> >::iterator ctPrev = ctItr;
                    ++ctItr;
                    copyTo.erase(ctPrev);
                } else {
                    ++ctItr;
                }
            }
        }
    }

//  actionsToProcessedStartNumericPreconditions = vector<list<NumericPrecondition> >(opCount);
    actionsToProcessedStartRPGNumericPreconditions = vector<list<int> >(opCount);
    initialUnsatisfiedProcessedStartNumericPreconditions = vector<int>(opCount);

    map<int, list<int> > precToActionMap;

    for (int i = 0; i < opCount; ++i) {

        if (!rogueActions[i]) {

            list<pair<ArtificialVariable*, bool> > localAVs;
            list<pair<const RPGNumericPrecondition*, bool> > localPres;

            list<int> & addToTwo = actionsToProcessedStartRPGNumericPreconditions[i] = actionsToRPGNumericStartPreconditions[i];

            //cout << "Before handing action " << i << " had " << addToTwo.size() << " numeric start pres\n";

            set<int> alreadyAtStart;
            alreadyAtStart.insert(actionsToRPGNumericStartPreconditions[i].begin(), actionsToRPGNumericStartPreconditions[i].end());


            bool eliminate = false;
            {


                list<int>::iterator liItr = actionsToRPGNumericInvariants[i].begin();
                const list<int>::iterator liEnd = actionsToRPGNumericInvariants[i].end();

                for (bool first=true; liItr != liEnd; ++liItr,first=false) {

                    if (Globals::globalVerbosity & 16 && first) {
                        cout << "Invariants of " << *(RPGBuilder::getInstantiatedOp(i)) << endl;
                    }
                    const RPGNumericPrecondition & invPre = rpgNumericPreconditions[*liItr];

                    pair<const RPGNumericPrecondition*, bool> preResult(0, false);
                    pair<ArtificialVariable*, bool> avResult(0, false);

                    const bool feasible = pushInvariantBackThroughStartEffects(invPre, actionsToRPGNumericStartEffects[i], linearDiscretisation[i], commonData, preResult, avResult);

                    if (!feasible) {
                        eliminate = true;
                        break;
                    }

                    if (avResult.first) localAVs.push_back(avResult);
                    if (preResult.first && alreadyAtStart.find(preResult.first->ID) == alreadyAtStart.end()) {
                        if (Globals::globalVerbosity & 16) {
                            cout << "Extra start precondition: " << *(preResult.first) << endl;
                        }
                        localPres.push_back(preResult);
                    }

                }

            }

            if (eliminate) {

                list<pair<ArtificialVariable*, bool> >::reverse_iterator avItr = localAVs.rbegin();
                const list<pair<ArtificialVariable*, bool> >::reverse_iterator avEnd = localAVs.rend();

                for (; avItr != avEnd; ++avItr) {
                    if (avItr->second) {
                        commonData.erase(avItr->first);
                    }
                }

                list<pair<const RPGNumericPrecondition*, bool> >::reverse_iterator preItr = localPres.rbegin();
                const list<pair<const RPGNumericPrecondition*, bool> >::reverse_iterator preEnd = localPres.rend();

                for (; preItr != preEnd; ++preItr) {
                    if (preItr->second) {
                        commonData.erase(preItr->first);
                    }
                }

                pruneIrrelevant(i);

            } else {

                list<pair<const RPGNumericPrecondition*, bool> >::iterator preItr = localPres.begin();
                const list<pair<const RPGNumericPrecondition*, bool> >::iterator preEnd = localPres.end();

                for (; preItr != preEnd; ++preItr) {
                    addToTwo.push_back(preItr->first->ID);
                    static const list<int> emptyList;
                    precToActionMap.insert(make_pair(preItr->first->ID, emptyList)).first->second.push_back(i);
                }

                initialUnsatisfiedProcessedStartNumericPreconditions[i] = addToTwo.size();

            }

        }

    }

    if (commonData.anyNewAVs()) {
        const int newAVLimit = rpgArtificialVariables.size() + commonData.anyNewAVs();
        rpgArtificialVariables.resize(newAVLimit);
        rpgArtificialVariablesToPreconditions.resize(newAVLimit);

        list<ArtificialVariable*>::iterator avItr = commonData.newAVs.begin();
        const list<ArtificialVariable*>::iterator avEnd = commonData.newAVs.end();

        for (; avItr != avEnd; ++avItr) {
            ArtificialVariable & currAV = rpgArtificialVariables[(*avItr)->ID - (2 * pneCount)] = *(*avItr);

            const int afflim = currAV.size;
            for (int aff = 0; aff < afflim; ++aff) {
                const int affVar = currAV.fluents[aff];
                if (affVar >= 0) rpgVariableDependencies[affVar].push_back(currAV.ID);
            }
        }
    }

    if (commonData.anyNewPres()) {
        const int newPreLimit = rpgNumericPreconditions.size() + commonData.anyNewPres();
        rpgNumericPreconditions.resize(newPreLimit);
        processedRPGNumericPreconditionsToActions.resize(newPreLimit);
        rpgNumericPreconditionsToActions.resize(newPreLimit);
        
        numericAchievedInLayer.resize(newPreLimit);
        numericAchievedInLayerReset.resize(newPreLimit);
//      negativeNumericAchievedInLayer.resize(newPreLimit);
//      negativeNumericAchievedInLayerReset.resize(newPreLimit);

        numericAchievedBy.resize(newPreLimit);
        numericAchievedByReset.resize(newPreLimit);
//      negativeNumericAchievedBy.resize(newPreLimit);
//      negativeNumericAchievedByReset.resize(newPreLimit);


        //cout << "Now have " << newPreLimit << " numeric pres\n";

        list<RPGNumericPrecondition*>::iterator preItr = commonData.newPres.begin();
        const list<RPGNumericPrecondition*>::iterator preEnd = commonData.newPres.end();

        for (; preItr != preEnd; ++preItr) {
            RPGNumericPrecondition & currPre = rpgNumericPreconditions[(*preItr)->ID] = *(*preItr);

            if (currPre.LHSVariable < pneCount) {
                rpgPositiveVariablesToPreconditions[currPre.LHSVariable].push_back(currPre.ID);
                assert(currPre.ID < newPreLimit);
            } else if (currPre.LHSVariable < 2 * pneCount) {
                rpgNegativeVariablesToPreconditions[currPre.LHSVariable - pneCount].push_back(currPre.ID);
                assert(currPre.ID < newPreLimit);
            } else {
                rpgArtificialVariablesToPreconditions[currPre.LHSVariable - 2 * pneCount].push_back(currPre.ID);
                assert(currPre.ID < newPreLimit);
            }

            numericAchievedInLayerReset[currPre.ID] = -1.0;
        }

        assert(rpgNumericPreconditions.size() == numericAchievedByReset.size()); /// EH?
    }

    map<int, list<int> >::iterator ptaItr = precToActionMap.begin();
    const map<int, list<int> >::iterator ptaEnd = precToActionMap.end();

    for (; ptaItr != ptaEnd; ++ptaItr) {
        list<pair<int, VAL::time_spec> > & destList = processedRPGNumericPreconditionsToActions[ptaItr->first];

        list<int>::iterator actItr = ptaItr->second.begin();
        const list<int>::iterator actEnd = ptaItr->second.end();

        for (; actItr != actEnd; ++actItr) {
            destList.push_back(make_pair(*actItr, VAL::E_AT_START));
        }
    }

    assert(rpgNumericPreconditions.size() == numericAchievedInLayerReset.size());

#ifndef NDEBUG

    for (int i = 0; i < opCount; ++i) {
        if (!rogueActions[i]) {
            assert(actionsToProcessedStartRPGNumericPreconditions[i].size() == ((unsigned int) initialUnsatisfiedProcessedStartNumericPreconditions[i]));
        }
    }

#endif

//  initialisedNumericPreTable = true;

}
void RPGBuilder::makeOneSided(pair<list<double>, list<int> > & LHSvariable, pair<list<double>, list<int> > & RHSvariable, const int & negOffset)
{
    //pushes variables to the LHS and constants to the RHS
    //result is an expression of the form (w.x [>,>=,<,<=,==] c )

    {
        list<double>::iterator dlItr = LHSvariable.first.begin();
        list<int>::iterator ilItr = LHSvariable.second.begin();
        const list<double>::iterator dlEnd = LHSvariable.first.end();

        while (dlItr != dlEnd) {
            if (*dlItr < 0.0) { // for negative weights
                if (*ilItr == -1) { // push constants to RHS
                    RHSvariable.first.push_back(0.0 - *dlItr);
                    RHSvariable.second.push_back(-1);
                    simplify(RHSvariable);
                    list<double>::iterator dlErr = dlItr;
                    list<int>::iterator ilErr = ilItr;
                    ++dlItr;
                    ++ilItr;
                    LHSvariable.first.erase(dlErr);
                    LHSvariable.second.erase(ilErr);
                } else { // keep vars here, but refer to negative instances and flip sign on weight
                    if (*ilItr >= 0) {
                        *ilItr += negOffset;
                    } else {
                        *ilItr -= 16;                        
                    }
                    
                    *dlItr = 0.0 - *dlItr;
                    ++dlItr;
                    ++ilItr;
                }
            } else { // positive weights are fine
                ++dlItr;
                ++ilItr;
            }
        }
    }


    { // finally, push constants to right, variables to left (sec 5.1, col 2)

        list<double>::iterator dlItr = RHSvariable.first.begin();
        list<int>::iterator ilItr = RHSvariable.second.begin();
        const list<double>::iterator dlEnd = RHSvariable.first.end();

        while (dlItr != dlEnd) {
            if (*ilItr == -1) {
                // leave it alone - it's a constant term :)
                ++dlItr;
                ++ilItr;
            } else {
                if (*dlItr > 0.0) {
                    LHSvariable.first.push_back(*dlItr);
                    if (*ilItr >= 0) {
                        LHSvariable.second.push_back(*ilItr + negOffset);
                    } else {
                        LHSvariable.second.push_back(*ilItr - 16);
                    }

                } else if (*dlItr == 0.0) {
                    // a null weight is a very silly thing

                } else {
                    LHSvariable.first.push_back(0.0 - *dlItr);
                    LHSvariable.second.push_back(*ilItr);
                }

                list<double>::iterator dlErr = dlItr;
                list<int>::iterator ilErr = ilItr;
                ++dlItr;
                ++ilItr;
                RHSvariable.first.erase(dlErr);
                RHSvariable.second.erase(ilErr);

            }
        }

        simplify(RHSvariable); // why not!
        simplify(LHSvariable); // why not!

    }

}

#ifdef STOCHASTICDURATIONS
void printStackTerm(list<double> & first, list<pair<int,PNE*> > & second)
{
    static const int pneCount = RPGBuilder::getPNECount();
    
    list<double>::iterator ldItr = first.begin();
    list<double>::iterator ldEnd = first.end();

    if (ldItr == ldEnd) {
        cout << "0.0";
        return;
    }
    
    list<pair<int,PNE*> >::iterator liItr = second.begin();

    for (int term = 0; ldItr != ldEnd; ++ldItr, ++liItr, ++term) {
        if (term) cout << " + ";
        if (!liItr->second && liItr->first == -1) {
            cout << *ldItr;
        } else {
            if (*ldItr != 1.0) cout << *ldItr << "*";
            if (liItr->second) {
                cout << *(liItr->second);
            } else {
                if (liItr->first >= 0) {
                    if (liItr->first >= pneCount) {                    
                        cout << "-" << *(RPGBuilder::getPNE(liItr->first - pneCount));
                    } else {
                        cout << *(RPGBuilder::getPNE(liItr->first));
                    }
                } else {
                    if (liItr->first == -3) {
                        cout << "?duration";
                    } else if (liItr->first == -2) {
                        cout << "#t";
                    } else if (liItr->first == -19) {
                        cout << "-?duration";
                    } else if (liItr->first == -18) {
                        cout << "-#t";
                    }
                }
            }
        }
    }
}
#endif

void printStackTerm(list<double> & first, list<int> & second)
{
    static const int pneCount = RPGBuilder::getPNECount();
    
    list<double>::iterator ldItr = first.begin();
    list<double>::iterator ldEnd = first.end();

    if (ldItr == ldEnd) {
        cout << "0.0";
        return;
    }
    
    list<int>::iterator liItr = second.begin();

    for (int term = 0; ldItr != ldEnd; ++ldItr, ++liItr, ++term) {
        if (term) cout << " + ";
        if (*liItr == -1) {
            cout << *ldItr;
        } else {
            if (*ldItr != 1.0) cout << *ldItr << "*";
            if (*liItr >= 0) {
                if (*liItr >= pneCount) {                    
                    cout << "-" << *(RPGBuilder::getPNE(*liItr - pneCount));
                } else {
                    cout << *(RPGBuilder::getPNE(*liItr));
                }
            } else {
                if (*liItr == -3) {
                    cout << "?duration";
                } else if (*liItr == -2) {
                    cout << "#t";
                } else if (*liItr == -19) {
                    cout << "-?duration";
                } else if (*liItr == -18) {
                    cout << "-#t";
                }
            }
        }
    }
}

#ifdef STOCHASTICDURATIONS

void RPGBuilder::makeWeightedSum(list<Operand> & formula, pair<list<double>, list<int> > & result)
{
    pair<list<double>, list<pair<int,PNE*> > > tmp;
    makeDurationWeightedSum(formula,tmp);
    
    list<double>::const_iterator wItr = tmp.first.begin();
    
    list<pair<int,PNE*> >::const_iterator tItr = tmp.second.begin();
    const list<pair<int,PNE*> >::const_iterator tEnd = tmp.second.end();
    
    for (; tItr != tEnd; ++tItr) {        
        if (tItr->second) {
            assert(EFT(tItr->second->getHead())->isStatic());
            result.first.push_back(*wItr * EFT(tItr->second->getHead())->getInitial(tItr->second->begin(), tItr->second->end()).second);
            result.second.push_back(-1);
        } else {
            result.first.push_back(*wItr);
            result.second.push_back(tItr->first);
        }
    }
    
    simplify(result);
}

typedef pair<list<double>, list<pair<int,PNE*> > > FormulaStackEntry;

pair<bool,double> entryIsConst(const FormulaStackEntry & e) {
    if (e.first.size() == 1 && !e.second.front().second && e.second.front().first == -1) {
        return make_pair(true, e.first.front());
    } else {
        return make_pair(false, std::numeric_limits< double >::signaling_NaN());
    }
}

const pair<int,PNE*> constFsEntry(-1,0);

void RPGBuilder::makeDurationWeightedSum(list<Operand> & formula, pair<list<double>, list<pair<int,PNE*> > > & result)
#else

void RPGBuilder::makeWeightedSum(list<Operand> & formula, pair<list<double>, list<int> > & result)
{
    makeDurationWeightedSum(formula,result);
}

typedef pair<list<double>, list<int> > FormulaStackEntry;

pair<bool,double> entryIsConst(const FormulaStackEntry & e) {
    if (e.first.size() == 1 && e.second.front() == -1) {
        return make_pair(true, e.first.front());  
    } else {
        return make_pair(false, std::numeric_limits< double >::signaling_NaN());
    }
}

const int constFsEntry = -1;

void RPGBuilder::makeDurationWeightedSum(list<Operand> & formula, pair<list<double>, list<int> > & result)
#endif
{

    const bool stackDebug = false;

    if (stackDebug) cout << "Making weighted sum\n";
    
    if (formula.empty()) {
        if (stackDebug) {
            cout << "\tEmpty formula - returning 0.0\n";
        }
        FormulaStackEntry toReturn;
        toReturn.first.push_front(0.0);
        toReturn.second.push_front(constFsEntry);
        result = toReturn;
        return;
    }
    
    list<FormulaStackEntry> formulaStack;

    list<Operand>::iterator opItr = formula.begin();
    const list<Operand>::iterator opEnd = formula.end();

    for (int st = 0; opItr != opEnd; ++opItr, ++st) {
        if (stackDebug) cout << "Stack term " << st << "\n";
        const Operand & currOperand = *opItr;
        const math_op currMathOp = currOperand.numericOp;
        switch (currMathOp) {
        case RPGBuilder::NE_ADD: {
            FormulaStackEntry oldFront = formulaStack.front();
            formulaStack.pop_front();
            if (stackDebug) {
                cout << "+ operation, two terms were previously:\n";
                {
                    cout << "\t";
                    printStackTerm(oldFront.first, oldFront.second);
                    cout << "\n";
                }
                {
                    cout << "\t";
                    printStackTerm(formulaStack.front().first, formulaStack.front().second);
                    cout << "\n";
                }
            }

            formulaStack.front().first.insert(formulaStack.front().first.begin(), oldFront.first.begin(), oldFront.first.end());
            formulaStack.front().second.insert(formulaStack.front().second.begin(), oldFront.second.begin(), oldFront.second.end());
            if (stackDebug) {
                cout << "Result:\n\t";
                printStackTerm(formulaStack.front().first, formulaStack.front().second);
                cout << "\n";

            }
            simplify(formulaStack.front());
            if (stackDebug) {
                cout << "Simplified:\n\t";
                printStackTerm(formulaStack.front().first, formulaStack.front().second);
                cout << "\n";

            }
        }
        break;
        case RPGBuilder::NE_SUBTRACT: {
            FormulaStackEntry oldFront = formulaStack.front();
            formulaStack.pop_front();
            if (stackDebug) {
                cout << "- operation, two terms were previously:\n";
                {
                    cout << "\t";
                    printStackTerm(oldFront.first, oldFront.second);
                    cout << "\n";
                }
                {
                    cout << "\t";
                    printStackTerm(formulaStack.front().first, formulaStack.front().second);
                    cout << "\n";
                }
            }

            list<double>::iterator negItr = oldFront.first.begin();
            const list<double>::iterator negEnd = oldFront.first.end();
            for (; negItr != negEnd; ++negItr) *negItr = -1.0 * (*negItr);
            formulaStack.front().first.insert(formulaStack.front().first.begin(), oldFront.first.begin(), oldFront.first.end());
            formulaStack.front().second.insert(formulaStack.front().second.begin(), oldFront.second.begin(), oldFront.second.end());
            if (stackDebug) {
                cout << "Result:\n\t";
                printStackTerm(formulaStack.front().first, formulaStack.front().second);
                cout << "\n";

            }

            simplify(formulaStack.front());
            if (stackDebug) {
                cout << "Simplified:\n\t";
                printStackTerm(formulaStack.front().first, formulaStack.front().second);
                cout << "\n";

            }

        }
        break;
        case RPGBuilder::NE_MULTIPLY: { // at least one of the terms has to be entirely conflict, otherwise we have var x * var y
            FormulaStackEntry oldFront = formulaStack.front();
            formulaStack.pop_front();
            FormulaStackEntry oldSecondFront = formulaStack.front();
            formulaStack.pop_front();

            if (stackDebug) {
                cout << "* operation, two terms were previously:\n";
                {
                    cout << "\t";
                    printStackTerm(oldFront.first, oldFront.second);
                    cout << "\n";
                }
                {
                    cout << "\t";
                    printStackTerm(oldSecondFront.first, oldSecondFront.second);
                    cout << "\n";
                }
            }
            
            const bool firstIsConst = entryIsConst(oldFront).first;
            const bool secondIsConst = entryIsConst(oldSecondFront).first;

            if (firstIsConst && secondIsConst) {
                formulaStack.push_front(FormulaStackEntry());
                formulaStack.front().second.push_back(constFsEntry);
                formulaStack.front().first.push_back(entryIsConst(oldFront).second * entryIsConst(oldSecondFront).second);
            } else if (firstIsConst && !secondIsConst) {
                const double constVal = entryIsConst(oldFront).second;
                if (constVal == 0.0) {
                    formulaStack.push_front(FormulaStackEntry());
                    formulaStack.front().second.push_back(constFsEntry);
                    formulaStack.front().first.push_back(0.0);
                } else {
                    list<double>::iterator negItr = oldSecondFront.first.begin();
                    const list<double>::iterator negEnd = oldSecondFront.first.end();
                    for (; negItr != negEnd; ++negItr) {
                        *negItr = constVal * (*negItr);
                    }
                    formulaStack.push_front(oldSecondFront);
                }
            } else if (!firstIsConst && secondIsConst) {
                const double constVal = entryIsConst(oldSecondFront).second;
                if (constVal == 0.0) {
                    formulaStack.push_front(FormulaStackEntry());
                    formulaStack.front().second.push_back(constFsEntry);
                    formulaStack.front().first.push_back(0.0);
                } else {
                    list<double>::iterator negItr = oldFront.first.begin();
                    const list<double>::iterator negEnd = oldFront.first.end();
                    for (; negItr != negEnd; ++negItr) {
                        *negItr = constVal * (*negItr);
                    }
                    formulaStack.push_front(oldFront);
                }
            } else {
                string theOp;

                {
                    #ifdef STOCHASTICDURATIONS
                    theOp = "Non-linear expression";
                    #else
                    ostringstream o;
                    o << "(";
                    {
                        list<double>::iterator ldItr = oldFront.first.begin();
                        list<double>::iterator ldEnd = oldFront.first.end();

                        list<int>::iterator liItr = oldFront.second.begin();

                        for (int term = 0; ldItr != ldEnd; ++ldItr, ++liItr, ++term) {
                            if (term) o << " + ";
                            if (*liItr == -1) {
                                o << *ldItr;
                            } else {
                                if (*ldItr != 1.0) o << *ldItr << "*";
                                if (*liItr >= 0) {
                                    o << *(RPGBuilder::getPNE(*liItr));
                                } else {
                                    if (*liItr == -3) {
                                        o << "?duration";
                                    } else if (*liItr == -2) {
                                        o << "#t";
                                    } else if (*liItr == -19) {
                                        o << "-?duration";
                                    } else if (*liItr == -18) {
                                        o << "-#t";
                                    }
                                }
                            }
                        }
                        o << ") * (";
                    }
                    {
                        list<double>::iterator ldItr = oldSecondFront.first.begin();
                        list<double>::iterator ldEnd = oldSecondFront.first.end();

                        list<int>::iterator liItr = oldSecondFront.second.begin();

                        for (int term = 0; ldItr != ldEnd; ++ldItr, ++liItr, ++term) {
                            if (term) o << " + ";
                            if (*liItr == -1) {
                                o << *ldItr;
                            } else {
                                if (*ldItr != 1.0) o << *ldItr << "*";
                                if (*liItr >= 0) {
                                    o << *(RPGBuilder::getPNE(*liItr));
                                } else {
                                    if (*liItr == -3) {
                                        o << "?duration";
                                    } else if (*liItr == -2) {
                                        o << "#t";
                                    } else if (*liItr == -19) {
                                        o << "-?duration";
                                    } else if (*liItr == -18) {
                                        o << "-#t";
                                    }
                                }
                            }
                        }
                        o << ")";
                    }
                    theOp = o.str();
                    #endif
                    
                }
                postmortem_noQuadratic(theOp);
            }

            if (stackDebug) {
                cout << "Result:\n\t";
                printStackTerm(formulaStack.front().first, formulaStack.front().second);
                cout << "\n";

            }
        }
        break;
        case RPGBuilder::NE_DIVIDE: {
            FormulaStackEntry oldFront = formulaStack.front();
            formulaStack.pop_front();
            const bool firstIsConst = entryIsConst(oldFront).first;
            if (!firstIsConst) {
                string theOp;

                {
                    #ifdef STOCHASTICDURATIONS
                    theOp = "Non-linear expression";
                    #else
                    ostringstream o;
                    o << "(";
                    {
                        list<double>::iterator ldItr = formulaStack.front().first.begin();
                        list<double>::iterator ldEnd = formulaStack.front().first.end();

                        list<int>::iterator liItr = formulaStack.front().second.begin();

                        for (int term = 0; ldItr != ldEnd; ++ldItr, ++liItr, ++term) {
                            if (term) o << " + ";
                            if (*liItr == -1) {
                                o << *ldItr;
                            } else {
                                if (*ldItr != 1.0) o << *ldItr << "*";
                                if (*liItr >= 0) {
                                    o << *(RPGBuilder::getPNE(*liItr));
                                } else {
                                    if (*liItr == -3) {
                                        o << "?duration";
                                    } else if (*liItr == -2) {
                                        o << "#t";
                                    } else if (*liItr == -19) {
                                        o << "-?duration";
                                    } else if (*liItr == -18) {
                                        o << "-#t";
                                    }
                                }
                            }
                        }
                        o << ") / (";
                    }
                    {
                        list<double>::iterator ldItr = oldFront.first.begin();
                        list<double>::iterator ldEnd = oldFront.first.end();

                        list<int>::iterator liItr = oldFront.second.begin();

                        for (int term = 0; ldItr != ldEnd; ++ldItr, ++liItr, ++term) {
                            if (term) o << " + ";
                            if (*liItr == -1) {
                                o << *ldItr;
                            } else {
                                if (*ldItr != 1.0) o << *ldItr << "*";
                                if (*liItr >= 0) {
                                    o << *(RPGBuilder::getPNE(*liItr));
                                } else {
                                    if (*liItr == -3) {
                                        o << "?duration";
                                    } else if (*liItr == -2) {
                                        o << "#t";
                                    } else if (*liItr == -19) {
                                        o << "-?duration";
                                    } else if (*liItr == -18) {
                                        o << "-#t";
                                    }
                                }
                            }
                        }
                        o << ")";
                    }

                    theOp = o.str();
                    #endif
                }
                postmortem_noQuadratic(theOp);
            }

            const double constVal = entryIsConst(oldFront).second;
            if (stackDebug) {
                cout << "/ operation, two terms were previously:\n";
                printStackTerm(formulaStack.front().first, formulaStack.front().second);
                cout << " / constant value " << constVal << "\n";
            }
            if (constVal == 0) {
                postmortem_mathsError("division by zero error", "", WhereAreWeNow);
            }
            list<double>::iterator negItr = formulaStack.front().first.begin();
            const list<double>::iterator negEnd = formulaStack.front().first.end();
            for (; negItr != negEnd; ++negItr) {
                *negItr = (*negItr) / constVal;
            }
        }
        break;
        case RPGBuilder::NE_CONSTANT: {
            formulaStack.push_front(FormulaStackEntry());
            formulaStack.front().first.push_front(currOperand.constantValue);
            formulaStack.front().second.push_front(constFsEntry);
        }
        break;
        case RPGBuilder::NE_FLUENT: {
            formulaStack.push_front(FormulaStackEntry());
            formulaStack.front().first.push_front(1.0);
            #ifdef STOCHASTICDURATIONS
            formulaStack.front().second.push_front(make_pair(currOperand.fluentValue, (PNE*)0));
            #else
            formulaStack.front().second.push_front(currOperand.fluentValue);
            #endif
            break;
        }        
        #ifdef STOCHASTICDURATIONS
        case RPGBuilder::NE_STOCHASTIC_DURATION_TERM: {
            if (stackDebug) {
                cout << "Stochastic duration term - " << *(currOperand.durationVar) << "\n";
            }
            formulaStack.push_front(FormulaStackEntry());
            formulaStack.front().first.push_front(1.0);
            formulaStack.front().second.push_front(make_pair(-1, currOperand.durationVar));
            break;
        }            
        #endif
        case RPGBuilder::NE_VIOLATION: {
            map<string, int>::iterator vID = prefNameToID.find(currOperand.isviolated);
            if (vID == prefNameToID.end()) {
                postmortem_isViolatedNotExist(currOperand.isviolated);
            }
            if (vID->second != -1) {
                formulaStack.push_front(FormulaStackEntry());
                formulaStack.front().first.push_front(1.0);
                #ifdef STOCHASTICDURATIONS
                formulaStack.front().second.push_front(make_pair(-1024 - vID->second, (PNE*)0));
                #else
                formulaStack.front().second.push_front(-1024 - vID->second);
                #endif
            } else {
                formulaStack.push_front(FormulaStackEntry());
                formulaStack.front().first.push_front(0.0);
                formulaStack.front().second.push_front(constFsEntry);
            }
        }
        break;
        default:
            // this should never happen
            assert(false);
        }

    }

    result = formulaStack.front();

}


bool RPGBuilder::processPreconditions(set<ArtificialVariable> & artificialVariableSet,
                                      map<RPGNumericPrecondition, list<pair<int, VAL::time_spec> > > & rpgNumericPreconditionSet,
                                      list<NumericPrecondition> & currPreList, list<int> & destList, int & toIncrement,
                                      const int & negOffset, const int & offset, int & precCount, int & avCount,
                                      vector<double> & localMaxNeed, const int & i, const VAL::time_spec & passTimeSpec)
{



    const bool debugRPGNum = (Globals::globalVerbosity & 16);

    toIncrement = 0;

    list<NumericPrecondition>::iterator cpItr = currPreList.begin();
    const list<NumericPrecondition>::iterator cpEnd = currPreList.end();

    for (; cpItr != cpEnd; ++cpItr) {
        if (debugRPGNum) {
            if (cpItr->polarity) {
                cout << "Converting " << *cpItr << "\n";
            } else {
                cout << "Converting ¬" << *cpItr << "\n";
            }
        }

        pair<list<double>, list<int> > LHSvariable;
        pair<list<double>, list<int> > RHSvariable;

        makeWeightedSum(cpItr->LHSformula, LHSvariable);

        if (debugRPGNum) {
            cout << "LHS is:\n\t";
            printStackTerm(LHSvariable.first, LHSvariable.second);
            cout << "\n";
        }

        makeWeightedSum(cpItr->RHSformula, RHSvariable);


        if (debugRPGNum) {
            cout << "RHS is:\n\t";
            printStackTerm(RHSvariable.first, RHSvariable.second);
            cout << "\n";
        }



        list<pair<list<double>, list<int> > > finalLHS;
        list<VAL::comparison_op> finalOp;
        list<pair<list<double>, list<int> > > finalRHS;


        if (cpItr->polarity) {
            switch (cpItr->op) {
            case VAL::E_GREATER: {
                makeOneSided(LHSvariable, RHSvariable, negOffset);
                finalLHS.push_back(LHSvariable);
                finalOp.push_back(VAL::E_GREATER);
                finalRHS.push_back(RHSvariable);
            }
            break;
            case VAL::E_GREATEQ: {
                makeOneSided(LHSvariable, RHSvariable, negOffset);
                finalLHS.push_back(LHSvariable);
                finalOp.push_back(VAL::E_GREATEQ);
                finalRHS.push_back(RHSvariable);
            }
            break;
            case VAL::E_LESS: {
                makeOneSided(RHSvariable, LHSvariable, negOffset);
                finalLHS.push_back(RHSvariable);
                finalOp.push_back(VAL::E_GREATER);
                finalRHS.push_back(LHSvariable);
            }
            break;
            case VAL::E_LESSEQ: {
                makeOneSided(RHSvariable, LHSvariable, negOffset);
                finalLHS.push_back(RHSvariable);
                finalOp.push_back(VAL::E_GREATEQ);
                finalRHS.push_back(LHSvariable);
            }
            break;
            case VAL::E_EQUALS: {
                pair<list<double>, list<int> > secondLHS(RHSvariable);
                pair<list<double>, list<int> > secondRHS(LHSvariable);

                makeOneSided(LHSvariable, RHSvariable, negOffset);

                finalLHS.push_back(LHSvariable);
                finalOp.push_back(VAL::E_GREATEQ);
                finalRHS.push_back(RHSvariable);

                makeOneSided(secondLHS, secondRHS, negOffset);

                finalLHS.push_back(secondLHS);
                finalOp.push_back(VAL::E_GREATEQ);
                finalRHS.push_back(secondRHS);
            }
            break;
            }
        } else {
            switch (cpItr->op) {
            case VAL::E_GREATER: {
                makeOneSided(LHSvariable, RHSvariable, negOffset);
                finalLHS.push_back(LHSvariable);
                finalOp.push_back(VAL::E_LESSEQ);
                finalRHS.push_back(RHSvariable);
            }
            break;
            case VAL::E_GREATEQ: {
                makeOneSided(LHSvariable, RHSvariable, negOffset);
                finalLHS.push_back(LHSvariable);
                finalOp.push_back(VAL::E_LESS);
                finalRHS.push_back(RHSvariable);
            }
            break;
            case VAL::E_LESS: {
                makeOneSided(RHSvariable, LHSvariable, negOffset);
                finalLHS.push_back(RHSvariable);
                finalOp.push_back(VAL::E_GREATEQ);
                finalRHS.push_back(LHSvariable);
            }
            break;
            case VAL::E_LESSEQ: {
                makeOneSided(RHSvariable, LHSvariable, negOffset);
                finalLHS.push_back(RHSvariable);
                finalOp.push_back(VAL::E_GREATER);
                finalRHS.push_back(LHSvariable);
            }
            break;
            case VAL::E_EQUALS: {
                postmortem_noADL();
            }
            break;
            }
        }

        {
            list<pair<list<double>, list<int> > >::iterator lhsItr = finalLHS.begin();
            const list<pair<list<double>, list<int> > >::iterator lhsEnd = finalLHS.end();
            list<VAL::comparison_op>::iterator opItr = finalOp.begin();
            list<pair<list<double>, list<int> > >::iterator rhsItr = finalRHS.begin();

            for (; lhsItr != lhsEnd; ++lhsItr, ++opItr, ++rhsItr) {
                if (debugRPGNum) {
                    cout << "Storing built precondition\n";
                    {
                        cout << "\t";
                        printStackTerm(lhsItr->first, lhsItr->second);
                        
                    }
                    if (*opItr == VAL::E_GREATER) {
                        cout << " > ";
                    } else {
                        cout << " >= ";
                    }
                    {
                        printStackTerm(rhsItr->first, rhsItr->second);                        
                    }
                    cout << "\n";
                }
                ++toIncrement;
                pair<list<double>, list<int> > & currLHS = *lhsItr;
                pair<list<double>, list<int> > & currRHS = *rhsItr;
                VAL::comparison_op currOp = *opItr;

                int lVar = -1;
                double lConst = 0.0;
                bool lIsConst = false;

                double rConst = 0.0;
                bool rIsConst = false;

                {
                    int rSize = currRHS.first.size();
                    if (rSize == 1) {
                        if (currRHS.second.front() == -1) {
                            rIsConst = true;
                            rConst = currRHS.first.front();
                        }
                    } else if (!rSize) {
                        rIsConst = true;
                    }
                }

                assert(rIsConst);

                {
                    int lSize = currLHS.first.size();
                    if (lSize == 1) {
                        if (currLHS.second.front() == -1) {
                            lIsConst = true;
                            lConst = currLHS.first.front();
                        }
                    } else if (!lSize) {
                        lIsConst = true;
                    }
                }



                if (!lIsConst) {


                    int lSize = currLHS.first.size();
                    if (lSize > 1) {
                        int a = 0;
                        double constTerm = 0.0;

                        vector<double> wVector(lSize);
                        vector<int> vVector(lSize);

                        list<double>::iterator wItr = currLHS.first.begin();
                        const list<double>::iterator wEnd = currLHS.first.end();
                        list<int>::iterator vItr = currLHS.second.begin();

                        for (; wItr != wEnd; ++wItr, ++vItr) {
                            if (*vItr == -1) {
                                constTerm = *wItr;
                            } else {
                                wVector[a] = *wItr;
                                vVector[a] = *vItr;
                                ++a;
                            }
                        }
                        ArtificialVariable newAV(avCount, a, wVector, vVector, constTerm, rConst);
                        pair<set<ArtificialVariable>::iterator, bool> insResult = artificialVariableSet.insert(newAV);
                        if (insResult.second) {
                            ++avCount;
                        } else {
                            if (debugRPGNum) {
                                cout << "Existing AV: " << *(insResult.first) << "\n";
                            }
                            (const_cast<ArtificialVariable*>(&(*insResult.first)))->updateMax(rConst);
                        }
                        lVar = insResult.first->ID;
                        if (debugRPGNum) {
                            cout << "LHS = artificial variable " << lVar << "\n";
                        }
                    } else if (lSize) {
                        const int vCheck = currLHS.second.front();
                        if (vCheck == -1) {
                            assert(false);  // see above
                            lConst = currLHS.first.front();
                            if (debugRPGNum) {
                                cout << "LHS = constant " << lConst << "\n";
                            }
                        } else {
                            lVar = vCheck;
                            lConst = currLHS.first.front();
                            if (debugRPGNum) {
                                cout << "LHS =";
                                if (lConst != 1.0) cout << " " << lConst << " *";
                                cout << " variable " << lVar << "\n";
                            }
                            assert(lConst > 0.0); // would be insane otherwise - the negative variables thing should have kicked in, and having 0*var as a function is just plain silly
                            const double newMaxNeed = rConst / lConst;
                            if (lVar >= 0) {
                                if (newMaxNeed > localMaxNeed[lVar]) localMaxNeed[lVar] = newMaxNeed;
                            }
                        }
                    } else {
                        assert(false);  // see above
                        if (debugRPGNum) {
                            cout << "LHS is empty\n";
                        }
                        lConst = 0.0;
                        // do nothing - side is empty, bizarrely enough
                    }
                } else {
                    if (currOp == VAL::E_GREATER) {
                        if (lConst <= rConst) {
                            return false;
                        }
                    } else {
                        if (lConst < rConst) {
                            return false;
                        }
                    } 
                }

                if (lIsConst) {
                    if (debugRPGNum) {
                        cout << "Tautologous condition - not bothering to create it\n";
                    }
                } else {
                    RPGNumericPrecondition newPrec(precCount, lVar, lConst, currOp, rConst);
                    map<RPGNumericPrecondition, list<pair<int, VAL::time_spec> > >::iterator insResult = rpgNumericPreconditionSet.find(newPrec);
                    if (insResult == rpgNumericPreconditionSet.end()) {
                        if (debugRPGNum) {
                            cout << "New RPGNumericPrecondition created, ID = " << newPrec.ID << "\n";
                            cout << "lv = " << newPrec.LHSVariable << ", lc = " << newPrec.LHSConstant << ", rv = " << newPrec.RHSVariable << ", rc = " << newPrec.RHSConstant << "\n";

                        }
                        list<pair<int, VAL::time_spec> > tmpList;
                        if (i >= 0) {
                            tmpList.push_back(pair<int, VAL::time_spec>(i, passTimeSpec));
                        }
                        rpgNumericPreconditionSet.insert(pair<RPGNumericPrecondition, list<pair<int, VAL::time_spec> > >(newPrec, tmpList));
                        destList.push_back(precCount);
                        ++precCount;
                        if (debugRPGNum) {
                            cout << "Registered that precondition applies to action " << i << "\n";
                            cout << "Registered that action uses precondition " << destList.back() << "\n";
                            cout << "precCount now " << precCount << "\n";

                        }
                    } else {

                        if (debugRPGNum) {
                            cout << "RPGNumericPrecondition reused, ID = " << insResult->first.ID << "\n";
                            cout << "lv = " << insResult->first.LHSVariable << ", lc = " << insResult->first.LHSConstant << ", rv = " << insResult->first.RHSVariable << ", rc = " << insResult->first.RHSConstant << "\n";

                        }
                        if (i >= 0) {
                            insResult->second.push_back(pair<int, VAL::time_spec>(i, passTimeSpec));
                        }
                        destList.push_back(insResult->first.ID);
                        if (debugRPGNum) {
                            cout << "Registered that precondition applies to action " << i << "\n";
                            cout << "Registered that action uses precondition " << destList.back() << "\n";


                        }
                    }
                }
            }

        }
    }

    if (debugRPGNum) {
        cout << "Action has " << toIncrement << " numeric preconditions\n";
    }
    
    return true;
}


void RPGBuilder::buildRPGNumericPreconditions()
{

    const bool debugRPGNum = (Globals::globalVerbosity & 16);

    set<ArtificialVariable> artificialVariableSet;
    map<RPGNumericPrecondition, list<pair<int, VAL::time_spec> > > rpgNumericPreconditionSet;
    const int negOffset = pnes.size();
    const int offset = negOffset * 2;
    int precCount = 0;
    int avCount = offset;

    const int opCount = instantiatedOps.size();

    rpgVariableDependencies = vector<list<int> >(offset);

    actionsToRPGNumericStartPreconditions = vector<list<int> >(opCount);
    actionsToRPGNumericInvariants = vector<list<int> >(opCount);
    actionsToRPGNumericEndPreconditions = vector<list<int> >(opCount);

    initialUnsatisfiedNumericStartPreconditions = vector<int>(opCount);
    initialUnsatisfiedNumericInvariants = vector<int>(opCount);
    initialUnsatisfiedNumericEndPreconditions = vector<int>(opCount);

    vector<double> localMaxNeed(offset);

    for (int i = 0; i < offset; ++i) localMaxNeed[i] = -DBL_MAX;

    for (int i = 0; i < opCount; ++i) {
 
        if (rogueActions[i]) {
            continue;
        }
        
        bool contradictoryPreconditions = false;
        
        for (int pass = 0; pass < 3; ++pass) {

            vector<list<NumericPrecondition> > * actionsToNumericPreconditions ;
            vector<list<int> > * actionsToRPGNumericPreconditions;
            vector<int> * initialUnsatisfiedNumericPreconditions;
            VAL::time_spec passTimeSpec;

            switch (pass) {

            case 0: {
                actionsToNumericPreconditions = &actionsToStartNumericPreconditions;
                actionsToRPGNumericPreconditions = &actionsToRPGNumericStartPreconditions;
                initialUnsatisfiedNumericPreconditions = &initialUnsatisfiedNumericStartPreconditions;
                passTimeSpec = VAL::E_AT_START;
                if (debugRPGNum) cout << "Building RPG Numeric Preconditions for start of " << *(instantiatedOps[i]) << "\n";
            }
            break;
            case 1: {
                actionsToNumericPreconditions = &actionsToNumericInvariants;
                actionsToRPGNumericPreconditions = &actionsToRPGNumericInvariants;
                initialUnsatisfiedNumericPreconditions = &initialUnsatisfiedNumericInvariants;
                passTimeSpec = VAL::E_OVER_ALL;
                if (debugRPGNum) cout << "Building RPG Numeric Preconditions for over all of " << *(instantiatedOps[i]) << "\n";
            }
            break;
            case 2: {
                actionsToNumericPreconditions = &actionsToEndNumericPreconditions;
                actionsToRPGNumericPreconditions = &actionsToRPGNumericEndPreconditions;
                initialUnsatisfiedNumericPreconditions = &initialUnsatisfiedNumericEndPreconditions;
                passTimeSpec = VAL::E_AT_END;
                if (debugRPGNum) cout << "Building RPG Numeric Preconditions for end of " << *(instantiatedOps[i]) << "\n";
            }
            break;
            default: {
                cout << "This should never happen\n";
                assert(false);
            }
            };

            

            if (!processPreconditions(artificialVariableSet, rpgNumericPreconditionSet, (*actionsToNumericPreconditions)[i], (*actionsToRPGNumericPreconditions)[i], (*initialUnsatisfiedNumericPreconditions)[i], negOffset, offset, precCount, avCount, localMaxNeed, i, passTimeSpec)) {
                contradictoryPreconditions = true;
                break;
            }


        }

        if (!actionsToConditionalEffects[i].empty()) {
            list<ProtoConditionalEffect*>::iterator ceItr = actionsToRawConditionalEffects[i].begin();
            const list<ProtoConditionalEffect*>::iterator ceEnd = actionsToRawConditionalEffects[i].end();
            list<ConditionalEffect>::iterator prcItr = actionsToConditionalEffects[i].begin();

            for (; !contradictoryPreconditions && ceItr != ceEnd; ++ceItr, ++prcItr) {

                ProtoConditionalEffect * const currRaw = *ceItr;
                ConditionalEffect & currCE = *prcItr;

                for (int pass = 0; pass < 3; ++pass) {

                    list<NumericPrecondition> * actionsToNumericPreconditions;
                    int unsat = 0;
                    list<int> dest;
                    VAL::time_spec passTimeSpec;

                    switch (pass) {

                    case 0: {
                        actionsToNumericPreconditions = &(currRaw->startPrecNumeric);
                        passTimeSpec = VAL::E_AT_START;
                        break;
                    }
                    case 1: {
                        actionsToNumericPreconditions = &(currRaw->invNumeric);
                        passTimeSpec = VAL::E_OVER_ALL;
                        break;
                    }
                    case 2: {
                        actionsToNumericPreconditions = &(currRaw->endPrecNumeric);
                        passTimeSpec = VAL::E_AT_END;
                        break;
                    }
                    default: {
                        cout << "This should never happen\n";
                        assert(false);
                    }
                    }



                    if (!processPreconditions(artificialVariableSet, rpgNumericPreconditionSet, *actionsToNumericPreconditions, dest, unsat, negOffset, offset, precCount, avCount, localMaxNeed, -2, passTimeSpec)) {
                        contradictoryPreconditions = true;
                        break;
                    }

                    list<int>::iterator destItr = dest.begin();
                    const list<int>::iterator destEnd = dest.end();

                    for (; destItr != destEnd; ++destItr) {
                        currCE.addCondition(*destItr, passTimeSpec);
                    }
                }
            }
        }
        
        if (contradictoryPreconditions) {
            pruneIrrelevant(i);
        }

    }


    { // now do the goals

        if (debugRPGNum) {
            cout << "Considering numeric goals\n";
        }

        list<NumericPrecondition>::iterator goalNumItr = numericGoals.begin();
        const list<NumericPrecondition>::iterator goalNumEnd = numericGoals.end();

        list<double>::const_iterator goalDeadItr = numericGoalDeadlines.begin();
        
        for (; goalNumItr != goalNumEnd; ++goalNumItr, ++goalDeadItr) {

            list<NumericPrecondition> justOne;
            justOne.push_back(*goalNumItr);

            int unsat = 0;
            list<int> dest;

            if (processPreconditions(artificialVariableSet, rpgNumericPreconditionSet, justOne, dest, unsat, negOffset, offset, precCount, avCount, localMaxNeed, -1, VAL::E_AT)) {
                

                const int destSize = dest.size();

                if (destSize == 2) {
                    if (debugRPGNum) cout << "Evaluated to pair " << dest.front() << ", " << dest.back() << "\n";
                    numericRPGGoals.push_back(pair<int, int>(dest.front(), dest.back()));
                    rpgNumericGoalDeadlines.push_back(*goalDeadItr);
                } else if (destSize == 1) {
                    if (debugRPGNum) cout << "Evaluated to single goal " << dest.front() << "\n";
                    numericRPGGoals.push_back(pair<int, int>(dest.front(), -1));
                    rpgNumericGoalDeadlines.push_back(*goalDeadItr);
                }
                
            } else {
                cout << "Problem cannot be solved: a numeric goal " << *goalNumItr << " evaluates to false\n";
                exit(0);
            }
        }

    }

    assert((int) rpgNumericPreconditionSet.size() == precCount);

    rpgNumericPreconditions = vector<RPGNumericPrecondition>(precCount);
    rpgNumericPreconditionsToActions = vector<list<pair<int, VAL::time_spec> > >(precCount);

    rpgArtificialVariables = vector<ArtificialVariable>(avCount - offset);
    rpgArtificialVariablesToPreconditions = vector<list<int> >(avCount - offset);
    rpgNegativeVariablesToPreconditions = vector<list<int> >(negOffset);
    rpgPositiveVariablesToPreconditions = vector<list<int> >(negOffset);

    maxNeeded = vector<double>(avCount);

    for (int i = 0; i < offset; ++i) maxNeeded[i] = localMaxNeed[i];

    {
        set<ArtificialVariable>::iterator avsItr = artificialVariableSet.begin();
        const set<ArtificialVariable>::iterator avsEnd = artificialVariableSet.end();

        for (; avsItr != avsEnd; ++avsItr) {
            const int oIndex = avsItr->ID - offset;
            rpgArtificialVariables[oIndex] = *avsItr;
            maxNeeded[avsItr->ID] = avsItr->maxNeed;
            if (debugRPGNum) cout << "Storing AV with ID " << avsItr->ID << " at index " << oIndex << "\n";
            {
                const int afflim = avsItr->size;
                for (int aff = 0; aff < afflim; ++aff) {
                    const int affVar = avsItr->fluents[aff];
                    if (affVar >= 0) {
                        rpgVariableDependencies[affVar].push_back(avsItr->ID);
                    }
                }
            }
        }
    }

    {
        map<RPGNumericPrecondition, list<pair<int, VAL::time_spec> > >::iterator rnpItr = rpgNumericPreconditionSet.begin();
        const map<RPGNumericPrecondition, list<pair<int, VAL::time_spec> > >::iterator rnpEnd = rpgNumericPreconditionSet.end();

        for (; rnpItr != rnpEnd; ++rnpItr) {
            const RPGNumericPrecondition & currPrec = rnpItr->first;
            const int currID = currPrec.ID;
            rpgNumericPreconditions[currID] = currPrec;
            rpgNumericPreconditionsToActions[currID] = rnpItr->second;

            if (debugRPGNum) cout << "Precondition ID: " << currID << "\n";

            {
                const int var = currPrec.LHSVariable;
                if (var >= offset) {
                    rpgArtificialVariablesToPreconditions[var-offset].push_back(currID);
                } else if (var >= negOffset) {
                    rpgNegativeVariablesToPreconditions[var-negOffset].push_back(currID);
                } else if (var >= 0) {
                    rpgPositiveVariablesToPreconditions[var].push_back(currID);
                }
            }



            if (debugRPGNum) {
                cout << "Built precondition " << currPrec << "\n";
                cout << "Applies to actions:\n";
                list<pair<int, VAL::time_spec> >::iterator ataItr = rpgNumericPreconditionsToActions[currID].begin();
                const list<pair<int, VAL::time_spec> >::iterator ataEnd = rpgNumericPreconditionsToActions[currID].end();
                for (; ataItr != ataEnd; ++ataItr) {
                    cout << "\t" << ataItr->first << " " << *(instantiatedOps[ataItr->first]) << " " << (ataItr->second == VAL::E_AT_START ? "start" : (ataItr->second == VAL::E_AT_END ? "end" : "over all")) << "\n";
                }
            }
        }

    }

    numericAchievedInLayer = vector<double>(precCount);
    numericAchievedInLayerReset = vector<double>(precCount);
    numericAchievedBy = vector<ActionFluentModification*>(precCount);
    numericAchievedByReset = vector<ActionFluentModification*>(precCount);

    {
        for (int i = 0; i < precCount; ++i) {
            numericAchievedInLayerReset[i] = -1.0;
            numericAchievedByReset[i] = 0;
        }
    }

    if (debugRPGNum) {
        cout << "All done!\n";
    }

}

void RPGBuilder::buildRPGNumericEffects()
{

    const bool localDebug = (Globals::globalVerbosity & 16);

    const int opCount = instantiatedOps.size();

    const int negOffset = pnes.size();

    realVariablesToRPGEffects = vector<list<int> >(negOffset * 2);

    actionsToRPGNumericStartEffects = vector<list<int> >(opCount);
    actionsToRPGNumericEndEffects = vector<list<int> >(opCount);

    map<RPGNumericEffect, list<pair<int, VAL::time_spec> > > rpgNumericEffectSet;


    const list<pair<int, VAL::time_spec> > emptyList;

    int effID = 0;

    for (int act = 0; act < opCount; ++act) {

        if (rogueActions[act]) {
            continue;
        }
        
        linearDiscretisation[act] = buildLE(act);

        bool doingMainEffects = true;

        list<ProtoConditionalEffect*>::iterator ceItr = actionsToRawConditionalEffects[act].begin();
        const list<ProtoConditionalEffect*>::iterator ceEnd = actionsToRawConditionalEffects[act].end();
        list<ConditionalEffect>::iterator prcItr = actionsToConditionalEffects[act].begin();

        while (doingMainEffects || ceItr != ceEnd) {

            for (int pass = 0; pass < 2; ++pass) {

                list<NumericEffect> & numEffs = (doingMainEffects
                                                 ? (pass ? actionsToEndNumericEffects[act] : actionsToStartNumericEffects[act])
                                                         : (pass ? (*ceItr)->startNumericEff : (*ceItr)->endNumericEff));

                list<NumericEffect>::iterator neItr = numEffs.begin();
                const list<NumericEffect>::iterator neEnd = numEffs.end();

                for (; neItr != neEnd; ++neItr) {

                    if (localDebug) {
                        if (doingMainEffects) {
                            cout << "Action " << act << " has " << (pass ? "end" : "start") << " effect:\n";
                        } else {
                            cout << "Conditional effect of action " << act << " has " << (pass ? "end" : "start") << " effect:\n";
                        }
                    }

                    NumericEffect & currEffect = *neItr;

                    pair<list<double>, list<int> > weightedSum;

                    makeWeightedSum(currEffect.formula, weightedSum);

                    if (currEffect.op == VAL::E_DECREASE) {
                        list<double>::iterator dlItr = weightedSum.first.begin();
                        const list<double>::iterator dlEnd = weightedSum.first.end();

                        for (; dlItr != dlEnd; ++dlItr) {
                            if (*dlItr != 0.0) *dlItr = 0.0 - *dlItr;
                        }
                    }

                    {
                        list<double>::iterator dlItr = weightedSum.first.begin();
                        list<int>::iterator ilItr = weightedSum.second.begin();
                        const list<double>::iterator dlEnd = weightedSum.first.end();

                        for (; dlItr != dlEnd; ++dlItr, ++ilItr) {
                            if (*dlItr < 0.0) { // for negative weights
                                if (*ilItr == -3) {
                                    *dlItr = 0.0 - *dlItr;
                                    *ilItr = -19;
                                } else if (*ilItr == -19) {
                                    *dlItr = 0.0 - *dlItr;
                                    *ilItr = -3;
                                } else if (*ilItr == -2) {
                                    *dlItr = 0.0 - *dlItr;
                                    *ilItr = -18;
                                } else if (*ilItr == -18) {
                                    *dlItr = 0.0 - *dlItr;
                                    *ilItr = -2;
                                } else if (*ilItr >= 0) {
                                    *ilItr += negOffset;
                                    *dlItr = 0.0 - *dlItr;
                                }
                            }

                        }
                    }


                    int lSize = weightedSum.first.size();

                    double constTerm = 0.0;

                    int a = 0;

                    vector<double> wVector(lSize);
                    vector<int> vVector(lSize);

                    list<double>::iterator wItr = weightedSum.first.begin();
                    const list<double>::iterator wEnd = weightedSum.first.end();
                    list<int>::iterator vItr = weightedSum.second.begin();

                    for (; wItr != wEnd; ++wItr, ++vItr) {
                        if (*vItr == -1) {
                            constTerm = *wItr;
                        } else {
                            wVector[a] = *wItr;
                            vVector[a] = *vItr;
                            ++a;
                        }
                    }

                    RPGNumericEffect rpgEffect(effID, currEffect.fluentIndex, currEffect.op == VAL::E_ASSIGN, wVector, vVector, a, constTerm);

                    pair<map<RPGNumericEffect, list<pair<int, VAL::time_spec> > >::iterator, bool> insItr
                    = rpgNumericEffectSet.insert(pair<RPGNumericEffect, list<pair<int, VAL::time_spec> > >(rpgEffect, emptyList));

                    if (insItr.second) {
                        {
                            list<double>::iterator dlItr = weightedSum.first.begin();
                            list<int>::iterator ilItr = weightedSum.second.begin();
                            const list<double>::iterator dlEnd = weightedSum.first.end();

                            for (; dlItr != dlEnd; ++dlItr, ++ilItr) {
                                if (*ilItr >= 0) {
                                    realVariablesToRPGEffects[*ilItr].push_back(insItr.first->first.ID);
                                }

                            }
                        }
                        ++effID;
                    }

                    insItr.first->second.push_back(pair<int, VAL::time_spec>(act, (pass ? VAL::E_AT_END : VAL::E_AT_START)));

                    if (doingMainEffects) {
                        insItr.first->second.push_back(pair<int, VAL::time_spec>(act, (pass ? VAL::E_AT_END : VAL::E_AT_START)));
                        (pass ? actionsToRPGNumericEndEffects : actionsToRPGNumericStartEffects)[act].push_back(insItr.first->first.ID);
                    } else {
                        (*prcItr).addNumericEffect(insItr.first->first.ID, (pass ? VAL::E_AT_END : VAL::E_AT_START));
                    }



                    if (localDebug) {

                        cout << "\t" << *(getPNE(currEffect.fluentIndex)) << " ";
                        if (currEffect.op == VAL::E_ASSIGN) {
                            cout << "= ";
                        } else {
                            cout << "+= ";
                        }

                        list<double>::iterator wItr = weightedSum.first.begin();
                        const list<double>::iterator wEnd = weightedSum.first.end();
                        list<int>::iterator vItr = weightedSum.second.begin();
                        bool isFirst = true;
                        for (; wItr != wEnd; ++wItr, ++vItr) {
                            if (!isFirst) cout << " + ";
                            if (*vItr == -1) {
                                cout << *wItr;
                            } else if (*vItr == -3) {
                                cout << "(" << *wItr << " * ?duration)";
                            } else if (*vItr == -19) {
                                cout << "(" << *wItr << " * -?duration)";
                            } else if (*vItr == -2) {
                                cout << "(" << *wItr << " * #t)";
                            } else if (*vItr == -18) {
                                cout << "(" << *wItr << " * -#t)";
                            } else if (*vItr < 0) {
                                cout << "(" << *wItr << " * <special>)";
                            } else if (*vItr >= negOffset) {
                                cout << "-";
                                if (*wItr != 1.0) cout << "(" << *wItr << " * ";
                                cout << *(getPNE(*vItr - negOffset));
                                if (*wItr != 1.0) cout << ")";
                            } else {
                                if (*wItr != 1.0) cout << "(" << *wItr << " * ";
                                cout << *(getPNE(*vItr));
                                if (*wItr != 1.0) cout << ")";
                            }
                            isFirst = false;
                        }
                        cout << "\n";
                        if (insItr.second) cout << "Effect hasn't been seen before anywhere\n";
                    }
                }
            }
            if (doingMainEffects) {
                doingMainEffects = false;
            } else {
                ++ceItr;
                ++prcItr;
            }
            
        }

        rpgNumericEffects = vector<RPGNumericEffect>(effID);
        rpgNumericEffectsToActions = vector<list<pair<int, VAL::time_spec> > >(effID);

        map<RPGNumericEffect, list<pair<int, VAL::time_spec> > >::iterator effItr = rpgNumericEffectSet.begin();
        const map<RPGNumericEffect, list<pair<int, VAL::time_spec> > >::iterator effEnd = rpgNumericEffectSet.end();

        for (; effItr != effEnd; ++effItr) {

            const RPGNumericEffect & currEff = effItr->first;
            const int currID = currEff.ID;

            rpgNumericEffects[currID] = effItr->first;
            rpgNumericEffectsToActions[currID] = effItr->second;

        }

    };

}

void RPGBuilder::buildMetric(VAL::metric_spec * ms)
{

    if (!ms) return;

    theMetric = new Metric(ms->opt == E_MINIMIZE);
    list<Operand> tmpFormula;
    ExpressionBuilder builder(tmpFormula, 0, 0);
    builder.buildFormula(const_cast<VAL::expression*>(ms->expr));
    pair<list<double>, list<int> > result;
    WhereAreWeNow = PARSE_METRIC;
    RPGBuilder::makeWeightedSum(tmpFormula, result);
    WhereAreWeNow = PARSE_UNKNOWN;
    theMetric->weights = result.first;
    theMetric->variables = result.second;
    
    const int varCount = getPNECount();
    
    list<int>::iterator varItr = theMetric->variables.begin();
    const list<int>::iterator varEnd = theMetric->variables.end();
    
    list<double>::iterator wItr = theMetric->weights.begin();
        
    while (varItr != varEnd) {
        if (*varItr == -1) {
            const list<int>::iterator varPrev = varItr; ++varItr;
            const list<double>::iterator wPrev = wItr; ++wItr;
            
            theMetric->variables.erase(varPrev);
            theMetric->weights.erase(wPrev);
        } else if (*varItr >= 0) {
            if (*varItr >= varCount) {
                *varItr -= varCount;
                *wItr *= -1;
            }
            metricVars.insert(*varItr);
            ++varItr;
            ++wItr;
        } else {
            if (*varItr <= -16) {
                *varItr += 16;
                metricVars.insert(*varItr + 16);                    
                if (*wItr != 0.0) *wItr = -*wItr;
            } else {
                metricVars.insert(*varItr);
            }
            ++varItr; ++wItr;
                            
        }                                                
    }

}

};
