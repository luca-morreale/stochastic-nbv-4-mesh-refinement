#include <opview/CplexVectorOperations.hpp>

namespace opview {

    IloNumVarList convertVector(unsigned int vecSize, IloEnv env)
    {
        IloNumVarList list;
        for (int i = 0; i < vecSize; i++) {
            IloNumVar var(env, MIN_VAR_DOMAIN, MAX_VAR_DOMAIN, ILOFLOAT);
            list.push_back(var);
        }
        return list;
    }

    IloExprList diff(CGALVec3 &a, IloNumVarList &b)
    {
        IloExprList exps;
        for (int i = 0; i < a.dimension(); i++) {
            exps.push_back(a[i] - b[i]);
        }

        return exps;
    }

    IloExpr dot(CGALVec3 &a, IloNumVarList &b)
    {
        IloExpr exp;
        for (int i = 0; i < a.dimension(); i++) {
            exp += a[i] * b[i];
        }

        return exp;
    }

    IloExpr dot(CGALVec3 &a, IloExprList &b)
    {
        IloExpr exp;
        for (int i = 0; i < a.dimension(); i++) {
            exp += a[i] * b[i];
        }

        return exp;
    }

    IloExpr norm(IloNumVarList &v)
    {
        IloExpr exp;
        for (int i = 0; i < v.size(); i++) {
            exp += IloSquare(v[i]);
        }
        return exp;
    }

    IloExpr norm(IloExprList &v)
    {
        IloExpr exp;
        for (int i = 0; i < v.size(); i++) {
            exp += IloSquare(v[i]);
        }
        return exp;
    }

    


} // namespace opview
