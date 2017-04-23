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

    IloNumExprArray absDifference(IloNumVarList &a, GLMVec3 &b)
    {
        IloNumExprArray diffs;
        for (int i = 0; i < a.size(); i++) {
            diffs.add(IloAbs(a[i]-b[i]));
        }
        return diffs;
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

    IloExpr sqrtDistance(IloNumVarList &a, GLMVec3 &b)
    {
        IloExpr distance;
        for (int i = 0; i < a.size(); i++) {
            distance += IloSquare(a[i] - b[i]);
        }
        return distance;
    }

    IloExpr linearDistance(IloNumVarList &a, GLMVec3 &b)
    {
        IloNumExprArray diffs = absDifference(a, b);
        return IloMin(diffs);    // IloMin(const IloNumExprArray exprs)
    }

    IloExpr pyramidDistance(IloNumVarList &a, GLMVec3 &b)
    {
        IloNumExprArray diffs = absDifference(a, b);
        return IloMax(diffs);
    }

    IloExpr octagonalDistance(IloNumVarList &a, GLMVec3 &b)
    {
        return 1007.0 / 1024.0 * pyramidDistance(a, b) + 441.0 / 1024.0 * linearDistance(a, b);
    }

    


} // namespace opview
