#include <opview/CplexVectorOperations.hpp>

namespace opview {

    // ============>> Should initialize all expressions with the environment before using them!

    IloNumVarList createCplexVariables(unsigned int vecSize, IloEnv env)
    {
        IloNumVarList list;
        for (int i = 0; i < vecSize; i++) {
            IloNumVar var(env, MIN_VAR_DOMAIN, MAX_VAR_DOMAIN, ILOFLOAT);
            list.push_back(var);
        }
        return list;
    }

    IloNumExprArray differenceExpr(IloNumVarList &a, CGALVec3 &b, IloEnv env)
    {
        IloNumExprArray diffs(env);
        for (int i = 0; i < a.size(); i++) {
            diffs.add(a[i] - b[i]);
        }
        return diffs;
    }

    IloNumExprArray differenceExpr(IloExprList &a, CGALVec3 &b, IloEnv env)
    {
        IloNumExprArray diffs(env);
        for (int i = 0; i < a.size(); i++) {
            diffs.add(a[i] - b[i]);
        }
        return diffs;
    }

    IloNumExprArray absDifferenceExpr(IloNumVarList &a, CGALVec3 &b, IloEnv env)
    {
        IloNumExprArray diffs(env);
        for (int i = 0; i < a.size(); i++) {
            diffs.add(IloAbs(a[i]-b[i]));
        }
        return diffs;
    }

    IloNumExprArray absDifferenceExpr(IloExprList &a, CGALVec3 &b, IloEnv env)
    {
        IloNumExprArray diffs(env);
        for (int i = 0; i < a.size(); i++) {
            diffs.add(IloAbs(a[i]-b[i]));
        }
        return diffs;
    }

    IloExpr dotExpr(CGALVec3 &a, IloNumVarList &b, IloEnv env)
    {
        IloExpr exp(env);
        for (int i = 0; i < a.dimension(); i++) {
            exp += a[i] * b[i];
        }
        return exp;
    }

    IloExpr dotExpr(CGALVec3 &a, IloExprList &b, IloEnv env)
    {
        IloExpr exp(env);
        for (int i = 0; i < a.dimension(); i++) {
            exp += a[i] * b[i];
        }
        return exp;
    }

    IloExpr dotExpr(CGALVec3 &a, IloNumExprArray &b, IloEnv env)
    {
        IloExpr exp(env);
        for (int i = 0; i < a.dimension(); i++) {
            exp += a[i] * b[i];
        }
        return exp;
    }

    IloExpr sqrtNormExpr(IloNumVarList &v, IloEnv env)
    {
        IloExpr exp(env);
        for (int i = 0; i < v.size(); i++) {
            exp += IloSquare(v[i]);
        }
        return exp;
    }

    IloExpr sqrtNormExpr(IloExprList &v, IloEnv env)
    {
        IloExpr exp(env);
        for (int i = 0; i < v.size(); i++) {
            exp += IloSquare(v[i]);
        }
        return exp;
    }

    IloExpr sqrtNormExpr(IloNumExprArray &v, IloEnv env)
    {
        IloExpr exp(env);
        for (int i = 0; i < 3; i++) {
            exp += IloSquare(v[i]);
        }
        return exp;
    }

    IloExprList middlePoint(IloNumVarList &a, CGALVec3 &b, IloEnv env)
    {
        IloExprList middle;
        for (int i = 0; i < a.size(); i++) {
            IloExpr tmp(env);
            tmp = (a[i] + b[i]) / 2.0;
            middle.push_back(tmp);
        }
        return middle;
    }

    IloExpr sqrtDistanceExpr(IloNumVarList &a, CGALVec3 &b, IloEnv env)
    {
        IloExpr distance(env);
        for (int i = 0; i < a.size(); i++) {
            distance += IloSquare(a[i] - b[i]);
        }
        return distance;
    }

    IloExpr sqrtDistanceExpr(IloExprList &a, CGALVec3 &b, IloEnv env)
    {
        IloExpr distance(env);
        for (int i = 0; i < a.size(); i++) {
            distance += IloSquare(a[i] - b[i]);
        }
        return distance;
    }

    IloExpr linearDistanceExpr(IloNumVarList &a, CGALVec3 &b, IloEnv env)
    {
        IloNumExprArray diffs = absDifferenceExpr(a, b, env);
        return IloMin(diffs);
    }

    IloExpr linearDistanceExpr(IloExprList &a, CGALVec3 &b, IloEnv env)
    {
        IloNumExprArray diffs = absDifferenceExpr(a, b, env);
        return IloMin(diffs);
    }

    IloExpr pyramidDistanceExpr(IloNumVarList &a, CGALVec3 &b, IloEnv env)
    {
        IloNumExprArray diffs = absDifferenceExpr(a, b, env);
        return IloMax(diffs);
    }

    IloExpr pyramidDistanceExpr(IloExprList &a, CGALVec3 &b, IloEnv env)
    {
        IloNumExprArray diffs = absDifferenceExpr(a, b, env);
        return IloMax(diffs);
    }

    IloExpr octagonalDistanceExpr(IloNumVarList &a, CGALVec3 &b, IloEnv env)
    {
        return 1007.0 / 1024.0 * pyramidDistanceExpr(a, b, env) + 441.0 / 1024.0 * linearDistanceExpr(a, b, env);
    }

    IloExpr octagonalDistanceExpr(IloExprList &a, CGALVec3 &b, IloEnv env)
    {
        return 1007.0 / 1024.0 * pyramidDistanceExpr(a, b, env) + 441.0 / 1024.0 * linearDistanceExpr(a, b, env);
    }

    IloExpr manhattanDistanceExpr(IloNumVarList &a, CGALVec3 &b, IloEnv env)
    {
        IloNumExprArray absDistances = absDifferenceExpr(a, b, env);
        return IloSum(absDistances);
    }

    IloExpr manhattanDistanceExpr(IloExprList &a, CGALVec3 &b, IloEnv env)
    {
        IloNumExprArray absDistances = absDifferenceExpr(a, b, env);
        return IloSum(absDistances);
    }
    


} // namespace opview
