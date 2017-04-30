#ifndef CAM_POSITION_GENERATOR_CPLEX_VECTOR_OPERATIONS_H
#define CAM_POSITION_GENERATOR_CPLEX_VECTOR_OPERATIONS_H

#include <ilconcert/iloexpression.h>

#include <opview/alias_definition.h>

namespace opview {

    #define MIN_VAR_DOMAIN -1000.0
    #define MAX_VAR_DOMAIN 1000.0

    IloNumVarList createCplexVariables(unsigned int vecSize, IloEnv env);
    
    IloNumExprArray differenceExpr(IloNumVarList &a, CGALVec3 &b, IloEnv env);
    IloNumExprArray differenceExpr(IloExprList &a, CGALVec3 &b, IloEnv env);
    IloNumExprArray absDifferenceExpr(IloNumVarList &a, CGALVec3 &b, IloEnv env);
    IloNumExprArray absDifferenceExpr(IloExprList &a, CGALVec3 &b, IloEnv env);

    IloExpr dotExpr(CGALVec3 &a, IloNumVarList &b, IloEnv env);
    IloExpr dotExpr(CGALVec3 &a, IloExprList &b, IloEnv env);
    IloExpr dotExpr(CGALVec3 &a, IloNumExprArray &b, IloEnv env);
    IloExpr sqrtNormExpr(IloNumVarList &v, IloEnv env);     // cplex can not handle root so not performed, not even needed to!
    IloExpr sqrtNormExpr(IloExprList &v, IloEnv env);
    IloExpr sqrtNormExpr(IloNumExprArray &v, IloEnv env);

    IloExprList middlePoint(IloNumVarList &a, CGALVec3 &b, IloEnv env);

    IloExpr sqrtDistanceExpr(IloNumVarList &a, CGALVec3 &b, IloEnv env);
    IloExpr sqrtDistanceExpr(IloExprList &a, CGALVec3 &b, IloEnv env);
    IloExpr linearDistanceExpr(IloNumVarList &a, CGALVec3 &b, IloEnv env);
    IloExpr linearDistanceExpr(IloExprList &a, CGALVec3 &b, IloEnv env);
    IloExpr pyramidDistanceExpr(IloNumVarList &a, CGALVec3 &b, IloEnv env);
    IloExpr pyramidDistanceExpr(IloExprList &a, CGALVec3 &b, IloEnv env);
    IloExpr octagonalDistanceExpr(IloNumVarList &a, CGALVec3 &b, IloEnv env);
    IloExpr octagonalDistanceExpr(IloExprList &a, CGALVec3 &b, IloEnv env);
    IloExpr manhattanDistanceExpr(IloNumVarList &a, CGALVec3 &b, IloEnv env);
    IloExpr manhattanDistanceExpr(IloExprList &a, CGALVec3 &b, IloEnv env);

}   // namespace opview

#endif // CAM_POSITION_GENERATOR_CPLEX_VECTOR_OPERATIONS_H
