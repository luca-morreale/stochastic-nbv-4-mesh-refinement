#ifndef CAM_POSITION_GENERATOR_CPLEX_VECTOR_OPERATIONS_H
#define CAM_POSITION_GENERATOR_CPLEX_VECTOR_OPERATIONS_H

#include <opview/alias_definition.h>

namespace opview {

    #define MIN_VAR_DOMAIN -1000.0
    #define MAX_VAR_DOMAIN 1000.0

    IloNumVarList convertVector(unsigned int vecSize, IloEnv env);
    IloExprList diff(CGALVec3 &a, IloNumVarList &b);
    IloExpr dot(CGALVec3 &a, IloNumVarList &b);
    IloExpr dot(CGALVec3 &a, IloExprList &b);
    IloExpr norm(IloNumVarList &v);      // cplex can not handle root so not performed, not even needed to!
    IloExpr norm(IloExprList &v);

}   // namespace opview

#endif // CAM_POSITION_GENERATOR_CPLEX_VECTOR_OPERATIONS_H
