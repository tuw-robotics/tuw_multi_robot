#ifndef __TUW_ORDERMANAGER_SIMPLESOLVER_H
#define __TUW_ORDERMANAGER_SIMPLESOLVER_H
#include "abstractsolver.h"

namespace tuw_order_manager {

struct dist_pair {
    float distance;
    std::string robot_name;
    int good_id;
};

class SimpleSolver : public AbstractSolver {
public:
    using AbstractSolver::AbstractSolver;
    std::vector<transport_pair> solve() override;
};

} // end namespace tuw_order_manager
#endif
