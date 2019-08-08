//
// Created by sigi on 19.06.19.
//

#ifndef EPIPHANY_DIFFEQUATIONSOLVER_HPP
#define EPIPHANY_DIFFEQUATIONSOLVER_HPP

namespace U {
    namespace Math {
        class DiffEquationSolver {
        public:
            explicit DiffEquationSolver(double stepSize);
            virtual ~DiffEquationSolver() = default;

            template <typename T>
            T solve(T dx);
        };

        class RungeKutta : public DiffEquationSolver{};
    }
}
#endif //EPIPHANY_DIFFEQUATIONSOLVER_HPP
