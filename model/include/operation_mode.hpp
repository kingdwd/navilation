//
// Created by sigi on 12.08.19.
//

#ifndef EPIPHANY_OPERATIONMODE_HPP
#define EPIPHANY_OPERATIONMODE_HPP

#include "util.h"

namespace epi{
    enum class operation_mode{
        AUTOMATIC, MANUAL
    };
    template<typename T>
    std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
    {
        return stream << static_cast<typename std::underlying_type<T>::type>(e);
    }

    struct OperationModeProvider {
        U::Var<operation_mode> state{operation_mode::MANUAL};
        void switchMode(){
            if(state.get() == operation_mode::MANUAL){
                state.set(operation_mode::AUTOMATIC);
            } else {
                state.set(operation_mode::MANUAL);
            }
            std::cout<<"Opertation mode changed to " << state.get() << "\n";
        }
    };

}
#endif //EPIPHANY_OPERATIONMODE_HPP
