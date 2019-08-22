//
// Created by sigi on 12.08.19.
//

#ifndef EPIPHANY_OPERATIONMODE_HPP
#define EPIPHANY_OPERATIONMODE_HPP

#include "U.h"

namespace epi{
    enum class OperationMode{
        AUTOMATIC, MANUAL
    };
    template<typename T>
    std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
    {
        return stream << static_cast<typename std::underlying_type<T>::type>(e);
    }

    struct OperationModeProvider {
        U::Var<OperationMode> state{OperationMode::MANUAL};
        void switchMode(){
            if(state.get() == OperationMode::MANUAL){
                state.set(OperationMode::AUTOMATIC);
            } else {
                state.set(OperationMode::MANUAL);
            }
            std::cout<<"Opertation mode changed to " << state.get() << "\n";
        }
    };

}
#endif //EPIPHANY_OPERATIONMODE_HPP
