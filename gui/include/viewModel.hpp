//
// Created by sigi on 08.08.19.
//

#ifndef EPIPHANY_VIEWMODEL_HPP
#define EPIPHANY_VIEWMODEL_HPP

#include <QtCore/QArgument>

namespace epi{
    class ViewModel {

    public slots:
        void defRoute();
        void resetRoute();
        void startRoute();
    };
}
#endif //EPIPHANY_VIEWMODEL_HPP
