//
// Created by sigi on 08.08.19.
//

#ifndef EPIPHANY_VIEWMODEL_HPP
#define EPIPHANY_VIEWMODEL_HPP

#include "U.h"
#include <QObject>
#include <memory>

#include "mouseClickHandler.hpp"
#include "viewControl.hpp"
#include "System.hpp"

namespace epi{
    class ViewModel : public QObject{
    public:
        ViewModel(std::shared_ptr<MouseClickHandler> mouseHandler
                , std::shared_ptr<epi::System> sys
                , ViewBuilder* vb);
        ~ViewModel();
    public slots:
        void switchOperationMode();

        void defRoute();
        void resetRoute();
        void startRoute();

    private:
        Q_OBJECT
        struct Impl;
        std::unique_ptr<Impl> pImpl;
        std::shared_ptr<MouseClickHandler> _mouseHandler;

    };
}
#endif //EPIPHANY_VIEWMODEL_HPP
