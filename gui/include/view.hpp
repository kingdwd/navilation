//
// Created by sigi on 04.08.19.
//

#ifndef EPIPHANY_VIEW_HPP
#define EPIPHANY_VIEW_HPP

#include <memory>
#include <utility>
#include <QtWidgets/QApplication>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include "viewModel.hpp"
#include "OperationMode.hpp"

namespace epi{

    class View {

    public:
        View(std::shared_ptr<ViewModel> model);
    private:
        std::shared_ptr<ViewModel> _modelPtr;
    };

}
#endif //EPIPHANY_VIEW_HPP
