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

namespace epi{

    class View : public QMainWindow {
        Q_OBJECT

    public slots:
        void defRoute();

    public:
        View(std::unique_ptr<ViewModel> model, QWidget* parent = 0);
    private:
        std::unique_ptr<ViewModel> _model;
    };

}
#endif //EPIPHANY_VIEW_HPP
