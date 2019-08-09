//
// Created by sigi on 09.08.19.
//

#include <view.hpp>

auto newLine(){
    auto lineA = new QFrame;
    lineA->setFrameShape(QFrame::HLine);
    lineA->setFrameShadow(QFrame::Sunken);
    return lineA;
}
epi::View::View(std::unique_ptr<epi::ViewModel> model, QWidget *parent) :
    QMainWindow(parent), _model{std::move(model)}
{
    QPushButton *quitButton = new QPushButton("&Quit");
    QObject::connect(quitButton, SIGNAL (clicked()), qApp, SLOT (quit()));
    QWidget *window = new QWidget();


    window->setWindowTitle("Control Bar");
    /* =============================================== */
    auto manualMode = new QRadioButton("Manual");
    /* =============================================== */

    /* =============================================== */
    auto posSetGroup = new QHBoxLayout;
    auto posLabel = new QLabel("Pose");
    auto xPos = new QLineEdit("x pos");
    auto yPos = new QLineEdit;
    auto phiPos = new QLineEdit;
    QPushButton *goPosButton = new QPushButton("GO");

    posSetGroup->addWidget(posLabel);
    posSetGroup->addWidget(xPos);
    posSetGroup->addWidget(yPos);
    posSetGroup->addWidget(phiPos);
    posSetGroup->addWidget(goPosButton);
    /* =============================================== */

    /* =============================================== */
    auto routeSetGroup = new QHBoxLayout;
    auto defRouteBtn = new QPushButton("Def Route");
    QObject::connect(defRouteBtn, SIGNAL(clicked()), this, SLOT(defRoute()));
    auto resetRouteBtn = new QPushButton("Reset");
    auto goRouteBtn = new QPushButton("GO");

    routeSetGroup->addWidget(defRouteBtn);
    routeSetGroup->addWidget(resetRouteBtn);
    routeSetGroup->addWidget(goRouteBtn);

    /* =============================================== */

    /* =============================================== */
    QVBoxLayout *vLayout = new QVBoxLayout;
    vLayout->addWidget(manualMode);
    vLayout->addWidget(newLine());
    vLayout->addLayout(posSetGroup);
    vLayout->addWidget(newLine());
    vLayout->addLayout(routeSetGroup);
    vLayout->addWidget(newLine());
    vLayout->addWidget(quitButton);
    /* =============================================== */

    window->setLayout(vLayout);
    window->show();
}

void epi::View::defRoute() {
    _model->defRoute();
}
