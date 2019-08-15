//
// Created by sigi on 09.08.19.
//

#include <view.hpp>

using epi::View;

auto newLine(){
    auto lineA = new QFrame;
    lineA->setFrameShape(QFrame::HLine);
    lineA->setFrameShadow(QFrame::Sunken);
    return lineA;
}

View::View(std::shared_ptr<epi::ViewModel> model) :
    _modelPtr{model}
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
    auto yPos = new QLineEdit("y pos");
    auto thetaPos = new QLineEdit("theta");
    QPushButton *goPosButton = new QPushButton("GO");

    posSetGroup->addWidget(posLabel);
    posSetGroup->addWidget(xPos);
    posSetGroup->addWidget(yPos);
    posSetGroup->addWidget(thetaPos);
    posSetGroup->addWidget(goPosButton);
    /* =============================================== */

    /* =============================================== */
    auto routeSetGroup = new QHBoxLayout;
    auto defRouteBtn = new QPushButton("Def Route");
    QObject::connect(defRouteBtn, SIGNAL(clicked()), _modelPtr.get(), SLOT(defRoute()));
    auto resetRouteBtn = new QPushButton("Reset");
    QObject::connect(resetRouteBtn, SIGNAL(clicked()), _modelPtr.get(), SLOT(resetRoute()));
    auto goRouteBtn = new QPushButton("GO");
    QObject::connect(goRouteBtn, SIGNAL(clicked()), _modelPtr.get(), SLOT(startRoute()));

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



