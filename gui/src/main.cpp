#include <iostream>
#include <string>
#include <opencv4/opencv2/core.hpp>


#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <thread>
#include <chrono>
#include <Spline.hpp>
#include "view.hpp"
#include "UU.h"
#include "data.h"
#include <QApplication>
#include <QTextEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include "U.h"

#include "viewControl.hpp"
#include "OperationMode.hpp"
#include "MpcFactory.hpp"


using namespace std;
using namespace epi;
using Clock = std::chrono::high_resolution_clock ;
using namespace cv;
using namespace std::literals;


enum Key{
    none = -1,
    j = 106,
    k = 107,
    l = 108,
    u = 117,
    i = 105,
    o = 111,
    h = 104,
    oe = 246
};


int waitUntilEscapeInManual(shared_ptr<System> sys){
    int key=0;
    while(key != 27) {
        key = waitKey(9);
        switch (key) {
            case Key::none:
                sys->move(0, 0);
                break;
            case Key::i:
                sys->move(1, 0);
                break;
            case Key::k:
                sys->move(-1, 0);
                break;

            case Key::u:
                sys->move(1, 1);
                break;
            case Key::j:
                sys->move(-1, 1);
                break;

            case Key::o:
                sys->move(1, -1);
                break;
            case Key::l:
                sys->move(-1, -1);
                break;

            case Key::h:
                sys->move(0, 1);
                break;
            case Key::oe:
                sys->move(0, -1);
                break;
        }
    }
    return 0;
    //return key != 27;
}


int main(int arg0, char** args)
{

    QApplication app(arg0, args);


    unique_ptr<grampc::Grampc> mpc{epi::createMpc()};
    shared_ptr<OperationModeProvider> modeProvider = make_shared<OperationModeProvider>();
    shared_ptr<Vehicle> car = make_shared<Vehicle>(make_unique<DynamicCarModel>());
    shared_ptr<System> sys = make_shared<System>(car, modeProvider, move(mpc));
    ViewBuilder viewBuilder(sys);

    viewBuilder.show();
    waitUntilEscapeInManual(sys);

    return app.exec();
}

