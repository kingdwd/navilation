#include <iostream>
#include <string>
#include <opencv4/opencv2/core.hpp>


#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <thread>
#include <chrono>
#include <spline.hpp>
#include "view.hpp"
#include "cv_util.hpp"
#include "data.hpp"
#include <QApplication>
#include <QTextEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include "util.h"

#include "view_control.hpp"
#include "operation_mode.hpp"
#include "mpc_factory.hpp"


using namespace std;
using namespace epi;

int main(int arg0, char** args)
{

    QApplication app(arg0, args);
    {
        shared_ptr<OperationModeProvider> modeProvider = make_shared<OperationModeProvider>();
        shared_ptr<Vehicle> car = make_shared<Vehicle>(make_unique<DynamicCarModel>());
        shared_ptr<System> sys = make_shared<System>(car, modeProvider);
        ViewControl viewBuilder(sys);

        viewBuilder.show();
    }
    cout<< "system reset \n";

    return app.exec();
}

