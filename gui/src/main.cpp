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






int main(int arg0, char** args)
{

    QApplication app(arg0, args);
    {
        shared_ptr<OperationModeProvider> modeProvider = make_shared<OperationModeProvider>();
        shared_ptr<Vehicle> car = make_shared<Vehicle>(make_unique<DynamicCarModel>());
        shared_ptr<System> sys = make_shared<System>(car, modeProvider);
        ViewBuilder viewBuilder(sys);

        viewBuilder.show();
    }
    cout<< "system reset \n";

    return app.exec();
}

