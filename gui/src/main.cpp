#include <iostream>
#include <string>
#include <opencv2/core.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "UU.h"
#include "data.h"
#include "U.h"
#include "ViewBuilder.cpp"

using namespace std;
using namespace cv;

auto car = std::make_shared<epi::Vehicle>(std::make_unique<epi::SimpleController>());

enum Key{
    j = 106,
    k = 107,
    l = 108,
    u = 117,
    i = 105,
    o = 111
};
bool waitUntilEscape(){
    int key = waitKey(0);
    cout<< "pressed key: " << key << "\n";

    switch (key){
        case Key::i: car->drive(1,0); break;
        case Key::k: car->drive(-1,0); break;

        case Key::u: car->drive(1,1); break;
        case Key::j: car->drive(-1,1); break;

        case Key::o: car->drive(1,-1); break;
        case Key::l: car->drive(-1,-1); break;
    }
    waitKey(15);

    return key != 27;
}

int main(int argc, char** args)
{
    ViewBuilder view(car);

    view.show();
    //createTrackbar("hi", "Display window", &pos, 100, show);
    while(waitUntilEscape()){} // Wait for a keystroke in the window
    return 0;

}

