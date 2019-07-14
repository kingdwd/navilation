#include <iostream>
#include <string>
#include <opencv2/core.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <thread>
#include <chrono>
#include "UU.h"
#include "data.h"
#include "U.h"
#include "ViewBuilder.cpp"

using namespace std;
using Clock = std::chrono::high_resolution_clock ;
using namespace cv;
using namespace std::literals;

auto stepSize = 0.01s;
auto car = std::make_shared<epi::Vehicle>(std::make_unique<epi::DynamicModel>(epi::System(epi::State{0,0,0,0,0}, stepSize.count())));

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
bool waitUntilEscape(){
    Clock::time_point t1 = Clock::now();
    int key = waitKey(5);
    cout<< "pressed key: " << key << "\n";

    switch (key){
        case Key::none: car->drive(0,0); break;
        case Key::i: car->drive(1,0); break;
        case Key::k: car->drive(-1,0); break;

        case Key::u: car->drive(1,1); break;
        case Key::j: car->drive(-1,1); break;

        case Key::o: car->drive(1,-1); break;
        case Key::l: car->drive(-1,-1); break;

        case Key::h: car->drive(0,1); break;
        case Key::oe: car->drive(0,-1); break;
    }
    Clock::time_point t2 = Clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 );

    std::this_thread::sleep_for(stepSize - duration);
    //cout <<"\n millis: state " << duration << "\n";

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

