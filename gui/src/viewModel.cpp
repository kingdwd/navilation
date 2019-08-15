//
// Created by sigi on 08.08.19.
//

#include <viewModel.hpp>
#include <iostream>
#include <mutex>
#include <Spline.hpp>

struct epi::ViewModel::Impl{
   std::unique_ptr<U::SubscriptionHandle> subHandle;
   epi::spline::Points points;
   std::mutex _mutex;
   ViewBuilder* _vb;
   std::shared_ptr<epi::Vehicle> car;
   Impl(ViewBuilder* vb, std::shared_ptr<epi::Vehicle> car)
   : _vb{vb}
   , car{car}
   {};

   void init(){
       points.push_back(cv::Point(car->pose.get().x-1, -car->pose.get().y));
       points.push_back(cv::Point(car->pose.get().x, -car->pose.get().y));
   }
   void subscribe(MouseClickHandler* clickHandler){
       std::unique_lock<mutex>(_mutex);
       if(!subHandle){
           subHandle = clickHandler->clickPos.onUpdate(this, &Impl::drawPoint);
           init();
       }
   }

   void unsubscribe(){
       std::unique_lock<mutex>(_mutex);
       if(subHandle){
           subHandle->unsubscribe();
       }
   }

   void reset(){
       std::unique_lock<mutex>(_mutex);
       unsubscribe();
       points.clear();
       subHandle.reset();
       _vb->resetSpline();
   }

   void drawPoint(const cv::Point& point){
       std::unique_lock<mutex>(_mutex);
       auto diffX = U::Math::sgn(point.x - points[points.size()-1].x);
       auto diffY = U::Math::sgn(point.y - points[points.size()-1].y);
       points.push_back(point);
       points.push_back(cv::Point(point.x+diffX, point.y+diffY));

       spline::Points spline = spline::interpol(points);
       _vb->drawSpline(spline);
   }
};

epi::ViewModel::ViewModel(std::shared_ptr<epi::MouseClickHandler> mouseHandler
        , std::shared_ptr<epi::Vehicle> car
        , ViewBuilder* vb)
    : _mouseHandler{mouseHandler}
    , pImpl{std::make_unique<Impl>(vb, car)}
{}

void epi::ViewModel::defRoute() {
    std::cout<<"def Route pressed \n";
    pImpl->subscribe(_mouseHandler.get());
}

void epi::ViewModel::resetRoute() {
    std::cout<<"reset Route pressed \n";
    pImpl->reset();
}

void epi::ViewModel::startRoute() {
    std::cout<<"start Route pressed \n";

    pImpl->unsubscribe();
}

epi::ViewModel::~ViewModel() = default;
