//
// Created by sigi on 08.08.19.
//

#include <viewModel.hpp>
#include <iostream>
#include <mutex>
#include <Spline.hpp>

struct epi::ViewModel::Impl{
   std::unique_ptr<U::SubscriptionHandle> subHandle;
   spline::Points points;
   spline::Points spline;
   std::mutex _mutex;
   ViewBuilder* _vb;
   std::shared_ptr<epi::System> sys;
   Impl(ViewBuilder* vb, std::shared_ptr<epi::System> sys)
   : _vb{vb}
   , sys{sys}
   {};

   void init(){
       points.push_back(cv::Point(sys->vehicle->pose.get().x-1, -sys->vehicle->pose.get().y));
       points.push_back(cv::Point(sys->vehicle->pose.get().x, -sys->vehicle->pose.get().y));
   }
   void subscribe(MouseClickHandler* clickHandler){
       //std::unique_lock<mutex>(_mutex);
       if(!subHandle){
           subHandle = clickHandler->clickPos.onUpdate(this, &Impl::drawPoint);
           init();
       }
   }

   void unsubscribe(){
       //std::unique_lock<mutex>(_mutex);
       if(subHandle){
           subHandle->unsubscribe();
       }
   }

   void reset(){
       //std::unique_lock<mutex>(_mutex);
       unsubscribe();
       points.clear();
       spline.clear();
       subHandle.reset();
       _vb->resetSpline();
   }

   void start(){
       sys->move(spline);
   }

   void drawPoint(const cv::Point& point){
       //std::unique_lock<mutex>(_mutex);
       auto diffX = U::Math::sgn(point.x - points[points.size()-1].x);
       auto diffY = U::Math::sgn(point.y - points[points.size()-1].y);
       points.push_back(point);
       points.push_back(cv::Point(point.x+diffX, point.y+diffY));

       spline = spline::interpol(points);
       _vb->drawSpline(spline);
   }
};

epi::ViewModel::ViewModel(std::shared_ptr<epi::MouseClickHandler> mouseHandler
        , std::shared_ptr<epi::System> sys
        , ViewBuilder* vb)
    : _mouseHandler{mouseHandler}
    , pImpl{std::make_unique<Impl>(vb, sys)}
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
    pImpl->start();
    pImpl->unsubscribe();
}

void epi::ViewModel::switchOperationMode() {
    pImpl->sys->modeProvider->switchMode();

}

epi::ViewModel::~ViewModel() = default;
