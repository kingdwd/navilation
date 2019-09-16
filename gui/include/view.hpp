//
// Created by sigi on 04.08.19.
//

#ifndef EPIPHANY_VIEW_HPP
#define EPIPHANY_VIEW_HPP

#include "view_model.hpp"

namespace epi{

    class View {

    public:
        View(std::shared_ptr<ViewModel> model);
    private:
        std::shared_ptr<ViewModel> _modelPtr;
    };

}
#endif //EPIPHANY_VIEW_HPP
