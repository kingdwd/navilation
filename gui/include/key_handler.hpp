//
// Created by sigi on 24.08.19.
//

#ifndef EPIPHANY_KEYHANDLER_HPP
#define EPIPHANY_KEYHANDLER_HPP

#include <system.hpp>

namespace epi{
    enum Key{
        none = -1,
        esc = 27,
        j = 106,
        k = 107,
        l = 108,
        u = 117,
        i = 105,
        o = 111,
        h = 104,
        oe = 246
    };

    void handleKey(Key key, System* sys);

}
#endif //EPIPHANY_KEYHANDLER_HPP
