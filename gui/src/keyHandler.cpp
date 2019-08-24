//
// Created by sigi on 24.08.19.
//

#include "keyHandler.hpp"

void epi::handleKey(Key key, System* sys) {
    switch (key) {
        case Key::none:
            sys->move(0, 0);
            break;
        case Key::i:
            // accelerate (acc)
            sys->move(1, 0);
            break;
        case Key::k:
            // decelerate (dec)
            sys->move(-1, 0);
            break;

        case Key::u:
            // acc + move left
            sys->move(1, 1);
            break;
        case Key::j:
            // dec + move left
            sys->move(-1, 1);
            break;

        case Key::o:
            // acc + move right
            sys->move(1, -1);
            break;
        case Key::l:
            // dec + move right
            sys->move(-1, -1);
            break;

        case Key::h:
            // move left
            sys->move(0, 1);
            break;
        case Key::oe:
            // move right
            sys->move(0, -1);
            break;
    }

}

