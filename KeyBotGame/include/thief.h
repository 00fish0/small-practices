//
// Created by SeparateWings on 9/28/23.
//

#ifndef KEYBOTGAME_THIEF_H
#define KEYBOTGAME_THIEF_H

#include "character.h"

namespace key_bot_game {

    class Thief : public Character {
    public:
        explicit Thief(const Position &position);

        Position thiefMove(MoveDirection moveDirection);
    };

} // key_bot_game

#endif //KEYBOTGAME_THIEF_H
