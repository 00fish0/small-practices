//
// Created by SeparateWings on 9/28/23.
//

#include "thief.h"

namespace key_bot_game {

    Thief::Thief(const Position &position) : Character(position) {}

    Position Thief::thiefMove(MoveDirection moveDirection) {
        switch (moveDirection) {
            case MoveDirection::UP:
                move(-1, 0);
                break;
            case MoveDirection::DOWN:
                move(1, 0);
                break;
            case MoveDirection::LEFT:
                move(0, -1);
                break;
            case MoveDirection::RIGHT:
                move(0, 1);
                break;
            default:
                break;
        }
        return getPosition();
    }

} // key_bot_game