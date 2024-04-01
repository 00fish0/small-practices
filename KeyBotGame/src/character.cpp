//
// Created by SeparateWings on 9/27/23.
//

#include "character.h"

namespace key_bot_game {
    Character::Character(const key_bot_game::Position &position) : position(position) {}

    Position Character::setPosition(Position position) {
        this->position = position;
        return this->position;
    }

    void Character::move(int moveX, int moveY) {
        position.x += moveX;
        position.y += moveY;
    }

    Position Character::getPosition() const {
        return position;
    }
}