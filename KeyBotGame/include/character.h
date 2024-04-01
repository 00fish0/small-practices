//
// Created by SeparateWings on 9/27/23.
//

#ifndef KEYBOTGAME_CHARACTER_H
#define KEYBOTGAME_CHARACTER_H

#include "data.h"

namespace key_bot_game {
    class Character {
    private:
        Position position;

    protected:
        explicit Character(const Position &position);

        virtual void move(int moveX, int moveY) final;

    public:
        virtual Position setPosition(Position position) final;

        virtual Position getPosition() const final;
    };
}

#endif //KEYBOTGAME_CHARACTER_H
