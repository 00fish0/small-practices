//
// Created by SeparateWings on 9/27/23.
//

#ifndef KEYBOTGAME_PLAYER_H
#define KEYBOTGAME_PLAYER_H

#include <iostream>
#include "character.h"

namespace key_bot_game {

    class Player : public Character {
    private:
        const PlayerType type;
        int speed;

    public:
        Player(const Position &position, PlayerType type);

        PlayerType getType() const;

        int getSpeed() const;

        Position playerMove(MoveDirection moveDirection);
    };

} // namespace key_bot_game

#endif //KEYBOTGAME_PLAYER_H
