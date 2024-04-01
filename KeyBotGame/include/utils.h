//
// Created by SeparateWings on 9/27/23.
//

#ifndef KEYBOTGAME_UTILS_H
#define KEYBOTGAME_UTILS_H

#include <random>
#include <termio.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include "data.h"

namespace key_bot_game {
    class Utils {
    private:
        static int getPositionAbsX(Position position1, Position position2);

        static int getPositionAbsY(Position position1, Position position2);

    public:
        // get the keyboard input
        static int scanKeyboard();
        // get a random number between 0 and max
        static int getRandom(int max);
        // get a random number between min and max
        static int getRandom(int min, int max);
        // get a random position in the map between (1, 1) and (size - 2, size - 2)
        static Position getRandomPosition(int size);
        // get the move direction from the keyboard input
        static MoveDirection getMoveDirection(int key);
        // check if the distance between two positions is less than or equal to distance
        static bool isPositionLessDistance(Position position1, Position position2, int distance);
        // get the opposite direction of the input direction
        static MoveDirection getOppositeDirection(MoveDirection direction);
    };
}

#endif //KEYBOTGAME_UTILS_H
