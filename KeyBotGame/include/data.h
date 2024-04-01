//
// Created by SeparateWings on 9/28/23.
//

#ifndef KEYBOTGAME_DATA_H
#define KEYBOTGAME_DATA_H

namespace key_bot_game {
    constexpr int MAP_SIZE = 20;

    enum class PointType {
        EMPTY = 0,
        WALL = 1,
        PLAYER = 2,
        THIEF = 3,
        EXIT = 4
    };

    enum class MoveDirection {
        UP = 0,
        DOWN = 1,
        LEFT = 2,
        RIGHT = 3,
        NONE = 4
    };

    enum class PlayerType {
        WALK = 0,
        CAR = 1,
    };

    struct Position {
        int x;
        int y;
    };

    struct Point {
        Position position;
        PointType type;

        // for the path finding
        MoveDirection thiefMoveDirection;
        int mark;
    };
}

#endif //KEYBOTGAME_DATA_H
