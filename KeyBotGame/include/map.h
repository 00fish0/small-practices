//
// Created by SeparateWings on 9/27/23.
//i

#ifndef KEYBOTGAME_MAP_H
#define KEYBOTGAME_MAP_H

#include <cstddef>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <list>
#include "data.h"
#include "player.h"
#include "thief.h"

namespace key_bot_game {
    class Map {

    private:
        Point map[MAP_MAX_SIZE][MAP_MAX_SIZE];
        Position exitPosition;
        Player player;
        Thief thief;
        
        static int playerScore;
        static int thiefScore;

        const int LOSE_DISTANCE = 1;

        void initMap();

        void FindPath();

        PointType getPointType(Position position);

        void setPoint(Position position, PointType pointType);

    public:
        Map(Position playerPosition, PlayerType playerType, Position thiefPosition, Position exitPosition);

        void drawMap();

        void thiefMove();

        void playerMove(MoveDirection moveDirection);

        void playerSkill();

        // return true if win
        bool ifWin() const;

        // return true if lose
        bool ifLose() const;

        void getScore();

        static void playerPlusScore();

        static void thiefPlusScore();

    };


} // namespace key_bot_game


#endif //KEYBOTGAME_MAP_H
