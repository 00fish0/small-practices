//
// Created by SeparateWings on 9/28/23.
//

#include "run.h"
#include "utils.h"

namespace key_bot_game {
    bool Run::run() {
        // welcome
        std::cout << "Welcome to KeyBotGame!" << std::endl;
        std::cout << "Please input the player type (0 for walk, 1 for car): ";
        int playerType;
        std::cin >> playerType;
        if (playerType != 0 && playerType != 1) {
            std::cout << "Wrong input! Try again!" << std::endl;
            return false;
        }
        std::cout << "Type wasd to move and q to exit the game, others to wait" << std::endl;
        std::cout << "Enter to start..." << std::endl;
        std::cin.get();

        // init map
        auto playerPosition = Utils::getRandomPosition(MAP_SIZE);
        auto thiefPosition = Utils::getRandomPosition(MAP_SIZE);
        auto exitPosition = Utils::getRandomPosition(MAP_SIZE);
        pMap = std::make_unique<Map>(playerPosition, PlayerType(playerType), thiefPosition, exitPosition);
        pMap->drawMap();

        // game loop
        while (true) {
            int key = Utils::scanKeyboard();
            // exit the game
            if (key == 'q') {
                std::cout << std::endl;
                std::cout << "Game exit." << std::endl;
                return true;
            }

            // move the player
            pMap->playerMove(Utils::getMoveDirection(key));

            // check if win
            if (checkWin()) {
                return true;
            }

            // move the thief
            pMap->thiefMove();

            // check if lose
            if (checkLose()) {
                return true;
            }

            // もう一度 check if win
            if (checkWin()) {
                return true;
            }

        }
    }

    bool Run::checkWin() {
        pMap->drawMap();
        if (pMap->ifWin()) {
            std::cout << std::endl;
            std::cout << "You catch the thief!" << std::endl;
            return true;
        }
        return false;
    }

    bool Run::checkLose() {
        pMap->drawMap();
        if (pMap->ifLose()) {
            std::cout << std::endl;
            std::cout << "Thief has escaped!" << std::endl;
            return true;
        }
        return false;
    }
} // key_bot_game