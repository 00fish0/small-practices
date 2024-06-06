//
// Created by SeparateWings on 9/28/23.
//

#include "run.h"
#include "utils.h"

namespace key_bot_game {
    bool Run::run() {
        // welcome
        std::cout << "Welcome to KeyBotGame!" << std::endl;
        std::cout << "Input -1 for exit\nPlease input the player type (0 for walk, 1 for car, 2 for ninja(wall-passer), 3 for enderman): ";
        int playerType;
        std::cin >> playerType;
        // getchar();
        if (playerType!=-1 && playerType!= 0 && playerType != 1 && playerType != 2 && playerType != 3) {
            std::cout << "Wrong input! Try again!" << std::endl;
            return false;
        }
        if(playerType==-1){
            std::cout<< "Game exit."<<std::endl;
            return true;
        }

        std::cout << "Type wasd to move and q to exit the game, others to wait" << std::endl;
        std::cout << "Enter to start..." << std::endl;
        std::cin.get();

        // init map
        std::cout<< "Please input the size of the map.(less than 40)"<<std::endl;
        std::cin>>mapSize;
        if(mapSize<5 || mapSize>40){
            std::cout<< "Wrong input! Try again!" << std::endl;
            return false;
        }

        auto playerPosition = Utils::getRandomPosition(mapSize);
        auto thiefPosition = Utils::getRandomPosition(mapSize);
        auto exitPosition = Utils::getRandomPosition(mapSize);
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

            else if (key == 'f') {
                pMap->playerSkill();
            }

            // move the player
            else pMap->playerMove(Utils::getMoveDirection(key));

            // check if win
            if (checkWin()) {
                pMap->getScore();
                return false;
            }

            // move the thief
            pMap->thiefMove();

            // check if lose
            if (checkLose()) {
                pMap->getScore();
                return false;
            }

            // もう一度 check if win
            if (checkWin()) {
                pMap->getScore();
                return false;
            }

        }
    }

    bool Run::checkWin() {
        pMap->drawMap();
        if (pMap->ifWin()) {
            std::cout << std::endl;
            std::cout << "You catch the thief!" << std::endl;
            pMap->playerPlusScore();
            return true;
        }
        return false;
    }

    bool Run::checkLose() {
        pMap->drawMap();
        if (pMap->ifLose()) {
            std::cout << std::endl;
            std::cout << "Thief has escaped!" << std::endl;
            pMap->thiefPlusScore();
            return true;
        }
        return false;
    }
} // key_bot_game