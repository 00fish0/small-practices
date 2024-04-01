//
// Created by SeparateWings on 9/28/23.
//

#ifndef KEYBOTGAME_RUN_H
#define KEYBOTGAME_RUN_H

#include "map.h"
#include <memory>

namespace key_bot_game {

    class Run {
    private:
        std::unique_ptr<Map> pMap;

        bool checkWin();
        bool checkLose();
    public:
        // return true if game exit normally
        bool run();
    };

} // key_bot_game

#endif //KEYBOTGAME_RUN_H
