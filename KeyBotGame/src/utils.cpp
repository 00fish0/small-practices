#include "utils.h"

namespace key_bot_game {
    int Utils::scanKeyboard() {
        int in;
        struct termios new_settings{};
        struct termios stored_settings{};
        tcgetattr(0, &stored_settings);
        new_settings = stored_settings;
        new_settings.c_lflag &= (~ICANON);
        new_settings.c_cc[VTIME] = 0;
        tcgetattr(0, &stored_settings);
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(0, TCSANOW, &new_settings);

        in = std::cin.get();

        tcsetattr(0, TCSANOW, &stored_settings);
        return in;
    }

    int Utils::getRandom(int min, int max) {
        static std::random_device rd;
        static std::default_random_engine eng(rd());
        std::uniform_int_distribution<int> distr(min, max);
        return distr(eng);
    }

    Position Utils::getRandomPosition(int size) {
        return Position(getRandom(1, size - 2), getRandom(1, size - 2));
    }

    MoveDirection Utils::getMoveDirection(int key) {
        switch (key) {
            case 'w':
                return MoveDirection::UP;
            case 's':
                return MoveDirection::DOWN;
            case 'a':
                return MoveDirection::LEFT;
            case 'd':
                return MoveDirection::RIGHT;
            default:
                return MoveDirection::NONE;
        }
    }

    int Utils::getPositionAbsX(Position position1, Position position2) {
        return std::abs(position1.x - position2.x);
    }

    int Utils::getPositionAbsY(Position position1, Position position2) {
        return std::abs(position1.y - position2.y);
    }

    bool Utils::isPositionLessDistance(Position position1, Position position2, int distance) {
        return getPositionAbsX(position1, position2) <= distance &&
               getPositionAbsY(position1, position2) <= distance;
    }

    MoveDirection Utils::getOppositeDirection(MoveDirection direction) {
        switch (direction) {
            case MoveDirection::UP:
                return MoveDirection::DOWN;
            case MoveDirection::DOWN:
                return MoveDirection::UP;
            case MoveDirection::LEFT:
                return MoveDirection::RIGHT;
            case MoveDirection::RIGHT:
                return MoveDirection::LEFT;
            default:
                return MoveDirection::NONE;
        }
    }

    int Utils::getRandom(int max) {
        return getRandom(0, max);
    }
} // namespace key_bot_game