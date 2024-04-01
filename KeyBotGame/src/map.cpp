#include "map.h"
#include "utils.h"

namespace key_bot_game {
    Map::Map(Position playerPosition, PlayerType playerType, Position thiefPosition, Position exitPosition)
            : player(playerPosition, playerType), thief(thiefPosition), exitPosition(exitPosition) {
        initMap();
    }

    PointType Map::getPointType(Position position) {
        return map[position.x][position.y].type;
    }

    void Map::initMap() {
        // init the map
        for (int i = 0; i < MAP_SIZE; i++) {
            for (int j = 0; j < MAP_SIZE; j++) {
                map[i][j].position = {i, j};
                map[i][j].type = PointType::EMPTY;
            }
        }

        // set the exit
        setPoint(exitPosition, PointType::EXIT);

        // set the thief
        setPoint(thief.getPosition(), PointType::THIEF);

        // set the player
        setPoint(player.getPosition(), PointType::PLAYER);

        // set the walls around the map
        for (int i = 0; i < MAP_SIZE; i++) {
            map[0][i].type = PointType::WALL;
            map[MAP_SIZE - 1][i].type = PointType::WALL;
            map[i][0].type = PointType::WALL;
            map[i][MAP_SIZE - 1].type = PointType::WALL;
        }

        // set the walls
        for (auto &i: map) {
            for (auto &j: i) {
                if (j.type == PointType::PLAYER || j.type == PointType::THIEF || j.type == PointType::EXIT) {
                    continue;
                }
                if (Utils::getRandom(100) % 100 < 5) {
                    j.type = PointType::WALL;
                }
            }
        }

        // Find the path
        FindPath();
    }

    void Map::drawMap() {
        std::cout << "\033c";
        for (auto &i: map) {
            for (auto &j: i) {
                if (j.type == PointType::EMPTY) {
                    std::cout << " ";
                } else if (j.type == PointType::WALL) {
                    std::cout << "#";
                } else if (j.type == PointType::PLAYER) {
                    std::cout << "P";
                } else if (j.type == PointType::THIEF) {
                    std::cout << "T";
                } else if (j.type == PointType::EXIT) {
                    std::cout << "E";
                }
            }
            std::cout << std::endl;
        }
    }

    void Map::setPoint(Position position, PointType pointType) {
        map[position.x][position.y].type = pointType;
    }

    void Map::FindPath() {
        std::list<Point> openList;
        std::list<Point> closeList;
        int exitX = exitPosition.x;
        int exitY = exitPosition.y;
        openList.push_back(map[exitX][exitY]);
        map[exitX][exitY].mark = 1;
        while (!openList.empty()) {
            Point currentPoint = openList.front();
            openList.pop_front();
            int x = currentPoint.position.x;
            int y = currentPoint.position.y;

            if (y >= 1 &&
                map[x][y - 1].type != PointType::WALL &&
                !map[x][y - 1].mark) {
                openList.push_back(map[x][y - 1]);
                map[x][y - 1].mark = 1;
                map[x][y - 1].thiefMoveDirection = MoveDirection::RIGHT;
            }
            if (y <= MAP_SIZE - 2 &&
                map[x][y + 1].type != PointType::WALL &&
                !map[x][y + 1].mark) {
                openList.push_back(map[x][y + 1]);
                map[x][y + 1].mark = 1;
                map[x][y + 1].thiefMoveDirection = MoveDirection::LEFT;
            }
            if (x >= 1 &&
                map[x - 1][y].type != PointType::WALL &&
                !map[x - 1][y].mark) {
                openList.push_back(map[x - 1][y]);
                map[x - 1][y].mark = 1;
                map[x - 1][y].thiefMoveDirection = MoveDirection::DOWN;
            }
            if (x <= MAP_SIZE - 2 &&
                map[x + 1][y].type != PointType::WALL &&
                !map[x + 1][y].mark) {
                openList.push_back(map[x + 1][y]);
                map[x + 1][y].mark = 1;
                map[x + 1][y].thiefMoveDirection = MoveDirection::UP;
            }

            closeList.push_back(currentPoint);
        }
    }

    void Map::thiefMove() {
        setPoint(thief.getPosition(), PointType::EMPTY);
        auto &movePoint = map[thief.getPosition().x][thief.getPosition().y];
        thief.thiefMove(movePoint.thiefMoveDirection);
        setPoint(thief.getPosition(), PointType::THIEF);
    }

    void Map::playerMove(MoveDirection moveDirection) {
        Position beforePlayerPosition = player.getPosition();
        player.playerMove(moveDirection);
        Position afterPlayerPosition = player.getPosition();
        if (getPointType(afterPlayerPosition) == PointType::WALL ||
            getPointType(afterPlayerPosition) == PointType::EXIT) {
            player.playerMove(Utils::getOppositeDirection(moveDirection));
        } else {
            setPoint(beforePlayerPosition, PointType::EMPTY);
            setPoint(afterPlayerPosition, PointType::PLAYER);
        }
    }

    bool Map::ifWin() const {
        if (Utils::isPositionLessDistance(thief.getPosition(), player.getPosition(), player.getSpeed())) {
            return true;
        } else {
            return false;
        }
    }

    bool Map::ifLose() const {
        if (Utils::isPositionLessDistance(thief.getPosition(), exitPosition, LOSE_DISTANCE)) {
            return true;
        } else {
            return false;
        }
    }

} // namespace key_bot_game