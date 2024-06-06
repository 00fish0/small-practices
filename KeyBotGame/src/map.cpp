#include "map.h"
#include "utils.h"
int key_bot_game::Map::playerScore=0;
int key_bot_game::Map::thiefScore=0;
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
        int mapsize;
        for (int i = 0; i < mapSize; i++) {
            for (int j = 0; j < mapSize; j++) {
                map[i][j].position = {i, j};
                map[i][j].type = PointType::EMPTY;
                map[i][j].mark = 0;
            }
        }

        // set the exit
        setPoint(exitPosition, PointType::EXIT);

        // set the thief
        setPoint(thief.getPosition(), PointType::THIEF);

        // set the player
        setPoint(player.getPosition(), PointType::PLAYER);

        // set the walls around the map
        for (int i = 0; i < mapSize; i++) {
            map[0][i].type = PointType::WALL;
            map[mapSize - 1][i].type = PointType::WALL;
            map[i][0].type = PointType::WALL;
            map[i][mapSize - 1].type = PointType::WALL;
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
        for (int i=0;i<mapSize;i++) {
            for (int j=0;j<mapSize;j++) {
                if (map[i][j].type == PointType::EMPTY) {
                    std::cout << " ";
                } else if (map[i][j].type == PointType::WALL) {
                    std::cout << "#";
                } else if (map[i][j].type == PointType::PLAYER || map[i][j].type == PointType::PLAYER_IN_WALL) {
                    std::cout << "P";
                } else if (map[i][j].type == PointType::THIEF) {
                    std::cout << "T";
                } else if (map[i][j].type == PointType::EXIT) {
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
            if (y <= mapSize - 2 &&
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
            if (x <= mapSize - 2 &&
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
        player.playerMove(moveDirection);//get player's afterPosition x,y
        Position afterPlayerPosition = player.getPosition();

        if ((getPointType(afterPlayerPosition) == PointType::WALL ||
            getPointType(afterPlayerPosition) == PointType::EXIT) &&
            player.getType() != PlayerType::NINJA)//cannot move
        {
            player.playerMove(Utils::getOppositeDirection(moveDirection));
        }
        else if(player.getType() == PlayerType::NINJA)
        {
            if(getPointType(afterPlayerPosition) == PointType::WALL &&
                getPointType(beforePlayerPosition) == PointType::PLAYER)
            {
                setPoint(beforePlayerPosition,PointType::EMPTY);
                setPoint(afterPlayerPosition,PointType::PLAYER_IN_WALL);
            }
            else if(getPointType(afterPlayerPosition) == PointType::EMPTY &&
                getPointType(beforePlayerPosition) == PointType::PLAYER)
            {
                setPoint(beforePlayerPosition,PointType::EMPTY);
                setPoint(afterPlayerPosition,PointType::PLAYER);
            }
            else if(getPointType(afterPlayerPosition) == PointType::WALL &&
                getPointType(beforePlayerPosition) == PointType::PLAYER_IN_WALL)
            {
                setPoint(beforePlayerPosition,PointType::WALL);
                setPoint(afterPlayerPosition,PointType::PLAYER_IN_WALL);
            }
            else if(getPointType(afterPlayerPosition) == PointType::EMPTY &&
                getPointType(beforePlayerPosition) == PointType::PLAYER_IN_WALL)
            {
                setPoint(beforePlayerPosition,PointType::WALL);
                setPoint(afterPlayerPosition,PointType::PLAYER);
            }
        }
        else// not ninja
        {
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

    void Map::getScore(){
        std::cout<< "****************"<<std::endl;
        std::cout<<"Your score: "<<playerScore<<std::endl;
        std::cout<<"Thief's score: "<<thiefScore<<std::endl;
        std::cout<< "****************"<<std::endl;

    }

    void Map::playerPlusScore(){
        playerScore+=1;
    }

    void Map::thiefPlusScore(){
        thiefScore+=1;
    }

    void Map::playerSkill(){
        if(player.getType() == PlayerType::ENDERMAN)
        {
            Position thiefPos = thief.getPosition();
            Position playerPos = player.getPosition();
            if(Utils::isPositionLessDistance(thiefPos,playerPos,10))
            {
                player.setPosition(thiefPos);
                thief.setPosition(playerPos);

                setPoint(thiefPos,PointType::PLAYER);
                setPoint(playerPos,PointType::THIEF);
            }
        }
        else std::cout<<"No skill"<<std::endl;
    }

} // namespace key_bot_game