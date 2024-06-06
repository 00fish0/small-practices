#include "player.h"

namespace key_bot_game {

    Player::Player(const Position &position, PlayerType type) : Character(position), type(static_cast<PlayerType>(type)) {
        switch (this->type) {
            case PlayerType::WALK:
                this->speed = 1;
                break;
            case PlayerType::CAR:
                this->speed = 2;
                break;
            case PlayerType::NINJA:
                this->speed = 1;
                break;
            case PlayerType::ENDERMAN:
                this->speed = 1;
                break;
                
        }
    }

    PlayerType Player::getType() const {
        return this->type;
    }

    Position Player::playerMove(MoveDirection moveDirection) {
        switch (moveDirection) {
            case MoveDirection::UP:
                move(-speed, 0);
                break;
            case MoveDirection::DOWN:
                move(speed, 0);
                break;
            case MoveDirection::LEFT:
                move(0, -speed);
                break;
            case MoveDirection::RIGHT:
                move(0, speed);
                break;
            case MoveDirection::NONE:
                break;
        }
        return getPosition();
    }

    int Player::getSpeed() const {
        return this->speed;
    }


} // namespace key_bot_game