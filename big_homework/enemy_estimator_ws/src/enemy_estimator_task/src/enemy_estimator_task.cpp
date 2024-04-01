#include <enemy_estimator_task/enemy_estimator_task.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EnemyEstimator>("EnemyEstimator");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}