 #include <task_interfaces/msg/input_msg.hpp>
 #include "task_interfaces/msg/output_msg.hpp"
 #include <Eigen/Dense>
 #include "rclcpp/rclcpp.hpp"
 #include "std_msgs/msg/string.hpp"
 #include <cmath>

class EnemyEstimator : public rclcpp::Node
{
public:
    using Vn = Eigen::Matrix<double, 9, 1>;
    using Vm = Eigen::Matrix<double, 4, 1>;
    using Mnn = Eigen::Matrix<double, 9, 9>;
    using Mmm = Eigen::Matrix<double, 4, 4>;
    using Mmn = Eigen::Matrix<double, 4, 9>;
    using Mnm = Eigen::Matrix<double, 9, 4>;

    inline static Vn init_P;
    static constexpr int n = 9;
    static constexpr int m = 4;

    // 滤波器参数
    inline static double sigma2_aa, sigma2_az, sigma2_aw;
    inline static double R_x, R_y, R_z, R_theta;

    inline static double r_0, z_0; // 给定的量
    inline static double delta_r_pri,delta_z_pri;
    double r_low,r_high,z_low,z_high;
    double t1;

    Vn Xe; 

    Mnn Pe;
    Mnn Q;
    Mmm R;
    Mnm K;
    
    EnemyEstimator(std::string name) : Node(name)
    {
        Pe = init_P.asDiagonal();
        command_publisher_ = this->create_publisher<task_interfaces::msg::OutputMsg>("command", 10);
        command_subscribe_ = this->create_subscription<task_interfaces::msg::InputMsg>("command", 10, std::bind(&EnemyEstimator::command_callback, this, std::placeholders::_1));
    }
private:
   void reset(const Vm &observe, double r)
    {
        Xe << observe[0] - r * cos(observe[3]), 0,
            observe[1] - r * sin(observe[3]), 0,
            observe[2], 0,
            observe[3], 0,
            r;
        Pe = init_P.asDiagonal();
    }

    Vn f(const Vn &x, double dT)
    {
        Vn result = Vn::Zero();
        result[0] = x[0] + x[1] * dT;
        result[1] = x[1];
        result[2] = x[2] + x[3] * dT;
        result[3] = x[3];
        result[4] = x[4] + x[5] * dT;
        result[5] = x[5];
        result[6] = x[6] + x[7] * dT;
        result[7] = x[7];
        result[8] = x[8];
        return result;
    }

    Vm h(const Vn &x)
    {
        Vm result = Vm::Zero();
        result[0] = x[0] + x[8] * cos(x[6]);
        result[1] = x[2] + x[8] * sin(x[6]);
        result[2] = x[4];
        result[3] = x[6];
        return result;
    }

    bool judge(const Vm &z)
    {
        constexpr double maxAngle = M_PI_2 * 0.8;
        if (z[3] - Xe[3] >= maxAngle)
            return true;
        else return false;
    }

    void update(const Vm &z, double dT)
    {
        static int changeTime=1;
        if (judge(z))
        {
            if (fabs(z[2] - z_low) < fabs(z[2] - z_high))
            {
                z_low=((double)changeTime/2*z_low+z[2])/((double)changeTime/2+1);
                r_low=((double)changeTime/2*r_low+Xe[8])/((double)changeTime/2+1);
                Xe[4]=z_low;
                Xe[8]=r_low;
            }
            else
            {
                z_high=((double)changeTime/2*z_high+z[2])/((double)changeTime/2+1);
                r_high=((double)changeTime/2*r_high+Xe[8])/((double)changeTime/2+1);
                Xe[4]=z_high;
                Xe[8]=r_high;
            }
            reset(z, Xe[8]);
            changeTime++;
        }

        Mnn F = Mnn::Zero();
        F << 1, dT, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, dT, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, dT, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, dT, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1;

        Mmn H = Mmn::Zero();
        H << 1, 0, 0, 0, 0, 0, -Xe[8] * sin(Xe[6]), 0, cos(Xe[6]),
            0, 0, 1, 0, 0, 0, Xe[8] * cos(Xe[6]), 0, sin(Xe[6]),
            0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0, 0;

        Vm R_diag;
        R_diag << R_x, R_y, R_z, R_theta;
        R = R_diag.asDiagonal();

        static double dTs[4];
        dTs[0] = dT;
        for (int i = 1; i < 4; i++)
            dTs[i] = dTs[i - 1] * dT;

        double q_dT4_a = dTs[3] / 4 * sigma2_aa;
        double q_dT3_a = dTs[2] / 2 * sigma2_aa;
        double q_dT2_a = dTs[1] * sigma2_aa;

        double q_dT4_w = dTs[3] / 4 * sigma2_aw;
        double q_dT3_w = dTs[2] / 2 * sigma2_aw;
        double q_dT2_w = dTs[1] * sigma2_aw;

        double q_dT4_z = dTs[3] / 4 * sigma2_az;
        double q_dT3_z = dTs[2] / 2 * sigma2_az;
        double q_dT2_z = dTs[1] * sigma2_az;

        double cos2Theta = cos(Xe[6]) * cos(Xe[6]);
        double sin2Theta = sin(Xe[6]) * sin(Xe[6]);
        double sinCosTheta = sin(Xe[6]) * cos(Xe[6]);

        Q << q_dT4_a * cos2Theta, q_dT3_a * cos2Theta, q_dT4_a * sinCosTheta, q_dT3_a * sinCosTheta, 0, 0, 0, 0, 0,
            q_dT3_a * cos2Theta, q_dT2_a * cos2Theta, q_dT3_a * sinCosTheta, q_dT2_a * sinCosTheta, 0, 0, 0, 0, 0,
            q_dT4_a * sinCosTheta, q_dT3_a * sinCosTheta, q_dT4_a * sin2Theta, q_dT3_a * sin2Theta, 0, 0, 0, 0, 0,
            q_dT3_a * sinCosTheta, q_dT2_a * sinCosTheta, q_dT3_a * sin2Theta, q_dT2_a * sin2Theta, 0, 0, 0, 0, 0,
            0, 0, 0, 0, q_dT4_z, q_dT3_z, 0, 0, 0,
            0, 0, 0, 0, q_dT3_z, q_dT2_z, 0, 0, 0,
            0, 0, 0, 0, 0, 0, q_dT4_w, q_dT3_w, 0,
            0, 0, 0, 0, 0, 0, q_dT3_w, q_dT2_w, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0;

        Vn X_pri = f(Xe, dT);
        Pe = F * Pe * F.transpose() + Q;
        K = Pe * H.transpose() * (H * Pe * H.transpose() + R).inverse();
        Vm Zp = h(X_pri);
        Xe = X_pri + K * (z - Zp);
        Pe = (Mnn::Identity() - K * H) * Pe;

        task_interfaces::msg::OutputMsg outputMsg;
        outputMsg.x=Xe[0];
        outputMsg.vx=Xe[1];
        outputMsg.y=Xe[2];
        outputMsg.vy=Xe[3];
        outputMsg.z=Xe[4];
        outputMsg.vz=Xe[5];
        outputMsg.theta=Xe[6];
        outputMsg.vtheta=Xe[7];

        outputMsg.timestamp=t1;
        outputMsg.r_offset=r_high-r_low;
        outputMsg.z_offset=z_high-z_low;

        command_publisher_->publish(outputMsg);
    }

    void command_callback(const task_interfaces::msg::InputMsg &msg)
    {
        static int count=0;
        Vm z;
        double t2,dT;
        z << msg.x, msg.y, msg.z, msg.yaw;

        if(count==0)
        {
            z_low = z_0;
            z_high = z_0+delta_z_pri;
            r_low = r_0;
            r_high = r_0+delta_r_pri;
            t1=msg.timestamp;

            reset(z, r_0);
        }
        else
        {
            t2 = msg.timestamp;
            dT = t2 - t1;
            t1=t2;
        }

        count++;
        update(z, dT);
    }

    rclcpp::Publisher<task_interfaces::msg::OutputMsg>::SharedPtr command_publisher_;
    rclcpp::Subscription<task_interfaces::msg::InputMsg>::SharedPtr command_subscribe_;
};
