#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/parameter.hpp>

#include <bits/stdc++.h>
#include <chrono>
#include <string>

#include "flight_patterns.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

namespace uosm
{
    namespace mavros
    {
        constexpr int PATTERNS_COUNT(5);
        constexpr float PUBLISH_RATE(20.0f); // publishing rate

        class OffboardControl : public rclcpp::Node
        {
        public:
            OffboardControl(std::string mavros_namespace) : Node("offboard_control_node")
            {
                // Params setup
                this->declare_parameter("flight_pattern", rclcpp::ParameterValue(0)); // Default to CIRCULAR
                rclcpp::Parameter flight_pattern_ = this->get_parameter("flight_pattern");
                const auto flight_pattern_int_ = flight_pattern_.as_int();

                if (flight_pattern_int_ < 0 || flight_pattern_int_ > PATTERNS_COUNT)
                {
                    RCLCPP_ERROR(this->get_logger(), "Invalid flight pattern %ld!", flight_pattern_int_);
                    return;
                }

                this->declare_parameter("max_iter", rclcpp::ParameterValue(2));
                this->declare_parameter("dt", rclcpp::ParameterValue(0.05f));
                this->declare_parameter("radius", rclcpp::ParameterValue(0.80f));
                this->declare_parameter("height", rclcpp::ParameterValue(1.00f));
                this->declare_parameter("speed", rclcpp::ParameterValue(0.30f));
                this->declare_parameter("min_speed", rclcpp::ParameterValue(0.05f));
                this->declare_parameter("offset_x", rclcpp::ParameterValue(0.00f));
                this->declare_parameter("offset_y", rclcpp::ParameterValue(0.00f));
                this->declare_parameter("offset_z", rclcpp::ParameterValue(0.00f));
                this->declare_parameter("offset_theta", rclcpp::ParameterValue(0.00f));
                this->declare_parameter("frequency", rclcpp::ParameterValue(0.00f));
                this->declare_parameter("ngram_vertices", rclcpp::ParameterValue(7));
                this->declare_parameter("ngram_step", rclcpp::ParameterValue(2));

                flight_params_ = {
                    this->get_parameter("dt").as_double(),
                    this->get_parameter("radius").as_double(),
                    this->get_parameter("height").as_double(),
                    this->get_parameter("speed").as_double(),
                    this->get_parameter("min_speed").as_double(),
                    this->get_parameter("offset_x").as_double(),
                    this->get_parameter("offset_y").as_double(),
                    this->get_parameter("offset_z").as_double(),
                    this->get_parameter("offset_theta").as_double(),
                    this->get_parameter("frequency").as_double(),
                    static_cast<int>(this->get_parameter("ngram_vertices").as_int()),
                    static_cast<int>(this->get_parameter("ngram_step").as_int()),
                    static_cast<int>(this->get_parameter("max_iter").as_int())};

                if (std::__gcd(flight_params_.ngram_vertices, flight_params_.ngram_step) != 1)
                {
                    RCLCPP_ERROR(this->get_logger(), "NGram_vertices and NGram_step must be co-prime!");
                    return;
                }

                pattern_ = static_cast<flight_pattern::PatternFactory::PatternType>(flight_pattern_int_);
                RCLCPP_INFO(this->get_logger(), "Flight Pattern: %d, Flight Height: %.2f", pattern_, flight_params_.height);

                // Publishers & Subscribers setup
                const auto qos_profile = rclcpp::QoS(10)
                                             .reliability(rclcpp::ReliabilityPolicy::Reliable)
                                             .history(rclcpp::HistoryPolicy::KeepLast);

                setpoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                    mavros_namespace + "setpoint_position/local", qos_profile);

                arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
                    mavros_namespace + "cmd/arming");

                set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
                    mavros_namespace + "set_mode");

                land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>(
                    mavros_namespace + "cmd/land");

                state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
                    mavros_namespace + "state", qos_profile,
                    [this](const mavros_msgs::msg::State::SharedPtr msg)
                    {
                        current_state_ = *msg;
                    });

                pose.pose.position.x = static_cast<float>(flight_params_.offset_x);
                pose.pose.position.y = static_cast<float>(flight_params_.offset_y);
                pose.pose.position.z = static_cast<float>(flight_params_.height);

                tf2::Quaternion q;
                q.setRPY(0, 0, 0);
                q.normalize();

                pose.pose.orientation = tf2::toMsg(q); // Convert tf2 quaternion to geometry_msgs

                is_init_ = true;
            }

            void arm();
            void disarm();
            void switch_to_offboard_mode();
            void publish_setpoint();
            void request_landing();

            bool is_init_ = false;
            flight_pattern::PatternParameters flight_params_;
            flight_pattern::PatternFactory::PatternType pattern_;

            mavros_msgs::msg::State current_state_;
            geometry_msgs::msg::PoseStamped pose;

            // Home position for return to land
            double home_x_ = 0.0;
            double home_y_ = 0.0;

        private:
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_publisher_;
            rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
            rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
            rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;

            void command_response_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future);
            void set_mode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future);
            void land_response_callback(rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future);
        };

        /**
         * @brief Send a command to switch to offboard mode
         */
        void OffboardControl::switch_to_offboard_mode()
        {
            RCLCPP_INFO(this->get_logger(), "requesting switch to Offboard mode");

            auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            request->custom_mode = "OFFBOARD";

            auto result = set_mode_client_->async_send_request(request,
                                                               std::bind(&OffboardControl::set_mode_response_callback, this, std::placeholders::_1));
        }

        /**
         * @brief Send a command to Arm the vehicle
         */
        void OffboardControl::arm()
        {
            RCLCPP_INFO(this->get_logger(), "requesting arm");

            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            request->value = true;

            auto result = arm_client_->async_send_request(request,
                                                          std::bind(&OffboardControl::command_response_callback, this, std::placeholders::_1));
        }

        /**
         * @brief Send a command to Disarm the vehicle
         */
        void OffboardControl::disarm()
        {
            RCLCPP_INFO(this->get_logger(), "requesting disarm");

            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            request->value = false;

            auto result = arm_client_->async_send_request(request,
                                                          std::bind(&OffboardControl::command_response_callback, this, std::placeholders::_1));
        }

        /**
         * @brief Publish setpoint position
         */
        void OffboardControl::publish_setpoint()
        {
            pose.header.stamp = this->get_clock()->now();
            pose.header.frame_id = "map";
            // RCLCPP_INFO(this->get_logger(), "Publishing setpoint x= %f y= %f z= %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            setpoint_publisher_->publish(pose);
        }

        void OffboardControl::request_landing()
        {
            RCLCPP_INFO(this->get_logger(), "requesting landing");

            auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            request->min_pitch = 0.0;
            request->yaw = 0.0;
            request->latitude = 0.0;  // 0 for local frame
            request->longitude = 0.0; // 0 for local frame
            request->altitude = 0.0;  // landing altitude

            auto result = land_client_->async_send_request(request,
                                                           std::bind(&OffboardControl::land_response_callback, this, std::placeholders::_1));
        }

        void OffboardControl::command_response_callback(
            rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                auto response = future.get();
                if (response->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Command accepted");
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Command rejected");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
            }
        }

        void OffboardControl::set_mode_response_callback(
            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                auto response = future.get();
                if (response->mode_sent)
                {
                    RCLCPP_INFO(this->get_logger(), "Mode change request sent");
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Mode change request rejected");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
            }
        }

        void OffboardControl::land_response_callback(
            rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready)
            {
                auto response = future.get();
                if (response->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Landing command accepted");
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Landing command rejected");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
            }
        }

    } // namespace mavros
} // namespace uosm

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    constexpr int wait_period_sec_ = 5;
    constexpr int hover_period_sec_ = 10;
    int connection_timeout_count_ = 0;

    enum STATE
    {
        WAITING_FOR_FCU = 0,
        DISARMED,
        OFFBOARD_ARMED,
        HOVERING,
        FLYING,
        LANDING
    } state_;

    state_ = STATE::WAITING_FOR_FCU;
    bool is_done_ = false;

    auto node = std::make_shared<uosm::mavros::OffboardControl>("/mavros/");
    if (node->is_init_)
    {
        rclcpp::Rate rate(uosm::mavros::PUBLISH_RATE);
        RCLCPP_INFO(node->get_logger(), "Waiting for FCU connection");

        // Wait for FCU connection
        while (rclcpp::ok() && !node->current_state_.connected)
        {
            rclcpp::spin_some(node);
            rate.sleep();
            connection_timeout_count_++;
            if (connection_timeout_count_ > 100)
            {
                RCLCPP_ERROR(node->get_logger(), "FCU connection timeout!");
                return 1;
            }
        }

        RCLCPP_INFO(node->get_logger(), "FCU connected");
        auto pattern = uosm::flight_pattern::PatternFactory::createPattern(node->pattern_, node->flight_params_);

        // Create flight pattern
        auto last_request = node->now();

        // Need to send setpoints consistently before switching to OFFBOARD
        for (int i = 0; i < 100; i++)
        {
            node->publish_setpoint();
            rclcpp::spin_some(node);
            rate.sleep();
        }

        state_ = STATE::DISARMED;

        while (rclcpp::ok())
        {
            node->publish_setpoint();
            rclcpp::spin_some(node);
            rate.sleep();

            switch (state_)
            {
            case STATE::DISARMED:
                RCLCPP_INFO_ONCE(node->get_logger(), "DISARMED");
                if (is_done_)
                {
                    RCLCPP_INFO(node->get_logger(), "Flight mission completed, Exiting!");
                    rclcpp::shutdown();
                    return 0;
                }
                state_ = STATE::OFFBOARD_ARMED;
                break;

            case STATE::OFFBOARD_ARMED:
                RCLCPP_INFO_ONCE(node->get_logger(), "OFFBOARD_ARMED");
                if (is_done_)
                {
                    state_ = STATE::DISARMED;
                }
                if ((node->now() - last_request).seconds() > wait_period_sec_)
                {
                    node->switch_to_offboard_mode();
                    node->arm();
                    last_request = node->now();
                }

                if (node->current_state_.mode == "OFFBOARD" &&
                    node->current_state_.armed)
                {
                    state_ = STATE::HOVERING;
                    last_request = node->now();
                }
                break;

            case STATE::HOVERING:
                RCLCPP_INFO_ONCE(node->get_logger(), "HOVERING");
                pattern->hover(node->pose);
                if ((node->now() - last_request).seconds() > hover_period_sec_)
                {
                    if (is_done_)
                    {
                        state_ = STATE::LANDING;
                    }
                    else
                    {
                        state_ = STATE::FLYING;
                    }
                }
                break;

            case STATE::FLYING:
                RCLCPP_INFO_ONCE(node->get_logger(), "FLYING");
                pattern->run(node->pose);
                if (pattern->is_done(node->flight_params_.max_iter))
                {
                    is_done_ = true;
                    last_request = node->now();
                    state_ = STATE::HOVERING;
                }
                break;

            case STATE::LANDING:
                RCLCPP_INFO_ONCE(node->get_logger(), "LANDING");
                if (node->current_state_.mode == "OFFBOARD")
                {
                    node->request_landing();
                    state_ = STATE::DISARMED;
                }
                break;

            default:
                RCLCPP_INFO(node->get_logger(), "STATE::UNKNOWN");
                break;
            }
        }
    }

    rclcpp::shutdown();
    return 0;
}