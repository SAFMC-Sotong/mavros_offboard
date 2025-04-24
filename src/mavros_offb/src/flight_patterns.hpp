#pragma once
/**
 * @file FlightPattern.hpp
 * @brief Predefined flight patterns
 * @author UoSM-CIRG Janitor
 * @date 2024-05-24
 */

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

namespace uosm
{
    namespace flight_pattern
    {
        constexpr double TWO_PI = 2.0 * M_PI;
        struct PatternParameters
        {
            double dt = 0.0f;
            double radius = 0.0f;
            double height = 0.0f;
            double speed = 0.0f; // linear or angular velocity
            double min_speed = 0.0f;
            double offset_x = 0.0f;
            double offset_y = 0.0f;
            double offset_z = 0.0f;
            double offset_theta = 0.0f;
            double frequency = 0.0f;
            int ngram_vertices = 7;
            int ngram_step = 2;
            int max_iter = 2;
        };

        /**
         * @brief Abstract class for flight patterns
         *
         * This class is an abstract base class for flight patterns. It provides
         * a common interface for all flight patterns, including the basic
         * functionalities and parameters.
         */
        class Pattern
        {
        protected:
            PatternParameters params_; // initial params
            double theta_ = 0.0f;      // progression of the pattern
            int iteration_ = 0;        // number of iterations

        public:
            /**
             * @brief Run the flight pattern
             *
             * This function updates the pose of the drone according to the
             * current flight pattern.
             *
             * @param pose The pose of the drone
             *
             */
            virtual void run(geometry_msgs::msg::PoseStamped &pose) = 0;
            virtual ~Pattern() = default;

            /**
             * @brief Set the pose to hover at a certain position defined by the parameters
             *
             * @param pose The pose of the drone
             *
             */
            void hover(geometry_msgs::msg::PoseStamped &pose) const
            {
                pose.pose.position.x = params_.offset_x;
                pose.pose.position.y = params_.offset_y;
                pose.pose.position.z = params_.height;
                tf2::Quaternion quat;
                quat.setRPY(0, 0, 0);
                pose.pose.orientation = tf2::toMsg(quat);
            }

            /**
             * @brief Increase the iteration counter by 1 every revolution
             */
            void increase_iteration()
            {
                if (theta_ >= TWO_PI + params_.offset_theta)
                {
                    theta_ = params_.offset_theta;
                    ++iteration_;
                }
            }
            /**
             * @brief Check if the flight pattern is done
             *
             * @param max_iteration The maximum allowed iteration
             *
             * @return True if the number of iterations exceeds the maximum
             *         allowed iteration, false otherwise
             *
             */
            bool is_done(const int max_iteration) const
            {
                return iteration_ >= max_iteration;
            }
        }; /* class Pattern */

        class CircularPattern : public Pattern
        {
        public:
            CircularPattern(const PatternParameters &params)
            {
                params_ = params;
                theta_ = params_.offset_theta;
            }

            void run(geometry_msgs::msg::PoseStamped &pose) override
            {
                pose.pose.position.x = params_.radius * cos(theta_) + params_.offset_x;
                pose.pose.position.y = params_.radius * sin(theta_) + params_.offset_y;
                pose.pose.position.z = params_.height;
                printf("X: %f, Y: %f, Z: %f\n", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                double angle_towards_middle = atan2(params_.offset_y - pose.pose.position.y, params_.offset_x - pose.pose.position.x);
                tf2::Quaternion quat;
                quat.setRPY(0, 0, angle_towards_middle);
                pose.pose.orientation = tf2::toMsg(quat);
                theta_ += params_.speed * params_.dt; // angular velocity
                increase_iteration();
            }
        }; /* class CircularPattern */

        class SpiralPattern : public Pattern
        {
        private:
            double min_height_ = 0.0f;
            double min_radius_ = 0.0f;

        public:
            SpiralPattern(const PatternParameters &params)
            {
                params_ = params;
                min_height_ = params_.height / 2.0;
                min_radius_ = params_.radius * 0.1;
                theta_ = params_.offset_theta;
            }

            void run(geometry_msgs::msg::PoseStamped &pose) override
            {
                double completion_rate = ((static_cast<double>(iteration_) + theta_ / TWO_PI) / params_.max_iter);
                double current_radius = params_.radius - (params_.radius - min_radius_) * completion_rate;
                double current_height = params_.height - (params_.height - min_height_) * completion_rate;
                pose.pose.position.x = current_radius * cos(theta_) + params_.offset_x;
                pose.pose.position.y = current_radius * sin(theta_) + params_.offset_y;
                pose.pose.position.z = current_height;

                double angle_towards_middle = atan2(params_.offset_y - pose.pose.position.y, params_.offset_x - pose.pose.position.x);
                tf2::Quaternion quat;
                quat.setRPY(0, 0, angle_towards_middle);
                pose.pose.orientation = tf2::toMsg(quat);

                theta_ += params_.speed * params_.dt; // angular velocity
                increase_iteration();
            }
        }; /* class SpiralPattern */

        class CloudPattern : public Pattern
        {
        private:
            double initial_progress_ = 0.0f;

        public:
            CloudPattern(const PatternParameters &params)
            {
                params_ = params;
                theta_ = params_.offset_theta;
            }

            void run(geometry_msgs::msg::PoseStamped &pose) override
            {
                double noise_amplitude = params_.radius / 4.0;

                double x = params_.radius * sin(theta_) + noise_amplitude * sin(params_.frequency * theta_) + params_.offset_x;
                double y = params_.radius * cos(theta_) + noise_amplitude * cos(params_.frequency * theta_) + params_.offset_y;

                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = params_.height;

                double angle = atan2(y - params_.offset_y, x - params_.offset_x);
                tf2::Quaternion quat;
                quat.setRPY(0, 0, angle);
                pose.pose.orientation = tf2::toMsg(quat);

                double speed_factor = abs(sin(2 * params_.frequency * theta_));
                double curr_speed = params_.min_speed + (params_.speed - params_.min_speed) * speed_factor;

                if (initial_progress_ >= (params_.radius + noise_amplitude))
                    theta_ += curr_speed * params_.dt;
                else
                    initial_progress_ += params_.speed * params_.dt;

                increase_iteration();
            }
        }; /* class CloudPattern */

        class SinePattern : public Pattern
        {
        private:
            double time_ = 0;        // Accumulated time for sine wave calculation
            int period_counter_ = 0; // Counter for completed sine wave periods
            double progress_ = 0;    // Progress within a single sine wave period
            bool first_it_ = true;
            bool forward_ = true; // Direction of motion
            double period_length_;

        public:
            SinePattern(const PatternParameters &params)
            {
                params_ = params;
                period_length_ = TWO_PI / params_.frequency;
                theta_ = params_.offset_theta;
            }

            void run(geometry_msgs::msg::PoseStamped &pose)
            {
                // Calculate the progress step for each update
                double progress_step = params_.speed * params_.dt;
                if (first_it_)
                {
                    pose.pose.position.x = params_.offset_x;
                    first_it_ = false;
                }
                // Update the x position based on the direction
                if (forward_)
                    pose.pose.position.x += progress_step;
                else
                    pose.pose.position.x -= progress_step;

                // Calculate the z position using the sine wave
                pose.pose.position.y = params_.offset_y; // y is constant
                pose.pose.position.z = params_.height + params_.offset_z * sin(params_.frequency * time_);

                // Orientation: Maintain a fixed orientation
                tf2::Quaternion quat;
                quat.setRPY(0, 0, 0);
                pose.pose.orientation = tf2::toMsg(quat);

                // Update time
                time_ += params_.dt;
                progress_ += params_.dt; // Update progress based on speed

                // Check if one period has been completed
                if (progress_ >= period_length_)
                {
                    period_counter_++; // Increment period counter
                    progress_ = 0.0;   // Reset progress for the next period

                    // std::cout << "Periods completed: " << period_counter_ << std::endl;
                }

                // Check if two periods have been completed for direction reversal
                if (period_counter_ == 2)
                {
                    if (!forward_)
                        theta_ = TWO_PI;

                    forward_ = !forward_; // Reverse direction after completing two periods
                    period_counter_ = 0;  // Reset period counter
                    // std::cout << "Direction reversed" << std::endl;
                }
                increase_iteration();
            }
        }; /* class SinePattern */

        class NGramPattern : public Pattern
        {
        private:
            struct Point
            {
                double x;
                double y;
            };
            std::vector<Point> vertices_;
            std::vector<int> order_;
            int segment_ = -1;
            double progress_ = 0.0f;
            double max_distance_ = 0.0f;

            std::vector<Point> calculate_vertices(double radius, int n, double offset_x = 0.0, double offset_y = 0.0)
            {
                std::vector<Point> vertices;
                for (int i = 0; i < n; ++i)
                {
                    double angle = TWO_PI * i / n;
                    double x = radius * cos(angle) + offset_x;
                    double y = radius * sin(angle) + offset_y;
                    vertices.push_back({x, y});
                }
                return vertices;
            }

            std::vector<int> generate_order(int n, int k)
            {
                std::vector<int> order;
                int current_vertex = 0;
                std::vector<bool> visited(n, false);
                while (!visited[current_vertex])
                {
                    order.push_back(current_vertex);
                    visited[current_vertex] = true;
                    current_vertex = (current_vertex + k) % n;
                }
                return order;
            }

        public:
            NGramPattern(const PatternParameters &params)
            {
                params_ = params;
                vertices_ = calculate_vertices(params_.radius, params_.ngram_vertices, params_.offset_x, params_.offset_y);
                order_ = generate_order(params_.ngram_vertices, params_.ngram_step);
                theta_ = params_.offset_theta;
            }

            void run(geometry_msgs::msg::PoseStamped &pose) override
            {
                double start_x, start_y, end_x, end_y = 0.0f;
                int start_index = 0;
                int end_index = 0;

                if (segment_ == -1) // -1 representing path to the first vertex
                {
                    start_x = 0.0;
                    start_y = 0.0;
                    end_x = vertices_[0].x;
                    end_y = vertices_[0].y;
                }
                else
                {
                    start_index = order_[segment_ % order_.size()];
                    end_index = order_[(segment_ + 1) % order_.size()];

                    start_x = vertices_[start_index].x;
                    start_y = vertices_[start_index].y;
                    end_x = vertices_[end_index].x;
                    end_y = vertices_[end_index].y;
                }
                pose.pose.position.x = start_x + (end_x - start_x) * progress_ + params_.offset_x;
                pose.pose.position.y = start_y + (end_y - start_y) * progress_ + params_.offset_y;
                pose.pose.position.z = params_.height;

                // Orientation: Face the next vertex
                double angle_towards_next = atan2(end_y - pose.pose.position.y, end_x - pose.pose.position.x);
                tf2::Quaternion quat;
                quat.setRPY(0, 0, angle_towards_next);
                pose.pose.orientation = tf2::toMsg(quat);

                if (segment_ == -1 || segment_ == 0)
                    max_distance_ = sqrt(pow(end_x - start_x, 2) + pow(end_y - start_y, 2));
                double distance_to_next = sqrt(pow(end_x - pose.pose.position.x, 2) + pow(end_y - pose.pose.position.y, 2));

                // Adjust speed based on the distance to the next vertex
                double speed_factor = distance_to_next / max_distance_;
                double speedi = params_.min_speed + (params_.speed - params_.min_speed) * speed_factor;

                // Update trajectory state
                progress_ += speedi * params_.dt;

                if (progress_ >= 1.0)
                {
                    progress_ = 0.0;
                    if (segment_ >= 0)
                    {
                        theta_ += TWO_PI / (params_.ngram_vertices);
                        increase_iteration();
                    }
                    segment_++;
                }
            }
        }; /* class NGramPattern */

        class SquarePattern : public Pattern
        {
        private:
            std::vector<std::pair<double, double>> waypoints_;
            int current_segment_ = 0;
            double distance_along_segment_ = 0.0;

        public:
            SquarePattern(const PatternParameters &params)
            {
                params_ = params;
                distance_along_segment_ = 0.0;

                // Define full flight path (start at origin, trace square, return)
                waypoints_ = {
                    {0.0, 0.0},    // Start
                    {params.radius, params.radius},
                    {-params.radius, params.radius},   // Enter square
                    {-params.radius, -params.radius},
                    {params.radius, -params.radius},
                    {params.radius, params.radius},   // End square
                    {0.0, 0.0}     // Return
                };
            }

            void run(geometry_msgs::msg::PoseStamped &pose) override
            {
                if (current_segment_ >= static_cast<int>(waypoints_.size()) - 1)
                    return;  // Done

                auto [x0, y0] = waypoints_[current_segment_];
                auto [x1, y1] = waypoints_[current_segment_ + 1];

                double dx = x1 - x0;
                double dy = y1 - y0;
                double length = std::hypot(dx, dy);
                double step = params_.speed * params_.dt;

                distance_along_segment_ += step;

                if (distance_along_segment_ >= length)
                {
                    distance_along_segment_ -= length;
                    ++current_segment_;
                    if (current_segment_ == static_cast<int>(waypoints_.size()) - 1)
                        ++iteration_;  // one full pattern completed
                    if (current_segment_ >= static_cast<int>(waypoints_.size()) - 1)
                        return;
                    x0 = waypoints_[current_segment_].first;
                    y0 = waypoints_[current_segment_].second;
                    x1 = waypoints_[current_segment_ + 1].first;
                    y1 = waypoints_[current_segment_ + 1].second;
                    dx = x1 - x0;
                    dy = y1 - y0;
                    length = std::hypot(dx, dy);
                }

                double t = distance_along_segment_ / length;
                double x = (1 - t) * x0 + t * x1 + params_.offset_x;
                double y = (1 - t) * y0 + t * y1 + params_.offset_y;
                double z = params_.height;

                double yaw = atan2(dy, dx);
                tf2::Quaternion quat;
                quat.setRPY(0, 0, yaw);

                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = z;
                pose.pose.orientation = tf2::toMsg(quat);
            }
        }; /* class SquarePattern */


        /**
         * @brief Factory class for creating flight patterns.
         */
        class PatternFactory
        {
        public:
            enum PatternType
            {
                CIRCULAR = 0,
                SPIRAL,
                CLOUD,
                SINE,
                NGRAM,
                SQUARE
            };

            /**
             * @brief Creates a new instance of flight pattern based on given type and parameters.
             *
             * @param type Type of the flight pattern.
             * @param params Parameters for the flight pattern.
             *
             * @return Unique pointer to the created flight pattern.
             *
             * @throws std::invalid_argument when unknown pattern type is provided.
             */
            static std::unique_ptr<Pattern> createPattern(const PatternType type, const PatternParameters &params)
            {
                switch (type)
                {
                case CIRCULAR:
                    return std::make_unique<CircularPattern>(params);
                case SPIRAL:
                    return std::make_unique<SpiralPattern>(params);
                case CLOUD:
                    return std::make_unique<CloudPattern>(params);
                case SINE:
                    return std::make_unique<SinePattern>(params);
                case NGRAM:
                    return std::make_unique<NGramPattern>(params);
                case SQUARE:
                    return std::make_unique<SquarePattern>(params);
                default:
                    throw std::invalid_argument("Unknown pattern type");
                }
            }
        }; /* class PatternFactory */
    } /* namespace flight_pattern */
} /* namespace uosm */