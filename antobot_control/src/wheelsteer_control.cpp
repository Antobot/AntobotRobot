#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <functional>
#include <array>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "antobot_platform_msgs/msg/float32_array.hpp"   // Wheel velocity command

// Common control base
#include "antobot_control/anto_control_base.hpp"

using std::placeholders::_1;

// ======================================================================
// Configuration parameters
// ======================================================================
namespace
{
	// Wheelbase (front to rear axle distance) [m]
	constexpr double WHEEL_BASE_M   = 1.156;

	// Track width (left to right wheel distance) [m]
	constexpr double TRACK_WIDTH_M  = 1.1;

	// Wheel radius [m]
	constexpr double WHEEL_RADIUS_M = 0.203;

	// Base angle for SPOTTURN (deg), other wheels derived from this
	constexpr double SPOTTURN_BASE_DEG = 46.42; // RF1, LR3

	// Base angle for LOCK (deg), other wheels derived from this
	constexpr double LOCK_BASE_DEG     = 45.0; // LF2, RR4

	// Default steering angle for CRAB when no clear direction or on mode entry [deg]
	constexpr double CRAB_INIT_ANGLE_DEG = 0.0;

	// Default steering angle for COUNTERPHASE when no clear direction or on mode entry [deg]
	constexpr double COUNTERPHASE_INIT_ANGLE_DEG = 0.0;

	// Small thresholds for command checking
	constexpr double EPS_V = 1e-3;
	constexpr double EPS_W = 1e-3;

	// Threshold for deciding if a new steering target is significantly different [deg]
	constexpr double STEER_RETARGET_TOL_DEG = 2.0;

	constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;

	// Mechanical limit for multi-turn steering (logical angle) [deg]
	constexpr double STEER_MIN_DEG = -90.0;
	constexpr double STEER_MAX_DEG = +90.0;

	// control freq [Hz]
	constexpr double CONTROL_FREQUENCY_HZ = 30.0;

	// Tolerance for checking turn position [deg]
	constexpr double TRANSITION_TOL_DEG = 2.0;

	// steady to active
	constexpr bool   ENABLE_STEADY_HOLD = false;

	// steady to active time [s]
	constexpr double STEADY_HOLD_SEC    = 0.5;

	constexpr double LONGITUDINAL_DEADBAND = 1e-4;

	// Physical wheel indexing mapping

	// Physical wheel index
	enum PhysicalWheelIndex {
		PHYS_LF = 0,
		PHYS_LR = 1,
		PHYS_RF = 2,
		PHYS_RR = 3
	};

	// Physical wheel positions in body frame: x forward, y left
	// Indexed by PhysicalWheelIndex
	constexpr std::array<std::pair<double, double>, 4> PHYSICAL_WHEEL_POS = {{
		{ +WHEEL_BASE_M * 0.5, +TRACK_WIDTH_M * 0.5 },  // PHYS_LF
		{ -WHEEL_BASE_M * 0.5, +TRACK_WIDTH_M * 0.5 },  // PHYS_LR
		{ +WHEEL_BASE_M * 0.5, -TRACK_WIDTH_M * 0.5 },  // PHYS_RF
		{ -WHEEL_BASE_M * 0.5, -TRACK_WIDTH_M * 0.5 }   // PHYS_RR
	}};


	// Steering array order (logical_pos_deg_ / drive_inverted_ / target_logical_angles_ etc.)
	// "index in steering arrays" -> "physical wheel index"
	// Default: 0=RF, 1=LF, 2=LR, 3=RR (your current steering convention)
	constexpr std::array<int, 4> STEER_INDEX_TO_PHYS = {
		PHYS_RF, // steering index 0
		PHYS_LF, // steering index 1
		PHYS_LR, // steering index 2
		PHYS_RR  // steering index 3
	};

	// Drive / bridge array order (wheel velocity command array)
	// "index in /antobridge/wheel_vel_cmd" -> "physical wheel index"
	// Default: 0=LF, 1=LR, 2=RF, 3=RR (bridge order)
	constexpr std::array<int, 4> DRIVE_INDEX_TO_PHYS = {
		PHYS_LF, // drive index 0
		PHYS_LR, // drive index 1
		PHYS_RF, // drive index 2
		PHYS_RR  // drive index 3
	};
}

// ======================================================================

class WheelSteerControl
	: public rclcpp::Node
	, public antobot_control::AntoControlBase
{
public:
	// four control mode
	enum ControlMode {
		CRAB = 0,
		COUNTERPHASE = 1,
		SPOTTURN = 2,     // defined angles
		LOCK = 3          // defined angles
	};

	// Steering high-level state
	enum SteerState {
		TRANSITIONING = 0,  // steering is moving to target, wheel speeds forced to zero
		ACTIVE        = 1   // steering is aligned, wheel speeds allowed
	};

	WheelSteerControl()
		: Node("wheelsteer_control")
	{
		wheel_count_ = 4;
		enabled_ = {true, true, true, true};
		zero_offset_deg_.assign(wheel_count_, 0.0);
		raw_pos_deg_.assign(wheel_count_, 0.0);
		logical_pos_deg_.assign(wheel_count_, 0.0);
		drive_inverted_.assign(wheel_count_, false);

		// Build reverse mapping: physical wheel -> steering index
		for (int steer_idx = 0; steer_idx < wheel_count_; ++steer_idx) {
			const int phys = STEER_INDEX_TO_PHYS[steer_idx];
			if (phys >= 0 && phys < wheel_count_) {
				phys_to_steer_[phys] = steer_idx;
			}
		}

		// Lock as initial mode
		current_mode_ = LOCK;

		// Initialize steering state machine
		steer_state_ = TRANSITIONING;
		transition_tol_deg_ = TRANSITION_TOL_DEG;
		target_logical_angles_.assign(wheel_count_, 0.0);
		final_logical_targets_.assign(wheel_count_, 0.0);
		has_target_ = false;

		// Debounce: require ACTIVE to hold this long before allowing a new steering target
		steady_hold_sec_   = STEADY_HOLD_SEC;
		enable_steady_hold_ = ENABLE_STEADY_HOLD;

		last_active_time_ = this->get_clock()->now();

		// Initialize common control parameters
		antobot_control::ControlParams params;
		params.wheel_count          = static_cast<std::size_t>(wheel_count_);
		params.track_width          = TRACK_WIDTH_M;
		params.wheel_radius         = WHEEL_RADIUS_M;
		params.control_frequency_hz = CONTROL_FREQUENCY_HZ;
		params.velocity_timeout_sec = 0.1;
		params.max_linear           = 0.2;
		params.min_linear           = -0.2;
		params.max_angular          = 0.2;
		params.min_angular          = -0.2;
		params.max_linear_accel     = 0.1;
		params.max_linear_decel     = -3.0;
		params.max_angular_accel    = 0.1;
		params.max_angular_decel    = -3.0;
		params.enable_smoothing     = true;
		params.enable_timeout       = true;
		params.enable_odom          = false;   // odom not used here
		setParams(params);

		RCLCPP_INFO(
			get_logger(),
			"WheelSteerControl started. Wheels: %d. Default Mode: %d",
			wheel_count_, current_mode_);

		// Raw feedback from bridge (multi-turn in degrees)
		raw_pos_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
			"/antobot/control/wheelsteer/real_pos_raw",
			50,
			std::bind(&WheelSteerControl::onRawPos, this, std::placeholders::_1));

		// steering command (manual override, not required in normal operation)
		cmd_pos_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
			"/antobot/control/wheelsteer/cmd_pos",
			50,
			std::bind(&WheelSteerControl::onCmdPos, this, std::placeholders::_1));

		// Set-zero command
		set_zero_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
			"/antobot/control/wheelsteer/set_zero",
			10,
			std::bind(&WheelSteerControl::onSetZero, this, std::placeholders::_1));

		// Mode command
		mode_sub_ = create_subscription<std_msgs::msg::Int32>(
			"/antobot/control/wheelsteer/mode",
			10,
			std::bind(&WheelSteerControl::onModeCmd, this, std::placeholders::_1));

		// Twist command (vx, vy, omega) for CRAB / COUNTERPHASE model
		twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
			"/antobot/control/wheelsteer/cmd_vel",
			50,
			std::bind(&WheelSteerControl::onTwistCmd, this, std::placeholders::_1));

		// Publishers
		cmd_pos_raw_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
			"/antobot/control/wheelsteer/cmd_pos_raw",
			50);

		real_pos_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
			"/antobot/control/wheelsteer/real_pos",
			50);

		// Wheel velocity command publisher
		pub_wheel_vel_cmd_ = this->create_publisher<antobot_platform_msgs::msg::Float32Array>(
			"/antobridge/wheel_vel_cmd",
			10);

		{
			// Initial steering command for the default mode
			std::vector<double> init_target(wheel_count_, 0.0);
			computeTargetAnglesForCurrentMode(
				init_target,
				/*use_preset_for_travel_modes=*/true);
			publishSteeringTargets(init_target);
			RCLCPP_INFO(
				get_logger(),
				"Initial steering command published for mode %d",
				static_cast<int>(current_mode_));
		}

		// ===================================
		// Periodic control loop timer for wheel speed computation
		// ===================================
		const double dt = 1.0 / params.control_frequency_hz;
		control_timer_ = this->create_wall_timer(
			std::chrono::duration<double>(dt),
			[this]()
			{
				this->wheel_speed_compute();
			});
	}

private:
	static double wrap360(double deg)
	{
		double x = std::fmod(deg, 360.0);
		if (x < 0.0) {
			x += 360.0;
		}
		return x;
	}

	static double clamp(double v, double lo, double hi)
	{
		return std::max(lo, std::min(hi, v));
	}

	// Mechanical limit in multi-turn mode:
	// - If allow_flip == false: clamp directly to [-90, +90].
	// - If allow_flip == true (CRAB/COUNTERPHASE): if angle is outside [-90,90],
	//   add/subtract 180deg to move it inside and mark invert_flag=true so that
	//   wheel longitudinal velocity will be inverted later.
	double apply_mech_limit(double desired_deg, bool &invert_flag, bool allow_flip) const
	{
		invert_flag = false;

		// Normalize to [-180,180] for easier reasoning
		double a = desired_deg;
		while (a > 180.0) a -= 360.0;
		while (a < -180.0) a += 360.0;

		if (allow_flip) {
			if (a > STEER_MAX_DEG) {
				// Example: +135deg -> -45deg with inverted wheel speed
				a -= 180.0;
				//invert_flag = true;
			} else if (a < STEER_MIN_DEG) {
				// Example: -135deg -> +45deg with inverted wheel speed
				a += 180.0;
				//invert_flag = true;
			}
		}

		// Final clamp in case we are still slightly out of range
		a = clamp(a, STEER_MIN_DEG, STEER_MAX_DEG);
		return a;
	}

	// Compute steering target angles for the current mode using the internal state.
	// All angles here are logical angles (deg) around mechanical zero, multi-turn aware,
	// but we keep them inside [-90, +90] via apply_mech_limit() when sending to the bridge.
	void computeTargetAnglesForCurrentMode(std::vector<double> & target_logical_angles,
										   bool use_preset_for_travel_modes)
	{
		target_logical_angles.assign(wheel_count_, 0.0);

		const auto & st  = getState();
		// Use raw command (before smoothing) for steering geometry
		const auto & raw = st.raw_cmd;
		const double vx  = raw.linear;    // raw vx
		const double vy  = raw.linear_y;  // joystick: right+, left-; here we use left+ convention
		double w         = raw.angular;   // raw omega

		switch (current_mode_) {
			case CRAB:
			{
				// CRAB: all wheels aligned with translational velocity (vx, vy)
				if (use_preset_for_travel_modes) {
					const double angle_deg = CRAB_INIT_ANGLE_DEG;
					for (int i = 0; i < wheel_count_; ++i) {
						target_logical_angles[i] = angle_deg;
					}
				} else {
					const double speed = std::hypot(vx, vy);

					RCLCPP_INFO(
						get_logger(),
						"CRAB mode: vx=%.3f, vy=%.3f, speed=%.6f",
						vx, vy, speed);

					if (speed < EPS_V) {
						// If no clear direction, use preset CRAB angle
						const double angle_deg = CRAB_INIT_ANGLE_DEG;
						for (int i = 0; i < wheel_count_; ++i) {
							target_logical_angles[i] = angle_deg;
						}
					} else {
						const double angle_deg = std::atan2(vy, vx) * RAD_TO_DEG; // [-180,180]
						for (int i = 0; i < wheel_count_; ++i) {
							target_logical_angles[i] = angle_deg;
						}
					}
				}
				break;
			}

			case COUNTERPHASE:
			{
				// COUNTERPHASE: wheels aligned with combined translational
				// and rotational velocity:
				//   v_i = [vx, vy] + omega × r_i
				// Here we use:
				//   vix = vx - omega * y_i
				//   viy = vy + omega * x_i

				if (use_preset_for_travel_modes) {
					// Use preset COUNTERPHASE angle when entering mode
					const double angle_deg = COUNTERPHASE_INIT_ANGLE_DEG;
					for (int i = 0; i < wheel_count_; ++i) {
						target_logical_angles[i] = angle_deg;
					}
				} else {
					if (std::fabs(vx) < EPS_V && std::fabs(vy) < EPS_V && std::fabs(w) < EPS_W) {
						const double angle_deg = COUNTERPHASE_INIT_ANGLE_DEG;
						for (int i = 0; i < wheel_count_; ++i) {
							target_logical_angles[i] = angle_deg;
						}
					} else {
						for (int steer_idx = 0; steer_idx < wheel_count_; ++steer_idx) {
							// Map steering index to physical wheel
							const int phys = STEER_INDEX_TO_PHYS[steer_idx];
							const double x_i = PHYSICAL_WHEEL_POS[phys].first;
							const double y_i = PHYSICAL_WHEEL_POS[phys].second;

							const double vix = vx - w * y_i;
							const double viy = vy + w * x_i;

							if (std::fabs(vix) < EPS_V && std::fabs(viy) < EPS_V) {
								// keep current logical if very small
								target_logical_angles[steer_idx] = logical_pos_deg_[steer_idx];
							} else {
								const double angle_deg = std::atan2(viy, vix) * RAD_TO_DEG; // [-180,180]
								target_logical_angles[steer_idx] = angle_deg;
							}
						}
					}
				}
				break;
			}

			case SPOTTURN:
			{
				// SPOTTURN fixed geometry around zero in multi-turn coordinates.
				// Use symmetric +/- SPOTTURN_BASE_DEG within [-90,90].
				if (wheel_count_ >= 4) {
					const double a = SPOTTURN_BASE_DEG;

					// Define angles in physical wheel space
					std::array<double, 4> phys_angle_deg{};
					// Example: RF/LR = +a, LF/RR = -a
					phys_angle_deg[PHYS_RF] = +a;
					phys_angle_deg[PHYS_LR] = +a;
					phys_angle_deg[PHYS_LF] = -a;
					phys_angle_deg[PHYS_RR] = -a;

					// Map physical angles to steering array indices
					for (int steer_idx = 0; steer_idx < wheel_count_; ++steer_idx) {
						const int phys = STEER_INDEX_TO_PHYS[steer_idx];
						target_logical_angles[steer_idx] = phys_angle_deg[phys];
					}
				}
				break;
			}

			case LOCK:
			{
				// LOCK fixed geometry around zero in multi-turn coordinates.
				if (wheel_count_ >= 4) {
					const double a = LOCK_BASE_DEG;

					// Define angles in physical wheel space
					std::array<double, 4> phys_angle_deg{};
					// Example: LF/RR = +a, RF/LR = -a
					phys_angle_deg[PHYS_LF] = +a;
					phys_angle_deg[PHYS_RR] = +a;
					phys_angle_deg[PHYS_RF] = -a;
					phys_angle_deg[PHYS_LR] = -a;

					// Map physical angles to steering array indices
					for (int steer_idx = 0; steer_idx < wheel_count_; ++steer_idx) {
						const int phys = STEER_INDEX_TO_PHYS[steer_idx];
						target_logical_angles[steer_idx] = phys_angle_deg[phys];
					}
				}
				break;
			}

			default:
			{
				// Default: keep current logical angles
				for (int i = 0; i < wheel_count_; ++i) {
					target_logical_angles[i] = logical_pos_deg_[i];
				}
				break;
			}
		}
	}

	// Check whether a new target set is significantly different from the current one.
	// In multi-turn mode we simply compare signed deltas (all within [-90,90]),
	// and we apply the same mechanical limit (with flip) before comparison.
	bool steeringTargetChangedSignificantly(const std::vector<double> & new_target) const
	{
		if (!has_target_ || new_target.size() != target_logical_angles_.size()) {
			return true;
		}

		const bool allow_flip =
			(current_mode_ == CRAB || current_mode_ == COUNTERPHASE);

		for (int i = 0; i < wheel_count_; ++i) {
			if (!enabled_[i]) {
				continue;
			}

			bool dummy_invert = false;
			double limited_new =
				apply_mech_limit(new_target[i], dummy_invert, allow_flip);

			double err = std::fabs(limited_new - target_logical_angles_[i]);
			if (err > STEER_RETARGET_TOL_DEG) {
				return true;
			}
		}
		return false;
	}

	// Publish steering raw targets and update state machine.
	void publishSteeringTargets(const std::vector<double> & target_logical_angles)
	{
		std_msgs::msg::Float64MultiArray raw_cmd;
		raw_cmd.data.resize(wheel_count_, 0.0);

		const bool allow_flip =
			(current_mode_ == CRAB || current_mode_ == COUNTERPHASE);

		for (int i = 0; i < wheel_count_; ++i) {
			if (!enabled_[i]) {
				raw_cmd.data[i] = raw_pos_deg_[i];
				final_logical_targets_[i] = logical_pos_deg_[i];
				continue;
			}

			// Apply mechanical limit for multi-turn steering.
			// For CRAB/COUNTERPHASE do 180deg flip with wheel inversion.
			bool invert = false;
			double desired_logical_deg =
				apply_mech_limit(target_logical_angles[i], invert, allow_flip);
			drive_inverted_[i] = invert;

			// Convert logical angle around zero to raw multi-turn angle by adding zero offset.
			double target_raw_deg = zero_offset_deg_[i] + desired_logical_deg;

			raw_cmd.data[i] = target_raw_deg;
			final_logical_targets_[i] = desired_logical_deg;
			RCLCPP_INFO(
				get_logger(),
				"Wheel %d [Mode %d]: desired=%.2f (orig=%.2f, invert=%d) -> raw=%.2f (offset=%.2f)",
				i + 1, current_mode_, desired_logical_deg, target_logical_angles[i],
				invert ? 1 : 0, target_raw_deg, zero_offset_deg_[i]);
		}

		cmd_pos_raw_pub_->publish(raw_cmd);

		// Update target angles for state machine and enter transitioning
		target_logical_angles_ = final_logical_targets_;
		has_target_ = true;
		steer_state_ = TRANSITIONING;
	}

	// ======================================================
	// Mode Callback
	// ======================================================
	void onModeCmd(const std_msgs::msg::Int32::SharedPtr msg)
	{
		int req_mode = msg->data;
		if (req_mode >= 0 && req_mode <= 3) {
			ControlMode new_mode = static_cast<ControlMode>(req_mode);
			if (new_mode != current_mode_) {
				current_mode_ = new_mode;
				RCLCPP_INFO(get_logger(), "Mode switched to: %d", current_mode_);

				// Enter transitioning state on mode change and immediately compute
				// steering targets for this mode using preset angles (if applicable).
				if (new_mode == CRAB) {
                    steer_state_ = ACTIVE;
                    has_target_ = true; 
                } else {
                    steer_state_ = TRANSITIONING;
                    has_target_ = false;
                }

				std::vector<double> target_logical_angles(wheel_count_, 0.0);
				// For CRAB/COUNTERPHASE we use preset angles on mode entry
				computeTargetAnglesForCurrentMode(target_logical_angles, /*use_preset_for_travel_modes=*/true);
				publishSteeringTargets(target_logical_angles);
			} else {
				RCLCPP_INFO(get_logger(), "Mode requested is same as current: %d", current_mode_);
			}
		} else {
			RCLCPP_WARN(get_logger(), "Invalid mode request: %d", req_mode);
		}
	}

	// ======================================================
	// Twist Callback (vx, vy, omega)
	// ======================================================
	void onTwistCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
	{
		const double now_sec = this->get_clock()->now().seconds();

		// Map geometry_msgs/Twist to common RobotCommand via base helper
		robot_cmd_vel_callback_2d(
			msg->linear.x,   // vx
			msg->linear.y,   // vy (joystick: right+, left-)
			msg->angular.z,  // omega
			now_sec);

		// For CRAB/COUNTERPHASE update steering targets based on current velocity command.
		if (current_mode_ == CRAB || current_mode_ == COUNTERPHASE) {

			std::vector<double> target_logical_angles(wheel_count_, 0.0);
			computeTargetAnglesForCurrentMode(
				target_logical_angles,
				/*use_preset_for_travel_modes=*/false);

			if (steeringTargetChangedSignificantly(target_logical_angles)) {
				publishSteeringTargets(target_logical_angles);
			}
		}

		velocity_smoother(now_sec);
	}

	// ======================================================
	// Feedback Callback
	// ======================================================
	void onRawPos(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
	{
		if (msg->data.empty()) return;

		const size_t n = std::min(msg->data.size(), static_cast<size_t>(wheel_count_));

		// raw_pos_deg_ now stores raw multi-turn angle from bridge directly (no wrapping).
		for (size_t i = 0; i < n; ++i) {
			raw_pos_deg_[i] = msg->data[i];
		}

		// logical_pos_deg_ is the angle around mechanical zero 
		for (int i = 0; i < wheel_count_; ++i) {
			logical_pos_deg_[i] = raw_pos_deg_[i] - zero_offset_deg_[i];
		}

		std_msgs::msg::Float64MultiArray out;
		out.data = logical_pos_deg_;
		real_pos_pub_->publish(out);

		// State machine: if transitioning, check whether all steering angles reached target
		if (steer_state_ == TRANSITIONING && has_target_) {

			double max_err = 0.0;

			for (int i = 0; i < wheel_count_; ++i) {
				if (!enabled_[i]) {
					RCLCPP_INFO(get_logger(), "Wheel %d disabled -> skip", i+1);
					continue;
				}

				double current = logical_pos_deg_[i];
				double target  = target_logical_angles_[i];
				double err     = std::fabs(current - target);

				RCLCPP_INFO(
					get_logger(),
					" Wheel %d: logical_pos = %.3f deg, target = %.3f deg, err = %.3f deg (tol=%.3f)",
					i+1, current, target, err, transition_tol_deg_
				);

				if (err > max_err) max_err = err;
			}

			RCLCPP_INFO(
				get_logger(),
				" -> Max error across wheels = %.3f deg (tol=%.3f)",
				max_err, transition_tol_deg_
			);

			if (max_err <= transition_tol_deg_) {
				steer_state_ = ACTIVE;
				last_active_time_ = this->get_clock()->now();

				RCLCPP_INFO(
					get_logger(),
					" *** Steering transition finished. State -> ACTIVE. Max error=%.3f deg ***",
					max_err
				);
			}
		}
        // State machine for CRAB: if angle diff between wheels > 5 deg, go TRANSITIONING
        if (current_mode_ == CRAB) {
            double max_angle = -9999, min_angle = 9999;

            for (int i = 0; i < wheel_count_; ++i) {
                if (!enabled_[i]) continue;
                max_angle = std::max(max_angle, logical_pos_deg_[i]);
                min_angle = std::min(min_angle, logical_pos_deg_[i]);
            }

            double diff = std::fabs(max_angle - min_angle);

            if (diff > 5.0) {
                steer_state_ = TRANSITIONING;

                RCLCPP_WARN(
                    get_logger(),
                    "*** CRAB mismatch detected: max=%.2f, min=%.2f, diff=%.2f > 5° -> TRANSITIONING ***",
                    max_angle, min_angle, diff
                );
            }
        }
	}

	// ======================================================
	// Command Callback  (manual override, optional)
	// ======================================================
	void onCmdPos(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
	{
		if (msg->data.size() < static_cast<std::size_t>(wheel_count_)) {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
				"cmd_pos size mismatch. Expected %d", wheel_count_);
			return;
		}

		// Manual override: directly use incoming logical angles (around zero) as targets.
		std::vector<double> target_logical_angles(wheel_count_, 0.0);
		for (int i = 0; i < wheel_count_; ++i) {
			target_logical_angles[i] = msg->data[i];
		}

		publishSteeringTargets(target_logical_angles);

		RCLCPP_INFO(
			get_logger(),
			"Manual cmd_pos override received and applied as steering targets (multi-turn logical).");
	}

	// ======================================================
	// Set Zero Callback
	// ======================================================
	void onSetZero(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
	{
		if (msg->data.empty()) return;
		int wheel_id = msg->data[0];

		if (wheel_id == 5) {
			for (int i = 0; i < wheel_count_; ++i) {
				if (!enabled_[i]) continue;
				zero_offset_deg_[i] = raw_pos_deg_[i];
				RCLCPP_INFO(get_logger(), "Set zero W%d: offset=%.2f (multi-turn raw)", i + 1, zero_offset_deg_[i]);
			}
			return;
		}

		if (wheel_id < 1 || wheel_id > wheel_count_) return;
		int idx = wheel_id - 1;
		if (!enabled_[idx]) return;

		zero_offset_deg_[idx] = raw_pos_deg_[idx];
		RCLCPP_INFO(get_logger(), "Set zero W%d: offset=%.2f (multi-turn raw)", wheel_id, zero_offset_deg_[idx]);
	}

	// ======================================================================
	// wrapper velocity_smoother to start smoothing only when wheels can move
	// ======================================================================
	void velocity_smoother(double now_time_sec)
	{
		if (steer_state_ != ACTIVE) {
			auto & st = getState();
			st.smoothed_cmd.linear   = 0.0;
			st.smoothed_cmd.linear_y = 0.0;
			st.smoothed_cmd.angular  = 0.0;
			st.clipped_cmd           = st.smoothed_cmd;
			return;
		}
		antobot_control::AntoControlBase::velocity_smoother(now_time_sec);
	}

	// ======================================================================
	// wheel_speed_compute
	// ======================================================================
	void wheel_speed_compute() override
	{
		// If steering is still transitioning, force all wheel speeds to zero.
		if (steer_state_ == TRANSITIONING) {
			antobot_platform_msgs::msg::Float32Array wheel_cmd;
			wheel_cmd.data.resize(wheel_count_, 0.0f);
			pub_wheel_vel_cmd_->publish(wheel_cmd);
			return;
		}

		// ACTIVE state: compute wheel speeds from smoothed command and current steering angles.
		const auto & st  = getState();
		const auto & cmd = st.smoothed_cmd;
		const double vx  = cmd.linear;    // smoothed vx [m/s], forward positive
		const double vy  = cmd.linear_y;  // smoothed vy [m/s], left positive
		const double w   = cmd.angular;   // smoothed omega [rad/s], CCW positive

		antobot_platform_msgs::msg::Float32Array wheel_cmd;
		wheel_cmd.data.resize(wheel_count_, 0.0f);

		for (int drive_idx = 0; drive_idx < wheel_count_; ++drive_idx) {
			// Map drive index -> physical wheel -> steering index
			const int phys = DRIVE_INDEX_TO_PHYS[drive_idx];
			const int steer_idx = phys_to_steer_[phys];

			// Use steering enable for this wheel
			if (steer_idx < 0 || steer_idx >= wheel_count_ || !enabled_[steer_idx]) {
				wheel_cmd.data[drive_idx] = 0.0f;
				continue;
			}

			const double x_i = PHYSICAL_WHEEL_POS[phys].first;
			const double y_i = PHYSICAL_WHEEL_POS[phys].second;

			// Linear velocity of the wheel center in body frame:
			//   v_i = [vx, vy] + omega × r_i
			// Implemented as:
			//   vix = vx - w * y_i
			//   viy = vy + w * x_i
			const double vix = vx - w * y_i;
			const double viy = vy + w * x_i;

			// Wheel steering direction in body frame comes from steering index
			const double theta_deg = logical_pos_deg_[steer_idx];
			const double theta_rad = theta_deg / RAD_TO_DEG;
			const double ex = std::cos(theta_rad);
			const double ey = std::sin(theta_rad);

			double v_long = vix * ex + viy * ey;

			if (std::fabs(v_long) < LONGITUDINAL_DEADBAND) {
				v_long = 0.0;
			}

			// Convert linear speed to wheel angular speed [rad/s]
			double omega_wheel = 0.0;
			if (WHEEL_RADIUS_M > 0.0) {
				omega_wheel = v_long / WHEEL_RADIUS_M;
			}

			// Apply 180deg flip inversion if used for this steering index
			if (drive_inverted_[steer_idx]) {
				omega_wheel = -omega_wheel;
			}

			// RCLCPP_INFO(
			// 	this->get_logger(),
			// 	"Wheel %d (phys %d, steer %d): vx=%.3f vy=%.3f w=%.3f | "
			// 	"x_i=%.3f y_i=%.3f | theta_deg=%.2f ex=%.3f ey=%.3f | "
			// 	"vix=%.3f viy=%.3f | v_long=%.3f inv=%d -> omega=%.3f",
			// 	drive_idx, phys, steer_idx,
			// 	vx, vy, w, x_i, y_i,
			// 	theta_deg, ex, ey, vix, viy,
			// 	v_long, drive_inverted_[steer_idx] ? 1 : 0, omega_wheel
			// );
            omega_wheel = -omega_wheel;

			// wheel_cmd uses drive / bridge order
			wheel_cmd.data[drive_idx] = static_cast<float>(omega_wheel);
		}

		pub_wheel_vel_cmd_->publish(wheel_cmd);
	}

	// ======================================================================
	// compute_robot_twist_from_wheels (not used in this node)
	// ======================================================================
	void compute_robot_twist_from_wheels(double & linear, double & angular) override
	{
		// This node does not compute odometry yet. Return zeros.
		linear  = 0.0;
		angular = 0.0;
	}

private:
	int wheel_count_{4};
	std::vector<bool> enabled_;
	std::vector<double> zero_offset_deg_;
	std::vector<double> raw_pos_deg_;      // raw multi-turn angle from bridge
	std::vector<double> logical_pos_deg_;  // angle around zero (multi-turn aware)

	// For CRAB/COUNTERPHASE invert wheel speed when using 180deg flip.
	std::vector<bool> drive_inverted_;

	ControlMode current_mode_;

	// Steering state machine members
	SteerState steer_state_;
	std::vector<double> target_logical_angles_;
	std::vector<double> final_logical_targets_;
	bool has_target_;
	double transition_tol_deg_;

	// Mapping from physical wheel index -> steering array index
	std::array<int, 4> phys_to_steer_;

	// Debounce: ACTIVE must hold for steady_hold_sec_ before accepting new steering target
	rclcpp::Time last_active_time_;
	double steady_hold_sec_;
	bool enable_steady_hold_{false};

	// Subscriptions
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr raw_pos_sub_;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pos_sub_;
	rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr set_zero_sub_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr           mode_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      twist_sub_;

	// Publishers
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pos_raw_pub_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr real_pos_pub_;
	rclcpp::Publisher<antobot_platform_msgs::msg::Float32Array>::SharedPtr pub_wheel_vel_cmd_;

	// Periodic control loop timer
	rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<WheelSteerControl>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
