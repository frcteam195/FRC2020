package com.team195.frc;

import com.team195.frc.constants.CalConstants;
import com.team195.frc.constants.Constants;
import com.team195.frc.subsystems.Drive;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

import java.util.Map;

public class RobotState {
	private static RobotState instance_ = new RobotState();

	public static RobotState getInstance() {
		return instance_;
	}

	private static final int kObservationBufferSize = 100;

	// FPGATimestamp -> Pose2d or Rotation2d
	private final InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
	private Twist2d vehicle_velocity_predicted_;
	private Twist2d vehicle_velocity_measured_;
	private double distance_driven_;

	private RobotState() {
		reset(0, new Pose2d());
	}

	/**
	 * Resets the field to robot transform (robot's position on the field)
	 */
	public void reset(double start_time, Pose2d initial_field_to_vehicle) {
		field_to_vehicle_.clear();
//		field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
		field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
		Drive.getInstance().setHeading(initial_field_to_vehicle.getRotation());
		vehicle_velocity_predicted_ = Twist2d.identity();
		vehicle_velocity_measured_ = Twist2d.identity();
		distance_driven_ = 0.0;
	}

	public void resetDistanceDriven() {
		distance_driven_ = 0.0;
	}

	/**
	 * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
	 * to fill in the gaps.
	 */
	public Pose2d getFieldToVehicle(double timestamp) {
		return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
	}

	public Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
		return field_to_vehicle_.lastEntry();
	}

	public Pose2d getPredictedFieldToVehicle(double lookahead_time) {
		return getLatestFieldToVehicle().getValue()
				.transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
	}

	public void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
		field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
	}

	public void addObservations(double timestamp, Twist2d measured_velocity,
	                                         Twist2d predicted_velocity) {
		addFieldToVehicleObservation(timestamp,
				Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), measured_velocity));
		vehicle_velocity_measured_ = measured_velocity;
		vehicle_velocity_predicted_ = predicted_velocity;
	}

	public Twist2d generateOdometryFromSensors(double left_encoder_delta_distance, double
			right_encoder_delta_distance, Rotation2d current_gyro_angle) {
//		final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
		final Twist2d delta = Kinematics.forwardKinematics(getLatestFieldToVehicle().getValue().getRotation(),
				left_encoder_delta_distance, right_encoder_delta_distance,
				current_gyro_angle);
		distance_driven_ += delta.dx; //do we care about dy here?
		return delta;
	}

	public double getDistanceDriven() {
		return distance_driven_;
	}

	public Twist2d getPredictedVelocity() {
		return vehicle_velocity_predicted_;
	}

	public Twist2d getMeasuredVelocity() {
		return vehicle_velocity_measured_;
	}
}
