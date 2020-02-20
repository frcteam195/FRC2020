package com.team195.frc.constants;

import com.team195.frc.subsystems.Turret;
import com.team195.lib.util.TurretHelper;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class CalConstants {
	/* ROBOT PHYSICAL CONSTANTS */

	// Wheels
	public static final double kDriveWheelTrackWidthInches = 23.5;
	public static final double kDriveWheelDiameterInches = 5.0 * 0.9625;
	public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
	public static final double kTrackScrubFactor = 1.0;  // Tune me!

	// Tuned dynamics
	public static final double kRobotLinearInertia = 68.946;  // kg TODO tune
	public static final double kRobotAngularInertia = 125;  // kg m^2 TODO tune
	public static final double kRobotAngularDrag = 0.1;  // N*m / (rad/sec) TODO tune
	public static final double kDriveVIntercept = 0.30165000; //0.781046438 angular  // V
	public static final double kDriveKv = 0.186163041;  // V per rad/s
	public static final double kDriveKa = 0.0086739979;  // V per rad/s^2

	// Gearing and mechanical constants.
	public static final double kDriveDownShiftVelocity = 9.5 * 12.0;  // inches per second
	public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
	public static final double kDriveUpShiftVelocity = 11.0 * 12.0;  // inches per second
	public static final double kPathKX = 4.0;  // units/s per unit of error
	public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
	public static final double kPathMinLookaheadDistance = 24.0;  // inches

	public static final double kDriveGearRatioMotorConversionFactor = 8.0;

	/* CONTROL LOOP GAINS */

	///////////////////////////////////////////////////////////////////////////
	//Drive
	public static final double kDriveDefaultVoltageCompensationSetpoint = 12.0;
	public static final double kDriveDefaultOpenLoopRampRate = 0.1;

	// PID gains for drive velocity loop (LOW GEAR)
	// Units: setpoint, error, and output are in ticks per second.
	public static final double kDriveLowGearVelocityKp = 0.002;
	public static final double kDriveLowGearVelocityKi = 0.0;
	public static final double kDriveLowGearVelocityKd = 0.000;
	public static final double kDriveLowGearVelocityKf = 0;//0.000176;
	public static final double kDriveLowGearVelocityDFilter = 1;
	public static final int kDriveLowGearVelocityIZone = 0;
	public static final double kDriveLowGearPositionCruiseVel = 2000;
	public static final double kDriveLowGearPositionAccel = 1000;
	public static final int kDriveLowGearCurrentLim = 50;
	///////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////
	//Color Panel Manipulator
	//12:1 ??
	public static final double kCPMPositionKp = 4.3;
	public static final double kCPMPositionKi = 0.0;
	public static final double kCPMPositionKd = 8.0;
	public static final double kCPMPositionKf = 0.400360;
	public static final int kCPMPositionCruiseVel = 350;
	public static final int kCPMPositionMMAccel = 350;
	public static final int kCPMPositionSCurveStrength = 5;
	public static final int kCPMContinuousCurrentLimit = 20;
	public static final int kCPMPeakCurrentThreshold = 0;
	public static final int kCPMPeakCurrentThresholdExceedDuration = 0;
	///////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////
	//Turret
	//50:1
	public static final double kTurretPositionKp = 4.3;
	public static final double kTurretPositionKi = 0.0;
	public static final double kTurretPositionKd = 8.0;
	public static final double kTurretPositionKf = 0.400360;
	public static final int kTurretPositionCruiseVel = 350;
	public static final int kTurretPositionMMAccel = 350;
	public static final int kTurretPositionSCurveStrength = 5;
	public static final int kTurretContinuousCurrentLimit = 8;
	public static final int kTurretPeakCurrentThreshold = 9;
	public static final int kTurretPeakCurrentThresholdExceedDuration = 0;
	public static final double kTurretBallShooterOpenLoopRamp = 0.2;
	public static final int kTurretBallShooterContinuousCurrentLimit = 15;
	public static final int kTurretBallShooterPeakCurrentThreshold = 25;
	public static final int kTurretBallShooterPeakCurrentThresholdExceedDuration = 450;
	//Units in rotations
	public static final double kTurretMinDegrees = 270;
	public static final double kTurretMaxDegrees = -kTurretMinDegrees;
	public static final double kTurretForwardSoftLimit = TurretHelper.convertTurretDegreesToRotations(kTurretMinDegrees);
	public static final double kTurretReverseSoftLimit = TurretHelper.convertTurretDegreesToRotations(kTurretMaxDegrees);
	public static final double kTurretSmallGearTeeth = 18;
	public static final double kTurretLargeGearTeeth = 124;

	// TODO measure on robot!
	public static final Pose2d kVehicleToTurret = new Pose2d(0.42, -1.75, Rotation2d.identity()); //Pose of turret w.r.t. robot
	// TODO measure on robot!
	public static final Pose2d kTurretToCamera = new Pose2d(9.737, -0.25, Rotation2d.identity()); //Pose of camera w.r.t. turret

	public static final double kColorWheelDiameterInches = 32.0;
	public static final double kColorWheelColorArcInches = 12.5;
	public static final double kColorWheelManipulatorDiameter = 4.0;
	///////////////////////////////////////////////////////////////////////////

	//Breaker model for trip time output in seconds y = a*(current_percent_over_rating)^b + c
	public static final double kPDPBreakerModelA = 282.2962;
	public static final double kPDPBreakerModelB = -6.6305;
	public static final double kPDPBreakerModelC = 0.5;
	public static final double kPDPDefaultSafetyFactor = 4.0;

	//TODO: Tune collision detection
	// Collision Detection
	public static final double kCollisionDetectionJerkThreshold = 950;
	public static final double kTippingThresholdDeg = 11;
}
