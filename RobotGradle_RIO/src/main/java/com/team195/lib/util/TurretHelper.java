package com.team195.lib.util;

import com.team195.frc.constants.CalConstants;
import com.team254.lib.geometry.Rotation2d;

public class TurretHelper {
//	public static double convertRotationsToTurretDegrees(double rotations) {
//		return rotations / CalConstants.kTurretGearRatioMotorToTurretGear / (CalConstants.kTurretLargeGearTeeth / CalConstants.kTurretSmallGearTeeth / 360.0);
//	}

	public static double convertTurretDegreesToRotations(double degrees) {
		return degrees * CalConstants.kTurretGearRatioMotorToTurretGear * (CalConstants.kTurretLargeGearTeeth / CalConstants.kTurretSmallGearTeeth / 360.0);
	}

	public static double calculateSetpointForRobotCentricRotation(double currentRotationDegrees, Rotation2d robotCentricSetpoint, double minDegrees, double maxDegrees) {
		Rotation2d currentRobotCentricRotation = Rotation2d.fromDegrees(currentRotationDegrees);
		Rotation2d setpoint = robotCentricSetpoint;

		Rotation2d error = setpoint.inverse().rotateBy(currentRobotCentricRotation);
		double newDemandRotationDegrees = currentRotationDegrees - error.getDegrees();
		if(newDemandRotationDegrees < minDegrees) {
			newDemandRotationDegrees += 360;
		}
		if(newDemandRotationDegrees > maxDegrees) {
			newDemandRotationDegrees -= 360;
		}
		return newDemandRotationDegrees;
	}

	public static Rotation2d calculateRobotCentricTurretRotation(Rotation2d fieldCentricDemand, Rotation2d robotRotation) {
		return fieldCentricDemand.rotateBy(robotRotation.inverse());
	}
}
