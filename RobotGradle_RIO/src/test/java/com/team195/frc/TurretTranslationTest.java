package com.team195.frc;

import com.team195.frc.constants.CalConstants;
import com.team195.frc.constants.TargetingConstants;
import com.team195.lib.util.TurretHelper;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import org.junit.Test;

import static org.junit.Assert.assertEquals;


public class TurretTranslationTest {

	@Test
	public void testTranslationMath() {
//		double turret_position = 0;
//		Pose2d fieldToVehicle = new Pose2d(new Translation2d(400, 125), Rotation2d.fromDegrees(-160));
//
//		Pose2d latestFieldToTurret = fieldToVehicle.transformBy(new Pose2d(CalConstants.kVehicleToTurret.getTranslation(), Rotation2d.fromDegrees(TurretHelper.convertRotationsToTurretDegrees(turret_position))));
//		Translation2d turretToTarget = TargetingConstants.fieldToOuterTarget.getTranslation().translateBy(latestFieldToTurret.getTranslation().inverse());
//		Rotation2d robotCentricSetpoint = turretToTarget.direction().rotateBy(fieldToVehicle.getRotation().inverse());
//		double rawDegreesOut = TurretHelper.calculateSetpointForRobotCentricRotation(TurretHelper.convertRotationsToTurretDegrees(turret_position), robotCentricSetpoint, CalConstants.kTurretMinDegrees, CalConstants.kTurretMinDegrees);
//		System.out.println("Raw Degrees Out:" + rawDegreesOut);
		//assertEquals(1, rawDegreesOut, 0);
	}
}
