package com.team195.frc.constants;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class TargetingConstants {
	public static final Translation2d fieldToInnerTarget = new Translation2d(-8, 67); //Inner
	public static final Pose2d fieldToOuterTarget = new Pose2d(0, 67, Rotation2d.identity()); //Outer
}
