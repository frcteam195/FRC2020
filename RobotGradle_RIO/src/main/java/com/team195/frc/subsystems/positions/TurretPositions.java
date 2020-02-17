package com.team195.frc.subsystems.positions;

import com.team195.frc.subsystems.Turret;
import com.team195.lib.util.TurretHelper;

public class TurretPositions {
	public static final double Home = 0;
	public static final double PositionDelta = 0.15;

	public static final double Left90 = TurretHelper.convertTurretDegreesToRotations(-90);
	public static final double Right90 = TurretHelper.convertTurretDegreesToRotations(90);
	public static final double Back180 = TurretHelper.convertTurretDegreesToRotations(180);

	//Ball Shooter
	public static final double BallShootSpeedNormal = 1;
	public static final double BallShootSpeedIntake = -1;
	public static final double BallShootSpeedOff = 0;
}
