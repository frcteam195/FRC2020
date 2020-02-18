package com.team195.lib.util;

public class DriveHelper {
	public static double velocityMaxAccelDecelFilter(double requestedVelocity, double currentVelocity, double maxAccel, double maxDecel, double dt) {
		double diffErr = requestedVelocity - currentVelocity;
		if (maxAccel == 0 || maxDecel == 0)
			return requestedVelocity;
		boolean accel = Math.abs(requestedVelocity) > Math.abs(currentVelocity) && Math.signum(requestedVelocity) == Math.signum(currentVelocity);
		double accelConst = accel ? maxAccel : maxDecel;
		double speedNew = currentVelocity + (accelConst * dt * Math.copySign(1.0, diffErr));
		speedNew = diffErr >= 0 ? Math.min(speedNew, requestedVelocity) : Math.max(speedNew, requestedVelocity);
		return speedNew;
	}
}
