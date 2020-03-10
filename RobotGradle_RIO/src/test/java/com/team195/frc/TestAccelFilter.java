package com.team195.frc;

import org.junit.Test;

public class TestAccelFilter {

	private double prevVel = 0;

//	@Test
//	public void kyleAccelFilter() {
//		for (int i = 0; i < 200; i++) {
//			prevVel = shooterVelocityMaxAccelFilterKyle(100, prevVel, 100, 100, 0.01);
//			System.out.println(prevVel);
//		}
//		for (int i = 0; i < 200; i++) {
//			prevVel = shooterVelocityMaxAccelFilterKyle(-100, prevVel, 100, 100, 0.01);
//			System.out.println(prevVel);
//		}
//		for (int i = 0; i < 200; i++) {
//			prevVel = shooterVelocityMaxAccelFilterKyle(0, prevVel, 100, 100, 0.01);
//			System.out.println(prevVel);
//		}
//		for (int i = 0; i < 200; i++) {
//			prevVel = shooterVelocityMaxAccelFilterKyle(100, prevVel, 100, 100, 0.01);
//			System.out.println(prevVel);
//		}
//		for (int i = 0; i < 200; i++) {
//			prevVel = shooterVelocityMaxAccelFilterKyle(0, prevVel, 100, 100, 0.01);
//			System.out.println(prevVel);
//		}
//	}

	@Test
	public void robAccelFilter() {
		for (int i = 0; i < 200; i++) {
			prevVel = shooterVelocityMaxAccelFilterRob(100, prevVel,  100, 0.01);
			System.out.println(prevVel);
		}
		for (int i = 0; i < 200; i++) {
			prevVel = shooterVelocityMaxAccelFilterRob(-100, prevVel,  100, 0.01);
			System.out.println(prevVel);
		}
		for (int i = 0; i < 200; i++) {
			prevVel = shooterVelocityMaxAccelFilterRob(0, prevVel,  100, 0.01);
			System.out.println(prevVel);
		}
		for (int i = 0; i < 200; i++) {
			prevVel = shooterVelocityMaxAccelFilterRob(100, prevVel,  100, 0.01);
			System.out.println(prevVel);
		}
		for (int i = 0; i < 200; i++) {
			prevVel = shooterVelocityMaxAccelFilterRob(0, prevVel,  100, 0.01);
			System.out.println(prevVel);
		}
	}

	public double shooterVelocityMaxAccelFilterRob(double requestedVelocity, double currentVelocity, double maxAccel, double dt) {
		double diffErr = requestedVelocity - currentVelocity;
		if (maxAccel == 0)
			return requestedVelocity;
		return currentVelocity + Math.min(Math.abs(diffErr), (maxAccel * dt)) * Math.copySign(1.0, diffErr);
	}

	public double shooterVelocityMaxAccelFilterKyle(double requestedVelocity, double currentVelocity, double maxAccel, double maxDecel, double dt) {
		double diffErr = requestedVelocity - currentVelocity;
		if (maxAccel == 0 || maxDecel == 0)
			return requestedVelocity;
		boolean accel = Math.abs(requestedVelocity) > Math.abs(currentVelocity) && Math.signum(requestedVelocity) == Math.signum(currentVelocity);
		double accelConst = accel ? maxAccel : maxDecel;
		double speedNew = currentVelocity + Math.min(Math.abs(diffErr), (accelConst * dt)) * Math.copySign(1.0, diffErr);
		return speedNew;
	}
}
