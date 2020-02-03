package com.team195.lib.util.arm;

import edu.wpi.first.wpilibj.Timer;

public class ARMElapsedTimer {
	private double startTime = 0;

	public ARMElapsedTimer() {

	}

	public synchronized void start() {
		startTime = ARMTimer.getCurrentTimestamp();
	}

	public double hasElapsed() {
		return ARMTimer.getCurrentTimestamp() - startTime;
	}
}
