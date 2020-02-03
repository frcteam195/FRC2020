package com.team195.lib.util.arm;

public class ARMTimeoutTimer {
	private double timeout;
	private ARMElapsedTimer eTimer = new ARMElapsedTimer();
	private boolean firstRun;

	public ARMTimeoutTimer(double timeout) {
		this.timeout = timeout;
		setFirstRun(true);
	}

	public boolean isTimedOut() {
		if (firstRun) {
			eTimer.start();
			setFirstRun(false);
		}
		return eTimer.hasElapsed() > timeout;
	}

	public double getTimeLeft() {
		return Math.max(timeout - eTimer.hasElapsed(), 0);
	}

	public void reset() {
		setFirstRun(true);
	}

	private synchronized void setFirstRun(boolean firstRun) {
		this.firstRun = firstRun;
	}
}
