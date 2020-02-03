package com.team195.lib.util.arm;

import com.team195.frc.reporters.ConsoleReporter;
import com.team195.frc.reporters.MessageLevel;
import com.team254.lib.util.MovingAverage;

public class ARMThreadRateControl {
	private MovingAverage mAverageLoopTime = new MovingAverage(20);

	private boolean started = false;
	private double prevDtCalcTime = 0;

	private final ARMElapsedTimer eTimer = new ARMElapsedTimer();

	public ARMThreadRateControl() {

	}

	public synchronized void start(boolean resetStart) {
		if (resetStart)
			started = false;
		start();
	}

	public synchronized void start() {
		if (!started) {
			eTimer.start();
			getDt();
			started = true;
		} else {
			ConsoleReporter.report("Thread rate control start called too many times!", MessageLevel.ERROR);
		}
	}

	/**
	 * Do rate control for loops
	 * @param minLoopTimeMs Time in ms
	 */
	public synchronized void doRateControl(int minLoopTimeMs) {
		double remainingTime = ((minLoopTimeMs / 1000.0) - eTimer.hasElapsed());
		if (remainingTime > 0) {
			ARMNotifierJNI.sleepForNano((int)(remainingTime * 1e9));
		}
		mAverageLoopTime.addNumber(eTimer.hasElapsed());
		eTimer.start();
	}

	public double getLoopTime() {
		return mAverageLoopTime.getLastSample();
	}

	public double getAverageLoopTime() {
		return mAverageLoopTime.getAverage();
	}

	public synchronized double getDt() {
		double currDtCalcTime = ARMTimer.getCurrentTimestamp();
		double dt = currDtCalcTime - prevDtCalcTime;
		prevDtCalcTime = currDtCalcTime;
		return dt;
	}
}
