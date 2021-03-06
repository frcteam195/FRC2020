package frc.robot;

import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.Timer;

public class ThreadRateControl {

	private final int m_notifier = NotifierJNI.initializeNotifier();

	private boolean started = false;
	private double prevDtCalcTime = 0;

	private final ElapsedTimer eTimer = new ElapsedTimer();

	public ThreadRateControl() {

	}

	@Override
	@SuppressWarnings({"NoFinalizer", "deprecation"})
	protected void finalize() {
		NotifierJNI.stopNotifier(m_notifier);
		NotifierJNI.cleanNotifier(m_notifier);
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
			//ConsoleReporter.report("Thread rate control start called too many times!", MessageLevel.ERROR);
		}
	}

	/**
	 * Do rate control for loops
	 * @param minLoopTimeMs Time in ms
	 */
	public synchronized void doRateControl(int minLoopTimeMs) {
		double remainingTime = ((minLoopTimeMs / 1000.0) - eTimer.hasElapsed());
		if (remainingTime > 0) {
			//Subtract constant offset for code delay (currently 150us)
			NotifierJNI.updateNotifierAlarm(m_notifier, (long) (((Timer.getFPGATimestamp() + remainingTime) * 1e6) - 150));
			NotifierJNI.waitForNotifierAlarm(m_notifier);
		}
		eTimer.start();
	}


	public synchronized double getDt() {
		double currDtCalcTime = Timer.getFPGATimestamp();
		double dt = currDtCalcTime - prevDtCalcTime;
		prevDtCalcTime = currDtCalcTime;
		return dt;
	}
}
