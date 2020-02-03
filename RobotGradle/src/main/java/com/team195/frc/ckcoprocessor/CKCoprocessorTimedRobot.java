package com.team195.frc.ckcoprocessor;

import com.team195.lib.util.arm.ARMThreadRateControl;
import java.util.function.Supplier;

public abstract class CKCoprocessorTimedRobot {
	public static final int kLoopTimeMs = 20;

	public abstract void robotInit();
	public abstract void disabledInit();
	public abstract void autonomousInit();
	public abstract void teleopInit();
	public abstract void testInit();
	public abstract void robotPeriodic();
	public abstract void disabledPeriodic();
	public abstract void autonomousPeriodic();
	public abstract void teleopPeriodic();
	public abstract void testPeriodic();

	private RIOToCoprocessorData.CKRIOData.RobotMode currRobotMode = RIOToCoprocessorData.CKRIOData.RobotMode.NOMODE;
	private RIOToCoprocessorData.CKRIOData.RobotMode prevRobotMode = RIOToCoprocessorData.CKRIOData.RobotMode.NOMODE;

	protected void loopFunc() {
		currRobotMode = CKCoprocessorDataReceiver.getInstance().getCachedRIOData().getRobotMode();
		boolean init = currRobotMode != prevRobotMode;
		prevRobotMode = currRobotMode;
		switch (currRobotMode) {
			case AUTO:
				if (init) {
					autonomousInit();
				}
				autonomousPeriodic();
				break;
			case TELEOP:
				if (init) {
					teleopInit();
				}
				teleopPeriodic();
				break;
			case TEST:
				if (init) {
					testInit();
				}
				testPeriodic();
				break;
			default:
				if (init) {
					disabledInit();
				}
				disabledPeriodic();
				break;
		}
		robotPeriodic();
	}

	/**
	 * Run the robot main loop.
	 */
	@SuppressWarnings({"PMD.AvoidInstantiatingObjectsInLoops", "PMD.AvoidCatchingThrowable",
			"PMD.CyclomaticComplexity", "PMD.NPathComplexity"})
	private static <T extends CKCoprocessorTimedRobot> void runRobot(Supplier<T> robotSupplier) {
		T robot;
		try {
			robot = robotSupplier.get();
		} catch (Throwable throwable) {
			Throwable cause = throwable.getCause();
			if (cause != null) {
				throwable = cause;
			}
			String robotName = "Unknown";
			StackTraceElement[] elements = throwable.getStackTrace();
			if (elements.length > 0) {
				robotName = elements[0].getClassName();
			}
			System.out.println("Unhandled exception instantiating robot " + robotName + " " + throwable.toString());
			System.out.println("Robots should not quit, but yours did!");
			System.out.println("Could not instantiate robot " + robotName + "!");
			return;
		}

		ARMThreadRateControl trc = new ARMThreadRateControl();
		robot.robotInit();
		trc.start();
		while (true) {
			robot.loopFunc();
			trc.doRateControl(kLoopTimeMs);
		}
	}

	public static <T extends CKCoprocessorTimedRobot> void startRobot(Supplier<T> robotSupplier) {
		Thread thread = new Thread(() -> {
			runRobot(robotSupplier);
		}, "CoprocessorMain");
		thread.setDaemon(true);
		thread.start();
		try {
			thread.join();
		} catch (InterruptedException ex) {
			Thread.currentThread().interrupt();
			System.out.println(ex.toString());
		}
	}
}
