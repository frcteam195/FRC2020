package com.team195.frc.controllers;

import com.team195.frc.constants.Constants;
import com.team195.frc.auto.actions.*;
import com.team195.frc.auto.autonomy.AutomatedAction;
import com.team195.frc.reporters.ConsoleReporter;
import com.team195.frc.subsystems.*;
import com.team195.frc.subsystems.positions.TurretPositions;
import com.team195.lib.drivers.dashjoy.CKDashJoystick;
import com.team195.lib.util.ElapsedTimer;
import com.team195.lib.util.TeleopActionRunner;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.CrashTrackingRunnable;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.Notifier;

public class HIDController {
	private static HIDController mInstance = null;
	public synchronized static HIDController getInstance() {
		if (mInstance == null)
			mInstance = new HIDController();

		return mInstance;
	}

	private final CKDashJoystick driveJoystick = new CKDashJoystick(0);
	private final CKDashJoystick armControlJoystick = new CKDashJoystick(1);
	private final CKDashJoystick buttonBox1 = new CKDashJoystick(2);
	private final CKDashJoystick buttonBox2 = new CKDashJoystick(3);

	private final Object taskRunningLock_ = new Object();

	private Drive mDrive = Drive.getInstance();

	private boolean firstRun = true;

	private static double mThrottle = 0;
	private static double mTurn = 0;
	private static final DriveSignal mDriveSignalOutput = new DriveSignal(0, 0);

	private static final double HID_RATE_CONTROL = 0.020;

	private final Notifier mHIDNotifier;

	private final CrashTrackingRunnable mHIDRunnable = new CrashTrackingRunnable() {
		@Override
		public void runCrashTracked() {
		synchronized (taskRunningLock_) {
			if (firstRun) {
				Thread.currentThread().setName("HIDController");
				Thread.currentThread().setPriority(Constants.kControllerThreadPriority);

				firstRun = false;
			}
			try {
				mThrottle = -driveJoystick.getNormalizedAxis(1, Constants.kJoystickDeadband);
				mTurn = driveJoystick.getNormalizedAxis(2, Constants.kJoystickDeadband);


				if (mDrive.getDriveControlState() == Drive.DriveControlState.OPEN_LOOP) {
					mDrive.setBrakeMode(driveJoystick.getTriggerPressed(3, Constants.kJoystickTriggerThreshold) || VisionTracker.getInstance().isVisionEnabled());
					mDriveSignalOutput.set(Math.max(Math.min(mThrottle + mTurn, 1), -1), Math.max(Math.min(mThrottle - mTurn, 1), -1));
					mDrive.setOpenLoop(mDriveSignalOutput);
				}

				if (driveJoystick.getRawButton(1)) {

				} else if (driveJoystick.getRawButton(2)) {

				} else {

				}

				//Left Trigger
//				if (driveJoystick.getRisingEdgeTrigger(3, Constants.kJoystickTriggerThreshold)) {
//
//				} else {
//
//				}

				//Right Trigger
				if (driveJoystick.getRisingEdgeTrigger(4, Constants.kJoystickTriggerThreshold)) {

				}

				if (buttonBox1.getRawButton(1)) {

				} else if (buttonBox1.getRisingEdgeButton(2)) {

				} else if (buttonBox1.getRisingEdgeButton(3)) {

				} else if (buttonBox1.getRisingEdgeButton(4)) {

				} else if (buttonBox1.getRisingEdgeButton(5)) {

				} else if (buttonBox1.getRisingEdgeButton(7)) {

				} else if (buttonBox1.getRisingEdgeButton(8)) {

				} else if (buttonBox1.getRisingEdgeButton(9)) {

				} else if (buttonBox1.getRisingEdgeButton(10)) {

				} else if (buttonBox1.getRisingEdgeButton(11)) {

				} else if (buttonBox1.getRisingEdgeButton(12)) {

				} else if (buttonBox1.getRisingEdgeButton(13)) {

				} else if (buttonBox1.getRisingEdgeButton(14)) {

				} else if (buttonBox1.getRisingEdgeButton(15)) {

				} else if (buttonBox1.getRisingEdgeButton(16)) {

				} else {

				}


				if (buttonBox2.getRisingEdgeButton(1)) {
					TeleopActionRunner.runAction(AutomatedAction.fromAction(
							new SetIntakeAction(false, () -> buttonBox1.getRawButton(8)), 300, Turret.getInstance()));
				} else if (buttonBox2.getRisingEdgeButton(3)) {

				} else if (buttonBox2.getRisingEdgeButton(4)) {

				} else if (buttonBox2.getRisingEdgeButton(5)) {

				} else if (buttonBox2.getRisingEdgeButton(6)) {

				} else if (buttonBox2.getRisingEdgeButton(7)) {

				} else if (buttonBox2.getRisingEdgeButton(8)) {

				} else if (buttonBox2.getRisingEdgeButton(9)) {

				} else if (buttonBox2.getRisingEdgeButton(10)) {

				} else if (buttonBox2.getRisingEdgeButton(11)) {

				} else if (buttonBox2.getRisingEdgeButton(12)) {

				} else if (buttonBox2.getRisingEdgeButton(13)) {

				} else if (buttonBox2.getRisingEdgeButton(14)) {
					//Flash LEDs
					LEDController.getInstance().setRequestedState(LEDController.LEDState.BLINK);
				}


				if (armControlJoystick.getRisingEdgeButton(1)) {
					//Flash LEDs to signal Human Player
					LEDController.getInstance().setLEDColor(Constants.kRequestGamePieceColor);
					LEDController.getInstance().setRequestedState(LEDController.LEDState.BLINK);
				} else if (armControlJoystick.getRisingEdgeButton(2)) {
//					TeleopActionRunner.runAction(AutomatedAction.fromAction(new SetTurretPositionJoystickAction((t) -> armControlJoystick.getRawButton(2),
//							(t) -> armControlJoystick.getNormalizedAxis(2, 0.1)), 300, Turret.getInstance()));
				} else if (armControlJoystick.getRisingEdgeButton(3)) {

				} else if (armControlJoystick.getRisingEdgeButton(4)) {

				} else if (armControlJoystick.getRisingEdgeButton(5)) {

				} else if (armControlJoystick.getRisingEdgeButton(6)) {

				} else if (armControlJoystick.getRisingEdgeButton(7)) {

				} else if (armControlJoystick.getRisingEdgeButton(8)) {
					//Turret Open Loop
					TeleopActionRunner.runAction(AutomatedAction.fromAction(new SetTurretOpenLoopAction(() -> armControlJoystick.getRawButton(8),
							() -> -armControlJoystick.getNormalizedAxis(2, 0.1) / 3.0), 300, Turret.getInstance()));
				} else if (armControlJoystick.getRisingEdgeButton(9)) {
					//Rehome Arm
				} else if (armControlJoystick.getRisingEdgeButton(11)) {
					//Rehome turret
					Turret.getInstance().zeroSensors();
					Turret.getInstance().setTurretControlMode(Turret.TurretControlMode.POSITION);
				} else if (armControlJoystick.getRisingEdgeButton(12)) {

				}

				switch (armControlJoystick.getPOV()) {
					case 0:
						break;
					case 90:
						break;
					case 180:
						break;
					case 270:
						break;
					default:
						break;
				}
			} catch (Exception ex) {
				ConsoleReporter.report(ex);
			} catch (Throwable t) {
				ConsoleReporter.report(t);
				CrashTracker.logThrowableCrash(t);
			}
		}

		TeleopActionRunner.processActions();
		}
	};

	private HIDController() {
		mHIDNotifier = new Notifier(mHIDRunnable);
	}

	public void start() {
		synchronized (taskRunningLock_) {
			mHIDNotifier.startPeriodic(HID_RATE_CONTROL);
		}
	}

	public void stop() {
		synchronized (taskRunningLock_) {
			mHIDNotifier.stop();
		}
	}
}
