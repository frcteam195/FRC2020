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

	private static final boolean USE_CHEESY_DRIVE = true;

	private final CKDashJoystick driveJoystick = Controllers.getInstance().getDriveJoystick();
	private final CKDashJoystick armControlJoystick = Controllers.getInstance().getArmControlJoystick();
	private final CKDashJoystick buttonBox1 = Controllers.getInstance().getButtonBox1();
	private final CKDashJoystick buttonBox2 = Controllers.getInstance().getButtonBox2();

	private final Object taskRunningLock_ = new Object();

	private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
	private Drive mDrive = Drive.getInstance();

	private boolean firstRun = true;
	private boolean stoppedAuto = false;

	private boolean autoBrake = false;
	private ElapsedTimer autoBrakeTimer = new ElapsedTimer();
	private static final double kBrakeTimeout = 1;

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
				if (Infrastructure.getInstance().isDuringAuto() && !stoppedAuto) {
					if (driveJoystick.isAxisInputActive()) {
//						if (Robot.mAutoModeExecutor.isSet())
//							Robot.mAutoModeExecutor.stop();
//						stoppedAuto = true;
					}
				} else {
					//User Control Interface code here

					if (!USE_CHEESY_DRIVE) {
						mThrottle = -driveJoystick.getSmoothedAxis(1, Constants.kJoystickDeadband, 2);
						mTurn = driveJoystick.getNormalizedAxis(4, Constants.kJoystickDeadband) * 0.5;
						if (VisionTracker.getInstance().isVisionEnabled() && VisionTracker.getInstance().getTargetMode() == VisionTracker.TargetMode.HATCH) {
							if (Turret.getInstance().getSetpoint() == TurretPositions.Right90) {
								if (VisionTracker.getInstance().isTargetFound())
									mThrottle = -Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 1), -1);
							} else if (Turret.getInstance().getSetpoint() == TurretPositions.Left90) {
								if (VisionTracker.getInstance().isTargetFound())
									mThrottle = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 1), -1);
							} else {
								if (VisionTracker.getInstance().isTargetFound())
									mTurn = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.007, 1), -1);
							}
						}


						if (mDrive.getDriveControlState() == Drive.DriveControlState.OPEN_LOOP) {
							mDrive.setBrakeMode(driveJoystick.getRawButton(5) || driveJoystick.getRawButton(6) || VisionTracker.getInstance().isVisionEnabled());
							mDriveSignalOutput.set(Math.max(Math.min(mThrottle + mTurn, 1), -1), Math.max(Math.min(mThrottle - mTurn, 1), -1));
							mDrive.setOpenLoop(mDriveSignalOutput);
						}
					} else {
						mThrottle = -driveJoystick.getNormalizedAxis(1, Constants.kJoystickDeadband);
						mTurn = driveJoystick.getNormalizedAxis(4, Constants.kJoystickDeadband) * 0.6;

						if (VisionTracker.getInstance().isVisionEnabled() && VisionTracker.getInstance().getTargetMode() == VisionTracker.TargetMode.HATCH) {
							if (Turret.getInstance().getSetpoint() == TurretPositions.Right90) {
								if (VisionTracker.getInstance().isTargetFound())
									mThrottle = -Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 1), -1);
							} else if (Turret.getInstance().getSetpoint() == TurretPositions.Left90) {
								if (VisionTracker.getInstance().isTargetFound())
									mThrottle = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 1), -1);
							} else {
								if (VisionTracker.getInstance().isTargetFound())
									mTurn = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 0.1), -0.1);
							}
						}

						if (mDrive.getDriveControlState() == Drive.DriveControlState.OPEN_LOOP) {
							boolean brake = driveJoystick.getRawButton(5) || driveJoystick.getRawButton(6) || VisionTracker.getInstance().isVisionEnabled();
							mDrive.setBrakeMode(brake || autoBrake);

							if (brake)
								autoBrakeTimer.start();

							autoBrake = autoBrakeTimer.hasElapsed() < kBrakeTimeout;


							boolean quickTurn = driveJoystick.getRawButton(6) || VisionTracker.getInstance().isVisionEnabled();
							if (quickTurn)
								mTurn *= 0.5;

							mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(mThrottle, mTurn, quickTurn, true));
						}
					}

					if (driveJoystick.getRawButton(1)) {
						VisionTracker.getInstance().setTargetMode(VisionTracker.TargetMode.HATCH);
						if (!VisionTracker.getInstance().isTargetAreaReached())
							VisionTracker.getInstance().setVisionEnabled(true);
						else
							VisionTracker.getInstance().setVisionEnabled(false);
					} else if (driveJoystick.getRawButton(2)) {
						VisionTracker.getInstance().setTargetMode(VisionTracker.TargetMode.HATCH);
						VisionTracker.getInstance().setVisionEnabled(true);
					} else {
						if (VisionTracker.getInstance().getTargetMode() == VisionTracker.TargetMode.HATCH)
							VisionTracker.getInstance().setVisionEnabled(false);
					}

					if (driveJoystick.getRisingEdgeTrigger(2, Constants.kJoystickTriggerThreshold)) {

					} else if (driveJoystick.getRisingEdgeTrigger(3, Constants.kJoystickTriggerThreshold)) {

					}

					if (buttonBox1.getRisingEdgeButton(1)) {

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

					}


					if (buttonBox2.getRisingEdgeButton(1)) {

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
						TeleopActionRunner.runAction(AutomatedAction.fromAction(new SetTurretPositionJoystickAction((t) -> armControlJoystick.getRawButton(2),
								(t) -> armControlJoystick.getNormalizedAxis(2, 0.1)), 300, Turret.getInstance()));
					} else if (armControlJoystick.getRisingEdgeButton(3)) {

					} else if (armControlJoystick.getRisingEdgeButton(4)) {

					} else if (armControlJoystick.getRisingEdgeButton(5)) {

					} else if (armControlJoystick.getRisingEdgeButton(6)) {

					} else if (armControlJoystick.getRisingEdgeButton(7)) {

					} else if (armControlJoystick.getRisingEdgeButton(8)) {
						//Turret Open Loop
						TeleopActionRunner.runAction(AutomatedAction.fromAction(new SetTurretOpenLoopAction((t) -> armControlJoystick.getRawButton(8),
								(t) -> -armControlJoystick.getNormalizedAxis(2, 0.1) / 3.0), 300, Turret.getInstance()));
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
