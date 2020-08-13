package com.team195.frc.controllers;

import com.team195.frc.constants.Constants;
import com.team195.frc.auto.actions.*;
import com.team195.frc.auto.autonomy.AutomatedAction;
import com.team195.frc.controlboard.CKControlsConsumer;
import com.team195.frc.reporters.ConsoleReporter;
import com.team195.frc.subsystems.*;
import com.team195.lib.drivers.dashjoy.CKDashJoystick;
import com.team195.lib.util.TeleopActionRunner;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.CrashTrackingRunnable;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.Notifier;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

@SuppressWarnings("DuplicatedCode")
public class DriveControls {
	private static DriveControls mInstance = null;
	public synchronized static DriveControls getInstance() {
		if (mInstance == null)
			mInstance = new DriveControls();

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

	//A list of functions that return true if they activated this run
	private final ArrayList<BooleanSupplier> mControlFunctions = new ArrayList<>();

	private final CrashTrackingRunnable mHIDRunnable = new CrashTrackingRunnable() {
		@Override
		public void runCrashTracked() {
			synchronized (taskRunningLock_) {
				if (firstRun) {
					Thread.currentThread().setName("DriveControls");
					Thread.currentThread().setPriority(Constants.kControllerThreadPriority);
					firstRun = false;
				}
				try {
					for (BooleanSupplier b : mControlFunctions) {
						b.getAsBoolean();   //TODO: Generate report of active functions
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

	private DriveControls() {
		mHIDNotifier = new Notifier(mHIDRunnable);
		registerControls();
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

	private void registerButtonPressControl(CKDashJoystick joystick, int button, CKControlsConsumer controlFunction) {
		mControlFunctions.add(() -> {
			if (joystick.getRisingEdgeButton(button)) {
				controlFunction.accept(joystick, button);
				return true;
			}
			return false;
		});
	}

	private void registerControls() {
		//Add Main Drive Control
		mControlFunctions.add(() -> {
			if (mDrive.getDriveControlState() == Drive.DriveControlState.OPEN_LOOP) {
				mThrottle = -driveJoystick.getNormalizedAxis(1, Constants.kJoystickDeadband);
				mTurn = driveJoystick.getNormalizedAxis(2, Constants.kJoystickDeadband);
				mDrive.setBrakeMode(driveJoystick.getTriggerPressed(3, Constants.kJoystickTriggerThreshold) || VisionTracker.getInstance().isVisionEnabled());
				mDriveSignalOutput.set(Math.max(Math.min(mThrottle + mTurn, 1), -1), Math.max(Math.min(mThrottle - mTurn, 1), -1));
				mDrive.setOpenLoop(mDriveSignalOutput);
				return true;
			}
			return false;
		});
/*
		//Shooter On
		registerButtonPressControl(buttonBox1, 1, (j, b) -> {
//			Turret.getInstance().setHoodPosition(80);
			Turret.getInstance().setShooterVelocity(5400);
			Turret.getInstance().setShooterControlMode(Turret.ShooterControlMode.VELOCITY);
		});

		registerButtonPressControl(buttonBox1, 2, (j, b) -> {
//			Turret.getInstance().setHoodPosition(80);
			Turret.getInstance().setShooterControlMode(Turret.ShooterControlMode.DISABLED);
		});

		//Intake On Button
		registerButtonPressControl(buttonBox1, 3, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
					new SetIntakeAction(false, () -> j.getRawButton(b)), 300));
		});

		//Feeder On Button
		registerButtonPressControl(buttonBox1, 4, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
					new SetFeederAction(false, () -> j.getRawButton(b)), 300));
		});


		//Blink LEDs Button
		registerButtonPressControl(buttonBox2, 14, (j, b) -> LEDController.getInstance().setRequestedState(LEDController.LEDState.BLINK));

		//Request Game Piece LEDs Button
		registerButtonPressControl(armControlJoystick, 1, (j, b) -> {
			LEDController.getInstance().setLEDColor(Constants.kRequestGamePieceColor);
			LEDController.getInstance().setRequestedState(LEDController.LEDState.BLINK);
		});

		//Open Loop Turret Control
		registerButtonPressControl(armControlJoystick, 8, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(new SetTurretOpenLoopAction(() -> j.getRawButton(b),
					() -> -armControlJoystick.getNormalizedAxis(2, 0.1) / 3.0), 300, Turret.getInstance()));
		});

		//Rehome Turret
		registerButtonPressControl(armControlJoystick, 11, (j, b) -> {
			Turret.getInstance().zeroSensors();
			Turret.getInstance().setTurretControlMode(Turret.TurretControlMode.POSITION);
		});
 */
	}
}
