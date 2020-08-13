package com.team195.frc;

import com.team195.frc.auto.AutoModeExecutor;
import com.team195.frc.auto.autonomy.AutomatedActions;
import com.team195.frc.constants.Constants;
import com.team195.frc.constants.TestConstants;
import com.team195.frc.controllers.DriveControls;
import com.team195.frc.controllers.LEDController;
import com.team195.frc.loops.Looper;
import com.team195.frc.monitors.ConnectionMonitor;
import com.team195.frc.paths.TrajectoryGenerator;
import com.team195.frc.reporters.ConsoleReporter;
import com.team195.frc.reporters.MessageLevel;
import com.team195.frc.subsystems.*;
import com.team195.lib.util.TeleopActionRunner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

@SuppressWarnings({"ResultOfMethodCallIgnored", "FieldCanBeLocal", "WeakerAccess"})
public class Robot extends TimedRobot {
	private Looper mEnabledLooper = new Looper("EnabledLooper");
	private Looper mDisabledLooper = new Looper("DisabledLooper");

	private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

	private SubsystemManager mSubsystemManager;

	private Drive mDrive;
	private LEDController mLED;
	private Infrastructure mInfrastructure;
	public static final AutoModeExecutor mAutoModeExecutor = new AutoModeExecutor();

	private DriveControls mDriveControls;

	public Robot() {
		CrashTracker.logRobotConstruction();
		Thread.currentThread().setPriority(Constants.kRobotThreadPriority);
		Thread.currentThread().setName("RobotMainThread");
	}

	@Override
	public void robotInit() {
		try {
			CrashTracker.logRobotInit();
			LiveWindow.disableAllTelemetry();
			Shuffleboard.disableActuatorWidgets();

			mDrive = Drive.getInstance();
			mLED = LEDController.getInstance();
			mInfrastructure = Infrastructure.getInstance();
			mDriveControls = DriveControls.getInstance();

			mSubsystemManager = SubsystemManager.getInstance(
					RobotStateEstimator.getInstance(),
					//VisionTracker.getInstance(),
					Drive.getInstance()//,
					//Turret.getInstance(),
					//Intake.getInstance(),
//					ControlPanelManipulator.getInstance(),
					//Infrastructure.getInstance()
			);

			ConsoleReporter.getInstance();
			ConsoleReporter.setReportingLevel(MessageLevel.INFO);

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			TrajectoryGenerator.getInstance().generateTrajectories();

			mLED.start();
			mLED.setRequestedState(LEDController.LEDState.BLINK);

			ConnectionMonitor.getInstance();

			Drive.getInstance().zeroSensors();
			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

			System.gc();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	private int count = 0;

	@Override
	public void robotPeriodic() {
		if (count++ % 20 == 0) {
//			ConsoleReporter.report(mEnabledLooper.generateReport());
//			ConsoleReporter.report(mDisabledLooper.generateReport());
//			ConsoleReporter.report("LeftDrivePos:" + Drive.getInstance().getLeftEncoderDistance() + ", RigthDrivePos:" + Drive.getInstance().getRightEncoderDistance());
//			ConsoleReporter.report("GyroRoll:" + Drive.getInstance().getRoll());
//			ConsoleReporter.report(mAutoModeSelector.getAutoMode().getClass().getSimpleName().toString());
//			ConsoleReporter.report("GyroDeg:" + Drive.getInstance().getRawYaw());
//			ConsoleReporter.report("Skew: " + VisionTracker.getInstance().getTargetSkew());

//			System.out.println("Drive Left Output: " + mDrive.getLeftDemand());
//			System.out.println("Drive Right Output: " + mDrive.getRightDemand());
		}
	}

	@Override
	public void autonomousInit() {
		try {
			CrashTracker.logAutoInit();

			mDisabledLooper.stop();
			mInfrastructure.setIsDuringAuto(true);
			Drive.getInstance().zeroSensors();
			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
			mDrive.setBrakeMode(true);
			mDrive.forceBrakeModeUpdate();
			mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
			mEnabledLooper.start();
			mDriveControls.start();

			if (mAutoModeExecutor.isSet()) {
				ConsoleReporter.report("Start Auto Mode");
				mAutoModeExecutor.start();
			}
		} catch (Exception ex) {
			CrashTracker.logThrowableCrash(ex);
		}
		catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void autonomousPeriodic() {
		try {

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopInit() {
		try {
			CrashTracker.logTeleopInit();
			mDisabledLooper.stop();

			if (mAutoModeExecutor.isSet()) {
				try {
					mAutoModeExecutor.stop();
				} catch (Exception ignored) {

				}
			}

			mInfrastructure.setIsDuringAuto(false);

			mEnabledLooper.start();
			mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
			mDrive.setOpenLoop(DriveSignal.NEUTRAL);
			mDrive.setBrakeMode(false);
			mDrive.forceBrakeModeUpdate();
			mDriveControls.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		try {

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
		try {
			ConsoleReporter.report("Starting systems check!", MessageLevel.INFO);

			mDisabledLooper.stop();
			mEnabledLooper.stop();
			mDriveControls.stop();

			if (TestConstants.RUN_INDIVIDUAL_TESTS) {
				if (mSubsystemManager.checkSystemsPassDiagnostics())
					ConsoleReporter.report("System passed test!", MessageLevel.DEFCON1);
				else
					ConsoleReporter.report("System failed test!", MessageLevel.DEFCON1);
			}
			else {
				mEnabledLooper.start();
				TeleopActionRunner.runAction(AutomatedActions.fullyAutomatedTest());
			}

			//Crash the JVM and force a reset
			System.exit(1);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void disabledInit() {
		try {
			CrashTracker.logDisabledInit();

			mDriveControls.stop();
			mEnabledLooper.stop();
			mDrive.setBrakeMode(false);
			mDrive.forceBrakeModeUpdate();
			if (mAutoModeExecutor.isSet()) {
				try {
					mAutoModeExecutor.stop();
				} catch (Exception ignored) {

				}
			}
			mDisabledLooper.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		try {
			mDrive.setBrakeMode(false);
			mAutoModeExecutor.reset();
//			mAutoModeExecutor.setAutoMode(mAutoModeSelector.getAutoMode());
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

}