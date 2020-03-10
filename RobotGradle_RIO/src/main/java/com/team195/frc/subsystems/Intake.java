package com.team195.frc.subsystems;

import com.team195.frc.constants.CalConstants;
import com.team195.frc.constants.DeviceIDConstants;
import com.team195.frc.loops.ILooper;
import com.team195.frc.loops.Loop;
import com.team195.frc.reporters.ReflectingLogDataGenerator;
import com.team195.lib.drivers.motorcontrol.*;
import com.team195.lib.util.*;

import java.util.List;

@SuppressWarnings("FieldCanBeLocal")
public class Intake extends Subsystem {

	private static Intake mInstance = new Intake();

	private final CKTalonFX mIntakeMotor;
	private final CKTalonFX mFeederMotor;

	private IntakeControlMode mIntakeControlMode = IntakeControlMode.OFF;
	private FeederControlMode mFeederControlMode = FeederControlMode.OFF;

	private PeriodicIO mPeriodicIO;
	private ReflectingLogDataGenerator<PeriodicIO> mLogDataGenerator = new ReflectingLogDataGenerator<>(PeriodicIO.class);

	private final ElapsedTimer loopTimer = new ElapsedTimer();

	private Intake() {
		mPeriodicIO = new PeriodicIO();

		mIntakeMotor = new CKTalonFX(DeviceIDConstants.kIntakeMotorId, false, PDPBreaker.B30A);
		mIntakeMotor.setInverted(true);
		mIntakeMotor.configCurrentLimit(CalConstants.kIntakeContinuousCurrentLimit, CalConstants.kIntakePeakCurrentThreshold, CalConstants.kIntakePeakCurrentThresholdExceedDuration);

		mFeederMotor = new CKTalonFX(DeviceIDConstants.kFeederMotorId, false, PDPBreaker.B30A);
		mFeederMotor.setInverted(true);
		mFeederMotor.configCurrentLimit(CalConstants.kFeederContinuousCurrentLimit, CalConstants.kFeederPeakCurrentThreshold, CalConstants.kFeederPeakCurrentThresholdExceedDuration);

		zeroSensors();
	}

	public static Intake getInstance() {
		return mInstance;
	}

	@Override
	public void stop() {
		mIntakeMotor.set(MCControlMode.PercentOut, 0, 0, 0);
		mFeederMotor.set(MCControlMode.PercentOut, 0, 0, 0);
	}

	@Override
	public synchronized boolean isSystemFaulted() {
		return false;
	}

	@Override
	public boolean runDiagnostics() {
		return false;
	}

	@Override
	public synchronized List<Object> generateReport() {
		loopTimer.start();
		mTmpHandle = mLogDataGenerator.generateData(mPeriodicIO);
		mPeriodicIO.intake_loop_time += loopTimer.hasElapsed();
		return mTmpHandle;
	}
	private List<Object> mTmpHandle;

	@Override
	public void zeroSensors() {

	}

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (Intake.this) {
				zeroSensors();
			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (Intake.this) {

			}
		}

		@SuppressWarnings("Duplicates")
		@Override
		public void onLoop(double timestamp) {
			loopTimer.start();
			synchronized (Intake.this) {
				switch (mIntakeControlMode) {
					case FORWARD:
						mIntakeMotor.set(MCControlMode.PercentOut, 1, 0, 0);
						break;
					case REVERSE:
						mIntakeMotor.set(MCControlMode.PercentOut, -1, 0, 0);
						break;
					case OFF:
						mIntakeMotor.set(MCControlMode.Disabled, 0, 0, 0);
						break;
				}
			}

			switch (mFeederControlMode) {
				case FORWARD:
					mFeederMotor.set(MCControlMode.PercentOut, 1, 0, 0);
					break;
				case REVERSE:
					mFeederMotor.set(MCControlMode.PercentOut, -1, 0, 0);
					break;
				case OFF:
					mFeederMotor.set(MCControlMode.Disabled, 0, 0, 0);
					break;
			}

			mPeriodicIO.intake_loop_time += loopTimer.hasElapsed();
		}

		@Override
		public void onStop(double timestamp) {
			stop();
		}

		@Override
		public String getName() {
			return "Intake";
		}
	};

	public synchronized void setIntakeControlMode(IntakeControlMode intakeControlMode) {
		if (mIntakeControlMode != intakeControlMode)
			mIntakeControlMode = intakeControlMode;
	}

	public synchronized void setFeederControlMode(FeederControlMode feederControlMode) {
		if (mFeederControlMode != feederControlMode)
			mFeederControlMode = feederControlMode;
	}


	public enum IntakeControlMode {
		FORWARD,
		REVERSE,
		OFF
	}

	public enum FeederControlMode {
		FORWARD,
		REVERSE,
		OFF
	}

	@Override
	public synchronized void readPeriodicInputs() {
		loopTimer.start();
		mPeriodicIO.intake_loop_time = loopTimer.hasElapsed();
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		loopTimer.start();
		mPeriodicIO.intake_loop_time += loopTimer.hasElapsed();
	}

	@SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {
		// Outputs
		public double intake_loop_time;
	}
}