package com.team195.frc.subsystems;

import com.team195.frc.constants.CalConstants;
import com.team195.frc.constants.DeviceIDConstants;
import com.team195.frc.loops.ILooper;
import com.team195.frc.loops.Loop;
import com.team195.frc.reporters.DiagnosticMessage;
import com.team195.frc.reporters.ReflectingLogDataGenerator;
import com.team195.lib.drivers.CKSolenoid;
import com.team195.lib.drivers.TCS34725;
import com.team195.lib.drivers.motorcontrol.CKTalonFX;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.PDPBreaker;
import com.team195.lib.util.*;

import java.util.List;

public class ControlPanelManipulator extends Subsystem {
	private static ControlPanelManipulator mInstance = new ControlPanelManipulator();
	public static ControlPanelManipulator getInstance() {
		return mInstance;
	}

	private final CKSolenoid mExtenderPiston;
	private final CKTalonFX mCPMRotationMotor;
	private final CachedValue<Boolean> mCPMHasReset;
	private final TCS34725 mColorSensor;

	private ControlPanelManipulatorControlMode mCPMControlMode = ControlPanelManipulatorControlMode.POSITION;

	private PeriodicIO mPeriodicIO;
	private ReflectingLogDataGenerator<PeriodicIO> mLogDataGenerator = new ReflectingLogDataGenerator<>(PeriodicIO.class);
	private final ElapsedTimer loopTimer = new ElapsedTimer();

	private ControlPanelManipulator() {
		mPeriodicIO = new PeriodicIO();
		mColorSensor = new TCS34725();
		mExtenderPiston = new CKSolenoid(DeviceIDConstants.kControlPanelExtenderSolenoid);
		mCPMRotationMotor = new CKTalonFX(DeviceIDConstants.kCPMRotationId, false, PDPBreaker.B30A);
		mCPMHasReset = new CachedValue<>(500, (t) -> mCPMRotationMotor.hasMotorControllerReset() != DiagnosticMessage.NO_MSG);


		mCPMRotationMotor.setInverted(true);
		mCPMRotationMotor.setSensorPhase(true);
		mCPMRotationMotor.setPIDF(CalConstants.kCPMPositionKp, CalConstants.kCPMPositionKi, CalConstants.kCPMPositionKd, CalConstants.kCPMPositionKf);
		mCPMRotationMotor.setMotionParameters(CalConstants.kCPMPositionCruiseVel, CalConstants.kCPMPositionMMAccel, CalConstants.kCPMPositionSCurveStrength);
		zeroSensors();
		mCPMRotationMotor.configCurrentLimit(CalConstants.kCPMContinuousCurrentLimit, CalConstants.kCPMPeakCurrentThreshold, CalConstants.kCPMPeakCurrentThresholdExceedDuration);
		mCPMRotationMotor.setControlMode(MCControlMode.MotionMagic);
	}

	@Override
	public void stop() {
		mCPMRotationMotor.set(MCControlMode.PercentOut, 0, 0, 0);
	}

	@Override
	public boolean isSystemFaulted() {
		return false;
	}

	@Override
	public boolean runDiagnostics() {
		return false;
	}

	@Override
	public List<Object> generateReport() {
		return mLogDataGenerator.generateData(mPeriodicIO);
	}

	public synchronized void setCPMPosition(double cpmPosition) {
		mPeriodicIO.cpm_position = cpmPosition;
	}

	@Override
	public void zeroSensors() {
		mCPMRotationMotor.setEncoderPosition(0);
		setCPMPosition(0);
		if (mCPMControlMode == ControlPanelManipulatorControlMode.POSITION)
			mCPMRotationMotor.set(MCControlMode.MotionMagic, 0, 0, 0);
	}

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (ControlPanelManipulator.this) {
				zeroSensors();
			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (ControlPanelManipulator.this) {

			}
		}

		@SuppressWarnings("Duplicates")
		@Override
		public void onLoop(double timestamp) {
			synchronized (ControlPanelManipulator.this) {
				switch (mCPMControlMode) {
					case POSITION:
						break;
					case OPEN_LOOP:
						break;
					case DISABLED:
						break;
				}
			}
		}

		@Override
		public void onStop(double timestamp) {
			stop();
		}

		@Override
		public String getName() {
			return "Turret";
		}
	};

	public enum ControlPanelManipulatorControlMode {
		POSITION,
		OPEN_LOOP,
		DISABLED;
	}

	@Override
	public synchronized void readPeriodicInputs() {
		loopTimer.start();
		mPeriodicIO.cpm_position = mCPMRotationMotor.getPosition();
		mPeriodicIO.cpm_velocity = mCPMRotationMotor.getVelocity();
		mPeriodicIO.cpm_reset = mCPMHasReset.getValue();
		mPeriodicIO.cpm_current_color = mColorSensor.getColorOutput();
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		mPeriodicIO.cpm_loop_time = loopTimer.hasElapsed();
	}

	@SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {
		//Making members public here will automatically add them to logs
		// INPUTS
		public double cpm_position;
		public double cpm_velocity;
		public double cpm_setpoint;
		public boolean cpm_reset;
		public ColorOutput cpm_current_color = ColorOutput.NONE;
		public ColorOutput cpm_desired_color = ColorOutput.NONE;

		// Outputs
		public double cpm_loop_time;
	}
}
