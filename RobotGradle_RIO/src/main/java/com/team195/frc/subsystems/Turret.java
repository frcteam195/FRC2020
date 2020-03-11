package com.team195.frc.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.team195.frc.constants.CalConstants;
import com.team195.frc.RobotState;
import com.team195.frc.constants.DeviceIDConstants;
import com.team195.frc.constants.TargetingConstants;
import com.team195.frc.loops.ILooper;
import com.team195.frc.loops.Loop;
import com.team195.frc.reporters.DiagnosticMessage;
import com.team195.frc.reporters.ReflectingLogDataGenerator;
import com.team195.lib.drivers.motorcontrol.*;
import com.team195.lib.util.*;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import java.util.List;

@SuppressWarnings("FieldCanBeLocal")
public class Turret extends Subsystem implements InterferenceSystem {

	private static Turret mInstance = new Turret();

	private final VisionTracker mVisionTracker = VisionTracker.getInstance();

	private final CKTalonFX mTurretRotationMotor;
	private final CKTalonFX mLeftShooterMotorMaster;
	private final CKTalonFX mRightShooterMotorSlave;
	private final CKTalonFX mTurretHoodMotor;

	private TurretControlMode mTurretControlMode = TurretControlMode.POSITION;
	private HoodControlMode mHoodControlMode = HoodControlMode.POSITION;
	private ShooterControlMode mShooterControlMode = ShooterControlMode.VELOCITY;

	private PeriodicIO mPeriodicIO;
	private ReflectingLogDataGenerator<PeriodicIO> mLogDataGenerator = new ReflectingLogDataGenerator<>(PeriodicIO.class);

	private final CachedValue<Boolean> mTurretMasterHasReset;
	private final CachedValue<Boolean> mHoodHasReset;

	private final ElapsedTimer loopTimer = new ElapsedTimer();
	private final ElapsedTimer shooter_dt = new ElapsedTimer();

	private Turret() {
		mPeriodicIO = new PeriodicIO();

		//Encoder on 50:1
		mTurretRotationMotor = new CKTalonFX(DeviceIDConstants.kTurretMotorId, false, PDPBreaker.B30A);
		mTurretRotationMotor.setInverted(true);
		mTurretRotationMotor.setGearRatioToOutputMechanism(CalConstants.kTurretOverallGearRatioDeg);
		mTurretRotationMotor.setPIDF(CalConstants.kTurretPositionKp, CalConstants.kTurretPositionKi, CalConstants.kTurretPositionKd, CalConstants.kTurretPositionKf);
		mTurretRotationMotor.setMotionParameters(CalConstants.kTurretPositionCruiseVel, CalConstants.kTurretPositionMMAccel, CalConstants.kTurretPositionSCurveStrength);
		mTurretRotationMotor.configForwardSoftLimitThreshold(CalConstants.kTurretMinDegrees);
		mTurretRotationMotor.configForwardSoftLimitEnable(true);
		mTurretRotationMotor.configReverseSoftLimitThreshold(CalConstants.kTurretMaxDegrees);
		mTurretRotationMotor.configReverseSoftLimitEnable(true);
		mTurretRotationMotor.configCurrentLimit(CalConstants.kTurretContinuousCurrentLimit, CalConstants.kTurretPeakCurrentThreshold, CalConstants.kTurretPeakCurrentThresholdExceedDuration);
//		mTurretRotationMotor.setControlMode(MCControlMode.MotionMagic);

		//Diameter = 17.75
		//Radius = 8.875
		//Teeth = 353
		mTurretHoodMotor = new CKTalonFX(DeviceIDConstants.kHoodMotorId, false, PDPBreaker.B30A);
		mTurretHoodMotor.setInverted(true);
		mTurretHoodMotor.setGearRatioToOutputMechanism(CalConstants.kTurretHoodOverallGearRatioDeg);
		mTurretHoodMotor.setPIDF(CalConstants.kTurretHoodKp, CalConstants.kTurretHoodKi, CalConstants.kTurretHoodKd, CalConstants.kTurretHoodKf);
		mTurretHoodMotor.setMotionParameters(CalConstants.kTurretHoodCruiseVel, CalConstants.kTurretHoodMMAccel, CalConstants.kTurretHoodSCurveStrength);
		mTurretHoodMotor.configForwardSoftLimitThreshold(CalConstants.kTurretHoodMinDegrees);
		mTurretHoodMotor.configForwardSoftLimitEnable(true);
		mTurretHoodMotor.configReverseSoftLimitThreshold(CalConstants.kTurretHoodMaxDegrees);
		mTurretHoodMotor.configReverseSoftLimitEnable(true);
		mTurretHoodMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, CalConstants.kTurretHoodContinuousStatorCurrentLimit, 0, 0));
		mTurretHoodMotor.setControlMode(MCControlMode.MotionMagic);

		mLeftShooterMotorMaster = new CKTalonFX(DeviceIDConstants.kLeftShooterMotorId, false, PDPBreaker.B30A);
		mRightShooterMotorSlave = new CKTalonFX(DeviceIDConstants.kRightShooterMotorId, mLeftShooterMotorMaster, PDPBreaker.B30A);
		mRightShooterMotorSlave.setInverted(true);
		mLeftShooterMotorMaster.configMasterAndSlaves((t) -> {
			t.setBrakeCoastMode(MCNeutralMode.Coast);
			t.disableCurrentLimit();
			t.configPeakOutputReverse(-0.4);
			return ErrorCode.OK;
		});
		mLeftShooterMotorMaster.setPIDF(CalConstants.kShooterWheelKp, CalConstants.kShooterWheelKi, CalConstants.kShooterWheelKd, CalConstants.kShooterWheelKf);

//		mLeftShooterMotorMaster.setControlMode(MCControlMode.Velocity);
//		TuneablePIDOSC x;
//		try {
//			x = new TuneablePIDOSC("Turret", 5804, true, mLeftShooterMotorMaster);
//		} catch (Exception ignored) {
//
//		}

		mTurretMasterHasReset = new CachedValue<>(500, (t) -> mTurretRotationMotor.hasMotorControllerReset() != DiagnosticMessage.NO_MSG);
		mHoodHasReset = new CachedValue<>(500, (t) -> mTurretHoodMotor.hasMotorControllerReset() != DiagnosticMessage.NO_MSG);

		zeroSensors();
	}

	public static Turret getInstance() {
		return mInstance;
	}

	@Override
	public void stop() {
		mTurretRotationMotor.set(MCControlMode.PercentOut, 0, 0, 0);
		mTurretHoodMotor.set(MCControlMode.PercentOut, 0, 0, 0);
		mLeftShooterMotorMaster.set(MCControlMode.PercentOut, 0, 0, 0);
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
		mPeriodicIO.turret_loop_time += loopTimer.hasElapsed();
		return mTmpHandle;
	}
	private List<Object> mTmpHandle;

	@Override
	public void zeroSensors() {
		mTurretRotationMotor.setEncoderPosition(0);
		mTurretHoodMotor.setEncoderPosition(0);
		setTurretPosition(0);
		setHoodPosition(0);
		if (mTurretControlMode == TurretControlMode.POSITION)
			mTurretRotationMotor.set(MCControlMode.MotionMagic, 0, 0, 0);
		if (mHoodControlMode == HoodControlMode.POSITION)
			mTurretHoodMotor.set(MCControlMode.MotionMagic, 0, 0, 0);
	}

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (Turret.this) {
				zeroSensors();
			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (Turret.this) {
				shooter_dt.start();
			}
		}

		@SuppressWarnings("Duplicates")
		@Override
		public void onLoop(double timestamp) {
			loopTimer.start();
			synchronized (Turret.this) {
				switch (mTurretControlMode) {
					case VISION_TRACK:
						if (mVisionTracker.isTargetFound()) {
							mPeriodicIO.turret_setpoint_deg = mPeriodicIO.turret_position_deg + mVisionTracker.getTargetHorizAngleDev();
						}
						//Fall through on purpose to set position -> no break;
					case POSITION:
						mTurretRotationMotor.set(MCControlMode.MotionMagic, mPeriodicIO.turret_setpoint_deg, 0, 0);
						break;
					case GIMBAL:
						Pose2d robotPose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
						Pose2d latestFieldToTurret = getLatestFieldToTurretPose();
						Translation2d turretToTarget = TargetingConstants.fieldToOuterTarget.getTranslation().translateBy(latestFieldToTurret.getTranslation().inverse());
						Rotation2d robotCentricSetpoint = turretToTarget.direction().rotateBy(robotPose.getRotation().inverse());
						double rawDegreesOut = TurretHelper.calculateSetpointForRobotCentricRotation(mPeriodicIO.turret_position_deg / 360.0, robotCentricSetpoint, CalConstants.kTurretMinDegrees, CalConstants.kTurretMinDegrees);
						double arbFF = 0;//-mTurretRotationMotor.getArbFFFromVelocity(-Drive.getInstance().getAngularVelocity() / (2 * Math.PI) * 60.0, mPeriodicIO.turret_velocity, mPeriodicIO.turret_loop_time);
						mTurretRotationMotor.set(MCControlMode.MotionMagic, rawDegreesOut, 0, arbFF);
						break;
					case OPEN_LOOP:
						mTurretRotationMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mPeriodicIO.turret_setpoint_deg, -1), 1), 0, 0);
						break;
					default:
						mTurretRotationMotor.set(MCControlMode.Disabled, 0, 0, 0);
						break;
				}
			}

			switch (mHoodControlMode) {
				case OPEN_LOOP:
					mTurretHoodMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mPeriodicIO.hood_setpoint_deg, -1), 1), 0, 0);
					break;
				case POSITION:
					mTurretHoodMotor.set(MCControlMode.MotionMagic, mPeriodicIO.hood_setpoint_deg, 0, 0);
					break;
				case DISABLED:
					mTurretHoodMotor.set(MCControlMode.Disabled, 0, 0, 0);
					break;
			}

			switch (mShooterControlMode) {
				case OPEN_LOOP:
					mLeftShooterMotorMaster.set(MCControlMode.PercentOut, Math.min(Math.max(mPeriodicIO.shooter_setpoint_rpm, -1), 1), 0, 0);
					break;
				case VELOCITY:
					mLeftShooterMotorMaster.set(MCControlMode.Velocity, getAccelFilteredShooterVelocity(), 0, 0);
					break;
				case DISABLED:
					mLeftShooterMotorMaster.set(MCControlMode.Disabled, 0, 0, 0);
					break;
			}
			mPeriodicIO.turret_loop_time += loopTimer.hasElapsed();
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

	public double getAccelFilteredShooterVelocity() {
		double diffErr = mPeriodicIO.shooter_setpoint_rpm - mPeriodicIO.shooter_velocity_rpm;
		if (CalConstants.kShooterWheelMaxAccel == 0)
			return mPeriodicIO.shooter_setpoint_rpm;
		double outputVel = mPeriodicIO.shooter_setpoint_rpm + Math.min(Math.abs(diffErr), (CalConstants.kShooterWheelMaxAccel * shooter_dt.hasElapsed())) * Math.copySign(1.0, diffErr);
		shooter_dt.start();
		return outputVel;
	}

	public synchronized void setTurretPosition(double turretPosition) {
		mPeriodicIO.turret_setpoint_deg = turretPosition;
	}

	public synchronized void setHoodPosition(double hoodPosition) {
		mPeriodicIO.hood_setpoint_deg = hoodPosition;
	}

	public synchronized void setShooterVelocity(double shooterVelocity) {
		mPeriodicIO.shooter_setpoint_rpm = shooterVelocity;
	}

	public synchronized void setTurretControlMode(TurretControlMode turretControlMode) {
		if (mTurretControlMode != turretControlMode)
			mTurretControlMode = turretControlMode;
	}

	public synchronized void setHoodControlMode(HoodControlMode hoodControlMode) {
		if (mHoodControlMode != hoodControlMode)
			mHoodControlMode = hoodControlMode;
	}

	public synchronized void setShooterControlMode(ShooterControlMode shooterControlMode) {
		if (mShooterControlMode != shooterControlMode)
			mShooterControlMode = shooterControlMode;
	}

	public boolean isTurretAtSetpoint(double posDelta) {
		return Math.abs(mPeriodicIO.turret_setpoint_deg - mPeriodicIO.turret_position_deg) < Math.abs(posDelta);
	}

	@Override
	public double getPosition() {
		return mPeriodicIO.turret_position_deg;
	}

	@Override
	public double getSetpoint() {
		return mPeriodicIO.turret_setpoint_deg;
	}

	public double getShooterVelocity() {
		return mPeriodicIO.shooter_velocity_rpm;
	}

	public Pose2d getLatestFieldToTurretPose() {
		return RobotState.getInstance().getLatestFieldToVehicle().getValue().transformBy(getLatestVehicleToTurretPose());
	}

	public Pose2d getLatestVehicleToTurretPose() {
		return new Pose2d(CalConstants.kVehicleToTurret.getTranslation(),
							Rotation2d.fromDegrees(mPeriodicIO.turret_position_deg));
	}

	public enum TurretControlMode {
		POSITION,
		OPEN_LOOP,
		AUTO_TRACK,
		VISION_TRACK,
		GIMBAL,
		DISABLED;
	}

	public enum HoodControlMode {
		OPEN_LOOP,
		POSITION,
		DISABLED;
	}

	public enum ShooterControlMode {
		OPEN_LOOP,
		VELOCITY,
		DISABLED;
	}

	@Override
	public synchronized void readPeriodicInputs() {
		loopTimer.start();
		mPeriodicIO.turret_position_deg = mTurretRotationMotor.getPosition();
		mPeriodicIO.turret_velocity_dps = mTurretRotationMotor.getVelocity();
		mPeriodicIO.turret_reset = mTurretMasterHasReset.getValue();
		mPeriodicIO.turret_loop_time = loopTimer.hasElapsed();
		mPeriodicIO.shooter_velocity_rpm = mLeftShooterMotorMaster.getVelocity();
		mPeriodicIO.hood_position_deg = mTurretHoodMotor.getPosition();
		mPeriodicIO.hood_reset = mHoodHasReset.getValue();
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		loopTimer.start();
		mPeriodicIO.turret_loop_time += loopTimer.hasElapsed();
	}

	@SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {
		//Making members public here will automatically add them to logs
		// INPUTS
		public double turret_position_deg;
		public double turret_velocity_dps;
		public double turret_setpoint_deg;
		public boolean turret_reset;

		public double shooter_velocity_rpm;
		public double shooter_setpoint_rpm;

		public double hood_position_deg;
		public double hood_setpoint_deg;
		public boolean hood_reset;

		// Outputs
		public double turret_loop_time;
	}
}