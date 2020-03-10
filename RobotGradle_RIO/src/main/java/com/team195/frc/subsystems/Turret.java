package com.team195.frc.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.team195.frc.constants.CalConstants;
import com.team195.frc.RobotState;
import com.team195.frc.constants.DeviceIDConstants;
import com.team195.frc.constants.TargetingConstants;
import com.team195.frc.loops.ILooper;
import com.team195.frc.loops.Loop;
import com.team195.frc.paths.TrajectoryGenerator;
import com.team195.frc.reporters.DiagnosticMessage;
import com.team195.frc.reporters.ReflectingLogDataGenerator;
import com.team195.frc.subsystems.positions.TurretPositions;
import com.team195.lib.drivers.motorcontrol.*;
import com.team195.lib.util.*;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import java.util.List;

public class Turret extends Subsystem implements InterferenceSystem {

	private static Turret mInstance = new Turret();

	private final VisionTracker mVisionTracker = VisionTracker.getInstance();

	private final CKTalonFX mTurretRotationMotor;
	private final CKTalonFX mLeftShooterMotorMaster;
	private final CKTalonFX mRightShooterMotorSlave;
	private final CKTalonFX mTurretHoodMotor;

	private TurretControlMode mTurretControlMode = TurretControlMode.POSITION;
	private HoodControlMode mHoodControlMode = HoodControlMode.POSITION;
	private ShooterControlMode mShooterControlMode = ShooterControlMode.OPEN_LOOP;

	private PeriodicIO mPeriodicIO;
	private ReflectingLogDataGenerator<PeriodicIO> mLogDataGenerator = new ReflectingLogDataGenerator<>(PeriodicIO.class);

	private final CachedValue<Boolean> mTurretMasterHasReset;

	private final ElapsedTimer loopTimer = new ElapsedTimer();

	private Turret() {
		mPeriodicIO = new PeriodicIO();

		//Encoder on 50:1
		mTurretRotationMotor = new CKTalonFX(DeviceIDConstants.kTurretMotorId, false, PDPBreaker.B30A);
		mTurretRotationMotor.setInverted(true);
		mTurretRotationMotor.setGearRatioToOutputMechanism(CalConstants.kTurretGearRatioMotorToTurretGear * (CalConstants.kTurretLargeGearTeeth / CalConstants.kTurretSmallGearTeeth));
		//mTurretRotationMotor.setSensorPhase(true);
		mTurretRotationMotor.setPIDF(CalConstants.kTurretPositionKp, CalConstants.kTurretPositionKi, CalConstants.kTurretPositionKd, CalConstants.kTurretPositionKf);
		mTurretRotationMotor.setMotionParameters(CalConstants.kTurretPositionCruiseVel, CalConstants.kTurretPositionMMAccel, CalConstants.kTurretPositionSCurveStrength);
//		mTurretRotationMotor.configForwardSoftLimitThreshold(CalConstants.kTurretForwardSoftLimit);
//		mTurretRotationMotor.configForwardSoftLimitEnable(true);
//		mTurretRotationMotor.configReverseSoftLimitThreshold(CalConstants.kTurretReverseSoftLimit);
//		mTurretRotationMotor.configReverseSoftLimitEnable(true);
//		mTurretRotationMotor.configCurrentLimit(CalConstants.kTurretContinuousCurrentLimit, CalConstants.kTurretPeakCurrentThreshold, CalConstants.kTurretPeakCurrentThresholdExceedDuration);
//		mTurretRotationMotor.setControlMode(MCControlMode.MotionMagic);
		mTurretRotationMotor.setControlMode(MCControlMode.PercentOut);


		//Diameter = 17.75
		//Radius = 8.875
		mTurretHoodMotor = new CKTalonFX(DeviceIDConstants.kHoodMotorId, false, PDPBreaker.B30A);
		mTurretHoodMotor.setInverted(true);
		mTurretHoodMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 0, 0));
		mTurretHoodMotor.setPIDF(CalConstants.kTurretHoodKp, CalConstants.kTurretHoodKi, CalConstants.kTurretHoodKd, CalConstants.kTurretHoodKf);
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

		mLeftShooterMotorMaster.setControlMode(MCControlMode.Velocity);
		TuneablePIDOSC x;
		try {
			x = new TuneablePIDOSC("Turret", 5804, true, mLeftShooterMotorMaster);
		} catch (Exception ignored) {

		}

		mTurretMasterHasReset = new CachedValue<>(500, (t) -> mTurretRotationMotor.hasMotorControllerReset() != DiagnosticMessage.NO_MSG);

		zeroSensors();
	}

	public static Turret getInstance() {
		return mInstance;
	}

	@Override
	public void stop() {
		mTurretRotationMotor.set(MCControlMode.PercentOut, 0, 0, 0);
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

			}
		}



		@SuppressWarnings("Duplicates")
		@Override
		public void onLoop(double timestamp) {
			loopTimer.start();
			synchronized (Turret.this) {
				Pose2d robotCurrentPos = RobotState.getInstance().getLatestFieldToVehicle().getValue();
				switch (mTurretControlMode) {
					case AUTO_TRACK:
						Translation2d currentRocketTarget;

						if (robotCurrentPos.getTranslation().x() > 0)
							//Track Left Rocket
							currentRocketTarget = TrajectoryGenerator.kLeftRocketPose.getTranslation();
						else
							//Track Right Rocket
							currentRocketTarget = TrajectoryGenerator.kRightRocketPose.getTranslation();

						double desiredTurretAngleDeg = Math.toDegrees(Math.atan2((currentRocketTarget.y() - robotCurrentPos.getTranslation().y()),
								(currentRocketTarget.x() - robotCurrentPos.getTranslation().x()))) - robotCurrentPos.getRotation().getDegrees();

						mPeriodicIO.turret_setpoint = TurretHelper.convertTurretDegreesToRotations(desiredTurretAngleDeg);
						//Fall through on purpose to set position -> no break;
					case VISION_TRACK:
						//Preempts Auto Track
						if (mVisionTracker.isTargetFound())
							mPeriodicIO.turret_setpoint = TurretHelper.convertTurretDegreesToRotations(mVisionTracker.getTargetHorizAngleDev());
						//Fall through on purpose to set position -> no break;
					case POSITION:
						mTurretRotationMotor.set(MCControlMode.MotionMagic, mPeriodicIO.turret_setpoint, 0, 0);
						break;
					case GIMBAL:
						Pose2d robotPose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
						Pose2d latestFieldToTurret = getLatestFieldToTurretPose();
						Translation2d turretToTarget = TargetingConstants.fieldToOuterTarget.getTranslation().translateBy(latestFieldToTurret.getTranslation().inverse());
						Rotation2d robotCentricSetpoint = turretToTarget.direction().rotateBy(robotPose.getRotation().inverse());
						double rawDegreesOut = TurretHelper.calculateSetpointForRobotCentricRotation(mPeriodicIO.turret_position / 360.0, robotCentricSetpoint, CalConstants.kTurretMinDegrees, CalConstants.kTurretMinDegrees);
						double arbFF = 0;//-mTurretRotationMotor.getArbFFFromVelocity(-Drive.getInstance().getAngularVelocity() / (2 * Math.PI) * 60.0, mPeriodicIO.turret_velocity, mPeriodicIO.turret_loop_time);
						mTurretRotationMotor.set(MCControlMode.MotionMagic, TurretHelper.convertTurretDegreesToRotations(rawDegreesOut), 0, arbFF);
						break;
					case OPEN_LOOP:
						mTurretRotationMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mPeriodicIO.turret_setpoint, -1), 1), 0, 0);
						break;
					default:
						mTurretRotationMotor.set(MCControlMode.Disabled, 0, 0, 0);
						break;
				}
			}

			switch (mHoodControlMode) {
				case OPEN_LOOP:
					break;
				case POSITION:
					mTurretHoodMotor.set(MCControlMode.MotionMagic, mPeriodicIO.hood_position, 0, 0);
					break;
				case DISABLED:
					break;
			}

			switch (mShooterControlMode) {
				case OPEN_LOOP:
					break;
				case VELOCITY:
					break;
				case DISABLED:
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

	public synchronized void setTurretPosition(double turretPosition) {
		mPeriodicIO.turret_setpoint = turretPosition;
	}

	public synchronized void setHoodPosition(double hoodPosition) {
		mPeriodicIO.hood_setpoint = hoodPosition;
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
		return Math.abs(mPeriodicIO.turret_setpoint - mPeriodicIO.turret_position) < Math.abs(posDelta);
	}

	@Override
	public double getPosition() {
		return mPeriodicIO.turret_position;
	}

	@Override
	public double getSetpoint() {
		return mPeriodicIO.turret_setpoint;
	}

	public double getShooterVelocity() {
		return mPeriodicIO.shooter_velocity;
	}

	public Pose2d getLatestFieldToTurretPose() {
		return RobotState.getInstance().getLatestFieldToVehicle().getValue().transformBy(getLatestVehicleToTurretPose());
	}

	public Pose2d getLatestVehicleToTurretPose() {
		return new Pose2d(CalConstants.kVehicleToTurret.getTranslation(),
							Rotation2d.fromDegrees(mPeriodicIO.turret_position / 360.0));
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
		mPeriodicIO.turret_position = mTurretRotationMotor.getPosition();
		mPeriodicIO.turret_velocity = mTurretRotationMotor.getVelocity();
		mPeriodicIO.turret_reset = mTurretMasterHasReset.getValue();
		mPeriodicIO.turret_loop_time = loopTimer.hasElapsed();
		mPeriodicIO.shooter_velocity = mLeftShooterMotorMaster.getVelocity();
		mPeriodicIO.hood_position = mTurretHoodMotor.getPosition();
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
		public double turret_position;
		public double turret_velocity;
		public double turret_setpoint;
		public boolean turret_reset;

		public double shooter_velocity;
		public double shooter_setpoint;

		public double hood_position;
		public double hood_setpoint;

		// Outputs
		public double turret_loop_time;
	}
}