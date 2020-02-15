package com.team195.frc.subsystems;

import com.team195.frc.Robot;
import com.team195.frc.constants.CalConstants;
import com.team195.frc.RobotState;
import com.team195.frc.constants.DeviceIDConstants;
import com.team195.frc.loops.ILooper;
import com.team195.frc.loops.Loop;
import com.team195.frc.paths.TrajectoryGenerator;
import com.team195.frc.reporters.DiagnosticMessage;
import com.team195.frc.reporters.ReflectingLogDataGenerator;
import com.team195.frc.subsystems.positions.TurretPositions;
import com.team195.lib.drivers.motorcontrol.CKTalonFX;
import com.team195.lib.drivers.motorcontrol.MCControlMode;
import com.team195.lib.drivers.motorcontrol.PDPBreaker;
import com.team195.lib.util.CachedValue;
import com.team195.lib.util.ElapsedTimer;
import com.team195.lib.util.InterferenceSystem;
import com.team195.lib.util.MotionInterferenceChecker;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import java.util.List;

public class Turret extends Subsystem implements InterferenceSystem {

	private static Turret mInstance = new Turret();

	private final VisionTracker mVisionTracker = VisionTracker.getInstance();

	private final CKTalonFX mTurretRotationMotor;

	private TurretControlMode mTurretControlMode = TurretControlMode.POSITION;

	private final MotionInterferenceChecker turretAnyPositionCheck;

	private PeriodicIO mPeriodicIO;
	private ReflectingLogDataGenerator<PeriodicIO> mLogDataGenerator = new ReflectingLogDataGenerator<>(PeriodicIO.class);

	private final CachedValue<Boolean> mTurretMasterHasReset;

	private final ElapsedTimer loopTimer = new ElapsedTimer();

	private Turret() {
		mPeriodicIO = new PeriodicIO();

		//Encoder on 50:1
		//Turret gear is another 36:252
		mTurretRotationMotor = new CKTalonFX(DeviceIDConstants.kTurretMotorId, false, PDPBreaker.B30A);
		mTurretRotationMotor.setInverted(true);
		mTurretRotationMotor.setSensorPhase(true);
		mTurretRotationMotor.setPIDF(CalConstants.kTurretPositionKp, CalConstants.kTurretPositionKi, CalConstants.kTurretPositionKd, CalConstants.kTurretPositionKf);
		mTurretRotationMotor.setMotionParameters(CalConstants.kTurretPositionCruiseVel, CalConstants.kTurretPositionMMAccel, CalConstants.kTurretPositionSCurveStrength);
		zeroSensors();
		mTurretRotationMotor.configForwardSoftLimitThreshold(CalConstants.kTurretForwardSoftLimit);
		mTurretRotationMotor.configForwardSoftLimitEnable(true);
		mTurretRotationMotor.configReverseSoftLimitThreshold(CalConstants.kTurretReverseSoftLimit);
		mTurretRotationMotor.configReverseSoftLimitEnable(true);
		mTurretRotationMotor.configCurrentLimit(CalConstants.kTurretContinuousCurrentLimit, CalConstants.kTurretPeakCurrentThreshold, CalConstants.kTurretPeakCurrentThresholdExceedDuration);
		mTurretRotationMotor.setControlMode(MCControlMode.MotionMagic);

//		TuneablePIDOSC x;
//		try {
//			x = new TuneablePIDOSC("Turret", 5804, true, mTurretRotationMotor);
//		} catch (Exception ignored) {
//
//		}

		turretAnyPositionCheck = new MotionInterferenceChecker(MotionInterferenceChecker.LogicOperation.AND, true,
				(t) -> (true)
		);

		mTurretMasterHasReset = new CachedValue<>(500, (t) -> mTurretRotationMotor.hasMotorControllerReset() != DiagnosticMessage.NO_MSG);
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
		return mLogDataGenerator.generateData(mPeriodicIO);
	}

	@Override
	public void zeroSensors() {
		mTurretRotationMotor.setEncoderPosition(0);
		setTurretPosition(0);
		if (mTurretControlMode == TurretControlMode.POSITION)
			mTurretRotationMotor.set(MCControlMode.MotionMagic, 0, 0, 0);
	}

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (Turret.this) {
//				zeroSensors();
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

						mPeriodicIO.turret_setpoint = convertTurretDegreesToRotations(desiredTurretAngleDeg);
						//Fall through on purpose to set position -> no break;
					case VISION_TRACK:
						//Preempts Auto Track
						if (mVisionTracker.isTargetFound())
							mPeriodicIO.turret_setpoint = convertTurretDegreesToRotations(mVisionTracker.getTargetHorizAngleDev());
						//Fall through on purpose to set position -> no break;
					case POSITION:
						if (turretAnyPositionCheck.hasPassedConditions() //|| Elevator.getInstance().getPosition() > ElevatorPositions.CargoBall
							|| Math.abs(mPeriodicIO.turret_setpoint - TurretPositions.Back180) < Turret.convertTurretDegreesToRotations(10)
							|| Math.abs(mPeriodicIO.turret_setpoint - TurretPositions.Home) < Turret.convertTurretDegreesToRotations(10))
							mTurretRotationMotor.set(MCControlMode.MotionMagic, mPeriodicIO.turret_setpoint, 0, 0);
//						else if (mPeriodicIO.turret_setpoint != TurretPositions.Back180 && mPeriodicIO.turret_setpoint != TurretPositions.Home)
//							mTurretRotationMotor.set(MCControlMode.MotionMagic, 0, 0, 0);
						break;
					case GIMBAL:
						//Field to Camera -> Camera to Target -> Field to Target
//						Translation2d latestFieldToTarget = new Translation2d(-10, -120); //Create this from Camera Data
						Pose2d robotPose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
						Pose2d latestFieldToTurret = RobotState.getInstance().getLatestFieldToTurretPose(convertRotationsToTurretDegrees(mPeriodicIO.turret_position));
						Translation2d latestFieldToTarget = new Translation2d(-8, 67); //Inner
                        Translation2d latestFieldToTargetO = new Translation2d(0, 67); //Outer
						Translation2d turretToTarget = latestFieldToTarget.getTranslation().translateBy(latestFieldToTurret.getTranslation().inverse());
						Rotation2d robotCentricSetpoint = turretToTarget.direction().rotateBy(robotPose.getRotation().inverse());
						double rawDegreesOut = calculateSetpointForRobotCentricRotation(convertRotationsToTurretDegrees(mPeriodicIO.turret_position), robotCentricSetpoint, CalConstants.kTurretMinDegrees, CalConstants.kTurretMinDegrees);
						double arbFF = -mTurretRotationMotor.getArbFFFromVelocity(-Drive.getInstance().getAngularVelocity() / (2 * Math.PI) * 60.0, mPeriodicIO.turret_velocity, mPeriodicIO.turret_loop_time);
						mTurretRotationMotor.set(MCControlMode.MotionMagic, convertTurretDegreesToRotations(rawDegreesOut), 0, arbFF);
						break;
					case OPEN_LOOP:
						mTurretRotationMotor.set(MCControlMode.PercentOut, Math.min(Math.max(mPeriodicIO.turret_setpoint, -1), 1), 0, 0);
						break;
					default:
						mTurretRotationMotor.set(MCControlMode.Disabled, 0, 0, 0);
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

	public synchronized void setTurretPosition(double turretPosition) {
		mPeriodicIO.turret_setpoint = turretPosition;
	}

	public synchronized void setTurretControlMode(TurretControlMode turretControlMode) {
		if (mTurretControlMode != turretControlMode)
			mTurretControlMode = turretControlMode;
	}

	public boolean isTurretAtSetpoint(double posDelta) {
		return Math.abs(mPeriodicIO.turret_setpoint - mPeriodicIO.turret_position) < Math.abs(posDelta);
	}

	public static double convertTurretRotationsToMotorRotations(double rotations) {
		return rotations * (CalConstants.kTurretLargeGearTeeth / CalConstants.kTurretSmallGearTeeth);
	}

	public static double convertMotorRotationsToTurretRotations(double rotations) {
		return rotations / (CalConstants.kTurretLargeGearTeeth / CalConstants.kTurretSmallGearTeeth);
	}

	public static double convertRotationsToTurretDegrees(double rotations) {
		return rotations / (CalConstants.kTurretLargeGearTeeth / CalConstants.kTurretSmallGearTeeth / 360.0);
	}

	public static double convertTurretDegreesToRotations(double degrees) {
		return degrees * (CalConstants.kTurretLargeGearTeeth / CalConstants.kTurretSmallGearTeeth / 360.0);
	}

	public static double calculateSetpointForRobotCentricRotation(double currentRotationDegrees, Rotation2d robotCentricSetpoint, double minDegrees, double maxDegrees) {
		Rotation2d currentRobotCentricRotation = Rotation2d.fromDegrees(currentRotationDegrees);
		Rotation2d setpoint = robotCentricSetpoint;

		Rotation2d error = setpoint.inverse().rotateBy(currentRobotCentricRotation);
		double newDemandRotationDegrees = currentRotationDegrees - error.getDegrees();
		if(newDemandRotationDegrees < minDegrees) {
			newDemandRotationDegrees += 360;
		}
		if(newDemandRotationDegrees > maxDegrees) {
			newDemandRotationDegrees -= 360;
		}
		return newDemandRotationDegrees;
	}

	public static Rotation2d calculateRobotCentricTurretRotation(Rotation2d fieldCentricDemand, Rotation2d robotRotation) {
		return fieldCentricDemand.rotateBy(robotRotation.inverse());
	}

	@Override
	public double getPosition() {
		return mPeriodicIO.turret_position;
	}

	@Override
	public double getSetpoint() {
		return mPeriodicIO.turret_setpoint;
	}

	public enum TurretControlMode {
		POSITION,
		OPEN_LOOP,
		AUTO_TRACK,
		VISION_TRACK,
		GIMBAL,
		DISABLED;
	}

	@Override
	public synchronized void readPeriodicInputs() {
		loopTimer.start();
		mPeriodicIO.turret_position = mTurretRotationMotor.getPosition();
		mPeriodicIO.turret_velocity = mTurretRotationMotor.getVelocity();
		mPeriodicIO.turret_reset = mTurretMasterHasReset.getValue();
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		mPeriodicIO.turret_loop_time = loopTimer.hasElapsed();
	}

	@SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {
		//Making members public here will automatically add them to logs
		// INPUTS
		public double turret_position;
		public double turret_velocity;
		public double turret_setpoint;
		public boolean turret_reset;

		// Outputs
		public double turret_loop_time;
	}
}