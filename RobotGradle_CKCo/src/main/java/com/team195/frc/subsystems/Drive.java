package com.team195.frc.subsystems;

import com.team195.frc.constants.CalConstants;
import com.team195.frc.constants.DeviceIDConstants;
import com.team195.frc.constants.TestConstants;
import com.team195.frc.loops.ILooper;
import com.team195.frc.loops.Loop;
import com.team195.frc.planners.DriveMotionPlanner;
import com.team195.frc.RobotState;
import com.team195.frc.reporters.ConsoleReporter;
import com.team195.frc.reporters.MessageLevel;
import com.team195.frc.reporters.ReflectingLogDataGenerator;
import com.team195.lib.drivers.CKIMU;
import com.team195.lib.drivers.NavX;
import com.team195.lib.drivers.motorcontrol.*;
import com.team195.lib.util.CachedValue;
import com.team195.lib.util.ElapsedTimer;
import com.team195.lib.util.MotorDiagnostics;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;

@SuppressWarnings("FieldCanBeLocal")
public class Drive extends Subsystem {

	private static final int kLowGearVelocityControlSlot = 0;
	private static Drive mInstance = new Drive();
	private final CKTalonFX mLeftMaster, mRightMaster, mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;
	private DriveControlState mDriveControlState;
	private CKIMU mGyro;
	private PeriodicIO mPeriodicIO;
	private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
	private ReflectingLogDataGenerator<PeriodicIO> mLogDataGenerator = new ReflectingLogDataGenerator<>(PeriodicIO.class);
	private DriveMotionPlanner mMotionPlanner;
	private Rotation2d mGyroOffset = Rotation2d.identity();
	private boolean mOverrideTrajectory = false;

	private AtomicBoolean mIsBrakeMode = new AtomicBoolean(false);
	private AtomicBoolean mForceBrakeUpdate = new AtomicBoolean(false);
	private boolean mPrevBrakeMode;

	private final CachedValue<Boolean> mGyroPresent;

	private final ElapsedTimer loopTimer = new ElapsedTimer();

	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (Drive.this) {

			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (Drive.this) {
				setOpenLoop(DriveSignal.NEUTRAL);
				if (mDriveControlState == DriveControlState.OPEN_LOOP) {
					setBrakeMode(false);
				}
				else {
					setBrakeMode(true);
				}
			}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized (Drive.this) {
				switch (mDriveControlState) {
					case OPEN_LOOP:
						break;
					case PATH_FOLLOWING:
						updatePathFollower();
						break;
					case CLIMB:
					case OPEN_LOOP_AUTOMATED:
					case VELOCITY:
						break;
					default:
						ConsoleReporter.report("Unexpected drive control state: " + mDriveControlState, MessageLevel.DEFCON1);
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
			return "Drive";
		}
	};

	private Drive() {
		mPeriodicIO = new PeriodicIO();

		mLeftMaster = new CKTalonFX(DeviceIDConstants.kLeftDriveMasterId, true, PDPBreaker.B40A);
		mLeftMaster.setInverted(false);
		mLeftMaster.setPIDF(CalConstants.kDriveLowGearVelocityKp, CalConstants.kDriveLowGearVelocityKi, CalConstants.kDriveLowGearVelocityKd, CalConstants.kDriveLowGearVelocityKf);
		mLeftMaster.setDFilter(CalConstants.kDriveLowGearVelocityDFilter);
		mLeftMaster.setMotionParameters(CalConstants.kDriveLowGearPositionCruiseVel, CalConstants.kDriveLowGearPositionAccel);
		mLeftMaster.setCurrentLimit(CalConstants.kDriveLowGearCurrentLim);

		mLeftSlaveA = new CKTalonFX(DeviceIDConstants.kLeftDriveSlaveAId, mLeftMaster, PDPBreaker.B40A, false);
		mLeftSlaveA.setCurrentLimit(CalConstants.kDriveLowGearCurrentLim);

		mLeftSlaveB = new CKTalonFX(DeviceIDConstants.kLeftDriveSlaveBId, mLeftMaster, PDPBreaker.B40A, false);
		mLeftSlaveB.setCurrentLimit(CalConstants.kDriveLowGearCurrentLim);


		mRightMaster = new CKTalonFX(DeviceIDConstants.kRightDriveMasterId, true, PDPBreaker.B40A);
		mRightMaster.setInverted(true);
		mRightMaster.setPIDF(CalConstants.kDriveLowGearVelocityKp, CalConstants.kDriveLowGearVelocityKi, CalConstants.kDriveLowGearVelocityKd, CalConstants.kDriveLowGearVelocityKf);
		mRightMaster.setMotionParameters(CalConstants.kDriveLowGearPositionCruiseVel, CalConstants.kDriveLowGearPositionAccel);
		mRightMaster.setCurrentLimit(CalConstants.kDriveLowGearCurrentLim);

		mRightSlaveA = new CKTalonFX(DeviceIDConstants.kRightDriveSlaveAId, mRightMaster, PDPBreaker.B40A, false);
		mRightSlaveA.setCurrentLimit(CalConstants.kDriveLowGearCurrentLim);

		mRightSlaveB = new CKTalonFX(DeviceIDConstants.kRightDriveSlaveBId, mRightMaster, PDPBreaker.B40A, false);
		mRightSlaveB.setCurrentLimit(CalConstants.kDriveLowGearCurrentLim);

		reloadGains();

		mGyro = new NavX();

		setOpenLoop(DriveSignal.NEUTRAL);

		// Force a CAN message across.
		mPrevBrakeMode = false;
		setBrakeMode(true);

		mMotionPlanner = new DriveMotionPlanner();

		mGyroPresent = new CachedValue<>(500, (t) -> mGyro.isPresent());

//		TuneablePIDOSC x;
//		try {
//			mLeftMaster.setControlMode(MCControlMode.Velocity);
//			mRightMaster.setControlMode(MCControlMode.Velocity);
//			x = new TuneablePIDOSC("Drive", 5804, true, mLeftMaster, mRightMaster);
//		} catch (Exception ignored) {
//
//		}
	}

	public static Drive getInstance() {
		return mInstance;
	}

	private static double rotationsToInches(double rotations) {
		return rotations * (CalConstants.kDriveWheelDiameterInches * Math.PI);
	}

	private static double rpmToInchesPerSecond(double rpm) {
		return rotationsToInches(rpm) / 60.0;
	}

	private static double inchesToRotations(double inches) {
		return inches / (CalConstants.kDriveWheelDiameterInches * Math.PI);
	}

	private static double inchesPerSecondToRpm(double inches_per_second) {
		return inchesToRotations(inches_per_second) * 60.0;
	}

	private static double radiansPerSecondToRPM(double rad_s) {
		return rad_s / (2.0 * Math.PI) * 60.0;
	}

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	public synchronized void setOpenLoop(DriveSignal signal) {
		if (mDriveControlState != DriveControlState.OPEN_LOOP) {
			setBrakeMode(false);
			setDriveControlState(DriveControlState.OPEN_LOOP);
		}
		mPeriodicIO.left_demand = signal.getLeft();
		mPeriodicIO.right_demand = signal.getRight();
		mPeriodicIO.left_feedforward = 0.0;
		mPeriodicIO.right_feedforward = 0.0;
	}

	public synchronized void setOpenLoopAutomated(DriveSignal signal) {
		if (mDriveControlState != DriveControlState.OPEN_LOOP_AUTOMATED) {
			setBrakeMode(true);
			setDriveControlState(DriveControlState.OPEN_LOOP_AUTOMATED);
		}
		mPeriodicIO.left_demand = signal.getLeft();
		mPeriodicIO.right_demand = signal.getRight();
		mPeriodicIO.left_feedforward = 0.0;
		mPeriodicIO.right_feedforward = 0.0;
	}

	public synchronized void setClimbLeft(double leftSignal) {
		if (mDriveControlState != DriveControlState.CLIMB) {
			setBrakeMode(true);
			setDriveControlState(DriveControlState.CLIMB);
		}
		mPeriodicIO.left_demand = leftSignal;
		mPeriodicIO.left_feedforward = 0.0;
	}

	public synchronized void setClimbRight(double rightSignal) {
		if (mDriveControlState != DriveControlState.CLIMB) {
			setBrakeMode(true);
			setDriveControlState(DriveControlState.CLIMB);
		}
		mPeriodicIO.right_demand = rightSignal;
		mPeriodicIO.right_feedforward = 0.0;
	}

	public DriveControlState getDriveControlState() {
		return mDriveControlState;
	}

	public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
		if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
			setBrakeMode(true);
			mLeftMaster.setPIDGainSlot(kLowGearVelocityControlSlot);
			mRightMaster.setPIDGainSlot(kLowGearVelocityControlSlot);

			setDriveControlState(DriveControlState.PATH_FOLLOWING);
		}
		mPeriodicIO.left_demand = signal.getLeft();
		mPeriodicIO.right_demand = signal.getRight();
		mPeriodicIO.left_feedforward = feedforward.getLeft();
		mPeriodicIO.right_feedforward = feedforward.getRight();
	}

	public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
		if (mMotionPlanner != null) {
			mOverrideTrajectory = false;
			mMotionPlanner.reset();
			mMotionPlanner.setTrajectory(trajectory);
			setDriveControlState(DriveControlState.PATH_FOLLOWING);
		}
	}

	public boolean isDoneWithTrajectory() {
		if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
			return false;
		}
		return mMotionPlanner.isDone() || mOverrideTrajectory;
	}

	public boolean isHighGear() {
		return false;
	}

	public synchronized void setHighGear(boolean wantsHighGear) {

	}

	public boolean isBrakeMode() {
		return mIsBrakeMode.get();
	}

	public void setBrakeMode(boolean on) {
		mIsBrakeMode.set(on);
	}

	public synchronized void setDriveControlState(DriveControlState driveControlState) {
		mDriveControlState = driveControlState;
	}

	public double getRoll() {
		return mPeriodicIO.gyro_roll;
	}

	public synchronized Rotation2d getHeading() {
		return mPeriodicIO.gyro_heading;
	}

	public synchronized void setHeading(Rotation2d heading) {
        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mGyro.getFusedHeading()).inverse());
        mPeriodicIO.gyro_heading = heading;
	}

	@Override
	public void stop() {
		setOpenLoop(DriveSignal.NEUTRAL);
	}

	public synchronized void resetEncoders() {
        mLeftMaster.setEncoderPosition(0);
        mRightMaster.setEncoderPosition(0);
		mPeriodicIO = new PeriodicIO();
	}

	@Override
	public void zeroSensors() {
		setHeading(Rotation2d.identity());
		resetEncoders();
	}

	public double getLeftEncoderDistance() {
		return rotationsToInches(mPeriodicIO.left_position_rotations);
	}

	public double getRightEncoderDistance() {
		return rotationsToInches(mPeriodicIO.right_position_rotations);
	}

	public double getRightLinearVelocity() {
		return rotationsToInches(mPeriodicIO.right_velocity_RPM / 60.0);
	}

	public double getLeftLinearVelocity() {
		return rotationsToInches(mPeriodicIO.left_velocity_RPM / 60.0);
	}

	public double getLinearVelocity() {
		return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
	}

	public double getAngularVelocity() {
		return (getRightLinearVelocity() - getLeftLinearVelocity()) / CalConstants.kDriveWheelTrackWidthInches;
	}

	public double getAverageInputVoltage() {
		return (mPeriodicIO.left_bus_voltage + mPeriodicIO.right_bus_voltage) / 2.0;
	}

	public void overrideTrajectory(boolean value) {
		mOverrideTrajectory = value;
	}

	public double getLeftEncoderVelocityRPM() {
		return mPeriodicIO.left_velocity_RPM;
	}

	public double getRightEncoderVelocityRPM() {
		return mPeriodicIO.right_velocity_RPM;
	}

	private static final DriveSignal pathVelWheelContainer = new DriveSignal(0,0 );
	private static final DriveSignal pathFFWheelContainer = new DriveSignal(0,0 );
	private static DriveMotionPlanner.Output motionPlannerOutput;
	private static double currPathFollowingTime;
	private void updatePathFollower() {
		if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
			currPathFollowingTime = Timer.getFPGATimestamp();

			motionPlannerOutput = mMotionPlanner.update(currPathFollowingTime, RobotState.getInstance().getFieldToVehicle(currPathFollowingTime));

			mPeriodicIO.error = mMotionPlanner.error();
			mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

			if (!mOverrideTrajectory) {
				pathVelWheelContainer.set(radiansPerSecondToRPM(motionPlannerOutput.left_velocity) * CalConstants.kDriveGearRatioMotorConversionFactor,
										  radiansPerSecondToRPM(motionPlannerOutput.right_velocity) * CalConstants.kDriveGearRatioMotorConversionFactor);
				pathFFWheelContainer.set(motionPlannerOutput.left_feedforward_voltage / 12.0,
										 motionPlannerOutput.right_feedforward_voltage / 12.0);
				setVelocity(pathVelWheelContainer, pathFFWheelContainer);

				mPeriodicIO.left_accel = radiansPerSecondToRPM(motionPlannerOutput.left_accel) / 1000.0;
				mPeriodicIO.right_accel = radiansPerSecondToRPM(motionPlannerOutput.right_accel) / 1000.0;
			} else {
				setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
				mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
			}
		} else {
			ConsoleReporter.report("Drive is not in path following state", MessageLevel.ERROR);
		}
	}

	public synchronized void reloadGains() {
		mLeftMaster.setPIDF(CalConstants.kDriveLowGearVelocityKp, CalConstants.kDriveLowGearVelocityKi, CalConstants.kDriveLowGearVelocityKd, CalConstants.kDriveLowGearVelocityKf);
		mLeftMaster.setIZone(CalConstants.kDriveLowGearVelocityIZone);
		mLeftMaster.writeToFlash();

		mRightMaster.setPIDF(CalConstants.kDriveLowGearVelocityKp, CalConstants.kDriveLowGearVelocityKi, CalConstants.kDriveLowGearVelocityKd, CalConstants.kDriveLowGearVelocityKf);
		mRightMaster.setIZone(CalConstants.kDriveLowGearVelocityIZone);
		mRightMaster.writeToFlash();
	}

	@Override
	public synchronized void readPeriodicInputs() {
		loopTimer.start();
		mPeriodicIO.prev_left_rotations = mPeriodicIO.left_position_rotations;
		mPeriodicIO.prev_right_rotations = mPeriodicIO.right_position_rotations;
		mPeriodicIO.left_position_rotations = mLeftMaster.getPosition();
		mPeriodicIO.right_position_rotations = mRightMaster.getPosition();
		mPeriodicIO.left_velocity_RPM = mLeftMaster.getVelocity();
		mPeriodicIO.right_velocity_RPM = mRightMaster.getVelocity();
		mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mGyro.getFusedHeading()).rotateBy(mGyroOffset);
		mPeriodicIO.gyro_roll = mGyro.getRoll();
		mPeriodicIO.left_drive_current = mLeftMaster.getSupplyCurrent();
		mPeriodicIO.right_drive_current = mRightMaster.getSupplyCurrent();
		mPeriodicIO.gyro_present = mGyroPresent.getValue();
		mPeriodicIO.left_bus_voltage = mLeftMaster.getMCInputVoltage();
		mPeriodicIO.right_bus_voltage = mRightMaster.getMCInputVoltage();





		mPeriodicIO.delta_left_rotations = (mPeriodicIO.left_position_rotations - mPeriodicIO.prev_left_rotations) * Math.PI;
		if (mPeriodicIO.delta_left_rotations > 0.0) {
			mPeriodicIO.left_distance += mPeriodicIO.delta_left_rotations * CalConstants.kDriveWheelDiameterInches;
		} else {
			mPeriodicIO.left_distance += mPeriodicIO.delta_left_rotations * CalConstants.kDriveWheelDiameterInches;
		}

		mPeriodicIO.delta_right_rotations = (mPeriodicIO.right_position_rotations - mPeriodicIO.prev_right_rotations) * Math.PI;
		if (mPeriodicIO.delta_right_rotations > 0.0) {
			mPeriodicIO.right_distance += mPeriodicIO.delta_right_rotations * CalConstants.kDriveWheelDiameterInches;
		} else {
			mPeriodicIO.right_distance += mPeriodicIO.delta_right_rotations * CalConstants.kDriveWheelDiameterInches;
		}

		if (mCSVWriter != null) {
			mCSVWriter.add(mPeriodicIO);
		}
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		if (mDriveControlState == DriveControlState.OPEN_LOOP
				|| mDriveControlState == DriveControlState.CLIMB
				|| mDriveControlState == DriveControlState.OPEN_LOOP_AUTOMATED) {
			mLeftMaster.set(MCControlMode.PercentOut, mPeriodicIO.left_demand, 0, 0.0);
			mRightMaster.set(MCControlMode.PercentOut, mPeriodicIO.right_demand, 0, 0.0);
		} else if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
			mLeftMaster.set(MCControlMode.Velocity, mPeriodicIO.left_demand, 0,
					mPeriodicIO.left_feedforward + CalConstants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / mLeftMaster.getNativeUnitsOutputRange());
			mRightMaster.set(MCControlMode.Velocity, mPeriodicIO.right_demand, 0,
					mPeriodicIO.right_feedforward + CalConstants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / mRightMaster.getNativeUnitsOutputRange());
		}

		if (mIsBrakeMode.get() != mPrevBrakeMode || mForceBrakeUpdate.get()) {
			MCNeutralMode mode = mIsBrakeMode.get() ? MCNeutralMode.Brake : MCNeutralMode.Coast;
			mRightMaster.setBrakeCoastMode(mode);
			mRightSlaveA.setBrakeCoastMode(mode);
			mRightSlaveB.setBrakeCoastMode(mode);

			mLeftMaster.setBrakeCoastMode(mode);
			mLeftSlaveA.setBrakeCoastMode(mode);
			mLeftSlaveB.setBrakeCoastMode(mode);

			mPrevBrakeMode = mIsBrakeMode.get();

			if (mForceBrakeUpdate.get())
				mForceBrakeUpdate.set(false);
		}
		mPeriodicIO.drive_loop_time = loopTimer.hasElapsed();
	}

	@Override
	public boolean runDiagnostics() {
		if (TestConstants.ENABLE_DRIVE_TEST) {
			ConsoleReporter.report("Testing DRIVE---------------------------------");
			final double kLowCurrentThres = TestConstants.kDriveBaseTestLowCurrentThresh;
			final double kLowRpmThres = TestConstants.kDriveBaseTestLowRPMThresh;

			ArrayList<MotorDiagnostics> mAllMotorsDiagArr = new ArrayList<>();
			ArrayList<MotorDiagnostics> mLeftDiagArr = new ArrayList<>();
			ArrayList<MotorDiagnostics> mRightDiagArr = new ArrayList<>();
			mLeftDiagArr.add(new MotorDiagnostics("Drive Left Master", mLeftMaster));
			mLeftDiagArr.add(new MotorDiagnostics("Drive Left Slave 1", mLeftSlaveA, mLeftMaster));
			mLeftDiagArr.add(new MotorDiagnostics("Drive Left Slave 2", mLeftSlaveB, mLeftMaster));
			mRightDiagArr.add(new MotorDiagnostics("Drive Right Master", mRightMaster));
			mRightDiagArr.add(new MotorDiagnostics("Drive Right Slave 1", mRightSlaveA, mRightMaster));
			mRightDiagArr.add(new MotorDiagnostics("Drive Right Slave 2", mRightSlaveB, mRightMaster));

			mLeftSlaveA.follow(mLeftMaster);
			mLeftSlaveB.follow(mLeftMaster);

			mRightSlaveA.follow(mRightMaster);
			mRightSlaveA.setInverted(true);
			mRightSlaveB.follow(mRightMaster);
			mRightSlaveB.setInverted(true);

			mAllMotorsDiagArr.addAll(mLeftDiagArr);
			mAllMotorsDiagArr.addAll(mRightDiagArr);

			boolean failure = false;

			for (MotorDiagnostics mD : mAllMotorsDiagArr) {
				mD.setZero();
			}

			for (MotorDiagnostics mD : mAllMotorsDiagArr) {
				ConsoleReporter.report("Testing motor: " + mD.getMotorName());
				mD.runTest();

				if (mD.isCurrentUnderThreshold(kLowCurrentThres)) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Current Low !!!!!!!!!!");
					failure = true;
				}

				if (mD.isRPMUnderThreshold(kLowRpmThres)) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " RPM Low !!!!!!!!!!");
					failure = true;
				}

				if (!mD.isSensorInPhase()) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Sensor Out of Phase !!!!!!!!!!");
					failure = true;
				}

				try {
					Thread.sleep(2000);
				} catch (Exception ex) {

				}
			}

			if (mLeftDiagArr.size() > 0 && mRightDiagArr.size() > 0 && mAllMotorsDiagArr.size() > 0) {
				List<Double> leftMotorCurrents = mLeftDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
				if (!Util.allCloseTo(leftMotorCurrents, leftMotorCurrents.get(0), TestConstants.kDriveBaseTestCurrentDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Drive Left2Cube Currents Different !!!!!!!!!!");
				}

				List<Double> rightMotorCurrents = mRightDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
				if (!Util.allCloseTo(rightMotorCurrents, rightMotorCurrents.get(0), TestConstants.kDriveBaseTestCurrentDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Drive Right2Cube Currents Different !!!!!!!!!!");
				}

				List<Double> driveMotorRPMs = mAllMotorsDiagArr.stream().map(MotorDiagnostics::getMotorRPM).collect(Collectors.toList());
				if (!Util.allCloseTo(driveMotorRPMs, driveMotorRPMs.get(0), TestConstants.kDriveBaseTestRPMDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!!! Drive RPMs different !!!!!!!!!!!!!!!!!!!");
				}
			} else {
				ConsoleReporter.report("Drive Testing Error Occurred in system. Please check code!", MessageLevel.ERROR);
			}

			return !failure;
		}
		else
			return true;
	}

	public void forceBrakeModeUpdate() {
		mForceBrakeUpdate.set(true);
	}

	@Override
	public synchronized List<Object> generateReport() {
		return mLogDataGenerator.generateData(mPeriodicIO);
	}

	@Override
	public synchronized boolean isSystemFaulted() {
		boolean navXFaulted = !mPeriodicIO.gyro_present;

		if (navXFaulted)
			ConsoleReporter.report("NavX Error", MessageLevel.DEFCON1);

		mPeriodicIO.drive_loop_time = loopTimer.hasElapsed();

		return navXFaulted;
	}

	public enum DriveControlState {
		OPEN_LOOP,
		PATH_FOLLOWING,
		VELOCITY,
		CLIMB,
		OPEN_LOOP_AUTOMATED
	}

	public enum ShifterState {
		FORCE_LOW_GEAR,
		FORCE_HIGH_GEAR,
		AUTO_SHIFT
	}

	@SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {
		//Making members public here will automatically add them to logs
		// INPUTS
		public double left_position_rotations;
		public double right_position_rotations;
		public double left_distance;
		public double right_distance;
		public double left_velocity_RPM;
		public double right_velocity_RPM;
		public Rotation2d gyro_heading = Rotation2d.identity();
		public double gyro_roll;
		public Pose2d error = Pose2d.identity();

		public double left_bus_voltage;
		public double right_bus_voltage;

		public double left_drive_current;
		public double right_drive_current;

		boolean gyro_present;

		double prev_left_rotations;
		double prev_right_rotations;
		double delta_left_rotations;
		double delta_right_rotations;


		// OUTPUTS
		public double left_demand;
		public double right_demand;
		public double left_accel;
		public double right_accel;
		public double left_feedforward;
		public double right_feedforward;
		TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<>(Pose2dWithCurvature.identity());
		public double drive_loop_time;
	}
}

