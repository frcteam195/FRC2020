package com.team195.frc.subsystems;

import com.team195.frc.RobotState;
import com.team195.frc.constants.CalConstants;
import com.team195.frc.constants.TargetingConstants;
import com.team195.frc.loops.ILooper;
import com.team195.frc.loops.Loop;
import com.team195.frc.reporters.ConsoleReporter;
import com.team195.frc.reporters.ReflectingLogDataGenerator;
import com.team195.lib.util.ElapsedTimer;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

public class VisionTracker extends Subsystem {
	private static VisionTracker mInstance = new VisionTracker();
	private PeriodicIO mPeriodicIO = new PeriodicIO();
	private ReflectingLogDataGenerator<PeriodicIO> mLogDataGenerator = new ReflectingLogDataGenerator<>(PeriodicIO.class);

	private TargetMode mTargetMode = TargetMode.AUTO_TARGET;
	private boolean mVisionEnabled = false;

	private NetworkTable mCurrentTargetingLimelightNT;

	private NetworkTable limelightTurret = NetworkTableInstance.getDefault().getTable("limelight-turret");

	private final ElapsedTimer loopTimer = new ElapsedTimer();

	public static VisionTracker getInstance() {
		return mInstance;
	}

	private VisionTracker() {

	}

		private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (VisionTracker.this) {

			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (VisionTracker.this) {
			}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized (VisionTracker.this) {
				switch (mTargetMode) {
					case AUTO_TARGET:
						mPeriodicIO.pipelineFront = mVisionEnabled ? 1 : 0;
						mCurrentTargetingLimelightNT = limelightTurret;
						break;
					default:
						mPeriodicIO.pipelineFront = mVisionEnabled ? 1 : 0;
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
			return "VisionTracker";
		}
	};

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	@Override
	public void stop() {
		setVisionEnabled(false);
	}

	@Override
	public synchronized boolean isSystemFaulted() {
		return false;
	}

	@Override
	public boolean runDiagnostics() {
		return true;
	}

	public boolean isVisionEnabled() {
		return mVisionEnabled;
	}

	public boolean isTargetFound() {
		return mVisionEnabled && mPeriodicIO.targetValid > 0;
	}

	public double getTargetDistance() {
		return mVisionEnabled ? mPeriodicIO.targetDistance : 0;
	}

	public double getTargetHorizAngleDev() {
		return mVisionEnabled ? mPeriodicIO.targetHorizontalDeviation : 0;
	}

	public double getTargetVertAngleDev() {
		return mVisionEnabled ? mPeriodicIO.targetVerticalDeviation : 0;
	}

	public synchronized void setVisionEnabled(boolean enabled) {
		mVisionEnabled = enabled;
	}

	public synchronized void setTargetMode(TargetMode targetMode) {
		mTargetMode = targetMode;
	}

	public double getTargetSkew() {
		return mPeriodicIO.targetSkew;
	}

	public Pose2d getCameraToTargetPose() {
		return mPeriodicIO.cameraToTargetPose;
	}

	@Override
	public synchronized List<Object> generateReport() {
		return mLogDataGenerator.generateData(mPeriodicIO);
	}

	@Override
	public synchronized void readPeriodicInputs() {
		loopTimer.start();
		try {
			if (mVisionEnabled) {
				mPeriodicIO.targetValid = mCurrentTargetingLimelightNT.getEntry("tv").getDouble(0);
				mPeriodicIO.targetHorizontalDeviation = mCurrentTargetingLimelightNT.getEntry("tx").getDouble(0);
				mPeriodicIO.targetVerticalDeviation = mCurrentTargetingLimelightNT.getEntry("ty").getDouble(0);
				mPeriodicIO.targetArea = mCurrentTargetingLimelightNT.getEntry("ta").getDouble(0);
				mPeriodicIO.targetSkew = mCurrentTargetingLimelightNT.getEntry("ts").getDouble(0);
				mPeriodicIO.targetLatency = mCurrentTargetingLimelightNT.getEntry("tl").getDouble(0);
				mPeriodicIO.targetShortSide = mCurrentTargetingLimelightNT.getEntry("tshort").getDouble(0);
				mPeriodicIO.targetLongSide = mCurrentTargetingLimelightNT.getEntry("tlong").getDouble(0);
				mPeriodicIO.targetHorizontalSide = mCurrentTargetingLimelightNT.getEntry("thor").getDouble(0);
				mPeriodicIO.targetVerticalSide = mCurrentTargetingLimelightNT.getEntry("tvert").getDouble(0);
				mPeriodicIO.getPipelineValue = mCurrentTargetingLimelightNT.getEntry("getpipe").getDouble(0);
				mPeriodicIO.cameraTranslation = new CameraTranslation(mCurrentTargetingLimelightNT.getEntry("camtran").getDoubleArray(mPeriodicIO.cameraTranslationRotationDefaultArray));
				mPeriodicIO.cameraToTargetPose = new Pose2d(new Translation2d(mPeriodicIO.cameraTranslation.x, mPeriodicIO.cameraTranslation.y), Rotation2d.fromDegrees(mPeriodicIO.cameraTranslation.yaw));
				if (!mPeriodicIO.cameraToTargetPose.equals(mPeriodicIO.prevCameraToTargetPose)) {
//					Pose2d targetToCamera = mPeriodicIO.cameraToTargetPose.inverse();
//					Pose2d cameraToTurret = CalConstants.kTurretToCamera.inverse();
//					Pose2d targetToTurret = targetToCamera.transformBy(cameraToTurret);
//					Pose2d turretToVehicle = CalConstants.kVehicleToTurret.inverse();
//					Pose2d targetToVehicle = targetToTurret.transformBy(turretToVehicle);
//					Pose2d fieldToVehicle = TargetingConstants.fieldToOuterTarget.transformBy(targetToVehicle);
//					RobotState.getInstance().addFieldToVehicleObservation(Timer.getFPGATimestamp(), fieldToVehicle);
					Pose2d latestAbsoluteFieldToVehicle = TargetingConstants.fieldToOuterTarget.transformBy(
							mPeriodicIO.cameraToTargetPose.inverse()    //May want to use robot angle here instead of angle from Camera pose
							.transformBy(CalConstants.kTurretToCamera.inverse())
							.transformBy(CalConstants.kVehicleToTurret.inverse())
					);
					RobotState.getInstance().addFieldToVehicleObservation(Timer.getFPGATimestamp(), latestAbsoluteFieldToVehicle);
					mPeriodicIO.prevCameraToTargetPose = mPeriodicIO.cameraToTargetPose;
				}
			}
			else {
				mPeriodicIO.targetValid = 0;
				mPeriodicIO.targetHorizontalDeviation = 0;
				mPeriodicIO.targetVerticalDeviation = 0;
				mPeriodicIO.targetArea = 0;
				mPeriodicIO.targetSkew = 0;
				mPeriodicIO.targetLatency = 0;
				mPeriodicIO.targetShortSide = 0;
				mPeriodicIO.targetLongSide = 0;
				mPeriodicIO.targetHorizontalSide = 0;
				mPeriodicIO.targetVerticalSide = 0;
				mPeriodicIO.getPipelineValue = 0;
				mPeriodicIO.cameraTranslation = CameraTranslation.identity;
				mPeriodicIO.cameraToTargetPose = Pose2d.identity();
				mPeriodicIO.targetDistance = 0;
			}
		}
		catch (Exception ex) {
			ConsoleReporter.report(ex);
		}
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		try {
			limelightTurret.getEntry("pipeline").setNumber(mPeriodicIO.pipelineFront);
		}
		catch (Exception ex) {
			ConsoleReporter.report(ex);
		}

//		NetworkTableEntry ledMode = mCurrentTargetingLimelightNT.getValue().getEntry("ledMode");
//		NetworkTableEntry camMode = mCurrentTargetingLimelightNT.getValue().getEntry("camMode");
//		NetworkTableEntry stream = mCurrentTargetingLimelightNT.getValue().getEntry("stream");
//		NetworkTableEntry snapshot = mCurrentTargetingLimelightNT.getValue().getEntry("snapshot");

		mPeriodicIO.vision_loop_time = loopTimer.hasElapsed();
	}

	public TargetMode getTargetMode() {
		return mTargetMode;
	}

	public static class CameraTranslation {
		public final double x;
		public final double y;
		public final double z;
		public final double pitch;
		public final double yaw;
		public final double roll;
		public static final CameraTranslation identity = new CameraTranslation(new double[6]);
		CameraTranslation(double[] inputData) {
			if (inputData.length == 6) {
				x = inputData[0];
				y = inputData[1];
				z = inputData[2];
				pitch = inputData[3];
				yaw = inputData[4];
				roll = inputData[5];
			} else {
				x = 0;
				y = 0;
				z = 0;
				pitch = 0;
				yaw = 0;
				roll = 0;
			}
		}

		@Override
		public String toString() {
			return "(x: " + x + ", y: " + y + ", theta: " + yaw + ")";
		}
	}

	@SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {
		//Making members public here will automatically add them to logs
		//Read values
		public double targetValid;
		public double targetHorizontalDeviation;
		double targetVerticalDeviation;
		public double targetArea;
		double targetSkew;
		double targetLatency;
		double targetShortSide;
		double targetLongSide;
		double targetHorizontalSide;
		double targetVerticalSide;
		double targetDistance;
		double getPipelineValue;
		public Pose2d cameraToTargetPose;
		public Pose2d prevCameraToTargetPose;
		public CameraTranslation cameraTranslation;
		double[] cameraTranslationRotationDefaultArray = new double[6];

		ArrayList<Translation2d> pointArray = new ArrayList<>();

		//Written values
		int pipelineFront;
		public double vision_loop_time;
	}

	public enum TargetMode {
		AUTO_TARGET;
	}
}
