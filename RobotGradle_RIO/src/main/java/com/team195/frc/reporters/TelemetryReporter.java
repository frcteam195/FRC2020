package com.team195.frc.reporters;

import com.google.protobuf.InvalidProtocolBufferException;
import com.illposed.osc.*;
import com.team195.frc.constants.Constants;
import com.team195.frc.subsystems.Drive;
import com.team195.frc.telemetryfusion.*;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import java.io.IOException;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.List;

/**
 * New version of the DataReporter. Requires a heartbeat from each subscriber.
 */
public class TelemetryReporter {
	private static final InetAddress ipAddr = getInetFromIpStr(Constants.COPROCESSOR_IP);
	private static InetAddress getInetFromIpStr(String ipAddr) {
		try {
			return InetAddress.getByName(ipAddr);
		} catch (Exception ex) {
			return null;
		}
	}

	private static final int portNumber = Constants.CKCO_TRANSPORT_PORT;

	private static boolean firstRun = true;

	@SuppressWarnings("FieldCanBeLocal")
	private static OSCPortIn oscPortIn;
	private static OSCPortOut oscPortOut;

	private static List tmpList;
	private static byte[] protoByteArr;
	private static CKStatusMessage.StatusMessage currStatusMessage;

	private static Runnable initializer = () -> {
		try {
			oscPortIn = new OSCPortIn(portNumber);
			OSCListener updateListener = (time, message) -> {
				tmpList = message.getArguments();
				if (tmpList.size() == 1) {
					protoByteArr = (byte[])tmpList.get(0);
					try {
						currStatusMessage = CKStatusMessage.StatusMessage.parseFrom(protoByteArr);
					} catch (InvalidProtocolBufferException invalidPbEx) {
						currStatusMessage = null;
						ConsoleReporter.report(invalidPbEx);
					}
				}
			};

			oscPortIn.addListener("/RobotPosition", updateListener);
			oscPortIn.startListening();
			firstRun = false;
		} catch (Exception ex) {
			if (oscPortIn != null) {
				try {
					oscPortIn.stopListening();
					oscPortIn.close();
				} catch (Exception ignored) {

				}
			}
			oscPortIn = null;

			firstRun = true;
			ConsoleReporter.report(ex);
		}
	};

	public static Pose2d getCurrRobotPosition() {
		return new Pose2d(
				currStatusMessage.getRobotPosition().getPositionInches().getX(),
				currStatusMessage.getRobotPosition().getPositionInches().getY(),
				Rotation2d.fromDegrees(currStatusMessage.getRobotPosition().getRotationDegrees().getYaw())
			);
	}

	private static final List<Object> oscDataList = new ArrayList<>(1);
	private static final OSCBoundListMessage boundOSCMesage = new OSCBoundListMessage("/RobotTelemetry", oscDataList);

	public static synchronized void sendTelemetry() {
		if (firstRun) {
			initializer.run();
		}

		if (oscPortOut == null) {
			try {
				oscPortOut = new OSCPortOut(ipAddr, portNumber);
			} catch (Exception ignored) {
				return;
			}
		}
		try {
			performUpdateStep();
			oscPortOut.send(boundOSCMesage);
		} catch (IOException ignored) {

		}
	}

	private static final CKControlMessage.ControlMessage.Builder ctrlMsgBuilder = CKControlMessage.ControlMessage.newBuilder();

	private static final TranslationOuterClass.Translation.Builder robotToCameraTranslation = TranslationOuterClass.Translation.newBuilder();
	private static final RotationOuterClass.Rotation.Builder cameraRotation = RotationOuterClass.Rotation.newBuilder();
	private static final OdometryInput.OdometryConfig.Builder odometryCfgBuilder = OdometryInput.OdometryConfig.newBuilder();
	//Setup Odometry Cfg messages
	static {
		robotToCameraTranslation.setX(0);
		robotToCameraTranslation.setY(0);
		robotToCameraTranslation.setZ(0);
		cameraRotation.setYaw(0);
		cameraRotation.setPitch(0);
		cameraRotation.setRoll(0);

		odometryCfgBuilder.setRobotToCameraTranslationInches(robotToCameraTranslation.build());
		odometryCfgBuilder.setCameraRotationDegrees(cameraRotation.build());

		ctrlMsgBuilder.setOdometryConfig(odometryCfgBuilder.build());
	}

	/**
	 * TODO: Add position reset
	 */
	private static final PositionResetOuterClass.PositionReset.Builder positionResetBuilder = PositionResetOuterClass.PositionReset.newBuilder();
	private static final PositionOuterClass.Position.Builder positionBuilder = PositionOuterClass.Position.newBuilder();

	private static final OdometryInput.OdometryUpdate.Builder odometryUpdBuilder = OdometryInput.OdometryUpdate.newBuilder();
	private static void performUpdateStep() {
		//Clear first to prevent old data being sent in the event an issue occurs
		odometryUpdBuilder.clearLeftWheelRotations();
		odometryUpdBuilder.clearRightWheelRotations();
		ctrlMsgBuilder.clearOdometryUpdate();
		odometryUpdBuilder.setLeftWheelRotations((float)Drive.getInstance().getLeftEncoderVelocityRPM());
		odometryUpdBuilder.setRightWheelRotations((float)Drive.getInstance().getRightEncoderVelocityRPM());
		ctrlMsgBuilder.setOdometryUpdate(odometryUpdBuilder.build());

		oscDataList.clear();
		oscDataList.add(ctrlMsgBuilder.build().toByteArray());
	}
}
