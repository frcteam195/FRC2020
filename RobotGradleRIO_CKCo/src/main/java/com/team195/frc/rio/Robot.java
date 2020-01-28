package com.team195.frc.rio;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.illposed.osc.OSCBoundListMessage;
import com.illposed.osc.OSCPortOut;
import com.team195.ckcoprocessor.RIOToCoprocessorData;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import java.net.InetAddress;
import java.util.ArrayList;
import java.util.List;

public class Robot extends TimedRobot {
	private static final double kDataReporterPeriod = 0.005;
	private static final String kCoprocessorIP = "10.1.95.19";

	@SuppressWarnings("FieldCanBeLocal")
	private TalonSRX mPhoenixInitializer;
	private NavXNetworkLite mNavX;
	private RIOToCoprocessorData.CKRIOData.GyroData.Builder mGyroDataBuilder = RIOToCoprocessorData.CKRIOData.GyroData.newBuilder();
	private RIOToCoprocessorData.CKRIOData.Builder mRIOtoCoBuilder = RIOToCoprocessorData.CKRIOData.newBuilder();
	private List<RIOToCoprocessorData.CKRIOData.JoystickData.Builder> mJoystickBuilderArr;

	private JoystickProtoBuilder mJoystickProtoBuilder = new JoystickProtoBuilder();

	private DriverStation mDS = DriverStation.getInstance();

	private byte[] mBlobData;

	private OSCPortOut oscPortOut;
	private List<Object> oscOutList = new ArrayList<>(1);
	private OSCBoundListMessage oscMsg = new OSCBoundListMessage("/ckco/rio", oscOutList);

	private ElapsedTimer eTimer = new ElapsedTimer();

	private Notifier mNotifier = new Notifier(() -> {
		buildRIOtoCoData();
		oscOutList.clear();
		oscOutList.add(mBlobData);
		try {
			if (oscPortOut == null) {
				oscPortOut = new OSCPortOut(InetAddress.getByName(kCoprocessorIP));
			}

			oscPortOut.send(oscMsg);
		} catch (Exception e) {
//			System.out.println(e.toString());
		}
	});

	@Override
	public void robotInit() {
		mPhoenixInitializer = new TalonSRX(0);
		mNavX = new NavXNetworkLite();

		try {
			oscPortOut = new OSCPortOut(InetAddress.getByName(kCoprocessorIP));
		} catch (Exception e) {
//			System.out.println(e.toString());
		}
		eTimer.start();
		mNotifier.startPeriodic(kDataReporterPeriod);
	}

	@Override
	public void robotPeriodic() {

	}

	private void buildRIOtoCoData() {
		mGyroDataBuilder.clear();
		mRIOtoCoBuilder.clear();

		mRIOtoCoBuilder.setIsEnabled(mDS.isEnabled());
		mRIOtoCoBuilder.setRobotMode(getRobotMode());
		mRIOtoCoBuilder.setIsFmsAttached(mDS.isFMSAttached());
		mRIOtoCoBuilder.setIsDsAttached(mDS.isDSAttached());
		mRIOtoCoBuilder.setMatchTime(mDS.getMatchTime());
		mRIOtoCoBuilder.setFpgaTimestamp(Timer.getFPGATimestamp());
		mRIOtoCoBuilder.setGameSpecificMessage(mDS.getGameSpecificMessage());
		mRIOtoCoBuilder.setEventName(mDS.getEventName());
		mRIOtoCoBuilder.setAlliancePosition(getAlliancePosition());
		mRIOtoCoBuilder.setMatchType(getMatchType());

		mRIOtoCoBuilder.setGyroData(mGyroDataBuilder
				.setYaw(mNavX.getYaw())
				.setPitch(mNavX.getPitch())
				.setRoll(mNavX.getRoll())
				.setFusedHeading(mNavX.getFusedHeading())
				.setYawRate(mNavX.getYawRateDegreesPerSec())
				.setIsPresent(mNavX.isPresent()
				).build());

		mJoystickBuilderArr = mJoystickProtoBuilder.getJoystickData();
		for (RIOToCoprocessorData.CKRIOData.JoystickData.Builder jb:
				mJoystickBuilderArr) {
			mRIOtoCoBuilder.addJoystickData(jb.build());
		}

		mRIOtoCoBuilder.setLoopRate(eTimer.hasElapsed());
		eTimer.start();

		mBlobData = mRIOtoCoBuilder.build().toByteArray();
	}

	private RIOToCoprocessorData.CKRIOData.RobotMode getRobotMode() {
		if (mDS.isAutonomous()) {
			return RIOToCoprocessorData.CKRIOData.RobotMode.AUTO;
		} else if (mDS.isOperatorControl()) {
			return RIOToCoprocessorData.CKRIOData.RobotMode.TELEOP;
		} else if (mDS.isTest()) {
			return RIOToCoprocessorData.CKRIOData.RobotMode.TEST;
		} else if (mDS.isEStopped()) {
			return RIOToCoprocessorData.CKRIOData.RobotMode.E_STOPPED;
		}
		return RIOToCoprocessorData.CKRIOData.RobotMode.NOMODE;
	}

	private RIOToCoprocessorData.CKRIOData.AlliancePosition getAlliancePosition() {
		return RIOToCoprocessorData.CKRIOData.AlliancePosition.forNumber(HAL.getAllianceStation().ordinal());
	}

	private RIOToCoprocessorData.CKRIOData.MatchType getMatchType() {
		return RIOToCoprocessorData.CKRIOData.MatchType.forNumber(mDS.getMatchType().ordinal());
	}
}
