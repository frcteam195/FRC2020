package com.team195.frc.rio;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team195.ckcoprocessor.RIOToCoprocessorData;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;

import java.util.ArrayList;

public class Robot extends TimedRobot {
	private TalonSRX mPhoenixInitializer;
	private NavXNetworkLite mNavX;
	private RIOToCoprocessorData.CKRIOData.GyroData mGyroData;
	private ArrayList<RIOToCoprocessorData.CKRIOData.JoystickDataOrBuilder> mJoystickDataList = new ArrayList<>(DriverStation.kJoystickPorts);

	private static byte[] mBlobData;

	@Override
	public void robotInit() {
		mPhoenixInitializer = new TalonSRX(0);
		mNavX = new NavXNetworkLite();

		for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
			mJoystickDataList.add(RIOToCoprocessorData.CKRIOData.JoystickData.newBuilder());
		}

		// Get the status of all of the joysticks
		for (byte stick = 0; stick < DriverStation.kJoystickPorts; stick++) {
			m_joystickAxesCache[stick].m_count =
					HAL.getJoystickAxes(stick, m_joystickAxesCache[stick].m_axes);
			m_joystickPOVsCache[stick].m_count =
					HAL.getJoystickPOVs(stick, m_joystickPOVsCache[stick].m_povs);
			m_joystickButtonsCache[stick].m_buttons = HAL.getJoystickButtons(stick, m_buttonCountBuffer);
			m_joystickButtonsCache[stick].m_count = m_buttonCountBuffer.get(0);
		}
	}

	@Override
	public void robotPeriodic() {
		mGyroData = RIOToCoprocessorData.CKRIOData.GyroData.newBuilder()
				.setYaw(mNavX.getYaw())
				.setPitch(mNavX.getPitch())
				.setRoll(mNavX.getRoll())
				.setFusedHeading(mNavX.getFusedHeading())
				.setYawRate(mNavX.getYawRateDegreesPerSec())
				.setIsPresent(mNavX.isPresent()).build();

		DriverStation.getInstance().

		mBlobData = RIOToCoprocessorData.CKRIOData.newBuilder().build().toByteArray();
	}
}
