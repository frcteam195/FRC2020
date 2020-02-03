package com.team195.frc.ckcoprocessor;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.*;

import java.util.ArrayList;
import java.util.List;

public class JoystickProtoBuilder {
	private ArrayList<RIOToCoprocessorData.CKRIOData.JoystickData.Builder> mJoystickDataArr = new ArrayList<>();

	private HALJoystickAxes[] mReportingAxesArr = new HALJoystickAxes[DriverStation.kJoystickPorts];
	private HALJoystickPOVs[] mReportingPOVsArr = new HALJoystickPOVs[DriverStation.kJoystickPorts];
	private HALJoystickButtons[] mReportingButtonsArr = new HALJoystickButtons[DriverStation.kJoystickPorts];

	public JoystickProtoBuilder() {
		for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
			mReportingAxesArr[i] = new HALJoystickAxes(HAL.kMaxJoystickAxes);
			mReportingPOVsArr[i] = new HALJoystickPOVs(HAL.kMaxJoystickPOVs);
			mReportingButtonsArr[i] = new HALJoystickButtons();
			mJoystickDataArr.add(RIOToCoprocessorData.CKRIOData.JoystickData.newBuilder());
		}
	}

	public List<RIOToCoprocessorData.CKRIOData.JoystickData.Builder> getJoystickData() {
		DriverStation.getInstance().getAllJoysticks(mReportingAxesArr, mReportingPOVsArr, mReportingButtonsArr);

		for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
			mJoystickDataArr.get(i).clear();
			for (int j = 0; j < mReportingAxesArr[i].m_count; j++) {
				mJoystickDataArr.get(i).addAxes(mReportingAxesArr[i].m_axes[j]);
			}
			for (int j = 0; j < mReportingPOVsArr[i].m_count; j++) {
				mJoystickDataArr.get(i).addPovs(mReportingPOVsArr[i].m_povs[j]);
			}
			mJoystickDataArr.get(i).setButtons(mReportingButtonsArr[i].m_buttons);
			mJoystickDataArr.get(i).setButtonsCount(mReportingButtonsArr[i].m_count);
		}

		return mJoystickDataArr;
	}
}
