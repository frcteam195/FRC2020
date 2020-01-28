package com.team195.frc.rio;

import com.team195.ckcoprocessor.RIOToCoprocessorData;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

public class JoystickProtoBuilder {
	private ArrayList<RIOToCoprocessorData.CKRIOData.JoystickData.Builder> mJoystickDataArr = new ArrayList<>();

	private DriverStation.HALJoystickAxes[] mReportingAxesArr = new DriverStation.HALJoystickAxes[DriverStation.kJoystickPorts];
	private DriverStation.HALJoystickPOVs[] mReportingPOVsArr = new DriverStation.HALJoystickPOVs[DriverStation.kJoystickPorts];
	private DriverStation.HALJoystickButtons[] mReportingButtonsArr = new DriverStation.HALJoystickButtons[DriverStation.kJoystickPorts];

	private DriverStation.HALJoystickAxes[] mTmpReportingAxesArr;
	private DriverStation.HALJoystickPOVs[] mTmpReportingPOVsArr;
	private DriverStation.HALJoystickButtons[] mTmpReportingButtonsArr;

	public JoystickProtoBuilder() {
		for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
			mJoystickDataArr.add(RIOToCoprocessorData.CKRIOData.JoystickData.newBuilder());
		}
	}

	public List<RIOToCoprocessorData.CKRIOData.JoystickData.Builder> getJoystickData() {
		mJoystickDataArr.clear();

		mTmpReportingAxesArr = DriverStation.getInstance().getAllJoystickAxes();
		mTmpReportingPOVsArr = DriverStation.getInstance().getAllJoystickPOVs();
		mTmpReportingButtonsArr = DriverStation.getInstance().getAllJoystickButtons();

		for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
			mReportingAxesArr[i].fromObj(mTmpReportingAxesArr[i]);
			mReportingPOVsArr[i].fromObj(mTmpReportingPOVsArr[i]);
			mReportingButtonsArr[i].fromObj(mTmpReportingButtonsArr[i]);

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
