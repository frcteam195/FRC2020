package frc.robot.dashjoy;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

@SuppressWarnings("FieldCanBeLocal")
public class CKDashJoystick {

	private final int mPort;

	private boolean[] prevButtonVal;
	private boolean[] prevTriggerVal;

	private Joystick backupJoystick;

	private DashJoyController dashJoyController = DashJoyController.getInstance();

	//Expire after 20ms, value in microseconds
	private static final long EXPIRATION_TIME = 20000;

	public CKDashJoystick(int port) {
		mPort = port;
		prevButtonVal = new boolean[DashJoyController.BUTTON_ARR_SIZE];
		prevTriggerVal = new boolean[DashJoyController.MAX_AXES];

		for (int i = 0; i < prevButtonVal.length; i++) {
			prevButtonVal[i] = false;
		}

		for (int i = 0; i < prevTriggerVal.length; i++) {
			prevTriggerVal[i] = false;
		}

		backupJoystick = new Joystick(mPort);
	}

	public int getPOV() {
		return getPOV(0);
	}

	public int getPOV(int pov) {
		if (isTimestampValid())
			return dashJoyController.getPOV(mPort);
		if (backupJoystick != null)
			return backupJoystick.getPOV();
		else
			return -1;
	}

	public double getRawAxis(int axis) {
		if (isTimestampValid())
			return dashJoyController.getRawAxis(mPort, axis);
		if (backupJoystick != null)
			return backupJoystick.getRawAxis(axis);
		else
			return 0;
	}

	public double getNormalizedAxis(int axis, double deadband) {
		return normalizeJoystickWithDeadband(getRawAxis(axis), deadband);
	}

	public double getSmoothedAxis(int axis, double deadband, double power) {
		double x = getRawAxis(axis);
		return Math.signum(x) * Math.min(Math.pow(Math.abs(normalizeJoystickWithDeadband(x, deadband)), power), 1);
	}

	public boolean getRawButton(int button) {
		if (isTimestampValid())
			return dashJoyController.getRawButton(mPort, button-1);
		if (backupJoystick != null)
			return backupJoystick.getRawButton(button);
		else
			return false;
	}

	private boolean currButtonRising;
	private boolean retValButtonRising;
	public boolean getRisingEdgeButton(int button) {
		try {
			currButtonRising = getRawButton(button);
			retValButtonRising = (currButtonRising != prevButtonVal[button-1]) && currButtonRising;
			setPrevButtonVal(button-1, currButtonRising);
			return retValButtonRising;
		} catch(Exception ex) {
			return false;
		}
	}

	private boolean currButtonFalling;
	private boolean retValButtonFalling;
	public boolean getFallingEdgeButton(int button) {
		try {
			currButtonFalling = getRawButton(button);
			retValButtonFalling = (currButtonFalling != prevButtonVal[button-1]) && !currButtonFalling;
			setPrevButtonVal(button-1, currButtonFalling);
			return retValButtonFalling;
		} catch(Exception ex) {
			return false;
		}
	}

	private synchronized void setPrevButtonVal(int idx, boolean val) {
		if (idx <= prevButtonVal.length) {
			prevButtonVal[idx] = val;
		}
	}

	private synchronized void setPrevTriggerVal(int idx, boolean val) {
		if (idx <= prevTriggerVal.length) {
			prevTriggerVal[idx] = val;
		}
	}

	private boolean isTimestampValid() {
		return (HALUtil.getFPGATime() - dashJoyController.getLastUpdateTimestamp()) < EXPIRATION_TIME;
	}

	private double normalizeJoystickWithDeadband(double val, double deadband) {
		val = (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;

		if (val != 0)
			val = Math.signum(val) * ((Math.abs(val) - deadband) / (1.0 - deadband));

		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	public boolean isAxisInputActive() {
		if (backupJoystick != null)
			for (int i = 0; i < backupJoystick.getAxisCount(); i++) {
				if (Math.abs(getRawAxis(i)) > 0.1) {
					return true;
				}
			}
		return false;
	}

	public boolean isButtonInputActive() {
		if (backupJoystick != null)
			for (int i = 1; i <= backupJoystick.getButtonCount(); i++) {
				if (getRawButton(i)) {
					return true;
				}
			}
		return false;
	}

	private boolean currTrigRising;
	private boolean retValTrigRising;
	public boolean getRisingEdgeTrigger(int axis, double threshold) {
		try {
			currTrigRising = Math.abs(getRawAxis(axis)) > threshold;
			retValTrigRising = (currTrigRising != prevTriggerVal[axis]) && currTrigRising;
			setPrevTriggerVal(axis, currTrigRising);
			return retValTrigRising;
		} catch(Exception ex) {
			return false;
		}
	}

	public void setRumble(double val) {
//		backupJoystick.setRumble(GenericHID.RumbleType.kLeftRumble, val);
		if (backupJoystick != null)
			backupJoystick.setRumble(GenericHID.RumbleType.kRightRumble, val);
	}

	public boolean isPOVInputActive() {
		return getPOV() != -1;
	}

	public boolean isJoystickInputActive() {
		return isAxisInputActive() || isButtonInputActive() || isPOVInputActive();
	}
}
