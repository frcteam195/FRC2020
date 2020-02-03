package com.team195.frc.auto.actions;

import com.team195.frc.constants.AutoConstants;
import com.team195.frc.constants.Constants;
import com.team195.frc.controllers.LEDController;
import com.team195.frc.subsystems.Turret;
import com.team195.lib.util.TimeoutTimer;

public class WaitForHatchOrTimeoutAction implements Action {
	private static final Turret mTurret = Turret.getInstance();
	private static final LEDController mLEDController = LEDController.getInstance();

	private final TimeoutTimer mTimeoutTimer;


	public WaitForHatchOrTimeoutAction() {
		this(AutoConstants.kLimitSwitchTriggerTimeout);
	}

	public WaitForHatchOrTimeoutAction(double timeout) {
		mTimeoutTimer = new TimeoutTimer(timeout);
	}

	@Override
	public boolean isFinished() {
		return (mTimeoutTimer.isTimedOut() || !mTurret.getLimitSwitchValue());
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
		if (!mTurret.getLimitSwitchValue()) {
			mLEDController.setLEDColor(Constants.kGotGamePieceColor);
			mLEDController.setRequestedState(LEDController.LEDState.BLINK);
		}
	}

	@Override
	public void start() {
		mTimeoutTimer.reset();
	}
}
