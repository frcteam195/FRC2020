package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.dashjoy.CKDashJoystick;

public class Robot extends TimedRobot {

	private TalonFX leftDriveMaster;
	private TalonFX leftDriveSlave1;
	private TalonFX leftDriveSlave2;
	private TalonFX rightDriveMaster;
	private TalonFX rightDriveSlave1;
	private TalonFX rightDriveSlave2;

	private CKDashJoystick driveJoystick;
	private static final double kJoystickDeadband = 0.05;

	private boolean brakeMode = true;

	@Override
	public void robotInit() {
		driveJoystick = new CKDashJoystick(0);
		leftDriveMaster = new TalonFX(53);
		leftDriveSlave1 = new TalonFX(55);
		leftDriveSlave2 = new TalonFX(58);
		rightDriveMaster = new TalonFX(54);
		rightDriveSlave1 = new TalonFX(56);
		rightDriveSlave2 = new TalonFX(57);

		leftDriveMaster.configFactoryDefault();
		leftDriveSlave1.configFactoryDefault();
		leftDriveSlave2.configFactoryDefault();
		rightDriveMaster.configFactoryDefault();
		rightDriveSlave1.configFactoryDefault();
		rightDriveSlave2.configFactoryDefault();
		
		leftDriveSlave1.follow(leftDriveMaster);
		leftDriveSlave2.follow(leftDriveMaster);

		rightDriveMaster.setInverted(true);
		rightDriveSlave1.follow(rightDriveMaster);
		rightDriveSlave1.setInverted(true);
		rightDriveSlave2.follow(rightDriveMaster);
		rightDriveSlave2.setInverted(true);

		setBrakeMode(false, true);
	}

	private void setBrakeMode(boolean brake, boolean force) {
		if (brakeMode != brake || force) {
			NeutralMode m = brake ? NeutralMode.Brake : NeutralMode.Coast;
			leftDriveMaster.setNeutralMode(m);
			leftDriveSlave1.setNeutralMode(m);
			leftDriveSlave2.setNeutralMode(m);
			rightDriveMaster.setNeutralMode(m);
			rightDriveSlave1.setNeutralMode(m);
			rightDriveSlave2.setNeutralMode(m);

			brakeMode = brake;
		}
	}

	@Override
	public void robotPeriodic() {
		
	}

	@Override
	public void autonomousInit() {

	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopPeriodic() {
		double y = -driveJoystick.getNormalizedAxis(1, kJoystickDeadband);
		double x = driveJoystick.getNormalizedAxis(4, kJoystickDeadband);
		leftDriveMaster.set(ControlMode.PercentOutput, Math.min(Math.max(y+x, -1), 1));
		rightDriveMaster.set(ControlMode.PercentOutput, Math.min(Math.max(y-x, -1), 1));

		if (Math.abs(driveJoystick.getRawAxis(2)) > 0.6) {
			setBrakeMode(true, false);
		} else {
			setBrakeMode(false, false);
		}
	}

	@Override
	public void testPeriodic() {

	}
}
