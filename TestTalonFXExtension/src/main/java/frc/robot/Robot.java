package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motorcontrol.CKTalonFX;
import frc.robot.motorcontrol.PDPBreaker;

public class Robot extends TimedRobot {

	private CKTalonFX ckfxMaster = null;
	private CKTalonFX ckfxSlave = null;

	@Override
	public void robotInit() {
		ckfxMaster = new CKTalonFX(35, true, PDPBreaker.B40A);;
		ckfxSlave = new CKTalonFX(61, ckfxMaster, PDPBreaker.B40A, true);;
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
	public void teleopInit() {
		ckfxMaster.set(ControlMode.PercentOutput, 0.2);
	}

	@Override
	public void teleopPeriodic() {

	}

	@Override
	public void testPeriodic() {
	}
}
