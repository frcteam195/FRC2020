package com.team195.frc.constants;

public class DeviceIDConstants {
	// Do not change anything after this line unless you rewire the robot and
	// update the spreadsheet!
	// Port assignments should match up with the spreadsheet here:
	// https://docs.google.com/spreadsheets/d/179YszqnEWPWInuHUrYJnYL48LUL7LUhZrnvmNu1kujE/edit#gid=0

	/* I/O */
	// (Note that if multiple talons are dedicated to a mechanism, any sensors
	// are attached to the master)

	// Drive
	public static final int kRightDriveMasterId = 4;
	public static final int kRightDriveSlaveAId = 5;
	public static final int kRightDriveSlaveBId = 6;
	public static final int kLeftDriveMasterId = 1;
	public static final int kLeftDriveSlaveAId = 2;
	public static final int kLeftDriveSlaveBId = 3;

	// Control Panel Manipulator
	public static final int kCPMRotationId = 13;

	//Climber
	public static final int kClimberWinchMotor = 14;
	public static final int kClimberArmMotor = 15;



	//Turret
	public static final int kTurretMotorId = 8;
	public static final int kHoodMotorId = 9;
	public static final int kLeftShooterMotorId = 10;
	public static final int kRightShooterMotorId = 11;

	//Intake Feeder
	public static final int kFeederMotorId = 7;
	public static final int kIntakeMotorId = 12;


	// Solenoids
	public static final int kControlPanelExtenderSolenoid = 0;

	public static final int kCANifierLEDId = 30;
	public static final int kNeoPixelPWMPort = 0;
}
