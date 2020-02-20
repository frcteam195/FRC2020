package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

  private TCS34725 colorSensor;

  @Override
  public void robotInit() {
    colorSensor = new TCS34725();
  }


  int printCounter = 0;
  @Override
  public void robotPeriodic() {
    if (printCounter++ % 50 == 0) {
      System.out.println(colorSensor.getRawData().toString() + colorSensor.getColorOutput().toString());
    }
  }


  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testPeriodic() {
  }
}
