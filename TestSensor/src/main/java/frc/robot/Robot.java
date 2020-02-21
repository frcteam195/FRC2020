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
      System.out.println("r:" + colorSensor.getmRGBBuffer()[0] + ",g:"+ colorSensor.getmRGBBuffer()[1] + ",b:" + colorSensor.getmRGBBuffer()[2]);
      System.out.println("c:" + colorSensor.getmCMYKBuffer()[0] + ",m:"+ colorSensor.getmCMYKBuffer()[1] + ",y:" + colorSensor.getmCMYKBuffer()[2] + ",k:" + colorSensor.getmCMYKBuffer()[3]);
      System.out.println("h:" + colorSensor.getmHSVBuffer()[0] + ",s:"+ colorSensor.getmHSVBuffer()[1] + ",v:" + colorSensor.getmHSVBuffer()[2]);
      System.out.println(colorSensor.getColor().toString());
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
