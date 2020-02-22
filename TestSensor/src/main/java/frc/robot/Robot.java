package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

//  private TCS34725 colorSensor;
  private CKCANTCS34725 colorSensorCAN;

  @Override
  public void robotInit() {
    colorSensorCAN = new CKCANTCS34725(1);
//    colorSensor = new TCS34725();
  }


  int printCounter = 0;
  @Override
  public void robotPeriodic() {
    if (printCounter++ % 50 == 0) {
      System.out.println("r:" + colorSensorCAN.getmRGBBuffer()[0] + ",g:"+ colorSensorCAN.getmRGBBuffer()[1] + ",b:" + colorSensorCAN.getmRGBBuffer()[2]);
      System.out.println("c:" + colorSensorCAN.getmCMYKBuffer()[0] + ",m:"+ colorSensorCAN.getmCMYKBuffer()[1] + ",y:" + colorSensorCAN.getmCMYKBuffer()[2] + ",k:" + colorSensorCAN.getmCMYKBuffer()[3]);
      System.out.println("h:" + colorSensorCAN.getmHSVBuffer()[0] + ",s:"+ colorSensorCAN.getmHSVBuffer()[1] + ",v:" + colorSensorCAN.getmHSVBuffer()[2]);
      System.out.println(colorSensorCAN.getColor().toString());
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
