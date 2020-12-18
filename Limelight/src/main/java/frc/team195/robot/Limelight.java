package frc.team195.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight {
    private NetworkTable table;
    private double[] initialize;

    public Limelight(){
        initialize = new double[6];
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void run() {
        NetworkTableEntry Values = table.getEntry("camtran");
        double[] Return = Values.getDoubleArray(initialize);
        System.out.println("X: " + Return[0] + ", Y: " + Return[1] + ", Z: " + Return[2] + ", Pitch: " + Return[3] + ", Yaw: " + Return[4] + ", Roll: " + Return[5]);

    }
}
