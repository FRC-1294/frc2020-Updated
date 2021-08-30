/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/////////////shacuando was here
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLightSubsystem extends SubsystemBase {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry tv = table.getEntry("tv");
  
  public static boolean targetAquired = false;
  public static double horizontalOffset = 0.0f;

  public double distanceFromTarget;

  public LimeLightSubsystem() {}

  public void pollCamera() {
    targetAquired = tv.getDouble(0) != 0;
    horizontalOffset = tx.getDouble(0.0);

    SmartDashboard.putBoolean("Limelight/TargetAquired", targetAquired);
    SmartDashboard.putNumber("Limelight/HorizontalOffset", horizontalOffset);
  }

  public double getHorizontalOffSet(){
    return -horizontalOffset;
  }

  public double getHeadingError() {
    return tx.getDouble(0.0) * -1;
  }

  public boolean isDetected() {
    return targetAquired;
  }

  public void setPipeline(int mode) {
    //opMode
    if (mode == 0) {
      table.getEntry("pipeline").setNumber(0);
      table.getEntry("camMode").setNumber(0);
      table.getEntry("ledMode").setNumber(3);
    }
    //driveMode
    else if (mode == 1) {
      table.getEntry("pipeline").setNumber(1);
      table.getEntry("camMode").setNumber(1);
      table.getEntry("ledMode").setNumber(1);
    }
  }

  public static void getInRange()  {
    // float kpDistance = -0.1f;
    // float currentDistance = Estimate_Distance();

    // float distanceError = desiredDistance - currentDistance;
    // drivingAdjust = kpDistance + distanceError;

    // leftCommand += distanceAdjust;
    // rightCommand += distanceAdjust;
  }

  @Override
  public void periodic() {   
    pollCamera();
    // System.out.println("HERE");
  } 
}