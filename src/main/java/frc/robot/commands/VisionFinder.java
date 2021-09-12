/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class VisionFinder extends CommandBase {
  DrivebaseSubsystem drive;
  LimeLightSubsystem Brutus;

  boolean isFinished;
  double currentAngle;
  double targetPositionRotations = 0.097;
  double[] startPos;
  double[] lastSpeed;
  double[] speed;

  Timer timer;
  Timer timerV2ElectricBoogaloo;

  public VisionFinder(DrivebaseSubsystem driver, LimeLightSubsystem visionSub) {
    isFinished = false;
    drive = driver;
    Brutus = visionSub;
  }

  @Override
  public void initialize() {
    drive.setFrontLeftSpeed(0);
    drive.setFrontRightSpeed(0);
    drive.setRearLeftSpeed(0);
    drive.setRearRightSpeed(0);
    
    timer = new Timer();
    timer.start();
    timer.reset();

    timerV2ElectricBoogaloo = new Timer();
    timerV2ElectricBoogaloo.start();
    timerV2ElectricBoogaloo.reset();
    
    startPos = new double[] {drive.getFrontLeftPosition(), drive.getFrontRightPosition()};
    speed = new double [] {0.3, -0.3};
    lastSpeed = new double[] {0.3, -0.3};
    currentAngle = 0;
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = (drive.getFrontLeftPosition()-startPos[0]) / targetPositionRotations;
    //System.out.println("SPEED: " + speed[0] + ", " + speed[1]);
    Brutus.setPipeline(0);

    if (Brutus.isDetected()) {
      if (timerV2ElectricBoogaloo.get() > 1) isFinished = true;

      speed[0] = 0;
      speed[1] = 0;
    }
    else if(timer.get() > 0.5 * 2){
      
      if (currentAngle > 90) {
        speed[0] = -0.3;
        speed[1] = 0.3;
        lastSpeed[0] = speed[0];
        lastSpeed[1] = speed[1];
      }
      else if (currentAngle < -90) {
        speed[0] = 0.3;
        speed[1] = -0.3;
        lastSpeed[0] = speed[0];
        lastSpeed[1] = speed[1];
      }
      else if (!Brutus.isDetected()) {
        speed[0] = -lastSpeed[0];
        speed[1] = -lastSpeed[1];
      }

      if(!Brutus.isDetected()) timerV2ElectricBoogaloo.reset();
      timer.reset();
    }

    drive.setFrontLeftSpeed(speed[0]);
    drive.setFrontRightSpeed(speed[1]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setFrontLeftSpeed(0);
    drive.setFrontRightSpeed(0);
    drive.setRearLeftSpeed(0);
    drive.setRearRightSpeed(0);
    
    Brutus.setPipeline(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
