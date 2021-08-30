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
import frc.robot.subsystems.UltrasonicSubsystem;

public class WallChecker extends CommandBase {
  final int PIDSlot = 1;
  final double margin = 90;
  int HAL9000 = 0;
  int amount = 0;
  UltrasonicSubsystem dracula;
  DrivebaseSubsystem whee;
  double[] threeMusketeers = new double[3];
  double[] threeAngles = new double[3];
  TurnByCommand tokyoDrift;
  boolean isFinished;

  Timer timer = new Timer();

  public WallChecker(int amount, DrivebaseSubsystem driver, UltrasonicSubsystem ultraBurst) {
    this.amount = amount;
    dracula = ultraBurst;
    whee = driver;
  }

  @Override
  public void initialize() {
    whee.setFrontLeftSpeed(0);
    whee.setFrontRightSpeed(0);
    whee.setRearLeftSpeed(0);
    whee.setRearRightSpeed(0);
    
    threeMusketeers = new double[3];
    threeAngles = new double[3];

    timer = new Timer();
    timer.start();
    timer.reset();
    
    HAL9000 = 1;
    isFinished = false;
    threeMusketeers[0] = dracula.getSensourLeft();
    threeAngles[0] = whee.getCurrentAngle();
    tokyoDrift = new TurnByCommand(amount, whee, PIDSlot);
    tokyoDrift.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!tokyoDrift.isScheduled()){
      if(HAL9000 == 1) {
        if (threeMusketeers[1] == 0) {
          threeMusketeers[1] = dracula.getSensourLeft();
          threeAngles[1] = whee.getCurrentAngle();
        }

        if (timer.get() > 0.2) {
          System.out.println();
          tokyoDrift = new TurnByCommand(-amount * 2, whee, PIDSlot);
          tokyoDrift.schedule();
          HAL9000 = 2;
        }
      }
      else if(HAL9000 == 2) {
        if (threeMusketeers[2] == 0) {
          threeMusketeers[2] = dracula.getSensourLeft();
          threeAngles[2] = whee.getCurrentAngle();
        }

        if (timer.get() > 0.2) {
          tokyoDrift = new TurnByCommand(amount, whee, PIDSlot);
          tokyoDrift.schedule();
          HAL9000 = 3;
        }
      }
      else if (HAL9000 == 3) {
        double baseVal = threeMusketeers[0] * Math.abs(Math.cos(amount * Math.PI / 180));
        double[] lowVal = new double[3];
        
        for (int i = 0; i < 3; i++) {
          if (threeAngles[i] < -margin) {
            lowVal[i] = threeAngles[i] + margin;
          }
          else if (threeAngles[i] > margin) {
            lowVal[i] = threeAngles[i] - margin;
          }
        }

        if (threeMusketeers[1] * Math.abs(Math.cos(amount * Math.PI / 180)) > baseVal+margin) {
          whee.setWall(false);
        }
        else if (threeMusketeers[2] * Math.abs(Math.cos(amount* Math.PI / 180)) > baseVal+margin) {
          whee.setWall(false);
        }
        else {
          whee.setWall(true);

        }

        isFinished = true;
      }
    }
    else {
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    whee.setFrontRightSpeed(0);
    whee.setFrontLeftSpeed(0);
    whee.setRearRightSpeed(0);
    whee.setRearLeftSpeed(0);

    System.out.println("\n\n\n\n");
    System.out.println(threeMusketeers[0] * Math.abs(Math.cos(threeAngles[0] * Math.PI / 180)) + " " + threeAngles[0]);
    System.out.println(threeMusketeers[1] * Math.abs(Math.cos(threeAngles[1] * Math.PI / 180)) + " " + threeAngles[1]);
    System.out.println(threeMusketeers[2] * Math.abs(Math.cos(threeAngles[2] * Math.PI / 180)) + " " + threeAngles[2]);
    System.out.println(Math.abs(Math.cos(60 * Math.PI / 180)));

    System.out.println(whee.getWall() + " ENDED");
    System.out.println("\n\n\n\n");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
