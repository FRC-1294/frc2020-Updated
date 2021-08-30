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

public class StalkerRoomba extends CommandBase {
  DrivebaseSubsystem m_robotDrive;
  UltrasonicSubsystem m_ultra;
  WallChecker wallChecker;
  TurnByCommand turner;
  double speed;
  boolean shouldCheckWall, hasChecked, hasTurned, isFinished;
  Timer wallCheckTimer, ultraUpdateTimer;
  double currentDistance;
  double targetDis;
  final double ultraMargin = 0.2;
  final double offSet = 24;

  public boolean isWall;

  public StalkerRoomba(final double dis, final DrivebaseSubsystem robotDrive, final UltrasonicSubsystem ultra) {
    targetDis = dis;
    currentDistance = 0.0;
    m_robotDrive = robotDrive;
    m_ultra = ultra;
    wallCheckTimer = new Timer();
    ultraUpdateTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //   m_robotDrive.setFrontLeftSpeed(0);
  //   m_robotDrive.setFrontRightSpeed(0);
  //   m_robotDrive.setRearLeftSpeed(0);
  //   m_robotDrive.setRearRightSpeed(0);

  //   resetVars();

  //   wallChecker = new WallChecker(40, m_robotDrive, m_ultra);
  //   turner = new TurnByCommand(90, m_robotDrive, 0);
  //   wallCheckTimer.reset();
  //   wallCheckTimer.start();
  //   ultraUpdateTimer.reset();
  //   ultraUpdateTimer.start();
  // }

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   currentDistance = m_ultra.getDistance();
    
  //   if (!wallChecker.isScheduled()) { //if code isn't simultaneously (shut up skye) running
  //     if (shouldCheckWall && !hasChecked) { //if needs to check
  //       System.out.println("Wall Checker Scheduled");
  //       hasChecked = true;
  //       //wallChecker.schedule();
  //     }
  //     else if (!isWall)  {//if should continue moving because is not wall
  //       if(currentDistance < targetDis + offSet){
  //         System.out.println("IN RANGE");

  //         if (wallCheckTimer.get() >= 1) { // if it's been a second since last checking wall
  //           shouldCheckWall = true;
  //         }

  //         //setting speed
  //         m_robotDrive.setFrontLeftSpeed(0);
  //         m_robotDrive.setFrontRightSpeed(0);
  //         m_robotDrive.setRearLeftSpeed(0);
  //         m_robotDrive.setRearRightSpeed(0);
  //       }
  //       else {
  //         System.out.println("OUT OF RANGE");
  //         wallChecker.cancel();
  //         wallCheckTimer.reset();
  //         hasChecked = false;
  //         shouldCheckWall = false;

  //         //setting speed
  //         m_robotDrive.setFrontLeftSpeed(0.3);
  //         m_robotDrive.setFrontRightSpeed(0.3);
  //         m_robotDrive.setRearLeftSpeed(0.3);
  //         m_robotDrive.setRearRightSpeed(0.3);
  //       }
  //     }
  //     //if at a wall
  //     else if (isWall) {
  //       if (!turner.isScheduled() ) { //if turner isn't alreado scheduled
  //         //first time, rotate left
  //         if (!hasTurned) {
  //           System.out.println("Turner Scheduled");
  //           //turner.schedule();
  //           hasTurned = true;
  //         }

  //         // phillip stop doing whateve ryou're doing 
  //         // izzy shut up/skye shut up TM

  //         //once rotated, end command
  //         else if (hasTurned) {
  //           System.out.println("Command Finished");
  //           isFinished = true;
  //         }
  //       }
  //     }
  //   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_robotDrive.setFrontLeftSpeed(0);
    m_robotDrive.setFrontRightSpeed(0);
    m_robotDrive.setRearLeftSpeed(0);
    m_robotDrive.setRearRightSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }

  public void resetVars() {
    speed = 0;
    shouldCheckWall = false;
    hasChecked = false;
    hasTurned = false;
    isFinished = false;
    wallCheckTimer.reset();
    ultraUpdateTimer.reset();
    currentDistance = 0.0;

    isWall = false;
  }
}