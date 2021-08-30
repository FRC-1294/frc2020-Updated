/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UltraFuseCommand extends CommandBase {
  UltrasonicSubsystem ultrasonic;
  DrivebaseSubsystem driveSub;
  final double kHoldDistance = 12.0; 
  final double kP = 0.05;
  double speed;
  double motorTurn;
  boolean isFinished;
  Timer timer = new Timer();

  public UltraFuseCommand(DrivebaseSubsystem driver, UltrasonicSubsystem ultra) {
    // Use addRequirements() here to declare subsystem dependencies.
    speed = 0.0;
    motorTurn = 0.0;
    ultrasonic = ultra;
    driveSub = driver;
    
    addRequirements(Robot.ultrasonic);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double[] currentDistance = new double[] {ultrasonic.getSensourLeft(), ultrasonic.getSensourRight()};
    double currentDistance = ultrasonic.getSensourLeft();
   
    if (currentDistance <= ultrasonic.MIN_DIS) {
      if (timer.get() > 0.1)
        isFinished = true;
    }
    else {
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.setMode("brake");
    driveSub.setFrontLeftSpeed(0);
    driveSub.setFrontRightSpeed(0);
    driveSub.setRearLeftSpeed(0);
    driveSub.setRearRightSpeed(0);
    driveSub.setMode("coast");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}