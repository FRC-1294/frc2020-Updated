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
import frc.robot.subsystems.GameMechSubsystem;
// import frc.robot.subsystems.DistanceSubsystem;
// import frc.robot.subsystems.LimeLightSubsystem;

public class AutoShoot extends CommandBase {
  DrivebaseSubsystem m_driveAuto;
  GameMechSubsystem m_shooter;

  MoveByCommand autoPath;
  // RotateTowardsGoal alignToTarget;
  FeedShooter feedShooterCommand;

  Timer timer = new Timer();

  boolean isFinished;
  boolean shouldShoot;
  boolean shooter;
  boolean shooterReady;
  int xTarget;
  double shooterSpeed;

  final int shootDis = 110;
  final double shootRPM = 6300;
  final int shootMargin = 50;
  final double shootTime = 5.0;

  int step = 0;

  public AutoShoot(DrivebaseSubsystem driveAuto, GameMechSubsystem shooter) {
    m_driveAuto = driveAuto;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    addRequirements(m_driveAuto);

    timer.start();
    timer.reset();
    resetVars();

    autoPath = new MoveByCommand(xTarget, m_driveAuto, 0);
    feedShooterCommand = new FeedShooter(m_shooter, shootTime);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    initShooter();

    //if current leg of path finished, schedule next in sequence
    if (!autoPath.isScheduled() && !feedShooterCommand.isScheduled()) {
      //align with hoop and shoot && get shooter ready
      if (step == 0) {
        if (shooterReady) {
          feedShooterCommand = new FeedShooter(m_shooter, shootTime);
          feedShooterCommand.schedule();
          step++;
        }
      }
      //reverse
      else if (step == 1) {
        autoPath = new MoveByCommand(-60, m_driveAuto, 0);
        autoPath.schedule();
        step++;
      }
      //end command
      else if (step >= 2) {
        shooter = false;
        isFinished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveAuto.setFrontLeftSpeed(0);
    m_driveAuto.setFrontRightSpeed(0);
    m_driveAuto.setRearLeftSpeed(0);
    m_driveAuto.setRearRightSpeed(0);
    m_shooter.setShooter(0);
    m_driveAuto.setCurrentAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }

  private void initShooter() {
    shooterSpeed = m_shooter.getShooterVelocity();
    m_shooter.setShooterPID(6300);

    boolean atSpeed = false;
    boolean timeHold = false;

    if(Math.abs(shooterSpeed) >= 6200 && Math.abs(shooterSpeed) <= 6400) {
      atSpeed = true;
    }

    if (atSpeed) {
      if(timer.get() >= 1) {
        timeHold = true;
      }
    }
    else {
      timer.reset();
    }

    if(atSpeed && timeHold) 
      shooterReady = true;
    else
      shooterReady = false;
  }

  private void resetVars() {
    isFinished = false;
    shooter = false;
    shooterReady = false;
  }
}
