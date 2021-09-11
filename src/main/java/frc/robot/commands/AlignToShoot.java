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
import frc.robot.subsystems.DistanceSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class AlignToShoot extends CommandBase {
  DrivebaseSubsystem m_driveAuto;
  DistanceSubsystem m_ultra;
  GameMechSubsystem m_shooter;
  LimeLightSubsystem m_vision;

  MoveByCommand autoPath;
  RotateTowardsGoal alignToTarget;
  FeedShooter feedShooterCommand;

  Timer timer = new Timer();

  boolean isFinished;
  boolean shouldShoot;
  boolean shooter;
  boolean shooterReady;
  int xTarget;
  double shooterSpeed;

  final int shootDis = 110;
  final double ticksPerRev = -2.59;
  final double shootRPM = 6300;
  final int shootMargin = 50;
  final double shootTime = 5.0;

  int step = 0;

  public AlignToShoot(DrivebaseSubsystem driveAuto, DistanceSubsystem ultra, GameMechSubsystem shooter, LimeLightSubsystem vision, int targetDis, boolean shoot) {
    m_driveAuto = driveAuto;
    m_ultra = ultra;
    m_shooter = shooter;
    m_vision = vision;

    shouldShoot = shoot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    addRequirements(m_driveAuto);
    addRequirements(m_ultra);

    timer.start();
    timer.reset();
    resetVars();

    autoPath = new MoveByCommand(xTarget, m_driveAuto, 0);
    feedShooterCommand = new FeedShooter(m_shooter, shootTime);
    alignToTarget = new RotateTowardsGoal(m_vision, m_driveAuto);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shouldShoot) initShooter();

    //if current leg of path finished, schedule next in sequence
    if (!autoPath.isScheduled() && !alignToTarget.isScheduled() && !feedShooterCommand.isScheduled()) {
      //align with hoop and shoot && get shooter ready
      if (step == 0) {
        alignToTarget = new RotateTowardsGoal(m_vision, m_driveAuto);
        alignToTarget.schedule();
        step++;
        m_driveAuto.setWall(false);
      }
      //move until shooting distance
      else if (step == 1) {
        xTarget = (int)m_ultra.getSensourLeft() - shootDis;
        autoPath = new MoveByCommand(xTarget, m_driveAuto, 0);
        autoPath.schedule();
        step++;
      }
      //realign
      else if (step == 2) {
        shooter = true;
        alignToTarget = new RotateTowardsGoal(m_vision, m_driveAuto);
        alignToTarget.schedule();
        step++;
      }
      //SHOOT
      else if (step == 3) {
        if (shouldShoot) {
          if (shooterReady) {
            feedShooterCommand = new FeedShooter(m_shooter, shootTime);
            feedShooterCommand.schedule();
            step++;
          }
        }
        else {
          step++;
        }
      }
      //reverse
      else if (step == 4) {
        autoPath = new MoveByCommand(-50, m_driveAuto, 0);
        autoPath.schedule();
        step++;
      }
      //end command
      else if (step >= 5) {
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
    if (shooter) {
      shooterSpeed = m_shooter.getShooterVelocity()/ticksPerRev;
      m_shooter.setShooterPID(7750*ticksPerRev);

      boolean atSpeed = false;
      boolean timeHold = false;

      System.out.println(shooterSpeed + " " + (shootRPM + shootMargin));

      if(Math.abs(shooterSpeed) >= 6300 && Math.abs(shooterSpeed) <= 6400) {
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
    else {
      m_shooter.setShooter(0);
    }
  }

  private void resetVars() {
    isFinished = false;
    shooter = false;
    shooterReady = false;
  }
}
