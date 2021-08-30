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

public class AutoNavCommand extends CommandBase {
  DrivebaseSubsystem m_driveAuto;
  DistanceSubsystem m_ultra;
  GameMechSubsystem m_shooter;
  LimeLightSubsystem m_vision;

  AutoPath autoPath;
  VisionFinder finder;
  AlignToShoot alignToTarget;
  TurnByCommand turner;
  MoveByCommand mover;
  UltraFuseCommand ultraFuse;

  Timer timer = new Timer();

  boolean isFinished;
  boolean left1;
  boolean left2;
  boolean shooter;
  boolean shooterReady;
  boolean loopComplete;
  int targetAngle;
  int xTarget;
  int yTarget;
  int moveAmount;
  int shooterSpeed;

  final int autoPathMargin = 2;
  final int robotFollowDis = 5*12;
  final int amountPastAutoLine = 5*12;
  final int shootDis = 11*12;
  final int shootRPM = 4200;
  final int shootMargin = 50;
  final double shootTime = 5.0;

  int step = 0;

  public AutoNavCommand(DrivebaseSubsystem driveAuto, DistanceSubsystem ultra, GameMechSubsystem shooter, LimeLightSubsystem vision) {
    m_driveAuto = driveAuto;
    m_ultra = ultra;
    m_shooter = shooter;
    m_vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    addRequirements(m_driveAuto);
    addRequirements(m_ultra);

    timer.start();
    timer.reset();

    resetVars();

    autoPath = new AutoPath(0, 0, left1, left2, m_driveAuto);
    ultraFuse = new UltraFuseCommand(m_driveAuto, m_ultra);
    finder = new VisionFinder(m_driveAuto, m_vision);
    turner = new TurnByCommand(180, m_driveAuto, 0);
    mover = new MoveByCommand(5*12, m_driveAuto, 0);
    alignToTarget = new AlignToShoot(m_driveAuto, m_ultra, m_shooter, m_vision, shootDis, true);

    ultraFuse.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //distance left to travel
    double xRem = Math.abs(xTarget - m_driveAuto.getAmountTraveled(0));
    double yRem = Math.abs(yTarget - m_driveAuto.getAmountTraveled(1));

    //if current leg of path finished, schedule next in sequence
    if (!finder.isScheduled() && !turner.isScheduled() && !mover.isScheduled() && !alignToTarget.isScheduled() && ultraFuse.isScheduled()) {
        //find edge of vision target

        //align and shoot
        if (step == 0) {
          alignToTarget = new AlignToShoot(m_driveAuto, m_ultra, m_shooter, m_vision, shootDis, true);
          alignToTarget.schedule();
          step++;
        }

        //turn around
        else if (step == 1) {
          turner = new TurnByCommand(180, m_driveAuto, 0);
          turner.schedule();
          step++;
        }

        //return to startPoint
        else if (step == 2) {
          xTarget = 0;
          yTarget = 0;
          moveAmount = (int) Math.sqrt(Math.pow(m_driveAuto.getAmountTraveled(0), 2) + Math.pow(m_driveAuto.getAmountTraveled(1), 2));
          mover = new MoveByCommand(moveAmount, m_driveAuto, 0);
          mover.schedule();
          step++;
        }

        //turn to the dark (our) side
        else if (step == 3) {
          moveAmount = 0;
          turner = new TurnByCommand(0-m_driveAuto.getCurrentAngle(), m_driveAuto, 0);
          turner.schedule();
          step++;
        }

        //move past auto line
        else if (step == 4) {
          xTarget = 0;
          yTarget = 0;
          moveAmount = amountPastAutoLine;
          mover = new MoveByCommand(moveAmount, m_driveAuto, 0);
          mover.schedule();
          step++;
        }

        //end command
        else if (step >= 5) {
          isFinished = true;
        }
    }
    

    //if obstacle detected during PID
    if (!ultraFuse.isScheduled() && !finder.isScheduled() && !turner.isScheduled() && !alignToTarget.isScheduled()) {
      //if stopping necessary
      if ((!m_driveAuto.getTurning() && !alignToTarget.isScheduled())) {
        mover.cancel();

        if (m_ultra.getSensourLeft() <= m_ultra.MIN_DIS) {
          //avoid?
        }
        else {
          if (targetAngle - m_driveAuto.getCurrentAngle() == 0) {
            left1 = false;
            left2 = false;
          } 
          else if (targetAngle - m_driveAuto.getCurrentAngle() == 90) {
            left1 = false;
            left2 = true;
          } 
          else if (targetAngle - m_driveAuto.getCurrentAngle() == 180) {
            left1 = true;
            left2 = true;
          }

          if (xTarget == 0 && yTarget == 0) {
            moveAmount = (int) Math.sqrt(Math.pow(m_driveAuto.getAmountTraveled(0), 2) + Math.pow(m_driveAuto.getAmountTraveled(1), 2));
            if (moveAmount >= autoPathMargin) {
              mover = new MoveByCommand(moveAmount, m_driveAuto, 0);
              mover.schedule();
            }
          }
          else {
            //resechedule path if obstacle avoided
            if (xRem >= autoPathMargin || yRem >= autoPathMargin) {
              autoPath = new AutoPath(xRem, yRem, left1, left2, m_driveAuto);
              autoPath.schedule();
            }
          }
        }
      }
      
      //keep ultraFuse running to check if obstacle moves
      ultraFuse.schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveAuto.setFrontLeftSpeed(0);
    m_driveAuto.setFrontRightSpeed(0);
    m_driveAuto.setRearLeftSpeed(0);
    m_driveAuto.setRearRightSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }

  public void resetVars() {
    isFinished = false;
    left1 = false;
    left2 = false;
    shooter = false;
    shooterReady = false;
    loopComplete = false;

    targetAngle = 0;
    xTarget = 0;
    yTarget = 0;
    shooterSpeed = 0;
    moveAmount = 0;
  }
}