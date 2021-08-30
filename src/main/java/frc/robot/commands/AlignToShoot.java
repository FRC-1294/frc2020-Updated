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
import frc.robot.commands.FeedShooterCommand;

public class AlignToShoot extends CommandBase {
  DrivebaseSubsystem m_driveAuto;
  DistanceSubsystem m_ultra;
  GameMechSubsystem m_shooter;
  LimeLightSubsystem m_vision;
  WallChecker glasses;

  MoveByCommand autoPath;
  DictatorLocator alignToTarget;
  FeedShooterCommand feedShooterCommand;
  UltraFuseCommand ultraFuse;

  Timer timer = new Timer();

  boolean isFinished;
  boolean left1;
  boolean left2;
  boolean shouldShoot;
  boolean shooter;
  boolean shooterReady;
  boolean loopComplete;
  int iterations;
  int targetAngle;
  int xTarget;
  int yTarget;
  double shooterSpeed;
  int shootDis = 110;
  double[] startAmount;

  final int autoPathMargin = 2;
  final int robotFollowDis = 5*12;
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
    glasses = new WallChecker(40, m_driveAuto, m_ultra);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    addRequirements(m_driveAuto);
    addRequirements(m_ultra);

    timer.start();
    timer.reset();
    resetVars();

    startAmount = new double[] {m_driveAuto.getAmountTraveled(0), m_driveAuto.getAmountTraveled(1)};
    left1 = false;
    left2 = false;
    autoPath = new MoveByCommand(xTarget, m_driveAuto, 0);
    ultraFuse = new UltraFuseCommand(m_driveAuto, m_ultra);
    feedShooterCommand = new FeedShooterCommand(m_shooter, shootTime);
    alignToTarget = new DictatorLocator(m_vision, m_driveAuto);

    ultraFuse.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //distance left to travel
    double xRem = Math.abs(xTarget - m_driveAuto.getAmountTraveled(0) + startAmount[0]);
    double yRem = Math.abs(yTarget - m_driveAuto.getAmountTraveled(1) + startAmount[1]);

    // System.out.println("Step: " + step + " " + autoPath.isScheduled());
    // System.out.println(!autoPath.isScheduled() && !alignToTarget.isScheduled() && !feedShooterCommand.isScheduled() && ultraFuse.isScheduled() && !glasses.isScheduled());

    if (shouldShoot) checkShooter();

    // System.out.println(step);

    //if current leg of path finished, schedule next in sequence
    if (!autoPath.isScheduled() && !alignToTarget.isScheduled() && !feedShooterCommand.isScheduled() && ultraFuse.isScheduled() && !glasses.isScheduled()) {
      //if (Math.abs(xRem) <= autoPathMargin && Math.abs(yRem) <= autoPathMargin) {
        //align with hoop and shoot && get shooter ready
        if (step == 0) {
          alignToTarget = new DictatorLocator(m_vision, m_driveAuto);
          alignToTarget.schedule();
          step++;
          m_driveAuto.setWall(false);
        }
        //check wall
        else if(step == 1){
          if(!m_driveAuto.getWall() && shouldShoot){
            glasses = new WallChecker(20, m_driveAuto, m_ultra);
            glasses.schedule();
          }
          else{
            m_driveAuto.setWall(false);
            step++;
          }
        }
        //move until shooting distance
        else if (step == 2) {
          xTarget = (int)m_ultra.getSensourLeft() - shootDis;
          autoPath = new MoveByCommand(xTarget, m_driveAuto, 0);
          autoPath.schedule();
          step++;
        }
        //realign
        else if (step == 3) {
          shooter = true;
          alignToTarget = new DictatorLocator(m_vision, m_driveAuto);
          alignToTarget.schedule();
          step++;
        }
        //SHOOT
        else if (step == 4) {
          if (shouldShoot) {
            if (shooterReady) {
              feedShooterCommand = new FeedShooterCommand(m_shooter, shootTime);
              feedShooterCommand.schedule();
              step++;
            }
          }
          else {
            step++;
          }
        }
        //reverse
        else if (step == 5) {
          autoPath = new MoveByCommand(-50, m_driveAuto, 0);
          autoPath.schedule();
          step++;
        }
        //end command
        else if (step >= 6) {
          shooter = false;
          isFinished = true;
        }
    }

    // System.out.println(shooterReady);
    

    // if obstacle detected during PID ONLY
    if (!ultraFuse.isScheduled() && !alignToTarget.isScheduled() && !feedShooterCommand.isScheduled()) {
      //if stopping necessary
      if ((!m_driveAuto.getTurning() && !alignToTarget.isScheduled())) {
        autoPath.cancel();

        if (m_ultra.getSensourLeft() <= m_ultra.MIN_DIS) {
          //avoid?
        }
        else {
          //resechedule path if obstacle avoided
          if (xRem >= autoPathMargin || yRem >= autoPathMargin) {
            autoPath = new MoveByCommand(xRem, m_driveAuto, 0);
            autoPath.schedule();
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
    m_shooter.setShooter(0);
    m_driveAuto.setCurrentAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }

  public void checkShooter() {
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

  public void resetVars() {
    isFinished = false;
    left1 = false;
    left2 = false;
    shooter = false;
    shooterReady = false;
    loopComplete = false;
  }
}
