package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DrivebaseSubsystem;

import com.revrobotics.ControlType;

public class TurnByCommand extends CommandBase {
  DrivebaseSubsystem m_driveAuto;
  int m_amount;
  //CHANGE TO ADJUST DEGREE FACTOR (if overshoots - lower, and vice versa)
  double targetPositionRotations = 0.12;
  double m_targetLeft;
  double m_targetRight;
  double startingGyro;
  double delta;
  Timer timer;
  double recordedTime = 0;
  int m_pidType;

  double leftSpeed;
  double rightSpeed;

  public TurnByCommand(int amount, DrivebaseSubsystem driveAuto, int pidType) {
    m_driveAuto = driveAuto;
    m_amount = amount;
    m_pidType = pidType;
    timer = new Timer();
    delta = amount*0.1 * targetPositionRotations;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetLeft = -(m_amount)*targetPositionRotations + m_driveAuto.getFrontLeftPosition();
    m_targetRight = (m_amount)*targetPositionRotations + m_driveAuto.getFrontRightPosition();

    m_driveAuto.setRamp(0.5);
    m_driveAuto.setTurning(true);

    timer.start();

    m_driveAuto.setCurrentAngle(m_driveAuto.getCurrentAngle() + m_amount);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveAuto.setFrontLeftPID(m_targetLeft, ControlType.kPosition, m_pidType);
    leftSpeed = m_driveAuto.getFrontLeftSpeed();
    m_driveAuto.setFrontRightPID(m_targetRight, ControlType.kPosition, m_pidType);
    rightSpeed = m_driveAuto.getFrontRightSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveAuto.setFrontLeftSpeed(0);
    m_driveAuto.setFrontRightSpeed(0);
    m_driveAuto.setRearLeftSpeed(0);
    m_driveAuto.setRearRightSpeed(0);

    m_driveAuto.setTurning(false);

    m_driveAuto.setRamp(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean atPos = false;
    boolean atSpeed = false;
    boolean timeHold = false;

    if (Math.abs(Math.abs(m_targetLeft)-Math.abs(m_driveAuto.getFrontLeftPosition())) < delta
    && Math.abs(Math.abs(m_targetRight)-Math.abs(m_driveAuto.getFrontRightPosition())) < delta) {
      atPos = true;
    }

    if (Math.abs(leftSpeed) <= 0.1 && Math.abs(rightSpeed) <= 0.1) {
      atSpeed = true;
    }

    if (atSpeed) {
      if(timer.get() >= 0.5){
        timeHold = true;
      }
    }
    else {
      timer.reset();
    }

    return atSpeed && timeHold;
  }
}
