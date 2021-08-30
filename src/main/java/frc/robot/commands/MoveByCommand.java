package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.ControlType;
import frc.robot.subsystems.DrivebaseSubsystem;

public class MoveByCommand extends CommandBase {
  DrivebaseSubsystem m_driveAuto;
  double targetPositionRotations;
  double m_targetLeft;
  double m_targetRight;
  double m_amount;
  int m_pidType;
  double startingEncoderPos;

  final double delta;
  double startingGyro;
  double recordedTime = 0;
  Timer timer = new Timer();
  double leftSpeed;
  double rightSpeed;

  public MoveByCommand(double amount, DrivebaseSubsystem driveAuto, int pidType) {
    m_driveAuto = driveAuto;
    m_amount = amount;
    m_pidType = pidType;

    targetPositionRotations = m_driveAuto.getMoveByFactor();
    delta = amount*0.1 * targetPositionRotations;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetLeft = (m_amount)*targetPositionRotations + m_driveAuto.getFrontLeftPosition();
    m_targetRight = (m_amount)*targetPositionRotations + m_driveAuto.getFrontRightPosition();

    startingEncoderPos = m_driveAuto.getFrontLeftPosition();

    m_driveAuto.setRamp(0.5);

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveAuto.setFrontLeftPID(m_targetLeft, ControlType.kPosition, 0);
    leftSpeed = m_driveAuto.getFrontLeftSpeed();
    m_driveAuto.setFrontRightPID(m_targetRight, ControlType.kPosition, 0);
    rightSpeed = m_driveAuto.getFrontRightSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveAuto.setFrontLeftSpeed(0);
    m_driveAuto.setFrontRightSpeed(0);
    m_driveAuto.setRearLeftSpeed(0);
    m_driveAuto.setRearRightSpeed(0);
    m_driveAuto.setRamp(1);

    double changePos = (m_driveAuto.getFrontLeftPosition() - startingEncoderPos) / targetPositionRotations;

    m_driveAuto.setAmountTraveled(0, (int)(m_driveAuto.getAmountTraveled(0) + (changePos * Math.cos(m_driveAuto.getCurrentAngle() * (Math.PI/180)))));
    m_driveAuto.setAmountTraveled(1, (int)(m_driveAuto.getAmountTraveled(1) + (changePos * Math.sin(m_driveAuto.getCurrentAngle() * (Math.PI/180)))));
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
      if(timer.get() >= 0.3){
        timeHold = true;
      }
    }
    else {
      timer.reset();
    }

    return atSpeed && timeHold;
  }
}
