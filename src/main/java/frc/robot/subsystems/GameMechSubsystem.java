package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class GameMechSubsystem extends SubsystemBase {
  private TalonFX shooter = new TalonFX(Constants.shooterFalcon);
  private TalonSRX indexer = new TalonSRX(Constants.indexerTalon);
  private TalonSRX intaker =  new TalonSRX(Constants.intakeTalon);
  private TalonSRX colorWheel = new TalonSRX(Constants.colorTalon);
  private boolean toReversed = false;
  private boolean toIndex = false;
  private boolean toIntake = false;
  private boolean toColor = false;
  private boolean toShoot = false;
  private double offset = 0.0;
  private boolean extendColor = false;
  private double ticksPerRev = -2.59;
  private double currentSpeed = 0.0;
  private XboxController gameJoystick = new XboxController(Constants.gameJoystick);
  public static Servo linActuator = new Servo(Constants.linearActuator);
  /**
   * Creates a new linearActuatorSubsystem.
   */
  public GameMechSubsystem() {
    shooter.configOpenloopRamp(5);
    shooter.configClosedloopRamp(0.5);
    shooter.config_kP(0, 0.1);
    shooter.config_kI(0, 0.00014);
    shooter.config_kD(0, 1);

    shooter.setNeutralMode(NeutralMode.Coast);
    shooter.configNominalOutputForward(0);
    shooter.configNominalOutputReverse(0);
    shooter.configPeakOutputForward(1);
    shooter.configPeakOutputReverse(-1);
  }

  //every loop it will check if any of the buttons are pressed and will do the coresponding task related with it
  @Override
  public void periodic() {
   double shooterSpeed = shooter.getSelectedSensorVelocity()/ticksPerRev;
   SmartDashboard.putNumber("Shooter RPM", shooterSpeed);
   SmartDashboard.putBoolean("Indexering", toIndex);
   SmartDashboard.putBoolean("Shooting", toShoot);

   //System.out.println(shooterSpeed);

    //indexer
    if(gameJoystick.getAButton()){
      toIndex = true;

      if (toReversed) {
        toReversed = false;
        toShoot = false;
      }
    }
    else toIndex = false;

    if (toIndex) setSRXSpeed(indexer, 0.5);
    else if (!toReversed && !Robot.inAuto) setSRXSpeed(indexer, 0);

    //intaker
    if(triggerDrive() != 0) setSRXSpeed(intaker, -triggerDrive()*0.5);
    else setSRXSpeed(intaker, 0);

    //shooter
    if (gameJoystick.getBumperPressed(Hand.kLeft)) {
      if ((!toShoot && shooterSpeed < 1000) || toShoot) {
        toShoot = !toShoot;
        toReversed = false;
        currentSpeed = 6300 + offset;
      }
    }
    else if (gameJoystick.getBumperPressed(Hand.kRight)) {
      if ((!toShoot && shooterSpeed < 1000) || toShoot) {
        toShoot = !toShoot;
        toReversed = false;
        currentSpeed = 7000 + offset;
      }
    }  
    else if (gameJoystick.getXButtonPressed()) {
      if ((!toShoot && shooterSpeed < 1000) || toShoot) {
        toShoot = !toShoot;
        toReversed = false;
        currentSpeed = 5900 + offset;
      }
    }    
    else if(gameJoystick.getBButtonPressed()){
      toReversed = !toReversed;

      if(toReversed){
        toShoot = true;
        setSRXSpeed(indexer, -0.3);
        currentSpeed = -500-offset;
      }
      else{
        toShoot = false;
        setSRXSpeed(indexer, 0);
      }
    }
    
    //shooter PID
    if (toShoot) shooter.set(TalonFXControlMode.Velocity, currentSpeed * ticksPerRev);
    else  setFXSpeed(shooter, 0);

    if (gameJoystick.getYButtonPressed()) {
      extendColor = !extendColor;
      if (extendColor) {
        linActuator.set(0.62);
      }
      else {
        linActuator.set(0.2);
      }
    }

    //colorer
    if(Math.abs(gameJoystick.getX(Hand.kRight)) >= 0.05) setSRXSpeed(colorWheel, -gameJoystick.getX(Hand.kRight));
    else setSRXSpeed(colorWheel, 0);
  }

  public double triggerDrive() {
    return gameJoystick.getTriggerAxis(Hand.kRight) - gameJoystick.getTriggerAxis(Hand.kLeft);
  }

  //better methods for increased versatility
  public void setSRXSpeed(TalonSRX controller, double speed) {
    controller.set(ControlMode.PercentOutput, speed);
  }

  public void setFXSpeed(TalonFX controller, double speed) {
    controller.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void setIndexer(double speed) {
    indexer.set(ControlMode.PercentOutput, speed);
  }

  public void setIntaker(double speed) {
    intaker.set(ControlMode.PercentOutput, speed);
  }

  public void setShooterPID(double velocity) {
    shooter.set(TalonFXControlMode.Velocity, velocity);
  }

  public void setShooter(double speed) {
    shooter.set(TalonFXControlMode.PercentOutput, speed);
  }

  public double getShooterVelocity() {
    return shooter.getSelectedSensorVelocity();
  }

  public void setZero() {
    setSRXSpeed(intaker, 0);
    setSRXSpeed(indexer, 0);
    setSRXSpeed(colorWheel, 0);
    setFXSpeed(shooter, 0);
    linActuator.set(0.2);
    
    toIndex = false;
    toIntake = false;
    toColor = false;
    toShoot = false;
    extendColor = false;
    toReversed = false;
  }
}