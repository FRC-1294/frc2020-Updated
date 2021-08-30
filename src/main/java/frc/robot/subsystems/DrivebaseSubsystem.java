package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax; //Shacuando was here
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj. XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Gains;

public class DrivebaseSubsystem extends SubsystemBase {
  //Spark diff drive init
  private CANSparkMax frontLeftSpark = new CANSparkMax(Constants.frontLeftSpark, MotorType.kBrushless);
  private CANPIDController frontLeftPID = frontLeftSpark.getPIDController();
  private CANSparkMax frontRightSpark = new CANSparkMax(Constants.frontRightSpark, MotorType.kBrushless);
  private CANPIDController frontRightPID = frontRightSpark.getPIDController();
  private CANSparkMax rearLeftSpark = new CANSparkMax(Constants.rearLeftSpark, MotorType.kBrushless);
  private CANPIDController rearLeftPID = rearLeftSpark.getPIDController();
  private CANSparkMax rearRightSpark = new CANSparkMax(Constants.rearRightSpark, MotorType.kBrushless);
  private CANPIDController rearRightPID = rearRightSpark.getPIDController();

  private SpeedControllerGroup sparkDriveLeft = new SpeedControllerGroup(frontLeftSpark, rearLeftSpark);
  private SpeedControllerGroup sparkDriveRight = new SpeedControllerGroup(frontRightSpark, rearRightSpark);
  private DifferentialDrive sparkDrive = new DifferentialDrive(sparkDriveLeft,sparkDriveRight);

  //control init
  public final XboxController driveJoystick = new XboxController(Constants.driveJoystick);

  private final double targetPositionRotations = 0.54;
  private static int currentAngle;
  private static double[] amountTraveled = new double[] {0, 0};
  private final Gains defaultPID = new Gains(0.05, 0.00001, 0.7, 0.0, 0.0, -0.5, 0.5, 0);
  private final Gains lowDisPID  = new Gains(0.05, 0.00001, 0.7, 0.0, 0.0,   -1,   1, 1);
  private Timer timer = new Timer();
  private Timer rumbleTime = new Timer();
  private boolean isTurning = false;
  private int rumble = 0;
  private double factor = 1;
  private boolean isWall;
  
  public DrivebaseSubsystem() {
    UsbCamera intakeCam = CameraServer.getInstance().startAutomaticCapture(0);
    UsbCamera indexCam = CameraServer.getInstance().startAutomaticCapture(1);
    frontLeftSpark.restoreFactoryDefaults(true);
    frontRightSpark.restoreFactoryDefaults(true);
    rearLeftSpark.restoreFactoryDefaults(true);
    rearRightSpark.restoreFactoryDefaults(true);

    frontLeftSpark.setMotorType(MotorType.kBrushless);
    frontRightSpark.setMotorType(MotorType.kBrushless);
    rearLeftSpark.setMotorType(MotorType.kBrushless);
    rearRightSpark.setMotorType(MotorType.kBrushless);

    frontLeftSpark.getEncoder();
    frontRightSpark.getEncoder();
    rearLeftSpark.getEncoder();
    rearRightSpark.getEncoder();

    frontLeftSpark.setSmartCurrentLimit(60);
    frontRightSpark.setSmartCurrentLimit(60);
    rearLeftSpark.setSmartCurrentLimit(60);
    rearRightSpark.setSmartCurrentLimit(60);

    frontLeftSpark.setOpenLoopRampRate(1);
    frontRightSpark.setOpenLoopRampRate(1);
    frontLeftSpark.setClosedLoopRampRate(1);
    frontRightSpark.setClosedLoopRampRate(1);
    rearLeftSpark.setClosedLoopRampRate(1);
    rearRightSpark.setClosedLoopRampRate(1);
    rearLeftSpark.setClosedLoopRampRate(1);
    rearRightSpark.setClosedLoopRampRate(1);
    setPidControllers(frontLeftPID, defaultPID, defaultPID.kSlot);
    setPidControllers(frontRightPID, defaultPID, defaultPID.kSlot);
    setPidControllers(rearLeftPID, defaultPID, defaultPID.kSlot);
    setPidControllers(rearRightPID, defaultPID, defaultPID.kSlot);
    setPidControllers(frontLeftPID, lowDisPID, lowDisPID.kSlot);
    setPidControllers(frontRightPID, lowDisPID, lowDisPID.kSlot);
    setPidControllers(rearLeftPID, lowDisPID, lowDisPID.kSlot);
    setPidControllers(rearRightPID, lowDisPID, lowDisPID.kSlot);
    setMode("coast");

    frontLeftSpark.setInverted(false);
    frontRightSpark.setInverted(true);
    rearLeftSpark.setInverted(false);
    rearRightSpark.setInverted(false);
    isWall = false;

    rearLeftSpark.follow(frontLeftSpark);
    rearRightSpark.follow(frontRightSpark);

    timer.start();
    rumbleTime.start();
  }

  @Override
  public void periodic() {
    sparkDrive.feedWatchdog();

    SmartDashboard.putString("AmountTraveled", getAmountTraveled(0) + " , " + getAmountTraveled(1));
    SmartDashboard.putNumber("currentAngle", getCurrentAngle());
    SmartDashboard.putNumber("encoder", getFrontLeftPosition());

    if (driveJoystick.getBumper(Hand.kRight)) {
      setMode("brake");
    }
    else {
      setMode("coast");
    }

    if (driveJoystick.getBumperPressed(Hand.kLeft)) {
      if (factor == 1) {
        factor = 0.5;
      }
      else {
        factor = 1;
      }
    }

    // if (driveJoystick.getYButton()) {
    //   Robot.cassius.setPipeline(0);
    // }
    // else {
    //   Robot.cassius.setPipeline(1);
    // }

    // if (driveJoystick.getAButtonPressed() && !visionMove.isScheduled() && !visionRotate.isScheduled()) {
    //   rumble = 0;
    //   visionMove = new AlignToShoot(this, Robot.ultrasonic, Robot.letsShoot, Robot.cassius, 10*12, false);
    //   visionMove.schedule();
    //   System.out.println("Scheduling visionMove");
    // }
    // else if (driveJoystick.getBButtonPressed() && !visionMove.isScheduled() && !visionRotate.isScheduled()) {
    //   rumble = 0;
    //   visionRotate = new DictatorLocator(Robot.cassius, this);
    //   visionRotate.schedule();
    //   System.out.println("Scheduling visionRotate");
    // }
    // else {
    //   rumble = 8;
    // }

    if (rumble != 0) {
      if (rumbleTime.get() > 1) {
        rumble = 0;
      }
    }
    else {
      rumbleTime.reset();
    }

    arcadeDrive(driveJoystick.getY(Hand.kLeft), driveJoystick.getX(Hand.kRight));
    driveJoystick.setRumble(RumbleType.kLeftRumble, rumble);
  }

  public void arcadeDrive(double forward, double turn) {
    sparkDrive.arcadeDrive(turn*0.5*factor, -forward);
  }

  public void setWall(boolean thiss){
    this.isWall = thiss;
  }

  public boolean getWall(){
    return this.isWall;
  }

  public void setTurning(boolean val) {
    isTurning = val;
  }

  public void setMode(String type) {
    if (type == "brake") {
      frontLeftSpark.setIdleMode(IdleMode.kBrake);
      frontRightSpark.setIdleMode(IdleMode.kBrake);
      rearLeftSpark.setIdleMode(IdleMode.kBrake);
      rearRightSpark.setIdleMode(IdleMode.kBrake);
    } else if (type == "coast") {
      frontLeftSpark.setIdleMode(IdleMode.kCoast);
      frontRightSpark.setIdleMode(IdleMode.kCoast);
      rearLeftSpark.setIdleMode(IdleMode.kCoast);
      rearRightSpark.setIdleMode(IdleMode.kCoast);
    }
  }

  private void setPidControllers (CANPIDController pidController, Gains pidSet, int slot) {
    pidController.setP(pidSet.kP, slot);
    pidController.setI(pidSet.kI, slot);
    pidController.setD(pidSet.kD, slot);
    pidController.setIZone(pidSet.kIz, slot);
    pidController.setFF(pidSet.kFF, slot);
    pidController.setOutputRange(pidSet.kMinOutput, pidSet.kMaxOutput, slot);
  }

  public int getCurrentAngle() {
    return currentAngle;
  }

  public double getFrontLeftSpeed() {
    return frontLeftSpark.get();
  }

  public double getFrontRightSpeed() {
    return frontRightSpark.get();
  }

  public double getFrontLeftVelocity() {
    return frontLeftSpark.getEncoder().getVelocity();
  }

  public double getFrontRightVelocity() {
    return frontRightSpark.getEncoder().getVelocity();
  }

  public double getFrontLeftPosition() {
    return frontLeftSpark.getEncoder().getPosition();
  }

  public double getFrontRightPosition() {
    return frontRightSpark.getEncoder().getPosition();
  }

  public double getRearLeftPosition() {
    return rearLeftSpark.getEncoder().getPosition();
  }

  public double getRearRightPosition() {
    return rearRightSpark.getEncoder().getPosition();
  }

  public double getMoveByFactor() {
    return targetPositionRotations;
  }

  public double getAmountTraveled(int id) {
    return amountTraveled[id];
  }

  public boolean getTurning() {
    return isTurning;
  }

  public void setCurrentAngle(int val) {
    currentAngle = val;
  }

  public void setAmountTraveled(int id, double val) {
    amountTraveled[id] = val;
  }

  public void setRamp(double time) {
    this.frontLeftSpark.setClosedLoopRampRate(0.5);
    this.frontRightSpark.setClosedLoopRampRate(0.5);
    this.rearLeftSpark.setClosedLoopRampRate(0.5);
    this.rearRightSpark.setClosedLoopRampRate(0.5);

    this.frontLeftSpark.setOpenLoopRampRate(0.5);
    this.frontRightSpark.setOpenLoopRampRate(0.5);
    this.rearLeftSpark.setOpenLoopRampRate(0.5);
    this.rearRightSpark.setOpenLoopRampRate(0.5);
  }

  public void setFrontLeftPID(double val, ControlType controlType, int slot) {
    this.frontLeftPID.setReference(val, controlType, slot);
  }
  
  public void setFrontRightPID(double val, ControlType controlType, int slot) {
    this.frontRightPID.setReference(val, controlType, slot);
  }

  public void setRearLeftPID(double val, ControlType controlType, int slot) {
    this.rearLeftPID.setReference(val, controlType, slot);
  }

  public void setRearRightPID(double val, ControlType controlType, int slot) {
    this.rearRightPID.setReference(val, controlType, slot);
  }

  public void resetEncoders() {
    this.frontLeftSpark.getEncoder().setPosition(0);
    this.frontRightSpark.getEncoder().setPosition(0);
    this.rearLeftSpark.getEncoder().setPosition(0);
    this.rearRightSpark.getEncoder().setPosition(0);
  }

  public void setFrontLeftSpeed(double val) {
    this.frontLeftSpark.set(val);
  }

  public void setFrontRightSpeed(double val) {
    this.frontRightSpark.set(val);
  }

  public void setRearLeftSpeed(double val) {
    this.rearLeftSpark.set(val);
  }

  public void setRearRightSpeed(double val) {
    this.rearRightSpark.set(val);
  }
}