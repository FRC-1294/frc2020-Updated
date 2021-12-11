package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.controller.PIDController;
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

  // private SpeedControllerGroup sparkDriveLeft = new SpeedControllerGroup(frontLeftSpark, rearLeftSpark);
  // private SpeedControllerGroup sparkDriveRight = new SpeedControllerGroup(frontRightSpark, rearRightSpark);
  // private DifferentialDrive sparkDrive = new DifferentialDrive(sparkDriveLeft,sparkDriveRight);

  //control init
  public final XboxController driveJoystick = new XboxController(Constants.driveJoystick);

  private final double targetPositionRotations = 0.54;
  //10 m/s to in/s
  private final double maxSpeed = 5500;
  private static int currentAngle;
  private static double[] amountTraveled = new double[] {0, 0};
  private final Gains defaultPID = new Gains(0.05, 0.00001, 0.7, 0.0, 0.0, -0.5, 0.5, 0);
  private final Gains lowDisPID  = new Gains(0.05, 0.00001, 0.7, 0.0, 0.0,   -1,   1, 1);
  private final Gains velocityPID  = new Gains(0.0005, 0, 0, 0, 0.0, -1, 1, 2);
  // private final PIDController arcadeTurningPID = new PIDController(0.05, 0.00001, 0.7);
  private Timer timer = new Timer();
  private boolean isTurning = false;
  private boolean isWall;
  
  public DrivebaseSubsystem() {
    // CameraServer.getInstance().startAutomaticCapture(0);
    // CameraServer.getInstance().startAutomaticCapture(1);
    frontLeftSpark.restoreFactoryDefaults(true);
    frontRightSpark.restoreFactoryDefaults(true);
    rearLeftSpark.restoreFactoryDefaults(true);
    rearRightSpark.restoreFactoryDefaults(true);

    frontLeftSpark.getEncoder();
    frontRightSpark.getEncoder();
    rearLeftSpark.getEncoder();
    rearRightSpark.getEncoder();

    frontLeftSpark.setSmartCurrentLimit(60);
    frontRightSpark.setSmartCurrentLimit(60);
    rearLeftSpark.setSmartCurrentLimit(60);
    rearRightSpark.setSmartCurrentLimit(60);

    setRampRates(0.5);
    setMode(idleMode.brake);

    setPidControllers(frontLeftPID, defaultPID, defaultPID.kSlot);
    setPidControllers(frontRightPID, defaultPID, defaultPID.kSlot);
    setPidControllers(rearLeftPID, defaultPID, defaultPID.kSlot);
    setPidControllers(rearRightPID, defaultPID, defaultPID.kSlot);

    setPidControllers(frontLeftPID, lowDisPID, lowDisPID.kSlot);
    setPidControllers(frontRightPID, lowDisPID, lowDisPID.kSlot);
    setPidControllers(rearLeftPID, lowDisPID, lowDisPID.kSlot);
    setPidControllers(rearRightPID, lowDisPID, lowDisPID.kSlot);

    setPidControllers(frontLeftPID, velocityPID, velocityPID.kSlot);
    setPidControllers(frontRightPID, velocityPID, velocityPID.kSlot);
    setPidControllers(rearLeftPID, velocityPID, velocityPID.kSlot);
    setPidControllers(rearRightPID, velocityPID, velocityPID.kSlot);

    frontLeftSpark.setInverted(false);
    frontRightSpark.setInverted(true);
    rearLeftSpark.setInverted(false);
    rearRightSpark.setInverted(false);
    isWall = false;

    rearLeftSpark.follow(frontLeftSpark);
    rearRightSpark.follow(frontRightSpark);

    timer.start();

    driveJoystick.setRumble(RumbleType.kLeftRumble, 0);
    driveJoystick.setRumble(RumbleType.kRightRumble, 0);
  }

  @Override
  public void periodic() {
    arcadeDrive(-driveJoystick.getY(Hand.kLeft), driveJoystick.getX(Hand.kRight), driveJoystick.getBumper(Hand.kLeft));

    //TODO: view velocities, change accordingly! Maybe add PID if there's time? 
    //TODO: check shooter PID and test AUTO!
    // if (frontLeftSpark.getEncoder().getVelocity() <= 0.5 && frontRightSpark.getEncoder().getVelocity() <= 0.5) {
    //   setMode(idleMode.brake);
    // }
    // else {
    //   setMode(idleMode.coast);
    // }
  }

  private enum idleMode {
    brake,
    coast
  }

  public void arcadeDrive(double forward, double turn, boolean precise) {
    final double deadZone = 0.05;
    final double minPower = 0.2;
    final double minTurn = 0.05;
    final double fastestTurn = 0.2;
    // final double maxTurnOffset = 0.1 * getSign(forward);
    
    double leftSpeed = 0;
    double rightSpeed = 0;
    
    SmartDashboard.putNumber("Input: turn", turn);

    //deadzone filter
    if (Math.abs(forward) <= deadZone) forward = 0;
    if (Math.abs(turn) <= deadZone) turn = 0;
    //precision mode
    if (precise) turn *= 0.6;

    //dynamic turn sensititvity and offset adjustments
    double turnFactor = (1-fastestTurn) * Math.pow(1-Math.pow(fastestTurn, 8/3), 6) + minTurn;
    //double turnOffset = Math.pow(forward, 2) * maxTurnOffset;

    //calculate turn correction PID
    //double turnCorrection = arcadeTurningPID.calculate(Math.abs(frontLeftSpark.getEncoder().getVelocity())-Math.abs(frontRightSpark.getEncoder().getVelocity()), 0);

    // if (turnCorrection > 1) turnCorrection = 1;
    // if (turnCorrection < -1) turnCorrection = -1;

    //apply power to inputs for higher percision at lower velocities, with applied power
    forward = ((1-minPower) * Math.abs(Math.pow(forward, 8/3)) + minPower) * getSign(forward);
    turn = ((1-minTurn) * Math.abs(Math.pow(turn, 8/3)) + minTurn) * getSign(turn) * turnFactor;// * getSign(turn);// + turnOffset;

    //differential drive logic
    leftSpeed = forward+turn;
    rightSpeed = forward-turn;

    double factor = Double.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
    if (factor > 1) {
      factor = 1/factor;
      leftSpeed *= factor;
      rightSpeed *= factor;
    }
    
    SmartDashboard.putNumber("Output: turn", turn);
    SmartDashboard.putNumber("Output: left", leftSpeed);
    SmartDashboard.putNumber("Output: right", rightSpeed);

    //apply to PID for open loop control
    frontLeftSpark.set(leftSpeed);
    frontRightSpark.set(rightSpeed);
    // setFrontLeftPID(maxSpeed*leftSpeed, ControlType.kVelocity, velocityPID.kSlot);
    // setFrontRightPID(maxSpeed*rightSpeed, ControlType.kVelocity, velocityPID.kSlot);
    // sparkDrive.feed();
  }

  //returns +1 or -1 based on num's sign
  private double getSign(double num) {
    double sign = num/Math.abs(num);
    if (Double.isNaN(sign)) sign = 0;

    return sign;
  }

  public void setRampRates(double time) {
    frontLeftSpark.setOpenLoopRampRate(time);
    frontLeftSpark.setClosedLoopRampRate(time);

    frontRightSpark.setOpenLoopRampRate(time);
    frontRightSpark.setClosedLoopRampRate(time);

    rearLeftSpark.setOpenLoopRampRate(time);
    rearLeftSpark.setClosedLoopRampRate(time);

    rearRightSpark.setOpenLoopRampRate(time);
    rearRightSpark.setClosedLoopRampRate(time);
  }

  public void stop() {
    // sparkDrive.arcadeDrive(0, 0);
    setFrontLeftSpeed(0);
    setFrontRightSpeed(0);
    setRearLeftSpeed(0);
    setRearRightSpeed(0);
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

  public void setMode(idleMode type) {
    if (type == idleMode.brake) {
      frontLeftSpark.setIdleMode(IdleMode.kBrake);
      frontRightSpark.setIdleMode(IdleMode.kBrake);
      rearLeftSpark.setIdleMode(IdleMode.kBrake);
      rearRightSpark.setIdleMode(IdleMode.kBrake);
    } else if (type == idleMode.coast) {
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