package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.PID;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private CANSparkMax openCloseMotor;

  private SparkMaxPIDController leftMotorPIDController;
  private RelativeEncoder leftMotorEncoder;

  private SparkMaxPIDController rightMotorPIDController;
  private RelativeEncoder rightMotorEncoder;

  private SparkMaxPIDController openCloseMotorPIDController;
  private RelativeEncoder openCloseMotorEncoder;

  private int state;
  // 0: Open
  // 1: Cube
  // 2: Cone

  public Manipulator() {
    leftMotor = new CANSparkMax(Constants.CANIDS.LEFTGRABBER_ID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.CANIDS.RIGHTGRABBER_ID, MotorType.kBrushless);
    openCloseMotor = new CANSparkMax(Constants.CANIDS.OPENCLOSE_ID, MotorType.kBrushless);

    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    // leftMotorPIDController = leftMotor.getPIDController();
    // leftMotorEncoder = leftMotor.getEncoder();

    // leftMotorPIDController.setP(Constants.Grabber.PID.P);
    // leftMotorPIDController.setI(Constants.Grabber.PID.I);
    // leftMotorPIDController.setD(Constants.Grabber.PID.D);

    // rightMotorPIDController = rightMotor.getPIDController();
    // rightMotorEncoder = rightMotor.getEncoder();

    // rightMotorPIDController.setP(Constants.Grabber.PID.P);
    // rightMotorPIDController.setI(Constants.Grabber.PID.I);
    // rightMotorPIDController.setD(Constants.Grabber.PID.D);

    openCloseMotorPIDController = openCloseMotor.getPIDController();
    openCloseMotorEncoder = openCloseMotor.getEncoder();

    setPID(Constants.openCloseMotor.openClosePID);

    state = 1;
    openCloseMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    openCloseMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);

  }

  public int getState() {
    return state;
  }

  public void enableAndSetSoftStop() { // TODO: call in calibrate
    openCloseMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    openCloseMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    openCloseMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
    openCloseMotor.setSoftLimit(SoftLimitDirection.kReverse, -22);
  }

  public void stop() {
    openCloseMotor.stopMotor();
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void stopIntake() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void setOC(double speed) {
    openCloseMotor.set(speed);
  }

  public void intake(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  public void open() {
    openCloseMotorPIDController.setReference(Constants.openCloseMotor.openPosition, CANSparkMax.ControlType.kPosition);
    state = 1;
    intake(-0.3);
  }

  // public void close

  // open: -50
  // cone: -1.15
  // cube:

  public void setPosition(int position) {
    openCloseMotorEncoder.setPosition(position);
  }

  // public void close() {
  //   // setPID(Constants.openCloseMotor.openClosePID);
  //   if (state == 1) {
  //     // openCloseMotorPIDController.setReference(Constants.openCloseMotor.conePosition,
  //     // CANSparkMax.ControlType.kPosition);
  //     getCone();
  //     state = 2;
  //   } else {
  //     openCloseMotorPIDController.setReference(Constants.openCloseMotor.cubePosition,
  //         CANSparkMax.ControlType.kPosition);
  //     state = 1;
  //   }
  //   System.out.println("CLOSING MOTOR ************************************ : " + state);
  //   stopIntake();
  // }

  public void setPID(PID pid) {
    openCloseMotorPIDController.setP(pid.p);
    openCloseMotorPIDController.setI(pid.i);
    openCloseMotorPIDController.setD(pid.d);
  }

  public void setManipulatorVelocity(int velocity) {
    leftMotorPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    rightMotorPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void setOpenCloseMotor(int velocity) {
    openCloseMotorPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void getCone() {
    openCloseMotorPIDController.setReference(Constants.openCloseMotor.conePosition, CANSparkMax.ControlType.kPosition);

  }

  public void getCube() {
      openCloseMotorPIDController.setReference(Constants.openCloseMotor.cubePosition, CANSparkMax.ControlType.kPosition);

    
  }

  public double getOCcurrent() {
    return openCloseMotor.getOutputCurrent();
  }

  public void resetEncoders() {
    openCloseMotorEncoder.setPosition(0);
    enableAndSetSoftStop();
  }

  // public void setManipulatorRightVelocity(int velocity){
  public void stopOpenClose() {
    openCloseMotor.disable();
    ;
  }

  public void getOpenCloseEncoder() {
    // SmartDashboard.putNumber("Open Close Encoder",
    // openCloseMotor.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(openCloseMotorEncoder.getPosition());
    // System.out.println(openCloseMotorPIDController.getP());
    SmartDashboard.putNumber("Manipulator Encoder", openCloseMotorEncoder.getPosition());

  }
}