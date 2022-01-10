// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveMotor extends SubsystemBase {
  /** Creates a new SwerveMotor. */


  //creating our motors and encoder
  public CANSparkMax driveMotor; //controls the drive motor
  public CANSparkMax rotationMotor; //controls the rotation of the unit
  private Encoder rotationEncoder; //encoder for the turning motor

  //creating variables
  public double encoderRemainingValue;
  public double pointSet = 0;
  
  //We need to find a good way to make this worj with SparkMaxs
  /**sets min and max output
   * 
   * @param peak - (double) The peak output for the motors, counts as forward or reverse. In amps
   * @return void
   */
   private void setMinMaxOutput(int peak){
     rotationMotor.setSmartCurrentLimit(peak);
   }

   /**Class for an entire swerve unit
    * 
    * @param motorDeviceNumber - (int) CAN Device ID of the Rotation Spark
    * @param driveMotorNumber - (int) Can device ID of the drive Spark
    * @param encSourceA - (int) The a channe; digital input for the encoder
    * @param encSourceB - (int) The b channel digital input for the encoder
    */
  public SwerveMotor(int rotate, int drive, int encSourceA, int encSourceB) {
    //when calling swerve motor we put in parameters which will assign these motors and encoders numbers and ports
    rotationMotor = new CANSparkMax(rotate, MotorType.kBrushless);
    driveMotor = new CANSparkMax(drive, MotorType.kBrushless);

    rotationEncoder = new Encoder(encSourceA, encSourceB);

    //this is where I would use SetMinMaxOutput
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //gets the value of an encoder from 0 to 420, we transfer negetive numbers to their positive counterpart
  public double currentEncoderCount(){
    if(rotationEncoder.get() >= 0){
      return rotationEncoder.get();
    }else{
      return rotationEncoder.get() + 420;
    }
  }

  /**sets the speed for the rotation motor
   * 
   * @param speed - (double) the desired power in percent [-1,1]
   * @return (void)
   */
  private void moveMotor(double speed){
    rotationMotor.set(speed);
  }

  //stops the rotation motor
  private void stopMotors(){
    rotationMotor.set(0);
  }

  /**
   * Takes a joystick input and turns the rotation motor to the equivilant of the position in ticks
   * 
   * @param targetX - (doubles) The joystick's current X-value
   * @param targetY - (doubles) The joystick's currentmY-Value
   * 
   * @return (void)
   */
  public void pointToTarget(double target){
    //some local variables to use in calculations
    double currentposition = currentEncoderCount();
    double desiredTarget = target;
    encoderRemainingValue = desiredTarget - currentposition;
    double directionMultiplier = 0;

    //preliminarily checking to see if it is at the value
    if((encoderRemainingValue != 0) || (encoderRemainingValue - 420 != 0) || (encoderRemainingValue +420 != 0)){

      //checks to see the direction needed to go, basically the formula a/|a|
      // if we need to go to a positive direction a/|a| = 1
      // if we need to go negetive -a/|-a| = -1
      if(encoderRemainingValue>210){
        directionMultiplier = Math.round((encoderRemainingValue-420)/Math.abs(encoderRemainingValue-420));
      }else if(encoderRemainingValue<210){
        directionMultiplier = Math.round((encoderRemainingValue+420)/Math.abs(encoderRemainingValue+420));
      }else if(encoderRemainingValue < 210 && encoderRemainingValue > -210 && encoderRemainingValue != 0){
        directionMultiplier = Math.round((encoderRemainingValue)/Math.abs(encoderRemainingValue));
      }else{
        directionMultiplier = 1;
      }

      //goes towards the point, if it is outside the large error it goes fast, if it is
      //in that range it goes at the slow speed untill smaller than the small error
      if(Math.abs(encoderRemainingValue) > Constants.LargeSwerveRotationError){
        moveMotor(Constants.FastSwerveRotationSpeed*-directionMultiplier);
      }else if(Math.abs(encoderRemainingValue) > Constants.SmallSwerveRotationError){
        moveMotor(Constants.SlowSwerveRotationSpeed*-directionMultiplier);
      }else{
        stopMotors();
      }
    }
  }

  //sets the encoder to zero
  public void zeroEncoder(){
    rotationEncoder.reset();
  }

  //gets the encoder value but keeps it on a range from -420 to 420
  public int encoderValue(){
    int encoder = rotationEncoder.get();

    if(encoder>420){
      encoder -= 420;
    }else if(encoder<-420){
      encoder += 420;
    }

    return encoder;
  }

  //points the swerve to zero
  public void goToZero(){
    pointToTarget(0);
  }


  //function to make the unite move
  public void drive(double speed, double angle){
    double revamp = 0;

    if(1 < Math.abs(speed)){
      revamp = (speed/Math.abs(speed));
    }else{
      revamp = (speed);
    }

    driveMotor.set(revamp);

    if(angle<0){
      pointSet = (Math.abs(1+angle)+1)*210;
    }else{
      pointSet = angle*210;
    }

    pointToTarget(pointSet);
  }

  public void staticAngle(double speed, double angle){
    pointToTarget(angle);
    driveMotor.set(speed);
  }
}