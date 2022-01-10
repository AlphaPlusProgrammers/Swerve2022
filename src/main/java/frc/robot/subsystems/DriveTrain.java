// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  public SwerveMotor motorFL;
  public SwerveMotor motorFR;
  public SwerveMotor motorRL;
  public SwerveMotor motorRR;

  public double FLAngle = 0;
  public double FRAngle = 0;
  public double RLAngle = 0;
  public double RRAngle = 0;

  //for the side length of the robot
  //it is undefined in constants rn
  public double l;
  //r is the hypotenuse
  public double r;


  //defining each swerve unit with the custom swerve class we made
  public DriveTrain() {
    //we need to change the parameters, based off of Swolenoid
    motorFL = new SwerveMotor(2, 1, 12, 10);
    motorFR = new SwerveMotor(4, 3, 18, 17);
    motorRL = new SwerveMotor(6, 5, 14, 13);
    motorRR = new SwerveMotor(8, 7, 16, 15);

    //defining l
    l = Constants.length;
    //this is a formula for hypotenuse with a 45 45 90 triangle
    r = (Math.sqrt(2)*l);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveSwerveAxis(double leftX, double leftY, double rightX){
    double swivel = rightX * (l / r);
    // a b c and d are the sides of the robot, wheels are made from the combination of sides
    double a = leftX-swivel;
    double b = leftX+swivel;
    double c = rightX-swivel;
    double d = rightX+swivel;

    //calculates the speed based on *vector math*
    double FLDesiredSpeed = -Math.sqrt((b*b)+(c*c));
    double FRDesiredSpeed = Math.sqrt((b*b)+(d*d));
    double RLDesiredSpeed = -Math.sqrt((a*a)+(c*c));
    double RRDesiredSpeed = Math.sqrt((a*a)+(d*d));

    //notorious problem area, rework if possible, atan will return zero and not tell you
    if(leftX == 0 && leftY == 0 && swivel == 0){
      //tries to avoid divide by zero error
      FLAngle = 0;
      FRAngle = 0;
      RLAngle = 0;
      RRAngle = 0;
    }else{
      //calculates the angle based of where each side wants to go
      FLAngle = Math.atan2(b, c)/Math.PI;
      FRAngle = Math.atan2(b, d)/Math.PI;
      RLAngle = Math.atan2(a, c)/Math.PI;
      RRAngle = Math.atan2(a, d)/Math.PI;
    }

    motorFL.drive(FLDesiredSpeed, FLAngle);
    motorFR.drive(FRDesiredSpeed, FRAngle);
    motorRL.drive(RLDesiredSpeed, RLAngle);
    motorRR.drive(RRDesiredSpeed, RRAngle);
  }

  //reaching into each motors "SwerveMotor" class to zero encoders
  public void zeroAllEncoders(){
    motorFL.zeroEncoder();
    motorFR.zeroEncoder();
    motorRL.zeroEncoder();
    motorRR.zeroEncoder();
  }


  //Pressing Alt+left Click will duplicate your curser 
  //so you can type in multiple spots at once
  public void findZero(){
    motorFL.goToZero();
    motorFR.goToZero();
    motorRL.goToZero();
    motorRR.goToZero();
  }


}
