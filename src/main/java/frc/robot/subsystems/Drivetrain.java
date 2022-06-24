package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

    public class Drivetrain extends SubsystemBase {
     TalonSRX leftFrontMotor = new TalonSRX(Constants.LEFT_FRONT_MOTOR);
     TalonSRX leftRearMotor = new TalonSRX(Constants.LEFT_REAR_MOTOR);
     TalonSRX rightFrontMotor = new TalonSRX(Constants.RIGHT_FRONT_MOTOR);
     TalonSRX rightRearMotor = new TalonSRX(Constants.RIGHT_REAR_MOTOR);

     PigeonIMU gyro = new PigeonIMU(leftFrontMotor);

    public Drivetrain() {
     rightRearMotor.setInverted(true);
     rightFrontMotor.setInverted(true);

     leftFrontMotor.setNeutralMode(NeutralMode.Brake);
     leftRearMotor.setNeutralMode(NeutralMode.Brake);
     rightFrontMotor.setNeutralMode(NeutralMode.Brake);
     rightRearMotor.setNeutralMode(NeutralMode.Brake);
        
     leftRearMotor.follow(leftFrontMotor);
     rightRearMotor.follow(rightFrontMotor);
    }

    // drive using control sticks
    public void drive() {
     double y = -1 * RobotContainer.driverPad.getLeftYAxis() * Constants.DRIVE_Y;
     double rot = RobotContainer.driverPad.getRightXAxis() * Constants.DRIVE_ROT;
    
     // Calculated Outputs (Limits Output to 12V)
     double leftOutput = rot + y ;
     double rightOutput = y - rot;
    
     // Assigns Each Motor's Power
     leftFrontMotor.set(ControlMode.PercentOutput, leftOutput);
     rightFrontMotor.set(ControlMode.PercentOutput, rightOutput);
     // leftRearMotor.set(ControlMode.PercentOutput, .2);
     // rightRearMotor.set(ControlMode.PercentOutput, .2);
    }

// Auto Methods

    public void stopDrive() {
     leftFrontMotor.set(ControlMode.PercentOutput, 0);
     rightFrontMotor.set(ControlMode.PercentOutput, 0);
     leftRearMotor.set(ControlMode.PercentOutput, 0);
     rightRearMotor.set(ControlMode.PercentOutput, 0);
    }

    public void driveForward(double speed) {
     leftFrontMotor.set(ControlMode.PercentOutput, speed);
     rightFrontMotor.set(ControlMode.PercentOutput, speed);
     leftRearMotor.set(ControlMode.PercentOutput, speed);
     rightRearMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setGyro(double degrees) {
     gyro.setYaw(degrees);
    }

    public void periodic() {
     drive();
    }    
}
