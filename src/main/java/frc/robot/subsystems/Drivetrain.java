package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

    public class Drivetrain extends SubsystemBase {
     CANSparkMax leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_MOTOR, MotorType.kBrushless);
     CANSparkMax leftRearMotor = new CANSparkMax(Constants.LEFT_REAR_MOTOR, MotorType.kBrushless);
     CANSparkMax rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_MOTOR, MotorType.kBrushless);
     CANSparkMax rightRearMotor = new CANSparkMax(Constants.RIGHT_REAR_MOTOR, MotorType.kBrushless);

    //  PigeonIMU gyro = new PigeonIMU();

    public Drivetrain() {
     rightRearMotor.setInverted(true);
     rightFrontMotor.setInverted(true);

     leftFrontMotor.setIdleMode(IdleMode.kBrake);
     leftRearMotor.setIdleMode(IdleMode.kBrake);
     rightFrontMotor.setIdleMode(IdleMode.kBrake);
     rightRearMotor.setIdleMode(IdleMode.kBrake);
        
     leftRearMotor.follow(leftFrontMotor);
     rightRearMotor.follow(rightFrontMotor);
    }

    // drive using control sticks
    public void drive() {
     double leftY = MathUtil.applyDeadband(RobotContainer.driverPad.getLeftYAxis(), 0.05);
     double rightX = MathUtil.applyDeadband(RobotContainer.driverPad.getRightXAxis(), 0.05);
     double y = -1 * leftY * Constants.DRIVE_Y;
     double rot = rightX * Constants.DRIVE_ROT;
    
     // Calculated Outputs (Limits Output to 12V)
     double leftOutput = rot + y ;
     double rightOutput = y - rot;
    
     // Assigns Each Motor's Power
     leftFrontMotor.set(leftOutput);
     rightFrontMotor.set(rightOutput);
     leftRearMotor.set(leftOutput);
     rightRearMotor.set(rightOutput);
    }

    

    // public void setGyro(double degrees) {
    //  gyro.setYaw(degrees);
    // }

    public void periodic() {
     drive();
    }    
}
