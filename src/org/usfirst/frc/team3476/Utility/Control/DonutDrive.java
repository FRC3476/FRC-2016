package org.usfirst.frc.team3476.Utility.Control;

import org.usfirst.frc.team3476.Utility.OrangeUtility;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;

public class DonutDrive extends RobotDrive
{
	private double driveDonutThreshold, rotateDonutThreshold, clamp;
	private boolean scale;
	
	public DonutDrive(int frontLeftMotor, int rearLeftMotor, int frontRightMotor, int rearRightMotor)
	{
		super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		setDefaults();
	}

	public DonutDrive(int leftMotorChannel, int rightMotorChannel)
	{
		super(leftMotorChannel, rightMotorChannel);
		setDefaults();
	}

	public DonutDrive(SpeedController frontLeftMotor, SpeedController rearLeftMotor, SpeedController frontRightMotor,
			SpeedController rearRightMotor)
	{
		super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		setDefaults();
	}

	public DonutDrive(SpeedController leftMotor, SpeedController rightMotor)
	{
		super(leftMotor, rightMotor);
		setDefaults();
	}
	
	private void setDefaults()
	{
		driveDonutThreshold = 0;
		rotateDonutThreshold = 0;
		clamp = 1;
		scale = false;
	}

	public void setDonutParams(double driveDonutThreshold, double rotateDonutThreshold)
	{
		this.driveDonutThreshold = driveDonutThreshold;
		this.rotateDonutThreshold = rotateDonutThreshold;
	}
	
	public void setClamp(double clamp)
	{
		this.clamp = clamp;
	}
	
	public void setScale(boolean scale)
	{
		this.scale = scale;
	}
	
	public void arcadeDrive(double moveValue, double rotateValue)
	{
		if(scale)
		{
			super.arcadeDrive(OrangeUtility.scalingDonut(moveValue, driveDonutThreshold, clamp, 1), OrangeUtility.scalingDonut(rotateValue, rotateDonutThreshold, clamp, 1));//we know the max(min) is 1 cause this is a motor
		}
		else
		{
			super.arcadeDrive(OrangeUtility.coerce(OrangeUtility.donut(moveValue, driveDonutThreshold), clamp, -clamp), OrangeUtility.coerce(OrangeUtility.donut(rotateValue, rotateDonutThreshold), clamp, -clamp));
		}
	}

}
