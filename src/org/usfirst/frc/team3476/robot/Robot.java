
package org.usfirst.frc.team3476.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.hal.PDPJNI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import java.io.File;
import java.io.FileWriter;

import org.usfirst.frc.team3476.Communications.Dashcomm;
import org.usfirst.frc.team3476.Main.Starter;
import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.ScriptableAuto.Clock;
import org.usfirst.frc.team3476.ScriptableAuto.Main;
import org.usfirst.frc.team3476.Subsystems.*;
import org.usfirst.frc.team3476.Subsystems.Drive.ShiftingState;
import org.usfirst.frc.team3476.Utility.RunningAverage;
import org.usfirst.frc.team3476.Utility.Toggle;
import org.usfirst.frc.team3476.Utility.Control.AtoD;
import org.usfirst.frc.team3476.Utility.Control.DifferentialAnalogGyro;
import org.usfirst.frc.team3476.Utility.Control.DifferentialSPIGyro;
import org.usfirst.frc.team3476.Utility.Control.DonutCANTalon;
import org.usfirst.frc.team3476.Utility.Control.MedianEncoder;
import org.usfirst.frc.team3476.Utility.Control.PIDCANTalonEncoderWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDCounterPeriodWrapper;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.SetValueMotionProfile;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot
{
	File logFile = new File("/usr/local/frc/logs/start.txt");
	FileWriter logger;
	
	Joystick xbox = new Joystick(0);
	Joystick joy = new Joystick(1);
	
	PowerDistributionPanel pdp = new PowerDistributionPanel();
	
	//TODO: get incorrect channels
	Talon flyTalon1 = new Talon(8),//correct
			flyTalon2 = new Talon(9),//correct
			loaderTalon = new Talon(7),//correct
			intake1 = new Talon(6),
			intake2 = new Talon(5);
											
	DonutCANTalon turretMotor = new DonutCANTalon(4, 0);
	
	CANTalon ddmotor = new CANTalon(3);
	
	//Flywheel constants
	final double FLY1 = -1, FLY2 = 1;
	
	RobotDrive drive = new RobotDrive(2, 3, 0, 1);

	Solenoid shifterSoleniod = new Solenoid(0);
	Solenoid hood = new Solenoid(1);
	
	enum Mode {DEFAULT, INTAKE, SHOOTUP, SHOOTDOWN}
    Mode mode = Mode.DEFAULT;
    
    //Joystick buttons
    final int DEFAULT = 12, TRIGGER = 1, REVERSE = 3;
    boolean defaultButton = joy.getRawButton(DEFAULT);
    boolean trigger = joy.getRawButton(TRIGGER);
    boolean reverseButton = joy.getRawButton(REVERSE);
    
    //Encoders
    MedianEncoder leftDrive = new MedianEncoder(10, 11, false, EncodingType.k4X, 5);
    MedianEncoder rightDrive = new MedianEncoder(12, 13, true, EncodingType.k4X, 5);
    
    DigitalInput halleffect = new DigitalInput(0);//TODO: ensure switch channel at a later time
    
    Main main;
    Subsystem[] systems;
    
    //Gyro analoggyro = new DifferentialAnalogGyro(0, 5);//TODO: replace with spi
    Gyro spiGyro = new DifferentialSPIGyro(SPI.Port.kOnboardCS0, 5);
    
    AnalogInput pressure = new AnalogInput(3), ballsensor = new AnalogInput(2);
    
    AtoD loaderSwitch = new AtoD(ballsensor, 2.3);
    
    DigitalInput banner = new DigitalInput(1);//comp is 1, program is 9
    PIDCounterPeriodWrapper tach = new PIDCounterPeriodWrapper(banner, 60);
    
    Starter starter;
    Thread starterThread;
    
    Toggle hoodToggle = new Toggle();
    
    int iters = 0;
    int threads = 0;
    boolean lastJoy = false;
    boolean lastJoy2 = false;
    
    boolean first = true, camfirst = true;
    
    enum CameraMode {VISION, INTAKE};
    
    boolean homed = false;
    
    boolean automatic = true;
    double joysetpoint;
    int joycount;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
    	System.out.println("robotInit");
        
    	drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
    	drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
    	tach.setSamplesToAverage(4);
    	spiGyro.calibrate();
    	turretMotor.setSafetyEnabled(false);
    	loaderTalon.setInverted(true);
    	flyTalon2.setInverted(true);
    	new PIDCANTalonEncoderWrapper(ddmotor, 1).reset();
    	
    	if(!automatic)
    	{
    		flyTalon2.setInverted(true);
    		ddmotor.setInverted(true);
        	loaderTalon.setInverted(true);
    	}
    	
    	//DONE BY SAIKIRAN TO TEST SHOOTER
//    	
//    	loaderTalon.setInverted(true);
    	
        //Systems
        systems = new Subsystem[10];
		systems[0] = new Drive(leftDrive, rightDrive, spiGyro, drive, shifterSoleniod);
		systems[1] = new Turret(turretMotor, halleffect);
		systems[2] = new Shooter(flyTalon1, flyTalon2, loaderTalon, (Turret)systems[1], tach, loaderSwitch);
		systems[3] = new Intake(intake1, intake2, ddmotor);
		systems[8] = new Watcher(systems, 10);
		systems[9] = new Clock(systems);
		((Clock)systems[9]).megaEnd();
		
		((Drive)systems[0]).autoShifting(false);
		
		//Main
		main = new Main();
		
		//Starter
		starter = new Starter(main, "2016", systems, 0, "10.94.76.11");
		starterThread = new Thread(starter);
		starter.resume();
		starterThread.start();
    }
    
	/**
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    @Override
	public void autonomousInit()
    {
    	first = true;
    }
    
    /**
     * This function is called periodically during autonomous.
     */
    public void autonomousPeriodic()
    {
    	if(starter.importantDone() && first)//WE ARE REEADY TO RUMMMMMMBLEEEEEEE and it's the first time
    	{
    		//DO THINGS
    		if(!homed)
    		{
    			((Turret)systems[1]).resetHomer();
    			homed = true;
    		}
    		main.startSubsystems();
    		main.startThread();
    		first = false;
    	}
    	
    	if(Timer.getMatchTime() > 15.0) main.stop();
    }
	
	@Override
	public void disabledInit()
	{
		System.out.println("disabledInit");
		first = true;
		camfirst = true;
		threads = 0;
	}
	
	public void disabledPeriodic()
	{
		if(starter.importantDone())//WE ARE REEADY TO RUMMMMMMBLEEEEEEE
		{
			if(first)
			{
				System.out.println("disabledPeriodic first");
				main.stopSubsystems();//Stop auto threads, we're not in auto
				main.stop();
				first = false;
			}
			if(camfirst && starter.cameraDone())
			{
				setCameramode(CameraMode.INTAKE);
			}
			main.updateData();
			main.robotDriveClear();
		}
    	if(threads != Thread.getAllStackTraces().keySet().size())
    	{
    		System.out.println("Threads changed: " + Thread.getAllStackTraces().keySet().size());
    	}
    	threads = Thread.getAllStackTraces().keySet().size();
    	
	}
    
	@Override
    public void teleopInit()
    {
		System.out.println("teleopInit");
    	first = true;
    	joysetpoint = 0;
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic()
    {
    	if(starter.importantDone())//WE ARE REEADY TO RUMMMMMMBLEEEEEEE
    	{
    		boolean axisprint = false, turretgo = true, buttons = false, tachprint = false,
    				currentprint = false, pressureprint = false, driveencoderprint = false,
    				spigyroprint = false, intakeenc = false, shooterout = false,
    				ballsensorprint = true;
    		
    		Drive drive = (Drive)systems[0];
    		Turret turret = (Turret)systems[1];
    		Shooter shooter = (Shooter)systems[2];
    		Intake intake = (Intake)systems[3];
    		
			PIDCANTalonEncoderWrapper encoder = turret.getTurretEncoder();
    		switch(first ? 1 : 0)
    		{
	    		case 1://first - For the first time in forever
	    			System.out.println("teleopPeriodic first");
					//DO THINGS
	    			((Watcher)systems[8]).watch(false);
	    			shooter.end();
	    			turret.end();
					main.startSubsystems();//we need subsystems but not main
					//systems[1].stopThreads();//stop shooter
					first = false;
					//if first, will continue to regular execution
					if(automatic)
					{
						turret.stopAim();
						if(!homed)
			    		{
			    			turret.resetHomer();
			    			homed = true;
			    		}
						intake.moveDropdown(20);
					}
					else
					{
						turret.killHomer();
					}
					
					
	    		default://!first - Normal execution
	    			if(automatic)
	    			{
	    				main.robotDriveClear();
	    				if(turretgo)
	    				{
	    					double 	xAxis = -xbox.getRawAxis(4),
		    		    			yAxis = -xbox.getRawAxis(1);
		    				drive.manualDrive(yAxis, xAxis);
		    				
	    					if(buttons)
	    					{
				    			if(joy.getRawButton(1))
				    			{
				    				turret.aim(0);
				    			}
				    			if(joy.getRawButton(7))
				    			{
				    				turret.aim(0.25);
				    			}
				    			if(joy.getRawButton(8))
				    			{
				    				turret.aim(-0.25);
				    			}
				    			if(joy.getRawButton(9))//bed tings
				    			{
				    				turret.aim(0.65);
				    			}
				    			if(joy.getRawButton(10))//bed tings
				    			{
				    				turret.aim(-0.65);
				    			}
	    					}
	    					else//joystick setpoint
	    					{
	    						if(joy.getRawButton(2))
	    						{
	    							if(!lastJoy2)
	    							{
	    								turret.printPID();
	    							}
	    							turret.aim();
	    						}
	    						else
	    						{
	    							if(lastJoy2)
	    							{
	    								joysetpoint = turret.getTurretEncoder().getDistance();
	    							}
	    							if(Math.abs(joy.getRawAxis(0)) > 0.1)
		    							joysetpoint += joy.getRawAxis(0)*0.01;
	    							else
	    							{
	    								joysetpoint = turret.getTurretEncoder().getDistance();
	    							}
		    						
	    							turret.aim(joysetpoint);
	    						}
	    					}
	    				}
	    				else
	    				{
	    					turret.manualTurret(joy.getAxis(AxisType.kX));//xaxis
	    				}
	    				
		    			double FLYWHEELMAX = 8900;
		    			
		    			//shooter.manualShooter((-joy.getRawAxis(3)+1)/2);//throttle
		    			//shooter.manualShooter(0);
		    			shooter.manualLoader(joy.getRawButton(1) ? 1 : 0);
		    			
	    				shooter.setFly(((-joy.getRawAxis(3) + 1)/2)*FLYWHEELMAX);
		    			
		    			double up = 20, horiz = 4310, down = 5230;
		    			
	    				if(joy.getRawButton(7))
	    				{
	    					intake.moveDropdown(up);
	    				}
	    				else if(joy.getRawButton(9))
	    				{
	    					intake.moveDropdown(horiz);
	    				}
	    				else if(joy.getRawButton(11))
	    				{
	    					intake.moveDropdown(down);
	    				}
		    			
	    				
	    				
		    			if(joy.getRawButton(3))
	    				{
	    					intake.manualIntake(1);
	    				}
	    				else if(joy.getRawButton(4))
	    				{
	    					intake.manualIntake(-1);
	    				}
	    				else if(joy.getRawButton(5))
	    				{
	    					intake.manualIntake(0.5);
	    				}
	    				else if(joy.getRawButton(6))
	    				{
	    					intake.manualIntake(-0.5);
	    				}
	    				else
						{
	    					intake.manualIntake(0);
						}
		    			
		    			shooter.manualLoader(joy.getRawButton(1) ? 1 : 0);
		    			
		    			hoodToggle.input(joy.getRawButton(12));
	    				hood.set(hoodToggle.get());
    				}
	    			else//manual
	    			{
	    				double 	xAxis = -xbox.getRawAxis(4),
	    		    			yAxis = -xbox.getRawAxis(1);
	    				boolean calibrate = false;
	    				
	    				if(calibrate)
	    				{
	    					if(xbox.getRawButton(1))
		    				{
		    					drive.manualDrive(1, 0);
		    				}
		    				else if(xbox.getRawButton(2))
		    				{
		    					drive.manualDrive(-1, 0);
		    				}
		    				else
		    				{
		    					drive.manualDrive(0, 0);
		    				}
	    				}
	    				else
	    				{
	    					drive.manualDrive(yAxis, xAxis);
	    				}
	    				
	    				turret.manualTurret(joy.getAxis(AxisType.kX));//xaxis
	    				shooter.manualShooter((-joy.getRawAxis(3)+1)/2);//throttle
	    				shooter.manualLoader(joy.getRawButton(1) ? 1 : 0);
	    				
	    				if(joy.getRawButton(3))
	    				{
	    					intake.manualIntake(1);
	    				}
	    				else if(joy.getRawButton(4))
	    				{
	    					intake.manualIntake(-1);
	    				}
	    				else if(joy.getRawButton(5))
	    				{
	    					intake.manualIntake(0.5);
	    				}
	    				else if(joy.getRawButton(6))
	    				{
	    					intake.manualIntake(-0.5);
	    				}
	    				else
						{
	    					intake.manualIntake(0);
						}
	    				intake.manualDropdown(joy.getRawAxis(1));//yaxis
	    				
	    				hoodToggle.input(joy.getRawButton(12));
	    				hood.set(hoodToggle.get());
	    				
	    				//drive.setShifterState(xbox.getRawAxis(3) > 0.5 ? ShiftingState.HIGH : ShiftingState.LOW);
	    				
	    				//System.out.println("Banner: " + banner.get());
	    			}
	    			
	    			if(ballsensorprint)
	    			{
	    				System.out.println("Voltage: " + ballsensor.getVoltage() + ", AtoD Value: " + loaderSwitch.above());
	    			}
	    			
	    			if(currentprint)
	    			{
	    				String print = "";
	    				for(int i = 0; i < 13; i++)
	    				{
	    					print += "Channel " + i + ": " + pdp.getCurrent(i) + "\n";
	    				}
	    				System.out.print(print);
	    			}
	    			
	    			if(intakeenc)
	    			{
	    				System.out.println("Encoder: " + intake.getIntakeEncoder().getDistance());
	    			}
	    			
	    			if(spigyroprint)
	    			{
	    				System.out.println("SPIGyro: " + spiGyro.getAngle());
	    			}
	    			
    				if(axisprint)
    				{
	    				System.out.println("X: " + joy.getAxis(AxisType.kX));
	    				System.out.println("Y: " + joy.getAxis(AxisType.kY));
	    				System.out.println("Z: " + joy.getAxis(AxisType.kZ));
	    				System.out.println("Throttle: " + joy.getAxis(AxisType.kThrottle));
    				}
    				
    				if(shooterout)
    				{
    					System.out.println("Fly1: " + flyTalon1.get() + ", Fly2: " + flyTalon2.get());
    				}
    				
    				if(tachprint)
    				{
		        		System.out.println("Tach: " + (60/tach.getPeriod()));
    				}
    				
    				if(pressureprint)
    				{
		        		System.out.println("Pressure: " + (250*pressure.getVoltage()/5 - 25));
    				}
    				
    				if(driveencoderprint)
    				{
    					System.out.println("Left: " + leftDrive.getDistance());
    					System.out.println("Right: " + rightDrive.getDistance());
    				}
    				
	    			//drive.manualDrive(yAxis, xAxis);
	    			//shooter.manualTurret(joy.getAxis(AxisType.kX));
	    			//turret.set(joy.getAxis(AxisType.kX));
	    			/*ystem.out.println("Joy: " + joy.getAxis(AxisType.kX));
	    			System.out.println("Encoder: " + turret.getPosition());
	    			System.out.println("DIO: " + banner.get());
	    			System.out.println("Hall: " + halleffect.get());*/
	    			
	    			setCameramode(CameraMode.VISION);
	    			
	    			//System.out.println("SPI Gyro: " + spiGyro.get());
    		}
		}
    	lastJoy = joy.getRawButton(1);
    	lastJoy2 = joy.getRawButton(2);
    	iters++;
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {}
    
    public void setCameramode(CameraMode mode)
    {
    	Dashcomm.putBoolean("data/visioncam", mode == CameraMode.VISION);
    }
}