package org.usfirst.frc.team3476.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.Joystick.RumbleType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team3476.Communications.Dashcomm;
import org.usfirst.frc.team3476.Main.Starter;
import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.ScriptableAuto.Clock;
import org.usfirst.frc.team3476.ScriptableAuto.Main;
import org.usfirst.frc.team3476.Subsystems.*;
import org.usfirst.frc.team3476.Subsystems.Drive.ShiftingState;
import org.usfirst.frc.team3476.Subsystems.Shooter.HoodState;
import org.usfirst.frc.team3476.Utility.ShooterLogger;
import org.usfirst.frc.team3476.Utility.Toggle;
import org.usfirst.frc.team3476.Utility.Control.DifferentialSPIGyro;
import org.usfirst.frc.team3476.Utility.Control.DonutCANTalon;
import org.usfirst.frc.team3476.Utility.Control.DonutDrive;
import org.usfirst.frc.team3476.Utility.Control.MedianEncoder;
import org.usfirst.frc.team3476.Utility.Control.MedianEncoderPair;
import org.usfirst.frc.team3476.Utility.Control.OrangeDigitalInput;
import org.usfirst.frc.team3476.Utility.Control.PIDCANTalonEncoderWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDCounterPeriodWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper.Data;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot
{
	//File logFile = new File("/usr/local/frc/logs/start.txt");
	//FileWriter logger;
	
	enum IntakePos{UP, MID, DOWN, DRAWBRIDGE}
	public static final double CURPOSEPSILON = 1/16;
	IntakePos intakePos = IntakePos.UP;
	
	Joystick xbox = new Joystick(0);
	Joystick joy = new Joystick(1);
	Joystick butbox = new Joystick(2);
	
	PowerDistributionPanel pdp = new PowerDistributionPanel(1);
	
	//TODO: get incorrect channels
	Talon flyTalon1 = new Talon(8),//correct
			flyTalon2 = new Talon(9),//correct
			loaderTalon = new Talon(7),//correct
			intake1 = new Talon(6),
			intake2 = new Talon(5),
			winch = new Talon(4);
											
	DonutCANTalon turretMotor = new DonutCANTalon(4, 0);
	
	PIDDashdataWrapper vision = new PIDDashdataWrapper(Data.VISIONX);
	
	CANTalon ddmotor = new CANTalon(3);
	
	//Flywheel constants
	final double FLY1 = -1, FLY2 = 1;
	
	//Drive
	DonutDrive drive = new DonutDrive(2, 3, 0, 1);

	Solenoid 	shifterSoleniod = new Solenoid(0), hood = new Solenoid(1), shooterflap = new Solenoid(2),
				wheelie = new Solenoid(4), hookRelease = new Solenoid(3);
	
	enum Mode {DEFAULT, INTAKE, SHOOTBATTER, SHOOTFAR, SHOOTFLEX, SHOOTWALL, SHOOTCLIMB}
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
    Subsystem dashPoster;
    
    //Gyro analoggyro = new DifferentialAnalogGyro(0, 5);//TODO: replace with spi
    DifferentialSPIGyro spiGyro = new DifferentialSPIGyro(SPI.Port.kOnboardCS0, 5);
    
    AnalogInput pressure = new AnalogInput(0), ballsensor = new AnalogInput(2);
    
    //AtoD loaderSwitch = new AtoD(ballsensor, 0.65, false);
    OrangeDigitalInput loaderSwitch = new OrangeDigitalInput(4);
    
    DigitalInput banner = new DigitalInput(1);
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
    
    final boolean autodropdown = true;
    
    boolean automatic = true;
    
    boolean shooterdatacollect = false;
    ShooterLogger shooterLogger;
    
    boolean[] 	joybuttons = new boolean[13], xboxbuttons = new boolean[11], butboxbuttons = new boolean[11],
    			lastjoybuttons = new boolean[joybuttons.length],
				lastxboxbuttons = new boolean[xboxbuttons.length],
				lastbutboxbuttons = new boolean[butboxbuttons.length];
    
    private double joysetpoint;
    int joycount;
    int lastPOV;
	private double testset = 0;
	private boolean posmove;
	private double watchset;
	private boolean rehome = false;
	private boolean lastforward;
	private boolean lastbackward;
	private boolean lasthold, gyrocomp = true;

	private boolean setIntake;

	private boolean lastposmove;
    
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
    	
    	leftDrive.setDistancePerPulse(0.05235987755982988730771072305464);
    	rightDrive.setDistancePerPulse(0.05235987755982988730771072305464);
    	
    	loaderSwitch.setInverted(false);
    	
    	intake1.setInverted(false);
    	intake2.setInverted(false);
    	
    	tach.setSamplesToAverage(4);
    	spiGyro.calibrate();
    	turretMotor.setSafetyEnabled(false);
    	loaderTalon.setInverted(true);
    	flyTalon2.setInverted(true);
    	new PIDCANTalonEncoderWrapper(ddmotor, 1).reset();
    	
    	flyTalon1.setSafetyEnabled(false);
    	flyTalon2.setSafetyEnabled(false);
    	
    	if(!automatic)
    	{
    		ddmotor.setInverted(true);
    	}
    	
    	if(shooterdatacollect)
    	{
    		shooterLogger = new ShooterLogger();
    	}
    	
        //Systems
        systems = new Subsystem[10];
		systems[0] = new Drive(leftDrive, rightDrive, spiGyro, drive, shifterSoleniod);
		systems[1] = new Turret(turretMotor, halleffect, vision, spiGyro);
		systems[2] = new Shooter(	flyTalon1, flyTalon2, loaderTalon, (Turret)systems[1], tach,
									loaderSwitch, vision, hood);
		systems[3] = new Intake(intake1, intake2, ddmotor, pdp);
		systems[4] = new Miscellaneous(pressure, hookRelease, winch);
		systems[8] = new Watcher(systems, 10);
		systems[9] = new Clock(systems);
		
		dashPoster = new DashboardPoster(systems);
		
		((Watcher)systems[8]).watch(false);
		
		//misc init
		((Clock)systems[9]).megaEnd();
		((Drive)systems[0]).autoShifting(false);
		((Intake)systems[3]).dropdownEncoderInverted(true);
		((Miscellaneous)systems[4]).climberEnable(true);
    	
    	if(!autodropdown)
    	{
    		((Intake)systems[3]).stopHoming();
    		((Intake)systems[3]).stopDD();
    	}
		
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
    		setCameramode(CameraMode.VISION);
    		main.startSubsystems();
    		main.startThread();
    		first = false;
    		
    		Dashcomm.put("match/enabled", true);
    		setMatchInfo(true);
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
				dashPoster.startThreads();//dashposter should still run
				first = false;
				
				Dashcomm.put("match/enabled", false);
				
			}
			if(camfirst && starter.cameraDone())
			{
				setCameramode(CameraMode.VISION);
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
    	int joyPOV = joy.getPOV(0);
    	int xboxPOV = xbox.getPOV(0);
    	if(starter.importantDone())//WE ARE REEADY TO RUMMMMMMBLEEEEEEE
    	{
    		//intake pos constants
    		final double up = 295, horiz = 4630, down = 5705, drawbridge = 3000;
    		
    		final double FLYWHEELMAX = 9000;
    		
    		final double CLOSESPEED = 6750, FARSPEED = 8000, WALLSPEED = 8000;
    		
    		//prints
    		boolean axisprint = false,  tachprint = false, currentprint = false,
    				pressureprint = false, driveencoderprint = false, spigyroprint = false,
    				intakeenc = false, shooterout = false, ballsensorprint = false,
    				distanceprint = false, shootervdist = false, povprint = false,
    				turretencoderprint = false, loaderstateprint = false, xboxaxisprint = false,
    				driveencoderrateprint = false, climberstateprint = false;
    		
    		//others
    		boolean turretgo = true, buttons = false, distmode = false,
    				searchgo = false, manualLoad = false, ddtuning = false,
    				scalerauto = true;
    		
    		for(int i = 1; i < joybuttons.length; i++)
    		{
    			joybuttons[i] = joy.getRawButton(i);
    		}
    		
    		for(int i = 1; i < xboxbuttons.length; i++)
    		{
    			xboxbuttons[i] = xbox.getRawButton(i);
    		}
    		
    		for(int i = 1; i < xboxbuttons.length; i++)
    		{
    			butboxbuttons[i] = butbox.getRawButton(i);
    		}
    		
    		double 	xAxis = -xbox.getRawAxis(4),
	    			yAxis = -xbox.getRawAxis(1);
    		
    		boolean joy1 = joy.getRawButton(1), joy2 = joy.getRawButton(2);
    		
    		boolean dropdowndown = butboxbuttons[2], dropdownmid = butboxbuttons[3], 
    				dropdownup = butboxbuttons[4], shootwall = butboxbuttons[5], shootoff = butboxbuttons[6],
    				shootfar = butboxbuttons[7], shootbatter = butboxbuttons[8],
					dropdownhome = joybuttons[11] && !lastjoybuttons[11],
					shootclimb = butboxbuttons[9], winchon = butboxbuttons[10] && !lastbutboxbuttons[10],
					winchoff = !butboxbuttons[10] && lastbutboxbuttons[10];
    		
    		boolean dropdowndrawbridge = xboxbuttons[5],
    				turrethome = joybuttons[12] && !lastjoybuttons[12],
					turretstophome = !joybuttons[12] && lastjoybuttons[12],
					shootmanual = joybuttons[6], gyrocompon = joybuttons[7], gyrocompoff = joybuttons[8],
					autofire = butboxbuttons[1], lastautofire = lastbutboxbuttons[1],
					scaleradvance = joybuttons[10] && joybuttons[9] &&
						(!lastjoybuttons[10] || !lastjoybuttons[9]);
    		
    		Drive drive = (Drive)systems[0];
    		Turret turret = (Turret)systems[1];
    		Shooter shooter = (Shooter)systems[2];
    		Intake intake = (Intake)systems[3];
    		Miscellaneous misc = (Miscellaneous)systems[4];
    		
			PIDCANTalonEncoderWrapper encoder = turret.getTurretEncoder();
    		switch(first ? 1 : 0)
    		{
	    		case 1://first - For the first time in forever
	    			System.out.println("teleopPeriodic first");
					//DO THINGS
	    			dashPoster.startThreads();
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
						if(shooterdatacollect)
						{
							shooterLogger.clearLogger();
						}
						if(searchgo)
						{
							turret.aim(0);
						}
					}
					else
					{
						turret.killHomer();
					}
					
					Dashcomm.put("match/enabled", true);
					
					
	    		default://!first - Normal execution
	    			if(automatic)
	    			{
	    				//============
	    				//===Scaler===
	    				//============
	    				if(scalerauto)
	    				{
	    					if(scaleradvance && !turret.isLocalAngleinRange(0.125, -0.125))
	    					{
	    						misc.startClimb();
	    						System.out.println("SCALER ADVANCE");
	    					}
	    					
	    					if(misc.isFired() || Dashcomm.get("data/misc/resetting", false))
    						{
	    						if(winchon)
		    					{
		    						misc.startWinch();
		    						System.out.println("WINCH ON");
		    					}
		    					
		    					if(winchoff)
		    					{
		    						misc.stopWinch();
		    						System.out.println("WINCH OFF");
		    					}
    						}
	    				}
	    				else
	    				{
		    				if(joybuttons[10])
		    				{
		    					winch.set(1);
		    				}
		    				/*else if(joybuttons[9])
		    				{
		    					winch.set(-1);
		    				}*/
		    				else
		    				{
		    					winch.set(0);
		    				}
		    				
		    				hookRelease.set(butboxbuttons[9]);
	    				}
	    				
	    				//===========
	    				//===Drive===
	    				//===========
	    				boolean forward = xboxPOV == 0, backward = xboxPOV == 180, hold = xboxbuttons[3];//3 = x - hold
	    				if(forward || backward)//Ramparts crossing
	    				{
	    					if(!(lastforward || lastbackward))
	    					{
	    						if(forward)
		    					{
		    						drive.doAuto(new double[]{300, 70}, "driven");
		    					}
		    					else
		    					{
		    						drive.doAuto(new double[]{-300, 70}, "driven");
		    					}
	    						drive.killDriveManual();
	    					}
	    				}
	    				else if(hold)
	    				{
	    					if(!lasthold)
    						{
	    						drive.doAuto(new double[]{0, 0}, "hold");
    						}
	    				}
	    				else//regular drive
	    				{
	    					if(lastforward || lastbackward || lasthold)
	    					{
	    						drive.killAuto();
	    					}
	    					drive.augmentedDrive(yAxis, xAxis);
	    				}
	    				lastforward = forward;
	    				lastbackward = backward;
	    				lasthold = hold;
	    				
	    				
	    				//==========================
	    				//===Shooting Mode Switch===
	    				//==========================
	    				if(shootbatter)
	    				{
	    					setCameramode(CameraMode.VISION);
	    					mode = Mode.SHOOTBATTER;
	    				}
	    				else if(shootfar)
	    				{
	    					setCameramode(CameraMode.VISION);
	    					mode = Mode.SHOOTFAR;
	    				}
	    				else if(shootoff)
	    				{
	    					setCameramode(CameraMode.INTAKE);
	    					mode = Mode.DEFAULT;
	    				}
	    				else if(shootmanual)
	    				{
	    					setCameramode(CameraMode.VISION);
	    					mode = Mode.SHOOTFLEX;
	    				}
	    				else if(shootwall)
	    				{
	    					setCameramode(CameraMode.VISION);
	    					mode = Mode.SHOOTWALL;
	    				}
	    				else if(shootclimb)
	    				{
	    					setCameramode(CameraMode.VISION);
	    					mode = Mode.SHOOTCLIMB;
	    				}
	    				
	    				
	    				//=========================
	    				//===CameraMode Override===
	    				//=========================
	    				if(xbox.getRawButton(2))
	    				{
	    					setCameramode(CameraMode.INTAKE);
	    				}
	    				
	    				boolean eject = joybuttons[4];
	    				
	    				if(eject)
	    				{
	    					shooter.resetLoader();
	    					shooter.manualLoader(-1);
	    				}
	    				
	    				
	    				//============
	    				//===Intake===
	    				//============
	    				if(eject)
	    				{
	    					intake.manualIntake(-1);
	    				}
	    				else if(joybuttons[3])
	    				{
	    					shooter.intakeNotify(true);
	    					intake.manualIntake(1*shooter.getIntakeScaling());
	    				}
	    				else if(joybuttons[5])
	    				{
	    					shooter.intakeNotify(true);
	    					intake.manualIntake(-1*shooter.getIntakeScaling());
	    				}
	    				else
						{
	    					shooter.intakeNotify(false);
	    					intake.manualIntake(0);
						}
	    				
	    				
	    				//=============
	    				//===Shooter===
	    				//=============
	    				if(eject)
	    				{
	    					//shooter.manualShooter(-1); //doesnt help
	    				}
	    				else
	    				{
		    				switch(mode)
		    				{
			    				case SHOOTBATTER:
			    					shooter.setHood(HoodState.HIGH);
			    					shooter.setFly(CLOSESPEED);
			    					shooterflap.set(false);
		    						break;
		    						
			    				case SHOOTFAR:
			    					shooter.setHood(HoodState.LOW);
			    					shooter.setFly(FARSPEED);
			    					shooterflap.set(false);
		    						break;
		    						
			    				case SHOOTFLEX:
			    					shooter.setHood(HoodState.LOW);
			    					shooterflap.set(false);
		    						if(distmode)
		    		    			{
		    		    				if(joy.getRawButton(2))
		    		    				{
		    		    					shooter.setFly();
		    		    				}
		    		    				else
		    		    				{
		    		    					shooter.stopFly();
		    		    				}
		    		    			}
		    		    			else//nondist
		    		    			{
		    		    				//System.out.println("Flyset: " + ((-joy.getRawAxis(3) + 1)/2)*FLYWHEELMAX);
		    		    				shooter.setFly(((-joy.getRawAxis(3) + 1)/2)*FLYWHEELMAX);
		    		    			}
		    						break;
		    						
			    				case SHOOTWALL:
			    					shooter.setHood(HoodState.LOW);
			    					shooter.setFly(WALLSPEED);
			    					shooterflap.set(true);
			    					break;
			    					
			    				case SHOOTCLIMB:
			    					shooter.setHood(HoodState.HIGH);
			    					shooter.setFly(CLOSESPEED);
			    					shooterflap.set(true);
			    					break;
		    						
			    				case DEFAULT:
			    					shooter.setHood(HoodState.LOW);
			    					shooter.setFly(0);
			    					shooterflap.set(false);
			    					break;
			    					
			    				default:
			    					System.out.println("Invalid Teleop State: " + mode);
			    					break;
		    				}
	    				}
	    				
	    				
	    				//============
	    				//===Turret===
	    				//============
	    				if(turretgo)
	    				{
	    					if(turretstophome)
	    					{
	    						turret.killHomer();
	    						rehome = false;
	    					}
	    					else if(turrethome)
	    					{
	    						//start homing
	    						turret.resetHomer();
	    						rehome = true;
	    					}
	    					
	    					if(gyrocompon)
	    					{
	    						gyrocomp = true;
	    					}
	    					else if(gyrocompoff)
	    					{
	    						gyrocomp = false;
	    					}
	    					
	    					if(!rehome)
	    					{
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
		    					else if(searchgo)//search testing
		    					{
		    						if(joy2 && !lastJoy2)
	    							{
		    							turret.search(-0.1);
	    							}
		    						else if(!turret.isSearching())
		    						{
		    							turret.aim();
		    						}
		    					}
		    					else//joystick setpoint
		    					{
		    						if(!autofire)
		    						{
		    							if(lastautofire)
				    					{
		    								System.out.println("lastautofire");
		    								joysetpoint = (gyrocomp ? turret.getCurrentGyroPos() : 0) +
			    									turret.getTurretEncoder().getDistance();
				    						turret.stopAim();
				    					}
			    						//if(intake.intakeRunning()) POV = 0;//turn the turret if the intake is running
			    						switch(joyPOV)
		    							{
			    							case -1://nothing
			    								watchset = Double.NaN;
			    								posmove = false;
			    								break;
			    								
			    							default:
			    								watchset = joyPOV/360.0 + 0.000000001;
			    								posmove = true;
		    							}
			    						
			    						if(posmove)//moving to position
			    						{
			    							if(turret.isAutoDone() && turret.getTurretSet() == watchset)//posmove done
			    							{
			    								posmove = false;
			    							}
			    							else//not there yet, set it yo
			    							{
			    								turret.aim(watchset);
			    							}
			    						}
			    						else//joystick setpoint
			    						{
				    						if(joybuttons[2])
				    						{
				    							if(!lastjoybuttons[2])
				    							{
				    								turret.aim();
				    							}
				    						}
				    						else
				    						{
				    							if(lastJoy2 || lastposmove || lastautofire)
				    							{
				    								joysetpoint = (gyrocomp ? turret.getCurrentGyroPos() : 0) +
				    									turret.getTurretEncoder().getDistance();
				    							}
				    							if(Math.abs(joy.getRawAxis(0)) > 0.1)
					    							joysetpoint += joy.getRawAxis(0)*0.01;
				    							else
				    							{
				    								joysetpoint = gyrocomp ? joysetpoint :
				    									turret.getTurretEncoder().getDistance();
				    								//System.out.println("joysetpoint: " + joysetpoint);
				    							}
				    							
				    							if(gyrocomp)
				    							{
				    								turret.gyroAim(joysetpoint);
				    							}
				    							else
				    							{
				    								turret.aim(joysetpoint);
				    							}
				    						}
			    						}//end joystick setpoint
			    						lastposmove = posmove;
			    					}//end !autofire
		    					}//end joysetpoint
	    					}
	    				}
	    				else
	    				{
	    					turret.manualTurret(joy.getAxis(AxisType.kX));//xaxis
	    				}
	    				
	    				//============
	    				//===Rumble===
	    				//============
	    				if(turret.isVisionTracking())
	    				{
	    					xbox.setRumble(RumbleType.kRightRumble, (float)1.0);
	    				}
	    				else
	    				{
	    					xbox.setRumble(RumbleType.kRightRumble, (float)0.0);
	    				}
		    			
		    			//shooter.manualShooter((-joy.getRawAxis(3)+1)/2);//throttle
		    			//shooter.manualShooter(0);
		    			//shooter.manualLoader(joy.getRawButton(1) ? 1 : 0);
		    			
	    				
	    				//==============
	    				//===Dropdown===
	    				//==============
	    				if(autodropdown)
	    				{
	    					if(ddtuning)
	    					{
	    						if(joybuttons[1])
	    						{
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
	    						}
	    						else
								{
	    							intake.stopDD();
								}
	    					}
	    					else
	    					{
	    						if(dropdownhome)
	    						{
	    							intake.home();
	    						}
	    						if(!intake.intakeRunning())
	    						{
		    						if(dropdownup)
				    				{
		    							intake.moveDropdown(up);
				    					intakePos = IntakePos.UP;
				    				}
				    				else if(dropdownmid)
				    				{
				    					intake.moveDropdown(horiz);
				    					intakePos = IntakePos.MID;
				    				}
				    				else if(dropdowndown)
				    				{
				    					intake.moveDropdown(down);
				    					intakePos = IntakePos.DOWN;
				    				}
				    				else if(dropdowndrawbridge)
				    				{
				    					intake.moveDropdown(drawbridge);
				    					intakePos = IntakePos.DRAWBRIDGE;
				    				}
		    						
		    						if(setIntake)//we need to set it back to what it was before
		    						{
			    						switch(intakePos)
			    						{
			    							case UP:
				    							intake.moveDropdown(up);
				    							break;
			    							case MID:
				    							intake.moveDropdown(horiz);
				    							break;
			    							case DOWN:
				    							intake.moveDropdown(down);
				    							break;
			    							case DRAWBRIDGE:
				    							intake.moveDropdown(drawbridge);
				    							break;
			    						}
		    						}
		    						
		    						setIntake = false;
	    						}
	    						else
	    						{
	    							if(intakePos == IntakePos.DOWN)
	    							{
	    								intake.moveDropdown(horiz);
	    							}
	    							setIntake = true;
	    						}
	    					}
	    				}
	    				else//manual dropdown
	    				{
	    					intake.stopDD();
	    					intake.manualDropdown(-joy.getRawAxis(1));//yaxis
	    				}
	    				
	    				
	    				//=================
	    				//===Wheelie Bar===
	    				//=================
	    				if(xboxbuttons[1])//a - extend
	    				{
	    					wheelie.set(true);
	    				}
	    				else if(xboxbuttons[4])//y - retract
	    				{
	    					wheelie.set(false);
	    				}
		    			
	    				
	    				//==========================
	    				//===Shooter Data Logging===
	    				//=======and Loading========
	    				//==========================
		    			if(shooterdatacollect)
		    			{
		    				if(joy1 && !lastJoy)
		    				{
		    					shooterLogger.prepValue(turret.getVisionWrapper().getClosestDist(), 60/tach.getPeriod());
		    				}
		    				
		    				if(shooterdatacollect && shooterLogger.valueToLog() && joy.getPOV(0) == 0)
		    				{
		    					shooterLogger.logPrepped();
		    				}
		    				
		    				if(shooterdatacollect && joyPOV == 180 && lastPOV != 180)
		    				{
		    					System.out.println(shooterLogger);
		    				}
		    			}
		    			else if(manualLoad)
		    			{
		    				if(joybuttons[1] && !lastjoybuttons[1])
		    				{
		    					System.out.println("DOING THE LOAD HACK\n\n\n\n\n\n");
		    					shooter.setLoaded();
		    					shooter.startFire();
		    				}
		    			}
		    			else//regular shooting
		    			{
		    				if(autofire)
		    				{
		    					if(!lastautofire)
		    					{
		    						turret.aim();
		    					}
		    					if(turret.isAutoDone() && shooter.getFlySet() > 0)//tracked and spinning
		    					{
		    						shooter.safeFire();
		    					}
		    				}
		    				else
		    				{
		    					if(joy.getRawButton(1) && shooter.getFlySet() > 0)
				    			{
				    				shooter.safeFire();
				    			}
		    				}
		    			}
		    			
		    			//==============
		    			//===Shifting===
		    			//==============
		    			if(xbox.getRawAxis(3) > 0.5 || xbox.getRawAxis(2) > 0.5)//manual
		    			{
		    				drive.manualShift(xbox.getRawAxis(3) > 0.5 ? ShiftingState.HIGH : ShiftingState.LOW);
		    			}
		    			else//auto
		    			{
		    				drive.autoShifting(true);
		    			}
		    			
    				}
	    			else//manual
	    			{
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
	    				
	    				boolean donuttest = false;
	    				
	    				if(donuttest)
	    				{
	    					if(joy.getRawButton(1) && !lastJoy)
	    					{
	    						testset += 0.005;
	    					}
	    					else if(joy.getRawButton(2) && !lastJoy2)
	    					{
	    						testset -= 0.005;
	    					}
	    					turret.manualTurret(testset);//xaxis
	    					System.out.println("Motor value: " + turretMotor.get());
	    				}
	    				else
	    				{
	    					turret.manualTurret(joy.getRawAxis(0));//xaxis
	    				}
	    				
	    				shooter.manualShooter((-joy.getRawAxis(3)+1)/2);//throttle
	    				//shooter.manualLoader(joy.getRawButton(1) ? 1 : 0);
	    				
	    				if(joybuttons[1] && !lastjoybuttons[1])
	    				{
	    					System.out.println("DOING THE LOAD HACK\n\n\n\n\n\n");
	    					shooter.setLoaded();
	    					shooter.startFire();
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
	    				intake.manualDropdown(joy.getRawAxis(1));//yaxis
	    				
	    				hoodToggle.input(joy.getRawButton(12));
	    				hood.set(hoodToggle.get());
	    				
	    				drive.setShifterState(xbox.getRawAxis(3) > 0.5 ? ShiftingState.HIGH : ShiftingState.LOW);
	    				
	    				//System.out.println("Banner: " + banner.get());
	    			}
	    			
	    			if(loaderstateprint)
	    			{
	    				System.out.println("Loader State: " + shooter.getLoaderState());
	    			}
	    			
	    			if(turretencoderprint)
	    			{
	    				System.out.println("Turret encoder: " + turret.getTurretEncoder().getDistance());
	    			}
	    			
	    			if(xboxaxisprint)
	    			{
	    				System.out.println("Xbox x: " + xAxis + ", Xbox y:" + yAxis);
	    			}
	    			
	    			if(povprint)
	    			{
	    				System.out.println("POV: " + joy.getPOV(0));
	    			}
	    			
	    			if(shootervdist)
	    			{
	    				System.out.println("{Tach, Dist}: {" + (60/tach.getPeriod()) + ", " + turret.getVisionWrapper().getClosestDist() + "}");
	    			}
	    			
	    			if(distanceprint)
	    			{
	    				System.out.println("Distance to goal: " + turret.getVisionWrapper().getClosestDist());
	    			}
	    			
	    			if(ballsensorprint)
	    			{
	    				System.out.println("Digital Value: " + loaderSwitch.get());
	    			}
	    			
	    			if(currentprint)
	    			{
	    				String print = "";
	    				
	    				boolean one = true;
	    				
	    				if(one)
	    				{
	    					int channel = 2;
	    					print += "Channel " + channel + ": " + pdp.getCurrent(channel) + "\n";
	    				}
	    				else
	    				{
	    					for(int i = 0; i < 13; i++)
		    				{
		    					print += "Channel " + i + ": " + pdp.getCurrent(i) + "\n";
		    				}
	    				}
	    				System.out.print(print);
	    			}
	    			
	    			if(intakeenc)
	    			{
	    				System.out.println("Setpoint: " + intake.getDDSet());
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
		        		if(iters % 10 == 0)
	        			{
		        			System.out.println("Tach: " + (60/tach.getPeriod()) + "\nSet:  " + shooter.getFlySet() + "\n");
	        			}
    				}
    				
    				if(pressureprint)
    				{
		        		System.out.println("Pressure: " + (250*pressure.getVoltage()/5 - 25));// + "\nVoltage: " + pressure.getVoltage());
    				}
    				
    				if(driveencoderprint)
    				{
    					System.out.println("Left: " + leftDrive.getDistance());
    					System.out.println("Right: " + rightDrive.getDistance());
    				}
    				
    				if(driveencoderrateprint)
    				{
    					System.out.println("Rate: " + ((Drive)systems[0]).getEncoderPair().getRate());
    				}
    				
    				if(climberstateprint)
    				{
    					System.out.println("ClimberState: " + misc.getClimberState());
    				}
    		}//end time switch
    		setMatchInfo(false);
		}//end importantdone
    	lastPOV = joyPOV;
    	lastJoy = joy.getRawButton(1);
    	lastJoy2 = joy.getRawButton(2);
    	
    	for(int i = 1; i < lastjoybuttons.length; i++)
		{
    		lastjoybuttons[i] = joybuttons[i];
		}
		
    	for(int i = 1; i < lastxboxbuttons.length; i++)
		{
			lastxboxbuttons[i] = xboxbuttons[i];
		}
    	
    	for(int i = 1; i < lastbutboxbuttons.length; i++)
		{
    		lastbutboxbuttons[i] = butboxbuttons[i];
		}
    	iters++;
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {}
    
    public void setCameramode(CameraMode mode)
    {
    	Dashcomm.put("data/visioncam", mode == CameraMode.VISION);
    }
    
    public void setMatchInfo(boolean auto)
    {
    	Dashcomm.put("match/auto?", auto);
    }
}