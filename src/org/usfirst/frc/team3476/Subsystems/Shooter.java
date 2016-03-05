package org.usfirst.frc.team3476.Subsystems;


import java.util.Arrays;

import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.Utility.DIOHomer;
import org.usfirst.frc.team3476.Utility.DIOHomer.HomeState;
import org.usfirst.frc.team3476.Utility.ManualHandler;
import org.usfirst.frc.team3476.Utility.OrangeUtility;
import org.usfirst.frc.team3476.Utility.RunningAverage;
import org.usfirst.frc.team3476.Utility.Control.BangBang;
import org.usfirst.frc.team3476.Utility.Control.PIDCANTalonEncoderWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDCounterPeriodWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper;
import org.usfirst.frc.team3476.Utility.Control.SmoothBangBang;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper.Data;
import org.usfirst.frc.team3476.Utility.Control.PIDMotorGroup;
import org.usfirst.frc.team3476.Utility.Control.TakeBackHalf;

import edu.wpi.first.wpilibj.CANSpeedController;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDController.Tolerance;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.hal.DIOJNI;

//TODO: javadoc comments if not specified by Subsystem
/*
 * Represents the robot's shooter.
 */
public class Shooter implements Subsystem
{
	private final String[] autoCommands = {"shooter", "visionaim", "encoderaim", "flywheel", "fire", "reload"};
	private final String[] constants = {"SHOOTEROUTPUTRANGEHIGH", "SHOOTEROUTPUTRANGELOW", "SHOOTERIGAIN",
			"FLY1DIR", "FLY2DIR", "FLYWHEELDEAD", "FLYWHEELMAXSPEED", "TURRETENCODERP", "TURRETENCODERI", "TURRETENCODERD",
			"TURRETENCODERDEAD", "LOADERSPEED", "SOFTLIMITS", "USESOFT", "HOMINGDIR", "HOMEHIGH",
			"HOMESEEKSPEED", "HOMESLOWSPEED", "HOMETIMEOUT", "NUMSMOOTHSAMPLES", "SHOOTERPGAIN", "SHOOTERDGAIN", "LOADERSPEEDLOW",
			"FIRINGTIME"};
	private final double RPSPERRPM = 60, TURRETOUTPUTRANGE = 0.25, ROTATIONSPERTICK = 2.2379557291666666666666666666667e-5;
	
	enum AimMode{VISION, ENCODER}
	AimMode aimmode;
	enum LoaderState{RELOADINGSLOW, RELOADINGFAST, LOADED, FIRING, WAITING}
	LoaderState loaderState;
	
	private double SHOOTEROUTPUTRANGEHIGH, SHOOTEROUTPUTRANGELOW, SHOOTERIGAIN, FLYWHEELDEAD,
	FLYWHEELMAXSPEED, TURRETENCODERP, TURRETENCODERI, TURRETENCODERD,
	TURRETENCODERDEAD, LOADERSPEED, SOFTLIMITS, SOFTRESETANGLE, USESOFT, HOMINGDIR, HOMEHIGH,
	HOMESEEKSPEED, HOMESLOWSPEED, HOMETIMEOUT, NUMSMOOTHSAMPLES,
	SHOOTERPGAIN, SHOOTERDGAIN, LOADERSPEEDLOW, FIRINGTIME;
	private int softDir;
	private int[] FLYDIRS;
	private boolean flyDone, loadDone, toFire, toReload, firingLast,
					pass1, intakeRunning;
	private SpeedController fly1, fly2, loader;
	private TakeBackHalf control;
	private PIDCounterPeriodWrapper tach;
	private Timer shootingTimer;
	
	private PIDController flywheelcontrol;
	private PIDMotorGroup flygroup;
	
	private DigitalInput loaderSwitch;
	
	private SubsystemTask task;
	private Thread shooterThread;
	
	private RunningAverage tachavg;
	
	private int iters;
	
	final long MANUALTIMEOUT = 50;//in ms
	final double FLYPIDEXECTIME = 1;
	private ManualHandler shooterManual, loaderManual, turretManual;
	
	private DIOHomer turretHomer;
	
	Turret turret;
	private boolean lastballsw;
	private double flyset, lastflyset;
	
	
	public Shooter(	SpeedController fly1in, SpeedController fly2in, SpeedController loaderin,
					Turret turretin, PIDCounterPeriodWrapper tachin, DigitalInput loaderSwitchin)
	{
		turret = turretin;
		
		//Shooter
		fly1 = fly1in;
		fly2 = fly2in;
		flyDone = true;
		loadDone = true;
		tach = tachin;
		tachavg = new RunningAverage(8);
		toFire = false;
		toReload = false;
		shootingTimer = new Timer();
		FLYDIRS = new int[]{0, 0};
		flygroup = new PIDMotorGroup(new SpeedController[]{fly1, fly2}, Arrays.copyOf(FLYDIRS, FLYDIRS.length));
		
		//Flywheel
		//control = new TakeBackHalf(new double[]{0, 0}, 0, 0);
		//control = new TakeBackHalf(new double[]{0, 0}, 1);
		flywheelcontrol = new PIDController(0, 0, 0, tach, flygroup, FLYPIDEXECTIME/1000.0);
		
		//Loader
		loaderSwitch = loaderSwitchin;
		loader = loaderin;
		loaderState = LoaderState.WAITING;
		
		//Manuals
		shooterManual = new ManualHandler(MANUALTIMEOUT);
		turretManual = new ManualHandler(MANUALTIMEOUT);
		loaderManual = new ManualHandler(MANUALTIMEOUT);
		
		iters = 0;
		task = new SubsystemTask(this, 10);//10ms minimum exec time
		shooterThread = new Thread(task, "shooterThread");
		shooterThread.start();
	}
	
	@Override
	public String[] getAutoCommands()
	{
		return autoCommands;
	}
	
	@Override
	public synchronized void doAuto(double[] params, String command)
	{
		switch(command)
		{
			case "shooter":
				setFly(params[1]);
				break;
			case "flywheel":
				setFly(params[0]);
				break;
			case "fire":
				startFire();
				break;
			case "reload":
				startFire();
				break;
			case "visionaim":
				turret.doAuto(params, command);
				break;
			case "encoderaim":
				turret.doAuto(params, command);
				break;
		}
	}
	
	@Override
	public synchronized boolean isAutoDone()
	{
		return flyDone && loadDone;
	}
	
	@Override
	public String[] getConstantRequest()//Request all needed constants
	{
		return constants;
	}
	
	@Override
	public synchronized void returnConstantRequest(double[] constantsin)//Get all needed constants
	{
		double 	prevshooterp = SHOOTERPGAIN,
				prevshooteri = SHOOTERIGAIN,
				prevshooterd = SHOOTERDGAIN,
				prevshooterdead = FLYWHEELDEAD,
				prevoutputrangehigh = SHOOTEROUTPUTRANGEHIGH,
				prevoutputrangelow = SHOOTEROUTPUTRANGELOW;
		
		int[] prevflydirs = Arrays.copyOf(FLYDIRS, FLYDIRS.length);
		
		int i = 0;
		SHOOTEROUTPUTRANGEHIGH = constantsin[i];
		i++;//1
		SHOOTEROUTPUTRANGELOW = constantsin[i];
		i++;//2
		SHOOTERIGAIN = constantsin[i];
		i++;//3
		FLYDIRS[i - 3] = (int)constantsin[i];
		i++;//4
		FLYDIRS[i - 3] = (int)constantsin[i];
		i++;//5
		FLYWHEELDEAD = constantsin[i];
		i++;//6
		FLYWHEELMAXSPEED = constantsin[i];
		i++;//7
		TURRETENCODERP = constantsin[i];
		i++;//8
		TURRETENCODERI = constantsin[i];
		i++;//9
		TURRETENCODERD = constantsin[i];
		i++;//10
		TURRETENCODERDEAD = constantsin[i];
		i++;//11
		LOADERSPEED = constantsin[i];
		i++;//12
		SOFTLIMITS = constantsin[i];
		SOFTRESETANGLE = Math.min(1.5*SOFTLIMITS, 0.95);
		i++;//13
		USESOFT = constantsin[i];
		i++;//14
		HOMINGDIR = constantsin[i];
		i++;//15
		HOMEHIGH = constantsin[i];
		i++;//16
		HOMESEEKSPEED = constantsin[i];
		i++;//17
		HOMESLOWSPEED = constantsin[i];
		i++;//18
		HOMETIMEOUT = constantsin[i];
		i++;//19
		NUMSMOOTHSAMPLES = constantsin[i];
		i++;//20
		SHOOTERPGAIN = constantsin[i];
		i++;//21
		SHOOTERDGAIN = constantsin[i];
		
		if(!Arrays.equals(FLYDIRS, prevflydirs))
		{
			flygroup.setDirs(Arrays.copyOf(FLYDIRS, FLYDIRS.length));
		}
		
		boolean pdiff = prevshooterp != SHOOTERPGAIN, idiff = prevshooteri != SHOOTERIGAIN,
				ddiff = prevshooterd != SHOOTERDGAIN, deaddiff = prevshooterdead != FLYWHEELDEAD,
				outdiff = prevoutputrangelow != SHOOTEROUTPUTRANGELOW || prevoutputrangehigh != SHOOTEROUTPUTRANGEHIGH;
		
		if(pdiff || idiff || ddiff || deaddiff || outdiff)//different or null
		{
			String print = "Different constants: ";
			print += pdiff ? "SHOOTERPGAIN " + SHOOTERPGAIN + " ": "";
			print += idiff ? "SHOOTERIGAIN " + SHOOTERIGAIN + " ": "";
			print += ddiff ? "SHOOTERDGAIN " + SHOOTERDGAIN + " ": "";
			print += deaddiff ? "FLYWHEELDEAD " + FLYWHEELDEAD + " ": "";
			print += outdiff ? "SHOOTEROUTPUTRANGEHIGH " + SHOOTEROUTPUTRANGEHIGH + " SHOOTEROUTPUTRANGELOW " + SHOOTEROUTPUTRANGELOW: "";
			System.out.println(print);
			
			flywheelcontrol.setPID(SHOOTERPGAIN, SHOOTERIGAIN, SHOOTERDGAIN);
			flywheelcontrol.setOutputRange(SHOOTEROUTPUTRANGELOW, SHOOTEROUTPUTRANGEHIGH);
			flywheelcontrol.setAbsoluteTolerance(FLYWHEELDEAD);
		}
		
		/*TakeBackHalf temptbh = new TakeBackHalf(new double[]{SHOOTEROUTPUTRANGELOW, SHOOTEROUTPUTRANGEHIGH}, SHOOTERIGAIN, FLYWHEELMAXSPEED);
		
		if(!temptbh.equals(control))
		{
			control.setGain(SHOOTERIGAIN);
			control.setMAX(FLYWHEELMAXSPEED);
			control.setOutputrange(new double[]{SHOOTEROUTPUTRANGEHIGH, SHOOTEROUTPUTRANGELOW});
		}*/
	}
	
	@Override
	public synchronized void update()//Flywheel and turret control loop
	{
		//=====================
		//=======Shooter=======
		//=====================
		if(shooterManual.isTimeUp())
		{
			if(!flywheelcontrol.isEnabled())
			{
				flywheelcontrol.enable();
			}
			
			if(flyset != lastflyset)//new value
			{
				flywheelcontrol.setSetpoint(flyset);
			}
			
			if(iters % 20 == 0)
			{
				System.out.println(OrangeUtility.PIDData(flywheelcontrol));
			}
			
			
			/*if(control == null)
			{
				throw new NullPointerException("No TakeBackHalf controller in Subsystem \"" + this +  "\" - constants not returned");
			}
			else
			{
				output = control.output(process);
			}
			//OrangeUtility.ControlData(control.getError(process), process, output)
			/*if(iters % 20 == 0)
			{
				String print = OrangeUtility.ControlLoopData(control, process) + "\nI: " + control.getGain() + "\nSetpoint: " + control.getMax();
				System.out.println(print);
			}*/
			//output = control.getSetpoint() > 0 ? output : 0;
			//fly1.set(output*FLYDIRS[0]);
			//fly2.set(output*FLYDIRS[1]);
			
			if(flywheelcontrol.onTarget() && Math.abs(flywheelcontrol.getError()) < FLYWHEELDEAD)
			{
				flyDone = true;
			}
			
			lastflyset = flyset;
		}
		else//manual control
		{
			if(flywheelcontrol.isEnabled())
			{
				flywheelcontrol.disable();
			}
		}
		
		//======================
		//========Loader========
		//======================
		boolean ballsw = loaderSwitch.get();
		if(loaderManual.isTimeUp())
		{
			switch(loaderState)
			{
				case WAITING:
					loader.set(0);
					if(intakeRunning || toReload)
					{
						loaderState = LoaderState.RELOADINGFAST;
					}
					loadDone = true;
					
					//Reset state to avoid unpredictable behavior
					toFire = false;
					break;
					
				case RELOADINGFAST:
					loader.set(LOADERSPEED);
					if(ballsw && !lastballsw)
					{
						loaderState = LoaderState.RELOADINGSLOW;
					}
					
					//Reset state to avoid unpredictable behavior
					toReload = false;
					toFire = false;
					break;
					
				case RELOADINGSLOW:
					loader.set(LOADERSPEEDLOW);
					if(!ballsw && lastballsw)
					{
						loaderState = LoaderState.LOADED;
					}
					
					//Reset state to avoid unpredictable behavior
					toReload = false;
					toFire = false;
					break;
					
				case LOADED:
					loader.set(0);
					if(toFire)
					{
						loaderState = LoaderState.FIRING;
						shootingTimer.reset();
						shootingTimer.start();
					}
					loadDone = true;
					
					//Reset state to avoid unpredictable behavior
					toReload = false;
					break;
					
				case FIRING:
					loader.set(LOADERSPEED);
					if(shootingTimer.get() > FIRINGTIME)//done firing
					{
						loaderState = LoaderState.WAITING;
					}
					//Reset state to avoid unpredictable behavior
					toReload = false;
					toFire = false;
					break;
			}
		}
		
		//Check if we're done here 
		//TODO: Decide if the flywheel needs to be in the deadzone for multiple iterations
		
		iters++;
		lastballsw = ballsw;
	}
	
	/**
	 * Controls the loader with simple motor values.
	 * @param move the movement value
	 */
	public void manualLoader(double move)
	{
		loaderManual.poke();
		loader.set(move);
	}
	
	/**
	 * Controls the turret with simple motor values.
	 * @param rotate the rotation value
	 */
	public void manualShooter(double speed)
	{
		shooterManual.poke();
		fly1.set(speed*FLYDIRS[0]);
		fly2.set(speed*FLYDIRS[1]);
	}
	
	/**
	 * Notifies the Shooter of the intake's activities.
	 * @param runnning if the shooter is running
	 */
	public void intakeNotify(boolean runnning)
	{
		intakeRunning = runnning;
	}
	
	/**
	 * Tells the loader to reload. Only has an effect when waiting(not loaded or moving).
	 */
	public synchronized void reload()
	{
		toReload = true;
		loadDone = false;
	}
	
	/**
	 * Tells the shooter to fire. Only has an effect when loaded(not waiting or moving).
	 */
	public synchronized void startFire()
	{
		toFire = true;
		loadDone = false;
	}
	
	/**
	 * Resets the loader state to WAITING in case of bad things.
	 */
	public synchronized void resetLoader()
	{
		loaderState = LoaderState.WAITING;
	}
	
	public synchronized boolean isFiring()
	{
		return loaderState == LoaderState.FIRING;
	}
	
	/**
	 * Tells the control loop to change the flywheel rpm. 
	 * @param rpm the setpoint for the flywheel in RPM
	 */
	public void setFly(double rpm)
	{
		flyDone = false;
		flyset = rpm;
	}
	
	public String toString()
	{
		return "Shooter";
	}
	
	@Override
	public void stopThreads()
	{
		task.hold();
	}
	
	@Override
	public void startThreads()
	{
		task.resume();
	}
	
	public void terminateThreads()
	{
		task.terminate();
		try
		{
			shooterThread.join();
			System.out.println("Ended " + this + " thread.");
		}
		catch(InterruptedException e)
		{
			System.out.println("Ended " + this + " thread.");
		}
	}
	
	public synchronized void end()
	{
		flyDone = true;
		//control.setSetpoint(0);
		flywheelcontrol.setSetpoint(0);
		flywheelcontrol.disable();
	}
	
	@Override
	public boolean threadsActive()
	{
		return task.isActive();
	}
}
