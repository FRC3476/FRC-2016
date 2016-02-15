package org.usfirst.frc.team3476.Subsystems;

import java.util.ResourceBundle.Control;

import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.Utility.OrangeUtility;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper.Data;
import org.usfirst.frc.team3476.Utility.Control.TakeBackHalf;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

//TODO: javadoc comments if not specified by Subsystem
/*
 * Represents the robot's shooter.
 */
public class Shooter implements Subsystem
{
	private final String[] autoCommands = {"shooter", "aim", "flywheel", "fire", "loader"};
	private final String[] constants = {"SHOOTEROUTPUTRANGEHIGH", "SHOOTEROUTPUTRANGELOW", "SHOOTERIGAIN", "FLY1DIR", "FLY2DIR", "FLYWHEELDEAD", "FLYWHEELMAXSPEED", "TURRETVISIONP", "TURRETVISIONI", "TURRETVISIOND", "TURRETVISIONDEAD", "TURRETENCODERP", "TURRETENCODERI", "TURRETENCODERD", "TURRETENCODERDEAD"};
	private final double RPMTORPS = 60, TURRETOUTPUTRANGE = 0.25;
	
	enum AimMode{VISION, ENCODER}
	AimMode aimmode;
	
	private double SHOOTEROUTPUTRANGEHIGH, SHOOTEROUTPUTRANGELOW, SHOOTERIGAIN, FLYWHEELDEAD, FLYWHEELMAXSPEED, TURRETVISIONP, TURRETVISIONI, TURRETVISIOND, TURRETVISIONDEAD, TURRETENCODERP, TURRETENCODERI, TURRETENCODERD, TURRETENCODERDEAD;
	private double aimangle;
	private double[] FLYDIRS;
	private boolean flyDone, loadDone, firing, firingLast, pass1, turretdone;
	private SpeedController fly1, fly2, turret;
	private TakeBackHalf control;
	private Counter tach;
	private Timer shootingTimer;
	
	private PIDDashdataWrapper vision;
	private Encoder encoder;
	private PIDController turretvisioncontrol;
	private PIDController turretencodercontrol;
	
	private SubsystemTask task;
	private Thread flyThread;
	
	private int iters;
	private boolean lastturretdone;
	
	public Shooter(SpeedController fly1in, SpeedController fly2in, SpeedController turretin, Counter tachin, Encoder encoderin)
	{
		//Turret setup
		turret = turretin;
		turretdone = true;
		aimmode = AimMode.ENCODER;
		
		//Vision tracking control
		vision = new PIDDashdataWrapper(Data.VISIONX);
		turretvisioncontrol = new PIDController(0, 0, 0, vision, turret);
		turretvisioncontrol.disable();
		turretvisioncontrol.setOutputRange(-TURRETOUTPUTRANGE, TURRETOUTPUTRANGE);
		
		//Encoder control
		aimangle = 0;
		encoder = encoderin;
		turretencodercontrol = new PIDController(0, 0, 0, encoder, turret);
		turretencodercontrol.disable();
		turretencodercontrol.setOutputRange(-TURRETOUTPUTRANGE, TURRETOUTPUTRANGE);
		
		//Shooter
		fly1 = fly1in;
		fly2 = fly2in;
		flyDone = true;
		loadDone = true;
		tach = tachin;
		firing = false;
		shootingTimer = new Timer();
		FLYDIRS = new double[2];
		
		iters = 0;
		task = new SubsystemTask(this, 10);
		flyThread = new Thread(task, "flyThread");
		flyThread.start();
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
				flyDone = false;
				
				control.setSetpoint(params[1]);
				break;
			//TODO: use the actual aim method correctly
			case "aim":
				break;
			case "flywheel":
				flyDone = false;
				
				control.setSetpoint(params[0]);
				break;
			//TODO: do firing right
			case "fire":
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
		int i = 0;
		SHOOTEROUTPUTRANGEHIGH = constantsin[i];
		i++;//1
		SHOOTEROUTPUTRANGELOW = constantsin[i];
		i++;//2
		SHOOTERIGAIN = constantsin[i];
		i++;//3
		FLYDIRS[i - 3] = constantsin[i];
		i++;//4
		FLYDIRS[i - 3] = constantsin[i];
		i++;//5
		FLYWHEELDEAD = constantsin[i];
		i++;//6
		FLYWHEELMAXSPEED = constantsin[i];
		i++;//7
		TURRETVISIONP = constantsin[i];
		i++;//8
		TURRETVISIONI = constantsin[i];
		i++;//9
		TURRETVISIOND = constantsin[i];
		i++;//10
		TURRETVISIONDEAD = constantsin[i];
		i++;//11
		TURRETENCODERP = constantsin[i];
		i++;//12
		TURRETENCODERI = constantsin[i];
		i++;//13
		TURRETENCODERD = constantsin[i];
		i++;//14
		TURRETENCODERDEAD = constantsin[i];
		
		//TODO: make this not lose state every five seconds
		control = new TakeBackHalf(new double[]{SHOOTEROUTPUTRANGEHIGH, SHOOTEROUTPUTRANGELOW}, SHOOTERIGAIN, FLYWHEELMAXSPEED);
		
		turretvisioncontrol.setPID(TURRETVISIONP, TURRETVISIONI, TURRETVISIOND);
		turretencodercontrol.setPID(TURRETENCODERP, TURRETENCODERI, TURRETENCODERD);
		//System.out.println("Setting PID");
		turretvisioncontrol.setAbsoluteTolerance(TURRETVISIONDEAD);
		turretencodercontrol.setAbsoluteTolerance(TURRETENCODERDEAD);
	}

	//TODO: add firing logic
	@Override
	public synchronized void update()//Flywheel and turret control loop
	{
		//Take back half control
		double output = 0;
		double process = tach.getRate()*RPMTORPS;//Get rps > to rpm
		if(control == null)
		{
			throw new NullPointerException("No TakeBackHalf controller in Subsystem \"" + this +  "\" - constants not returned");
		}
		else
		{
			output = control.output(process);
		}
		output = control.getSetpoint() > 0 ? 1 : 0;
		fly1.set(output*FLYDIRS[0]);
		fly2.set(output*FLYDIRS[1]);
		
		//Turret update
		if(!turretdone)
		{
			switch(aimmode)
			{
				case VISION:
					//If first exec, make sure we're using the right control
					if(lastturretdone)
					{
						turretencodercontrol.disable();
						turretvisioncontrol.enable();
					}
					
					//If not enabled, do it
					if(targetAvailable())
					{
						if(iters % 5 == 0) System.out.println(OrangeUtility.PIDData(turretvisioncontrol));
						
						//If not enabled, do it
						if(!turretvisioncontrol.isEnabled())
						{
							turretvisioncontrol.enable();
						}
						
						turretvisioncontrol.setSetpoint(0);
						
						if(turretvisioncontrol.onTarget())
						{
							turretdone = true;
							turretvisioncontrol.disable();
						}
					}
					else
					{
						turretvisioncontrol.disable();
					}
					break;
					
				case ENCODER:
					//If first exec, make sure we're using the right control
					if(lastturretdone)
					{
						turretencodercontrol.enable();
						turretvisioncontrol.disable();
					}
					
					//If not enabled, do it
					if(!turretencodercontrol.isEnabled())
					{
						turretencodercontrol.enable();
					}
					
					turretencodercontrol.setSetpoint(aimangle);
					
					if(turretencodercontrol.onTarget())
					{
						turretdone = true;
						turretencodercontrol.disable();
					}
					break;
			}
		}
		else
		{
			turretencodercontrol.disable();
			turretvisioncontrol.disable();
		}
		
		//Check if we're done here 
		//TODO: Decide if the flywheel needs to be in the deadzone for multiple iterations
		
		if(Math.abs(control.getSetpoint() - process) < FLYWHEELDEAD)
		{
			flyDone = true;
		}
		if(!firing)
		{
			loadDone = true;
		}
		iters++;
		
		lastturretdone = turretdone;
	}
	
	public synchronized void startFire()
	{
		firing = true;
	}
	
	public synchronized boolean isFiring()
	{
		return firing;
	}
	
	/**
	 * @return True if a vision target is available
	 */
	public synchronized boolean targetAvailable()
	{
		return vision.pidGet() != Double.NaN;
	}
	
	/**
	 * Aims the turret at the largest vision target.
	 */
	public synchronized void aim()
	{
		System.out.println("Aim called");
		turretdone = false;
		aimmode = AimMode.VISION;
		System.out.println("turretdone = " + turretdone);
	}
	
	/**
	 * Aims the turret to the specified angle.
	 * @param angle the angle to turn to
	 */
	public synchronized void aim(double angle)
	{
		turretdone = false;
		aimangle = angle;
		aimmode = AimMode.ENCODER;
	}
	
	/**
	 * Stops the aiming for the turret.
	 * In the case of encoder aiming, stops prematurely.
	 * In the case of vision aiming, stops normally.
	 */
	public synchronized void stopAim()
	{
		System.out.println("Stopaim");
		turretdone = true;
	}
	
	//TODO: do we need this?
	/*public synchronized void loader(Load dir)
	{
		switch(dir)
		{
			case IN:
				loader.set(false);
				break;
			case OUT:
				loader.set(true);
				break;
		}
	}
	
	public synchronized Load getLoader()
	{
		return loader.get() ? Load.OUT : Load.IN;
	}*/
	
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
			flyThread.join();
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
		turretdone = true;
		control.setSetpoint(0);
	}
	
	@Override
	public boolean threadsActive()
	{
		return task.isActive();
	}
}
