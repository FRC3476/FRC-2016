package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Main.Subsystem;
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
	private final String[] constants = {"SHOOTEROUTPUTRANGEHIGH", "SHOOTEROUTPUTRANGELOW", "SHOOTERIGAIN", "FLY1DIR", "FLY2DIR", "FLYWHEELDEAD", "FLYWHEELMAXSPEED", "TURRETP", "TURRETI", "TURRETD", "TURRETDEAD"};
	private final double RPMTORPS = 60;
	
	enum AimMode{VISION, ENCODER}
	AimMode aimmode;
	
	private double SHOOTEROUTPUTRANGEHIGH, SHOOTEROUTPUTRANGELOW, SHOOTERIGAIN, FLYWHEELDEAD, FLYWHEELMAXSPEED, TURRETP, TURRETI, TURRETD, TURRETDEAD;
	private double aimangle;
	private double[] FLYDIRS;
	private boolean flyDone, loadDone, firing, firingLast, pass1, turretdone;
	private SpeedController fly1, fly2, turret;
	private TakeBackHalf control;
	private Counter tach;
	private Timer shootingTimer;
	
	private PIDDashdataWrapper vision;
	private Encoder encoder;
	private PIDSource turretsource;
	private PIDController turretcontrol;
	
	private SubsystemTask task;
	private Thread flyThread;
	
	public Shooter(SpeedController fly1in, SpeedController fly2in, SpeedController turretin, Counter tachin, Encoder encoderin)
	{
		//Turret setup
		turret = turretin;
		vision = new PIDDashdataWrapper(Data.VISIONX);
		encoder = encoderin;
		aimangle = 0;
		turretsource = encoder;
		turretdone = true;
		aimmode = AimMode.ENCODER;
		
		fly1 = fly1in;
		fly2 = fly2in;
		flyDone = true;
		loadDone = true;
		tach = tachin;
		firing = false;
		shootingTimer = new Timer();
		FLYDIRS = new double[2];
		
		task = new SubsystemTask(this);
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
		TURRETP = constantsin[i];
		i++;//8
		TURRETI = constantsin[i];
		i++;//9
		TURRETD = constantsin[i];
		i++;//10
		TURRETDEAD = constantsin[i];
		
		control = new TakeBackHalf(new double[]{SHOOTEROUTPUTRANGEHIGH, SHOOTEROUTPUTRANGELOW}, SHOOTERIGAIN, FLYWHEELMAXSPEED);
		control.setSetpoint(0);
		
		turretcontrol = new PIDController(TURRETP, TURRETI, TURRETD, vision, turret);
		turretcontrol.disable();
		turretcontrol.setAbsoluteTolerance(TURRETDEAD);
		
		startThreads();
	}

	//TODO: add firing logic
	@Override
	public synchronized void update()//Flywheel and turret control loop
	{
		//Take back half control
		double output = 0;
		/*double process = tach.getRate()*RPMTORPS;//Get rps > to rpm
		if(control == null)
		{
			throw new NullPointerException("No TakeBackHalf controller in Subsystem \"" + this +  "\" - constants not returned");
		}
		else
		{
			output = control.output(process);
		}*/
		output = control.getSetpoint() > 0 ? 1 : 0;
		fly1.set(output*FLYDIRS[0]);
		fly2.set(output*FLYDIRS[1]);
		
		//Turret update
		if(!turretdone)
		{
			switch(aimmode)
			{
				case VISION:
					if(turretsource != vision)
					{
						turretsource = vision;
					}
					if(targetAvailable())
					{
						if(!turretcontrol.isEnabled())
						{
							turretcontrol.enable();
						}
						turretcontrol.setSetpoint(0);
					}
					else
					{
						turretcontrol.disable();
					}
					break;
					
				case ENCODER:
					if(turretsource != encoder)
					{
						turretsource = encoder;
					}
					if(!turretcontrol.isEnabled())
					{
						turretcontrol.enable();
					}
					turretcontrol.setSetpoint(aimangle);
					if(turretcontrol.onTarget())
					{
						turretdone = true;
					}
					break;
			}
		}
		else
		{
			turretcontrol.disable();
		}
		
		//Check if we're done here 
		//TODO: Decide if the flywheel needs to be in the deadzone for multiple iterations
		if(true /*Math.abs(control.getSetpoint() - process) < FLYWHEELDEAD*/) flyDone = true;
		if(!firing) loadDone = true;
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
		turretdone = false;
	}
	
	/**
	 * Aims the turret to the specified angle.
	 * @param angle the angle to turn to
	 */
	public synchronized void aim(double angle)
	{
		turretdone = false;
		aimangle = angle;
	}
	
	/**
	 * Stops the aiming for the turret.
	 * In the case of encoder aiming, stops prematurely.
	 * In the case of vision aiming, stops normally.
	 */
	public synchronized void stopAim()
	{
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
