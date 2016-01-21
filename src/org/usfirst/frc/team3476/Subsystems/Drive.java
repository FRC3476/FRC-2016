	package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.Utility.OrangeUtility;
import org.usfirst.frc.team3476.Utility.RunningAverage;
import org.usfirst.frc.team3476.Utility.Control.BangBang;
import org.usfirst.frc.team3476.Utility.Control.DifferentialGyro;
import org.usfirst.frc.team3476.Utility.Control.MedianEncoder;
import org.usfirst.frc.team3476.Utility.Control.MedianEncoderPair;
import org.usfirst.frc.team3476.Utility.Control.PIDOutputWrapper;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class Drive implements Subsystem
{
	private final String[] autoCommands = {"turn", "drive", "driven", "shiftit", "clear"};
	private final String[] constants = {"DRIVEDEAD", "DRIVESTRAIGHTDEAD", "TURNDEAD", "USELEFT", "USERIGHT", "STRAIGHTP", "STRAIGHTI", "STRAIGHTD", "DRIVEP", "DRIVEI", "DRIVED", "TURNP", "TURNI", "TURND", "SHIFTINGSPEED", "SHIFTINGHYS"};
	final int ENCODERSAMPLES = 16;
	final double TURNTIMEOUT = 6/360.0, TURNDONUTTHRESHOLD = 0.33, TURNCLAMP = 0.6, DONETIME = 0.5;
	final double SLOWSPEED = 0.4, SPECIALDIST = 12.0;
	
	private boolean done, driveStraight, simple, autoShifting, USELEFT, USERIGHT, clear;
	private double turnTimeout, DRIVEDEAD, DRIVESTRAIGHTDEAD, TURNDEAD, DRIVEP, DRIVEI, DRIVED, TURNP, TURNI, TURND, STRAIGHTP, STRAIGHTI, STRAIGHTD, SHIFTINGSPEED, SHIFTINGHYS;
	
	private MedianEncoder left, right;
	private MedianEncoderPair both;
	private DifferentialGyro gyro;
	private RunningAverage encoderAvg, avgRate;
	private RobotDrive driveTrain;
	private Solenoid shifters;
	
	private PIDController drive, turn, straightTurn;
	private PIDOutputWrapper driveWrapper, turnWrapper, straightWrapper;
	private BangBang driven;
	
	private SubsystemTask task;
	private Thread driveThread;
	
	public enum ShiftingState {LOW, HIGH}
	ShiftingState shiftingState;
	
	Timer doneTimer;
	Timer mainTimer;
	
	public Drive(MedianEncoder leftin, MedianEncoder rightin, DifferentialGyro gyroin, RobotDrive driveTrainin, Solenoid shiftersin)
	{
		done = true;
		driveStraight = true;
		simple = false;
		clear = true;
		autoShifting = true;
		shiftingState = ShiftingState.LOW;
		doneTimer = new Timer();
		doneTimer.stop();
		doneTimer.reset();
		
		mainTimer = new Timer();
		mainTimer.stop();
		mainTimer.reset();
		
		left = leftin;
		right = rightin;
		both = new MedianEncoderPair(left, right);
		gyro = gyroin;
		driveTrain = driveTrainin;
		shifters = shiftersin;
		
		encoderAvg = new RunningAverage(ENCODERSAMPLES);
		avgRate = new RunningAverage(ENCODERSAMPLES);
		driveWrapper = new PIDOutputWrapper(false);
		turnWrapper = new PIDOutputWrapper(true);
		straightWrapper = new PIDOutputWrapper(true);
		
		//Driving PID
		drive = new PIDController(DRIVEP, DRIVEI, DRIVED, both, driveWrapper);
		drive.setOutputRange(-1, 1);
		drive.enable();
		
		//Turn PID
		turn = new PIDController(TURNP, TURNI, TURND, gyro, turnWrapper);
		turn.setOutputRange(TURNDONUTTHRESHOLD - TURNCLAMP, TURNCLAMP - TURNDONUTTHRESHOLD);
		turn.enable();
		
		//Drivestraight PID
		straightTurn = new PIDController(STRAIGHTP, STRAIGHTI, STRAIGHTD, gyro, straightWrapper);
		straightTurn.setOutputRange(-0.3, 0.3);
		straightTurn.enable();
		
		driven = new BangBang(new double[]{1, -1});
		
		task = new SubsystemTask(this);
		driveThread = new Thread(task, "driveThread");
		driveThread.start();
	}
	
	@Override
	public String[] getAutoCommands()
	{
		return autoCommands;
	}

	@Override
	public synchronized void doAuto(double[] params, String command)
	{
		autoShifting = false;
		done = false;
		clear = false;
		mainTimer.reset();
		mainTimer.start();
		doneTimer.reset();
		doneTimer.start();
		switch(command)
		{
			case "turn":
				executeTurn(params[0]);
				break;
			case "drive":
				executeDrive(params[0]);
				break;
			case "driven":
				executeSimpleDrive(params[0], params[1]);
				break;
			case "shiftit":
				setShifterState(params[0] == 1 ? ShiftingState.HIGH : ShiftingState.LOW);
				done = true;
				break;
			case "clear":
				clear = true;
				done = true;
				break;
		}
	}

	@Override
	public synchronized boolean isAutoDone()
	{
		return done;
	}

	@Override
	public String[] getConstantRequest()
	{
		return constants;
	}

	@Override
	public synchronized void returnConstantRequest(double[] constantsin)
	{
		int i = 0;
		DRIVEDEAD = constantsin[i];
		i++;//1
		DRIVESTRAIGHTDEAD = constantsin[i];
		i++;//2
		TURNDEAD = constantsin[i];
		i++;//3
		USELEFT = constantsin[i] == 1;
		i++;//4
		USERIGHT = constantsin[i] == 1;
		i++;//5
		STRAIGHTP = constantsin[i];
		i++;//6
		STRAIGHTI = constantsin[i];
		i++;//7
		STRAIGHTD = constantsin[i];
		i++;//8
		DRIVEP = constantsin[i];
		i++;//9
		DRIVEI = constantsin[i];
		i++;//10
		DRIVED = constantsin[i];
		i++;//11
		TURNP = constantsin[i];
		i++;//12
		TURNI = constantsin[i];
		i++;//13
		TURND = constantsin[i];
		i++;//14
		SHIFTINGSPEED = constantsin[i];
		i++;//15
		SHIFTINGHYS = constantsin[i];
		//System.out.println("P: " + TURNP + " I: " + TURNI + " D: " + TURND + " Isvalid: " + constantsin.length);
		startThreads();
		both.setUse(USELEFT, USERIGHT);
	}

	@Override
	public synchronized void update()
	{
		//Poll the encoders and gyro - see what up
		if(!clear)
		{
			if (!done)
			{
				if (driveStraight)
				{
					if (!simple)
					{
						driveTrain.arcadeDrive(driveWrapper.getOutput(), straightWrapper.getOutput());
					}
					else
					{
						double bangbang = specialBangBang(both.getDistance());
						//System.out.println("Simple drive with bang bang: " + bangbang + " error: " + driven.getError(both.getDistance()));
						System.out.println("Straight drive: " + straightTurn.get() + " error: " + straightTurn.getError());
						driveTrain.arcadeDrive(bangbang, straightWrapper.getOutput());
						//System.out.println("Drive setpoint: " + driven.getSetpoint() + " Current pos: " + both.getDistance());
					}
				}
				else//Turning
				{
					if(mainTimer.get() > turnTimeout) done = true;
					//System.out.print("Turning with output: " + turnWrapper.getOutput() + "  ");
					//System.out.println("Gyro: "  + gyro.calcDiff());
					driveTrain.arcadeDrive(0, OrangeUtility.donut(turnWrapper.getOutput(), TURNDONUTTHRESHOLD));
				}
				
				//Check if we're done here 
				//TODO: Decide if the drive needs to be in the deadzone for multiple iterations
				boolean driveDone = Math.abs(drive.getSetpoint() - encoderAvg.pidGet()) < DRIVEDEAD;
				boolean drivenDone = Math.abs(driven.getSetpoint() - both.getDistance()) < DRIVEDEAD;
				boolean turnDone = Math.abs(turn.getError()) < TURNDEAD;
				//System.out.println("Turn PID output: " + turn.get() + " error: " + (turn.getError()));
				if(!(driveStraight ? (simple ? drivenDone : driveDone) : turnDone))
				{
					doneTimer.reset();
				}
				if(doneTimer.get() > DONETIME)
				{
					done = true;
				}
			}
			else
			{
				driveTrain.arcadeDrive(0, 0);
			}
		}
		else
		{
			driveTrain.arcadeDrive(0, 0);
		}
		
		if(autoShifting)
		{
			switch(shiftingState)
	    	{
	    		case HIGH:
	    			//System.out.print("Case: HIGH, Rate = " + Math.abs(avgRate.getAverage()));
	    			if(Math.abs(avgRate.getAverage()) <= SHIFTINGSPEED - SHIFTINGHYS)
	    			{
	    				shiftingState = ShiftingState.LOW;
	    				setShifterState(shiftingState);
	    			}
	    			break;
	    		case LOW:
	    			//System.out.print("Case: LOW, Rate = " + Math.abs(avgRate.getAverage()));
	    			if(Math.abs(avgRate.getAverage()) >= SHIFTINGSPEED + SHIFTINGHYS)
    				{
	    				shiftingState = ShiftingState.HIGH;
	    				setShifterState(shiftingState);
    				}
	    			break;
	    	}
		}
	}
	
	public synchronized void executeTurn(double delta)
	{
		simple = false;
		driveStraight = false;
		turnTimeout = Math.abs(TURNTIMEOUT*delta);
		
		//turn init
		gyro.reset();
		turn.reset();
		turn.setSetpoint(delta);
		turn.setPID(TURNP, TURNI, TURND);
		turn.enable();
	}
	
	public synchronized void executeDrive(double delta)
	{
		simple = false;
		driveStraight = true;
		
		//drive init
		
		drive.reset();
		drive.setSetpoint(both.getDistance() + delta);
		drive.setPID(DRIVEP, DRIVEI, DRIVED);
		drive.enable();
		
		//straightTurn init
		gyro.reset();
		straightTurn.reset();
		straightTurn.setSetpoint(0);
		straightTurn.setPID(STRAIGHTP, STRAIGHTI, STRAIGHTD);
		straightTurn.enable();
	}
	
	public synchronized void executeSimpleDrive(double delta, double percentSpeed)
	{
		simple = true;
		driveStraight = true;
		
		driven.setOutputrange(new double[]{percentSpeed/100, -percentSpeed/100});
		driven.setSetpoint(both.getDistance() + delta);
		
		
		gyro.reset();
		straightTurn.reset();
		straightTurn.setSetpoint(0);
		straightTurn.setPID(STRAIGHTP, STRAIGHTI, STRAIGHTD);
		straightTurn.enable();
	}
	
	public String toString()
	{
		return "Drive";
	}
	
	public void stopThreads()
	{
		task.hold();
	}
	
	public void terminateThreads()
	{
		task.terminate();
		try
		{
			driveThread.join();
			System.out.println("Ended " + this + " thread.");
		}
		catch(InterruptedException e)
		{
			System.out.println("Ended " + this + " thread.");
		}
	}
	
	public void autoShifting(boolean auto)
	{
		autoShifting = auto;
	}
	
	public synchronized void setShifterState(ShiftingState state)
	{
		switch(state)
		{
			case HIGH:
				shifters.set(false);
				break;
			case LOW:
				shifters.set(true);
				break;
		}
	}
	
	@Override
	public void startThreads()
	{
		task.resume();
	}

	@Override
	public void end()
	{
		clear = true;
		setShifterState(ShiftingState.LOW);
	}
	
	private double specialBangBang(double process)
	{
		if(Math.abs(driven.getError(process)) < SPECIALDIST)
		{
			double speed = Math.min(Math.abs(driven.getOutputrange()[0]), SLOWSPEED);
			driven.setOutputrange(new double[]{speed, -speed});
		}
		return driven.output(process);
	}
}
