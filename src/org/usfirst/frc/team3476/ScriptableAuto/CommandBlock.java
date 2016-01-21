package org.usfirst.frc.team3476.ScriptableAuto;

import java.util.ArrayList;

public class CommandBlock
{
	ArrayList<Command> commands;
	//int curCommand;
	
	public CommandBlock()
	{
		commands = new ArrayList<Command>();
		//curCommand = 0;
	}
	
	public CommandBlock(ArrayList<Command> commandsin)
	{
		commands = commandsin;
		//curCommand = 0;
	}
	
	public Command getCommand()
	{
		return commands.get(0);
	}
	
	public void finishCommand()
	{
		commands.remove(0);//Remove last command given
	}
	
	public boolean hasNext()
	{
		return !commands.isEmpty();
	}
	
	public String toString()
	{
		return commands.toString();
	}
}
