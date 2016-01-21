package org.usfirst.frc.team3476.ScriptableAuto;

import java.util.ArrayList;

/**
 * Contains the commands that should happen in sequence.
 * @author Anthony Demetrescu
 *
 */
public class CommandBlock
{
	ArrayList<Command> commands;
	
	/**
	 * Constructs an empty CommandBlock.
	 */
	public CommandBlock()
	{
		commands = new ArrayList<Command>();
	}
	
	/**
	 * Constructs a CommandBlock with the given commands.
	 * @param commandsin the commands to be contained
	 */
	public CommandBlock(ArrayList<Command> commandsin)
	{
		commands = commandsin;
	}
	
	/**
	 * Gets the next command.
	 * @return the next command.
	 */
	public Command getCommand()
	{
		return commands.get(0);
	}
	
	/**
	 * Finishes the current command.
	 */
	public void finishCommand()
	{
		commands.remove(0);//Remove last command given
	}
	
	/**
	 * Checks to see if this command block is finished.
	 * @return True if there is another command
	 */
	public boolean hasNext()
	{
		return !commands.isEmpty();
	}
	
	public String toString()
	{
		return commands.toString();
	}
}
