package se.oru.assignment.assignment_oru.gui;

import javax.swing.JTable;

public abstract class GUI <T> {


	protected JTable table;


	public abstract void updateGUI( T ... materials);
	
}








