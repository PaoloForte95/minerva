package se.oru.assignment.assignment_oru.gui;


import java.awt.BorderLayout;
import java.awt.GridLayout;
import java.util.ArrayList;

import javax.swing.BorderFactory;
import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.swing.table.DefaultTableModel;

import se.oru.assignment.assignment_oru.Task;

public final class TaskGUI extends GUI<Task> {
    
	public TaskGUI (Task ... tasks)  {
		
		JFrame frame = new JFrame();
		JPanel panel = new JPanel();
		ArrayList<String[]> localData = new ArrayList<String[]>();
		
		String[] headers = new String[]{"ID", "Start", "Goal", "Start", "Goal", "Deadline", "Priority", "Assigned Robot"};
		
		
		for (Task task : tasks) {
	
			String [] arrayData = new String [headers.length];
			arrayData[0] = String.valueOf(task.getID());
			arrayData[1] = task.getStartPose().getPosition().toString().replace(", NaN)", ")");
			arrayData[2] = task.getGoalPose().getPosition().toString().replace(", NaN)", ")");
			arrayData[3] = task.getStartLocation();
			arrayData[4] = task.getGoalLocation();
			arrayData[5] = String.valueOf(task.getDeadline());
			arrayData[6] = String.valueOf(task.isPriority());
			arrayData[7] = String.valueOf(task.isTaskAssignedID());
			localData.add(arrayData);
			
			
		}	
		String[][] data = new String[localData.size()][headers.length];
		data = localData.toArray(data);
		for (String[] x : data)
		System.out.print(x + " ");
			//labelsMaterials.put(material.getID(), labelmat);
			//panel.add(labelmat);
		
		DefaultTableModel model = new DefaultTableModel(data,headers);
		table = new JTable(model);
		panel.setBorder(BorderFactory.createTitledBorder(
		         BorderFactory.createEtchedBorder(), "Task manager", TitledBorder.CENTER, TitledBorder.TOP));
		panel.setLayout(new GridLayout(0, 1 ));
	
		panel.add(new JScrollPane(table));
		frame.add(panel, BorderLayout.CENTER);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setTitle("Task GUI Manager");
		frame.pack();
		frame.setVisible(true);
	}


    public void updateGUI(Task ... tasks) {
		int i = 0;
		for (Task task : tasks) {

            table.getModel().setValueAt(String.valueOf(task.getID()), i,0);
            table.getModel().setValueAt(task.getStartPose().getPosition().toString().replace(", NaN)", ")"), i,1);
            table.getModel().setValueAt(task.getGoalPose().getPosition().toString().replace(", NaN)", ")"), i,2);
            table.getModel().setValueAt(task.getStartLocation(), i,3);
            table.getModel().setValueAt(task.getGoalLocation(), i,4);
            table.getModel().setValueAt(String.valueOf(task.getDeadline()), i,5);
            table.getModel().setValueAt(String.valueOf(task.isPriority()), i,6);
            table.getModel().setValueAt(String.valueOf(task.isTaskAssignedID()), i,7);

            table.repaint();
            i ++;
			
		}
	}
}
