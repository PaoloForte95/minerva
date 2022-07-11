package se.oru.assignment.assignment_oru;

import se.oru.coordination.coordination_oru.Mission;

public interface TaskAssignmentCallback {
	
	public void onTaskAssignment(int robotID, int taskID, int pathID, Mission[] missions);

}
