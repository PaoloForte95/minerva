package se.oru.assignment.assignment_oru;

import se.oru.coordination.coordination_oru.Mission;

public interface TaskAssignmentCallback {
	
	public void onTaskAssignment(Mission[] missions);

}
