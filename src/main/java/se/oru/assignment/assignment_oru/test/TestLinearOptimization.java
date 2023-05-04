package se.oru.assignment.assignment_oru.test;

import java.util.Calendar;

import se.oru.assignment.assignment_oru.Robot;
import se.oru.assignment.assignment_oru.Task;
import se.oru.assignment.assignment_oru.methods.SystematicAlgorithm;
import se.oru.assignment.assignment_oru.problems.LinearOptimizationProblem;
import se.oru.assignment.assignment_oru.util.robotType.ROBOT_TYPE;
import se.oru.coordination.coordination_oru.demo.DemoDescription;

@DemoDescription(desc = "Test example for the linear optimization software. Evaluate the computational time to evaluate all the possible solutions for an optimization problem. The problem is solved using an MP solver")
public class TestLinearOptimization {


	public static double [][][] createCostMatrix(int numRobots, int numTasks, int numPaths){
		double [][][] costs = new double [numRobots][numTasks][numPaths];

		for(int i=0; i < numRobots; i++){
			for(int j=0; j < numTasks; j++){
				for(int s=0; s < numPaths; s++){
					costs[i][j][s] = 20;
					if(i==j ){
						costs[i][j][s] = 10;
					}
				}
			}
		}
		return costs;
	}

	public static void main(String[] args) throws InterruptedException {
		
		long startTime = Calendar.getInstance().getTimeInMillis();
		LinearOptimizationProblem problem = new LinearOptimizationProblem();
		int numRobots = 6;
		int numTasks = 6;
		int numPaths = 1;

		for(int i=0; i < numRobots; i++){
			Robot rb = new Robot(i+1);
			problem.addRobot(rb);
		}

		for(int j=0; j < numTasks; j++){
			Task tk = new Task(j+1,ROBOT_TYPE.CARLIKE);
			problem.addTask(tk);

		}
		problem.setmaxNumberOfAlternativePaths(numPaths);
		double [][][] costs = createCostMatrix(numRobots, numTasks, numPaths);

		problem.setInterferenceFreeCostMatrix(costs);

		SystematicAlgorithm alg = new SystematicAlgorithm();
		problem.findOptimalAssignment(alg);
		
		problem.printOptimalAssignment();

		problem.getFeasibleSolutions();

		//problem.printFeasibleAssignments();
		
		long finalTime = Calendar.getInstance().getTimeInMillis();
		long totalTime = finalTime - startTime;
		System.out.println("Time to evaluate all the possible solutions with the MP Solver: " + totalTime/1000 + " seconds");

	}
}