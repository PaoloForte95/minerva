package se.oru.assignment.assignment_oru.methods;

import java.util.ArrayList;
import java.util.Random;

import org.metacsp.utility.logging.MetaCSPLogging;

import se.oru.assignment.assignment_oru.ConstraintOptimizationProblem;
import se.oru.assignment.assignment_oru.LinearOptimizationProblem;

import com.google.ortools.linearsolver.*;



public class SimulatedAnnealingAlgorithm extends AbstractOptimizationAlgorithm {
	
		
	public SimulatedAnnealingAlgorithm(){
		super();
		logger = MetaCSPLogging.getLogger(this.getClass());
	}
	
	public static double probability(double f1, double f2, int iteration) {
        if (f2 < f1) return 1;
        return 1-Math.exp((f1 - f2) / iteration);
    }
	

	
	public int [][][] solveOptimizationProblem(LinearOptimizationProblem oap){
		return solveOptimizationProblem(oap,-1);
	
	}



	public int [][][] solveOptimizationProblem(LinearOptimizationProblem oap,int iterations){
		logger.info("Solving the problem using the Simulated Annealing Algorithm");
		Random rand = new Random(3455343);
		MPSolver optimizationModel = oap.getModel();
		ArrayList<Integer> IDsAllRobots = oap.getRobotsIDs();
		ArrayList<Integer> IDsAllTasks = oap.getTasksIDs();
		
		int numRobotAug = IDsAllRobots.size();
		int numTaskAug = IDsAllTasks.size();
		int maxNumPaths = oap.getmaxNumberOfAlternativePaths();
		double [][][] costValuesMatrix = oap.getInterferenceFreeCostMatrix();
		
		double linearWeight = oap.getLinearWeight();
		
	
	
		//Initialize the optimal assignment and the cost associated to it
		int [][][] optimalAssignmentMatrix = new int[numRobotAug][numTaskAug][maxNumPaths];
		double objectiveOptimalValue = 100000000;
		//Solve the optimization problem
		optimizationModel.solve();
		int [][][] AssignmentMatrix = oap.getAssignmentMatrix();
		int randomRobotID1 = 0;
		int prova2 = 0;
		
		int numIteration =1;
		for(int fat= 1 ; fat <= numRobotAug; fat++) {
			numIteration *= fat;
		}
		if(iterations > numIteration || iterations == -1) {
			iterations =  numIteration;
		}else {
			numIteration = iterations;
		}
		int index1i = 0;
		int index1j = 0;
		int index1s = 0;
		int index2i = 0;
		int index2j = 0;
		int index2s = 0;
		int [][][] newAssignmentMatrix = new int[numRobotAug][numTaskAug][maxNumPaths];
		for(int i=0; i< AssignmentMatrix.length;i ++) {
			for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
				for(int s = 0; s < maxNumPaths; s++) {
					newAssignmentMatrix[i][j][s] = AssignmentMatrix[i][j][s];
				}
			}
		}
		boolean solutionAlreadyFound = false;
		double[][][][] matrixSolutionFound= new double[iterations+1][numRobotAug][numTaskAug][maxNumPaths];
		boolean [] solutionFound = new boolean [numRobotAug*numTaskAug*maxNumPaths];
		double [] costSolutions = new double [iterations+1];
		int indexSolutionFound = 0;
		ArrayList <Integer> IDsRandomRobots = new ArrayList <Integer>(IDsAllRobots);
		ArrayList <Integer> IDsRandomRobots2 = new ArrayList <Integer>();

		int randomIndex  = (int) Math.floor(rand.nextDouble()*IDsRandomRobots.size());
		randomRobotID1 = IDsRandomRobots.get(randomIndex);
	
		for(int robotID: IDsAllRobots) {
			if(oap.getRealRobotsIDs().contains(robotID)){			
				if(oap.getRobotType(robotID) == oap.getRobotType(randomRobotID1)) {
					IDsRandomRobots2.add(robotID);
					
				}
			}else {//virtual robots
				IDsRandomRobots2.add(robotID);
			}
		}
		IDsRandomRobots2.remove(IDsRandomRobots2.indexOf(randomRobotID1));
		long timeOffset = 0;
		
		for(int iteration=0; iteration <= iterations ;iteration++){
			if(timeOffset < timeOut) {
	
			double costofAssignment = 0;
			double costofAssignmentForConstraint = 0;
			double costF = 0;
			if(iteration >0) {	
					if(IDsRandomRobots2.size() == 0) {
						//prova1 += 1;
						if(IDsRandomRobots.size()==0) {
							IDsRandomRobots.addAll(IDsAllRobots);
						}
						randomIndex  = (int) Math.floor(rand.nextDouble()*IDsRandomRobots.size());
						randomRobotID1 = IDsRandomRobots.get(randomIndex);
						IDsRandomRobots.remove(randomIndex);
						for(int robotID: IDsAllRobots) {
							if(oap.getRealRobotsIDs().contains(robotID)){			
								if(oap.getRobotType(robotID) == oap.getRobotType(randomRobotID1)) {
									IDsRandomRobots2.add(robotID);
									
								}
							}else {//virtual robots
								IDsRandomRobots2.add(robotID);
							}
						}
						IDsRandomRobots2.remove(IDsRandomRobots2.indexOf(randomRobotID1));
					}
					int randomIndex2 = (int) Math.floor(rand.nextDouble()*IDsRandomRobots2.size());
					if(IDsRandomRobots2.size()>0) {
						prova2 = IDsRandomRobots2.get(randomIndex2);
						IDsRandomRobots2.remove(randomIndex2);
					}
				for(int i=0; i< AssignmentMatrix.length;i ++) {
					for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
						for(int s = 0; s < maxNumPaths; s++) {
							newAssignmentMatrix[i][j][s] = AssignmentMatrix[i][j][s];
						}
					}
				}
				
				for (int robotID : IDsAllRobots ) {
					int i = IDsAllRobots.indexOf(robotID);
					for (int taskID : IDsAllTasks ) {
						int j = IDsAllTasks.indexOf(taskID);
						for(int s = 0; s < maxNumPaths; s++) {
							if (AssignmentMatrix[i][j][s]==1  && robotID == randomRobotID1) {
								index1i = i;	
								index1j = j;
								index1s = s;
							}
							if(AssignmentMatrix[i][j][s]==1 && robotID == prova2) {
								index2i = i;	
								index2j = j;
								index2s = s;		
							}
						}
					}
				}
				
				newAssignmentMatrix[index1i][index1j][index1s] = 0;
				newAssignmentMatrix[index1i][index2j][index1s] = 1;
				newAssignmentMatrix[index2i][index1j][index2s] = 1;
				newAssignmentMatrix[index2i][index2j][index2s] = 0;
				
				for(int it=0; it < iteration;it ++) {
					for(int i=0; i< AssignmentMatrix.length;i ++) {
						for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
							for(int s = 0; s < maxNumPaths; s++) {
								if(matrixSolutionFound[it][i][j][s] ==  newAssignmentMatrix[i][j][s]) {
									solutionFound[i*numTaskAug*maxNumPaths+j*maxNumPaths+s] = true;
								}else {
									solutionFound[i*numTaskAug*maxNumPaths+j*maxNumPaths+s] = false;
								}
							}
						}
					}
					solutionAlreadyFound = true;
					for(int f=0; f < solutionFound.length;f++) {
						if(!solutionFound[f]) {
							solutionAlreadyFound = false;
							break;
						}
					}
					if(solutionAlreadyFound) {
						indexSolutionFound = it;
						break;
					}
				}
			}
			solutionAlreadyFound = false;
			if(!solutionAlreadyFound) {
					
				//Take time to understand how much time require this function
				for (int robotID : IDsAllRobots ) {
					int i = IDsAllRobots.indexOf(robotID);
					for (int taskID : IDsAllTasks ) {
						int j = IDsAllTasks.indexOf(taskID);
						for(int s = 0; s < maxNumPaths; s++) {
							if ( newAssignmentMatrix[i][j][s] > 0) {
								if (linearWeight != 1) {
									//Evaluate cost of F function only if alpha is not equal to 1
									double costB = optimizationModel.objective().getCoefficient(optimizationModel.variables()[i*numTaskAug*maxNumPaths+j*maxNumPaths+s]);
									costF = oap.evaluateInterferenceCost(robotID,taskID,s,AssignmentMatrix);
									costofAssignment = linearWeight*costB + (1-linearWeight)*costF + costofAssignment ;
									costofAssignmentForConstraint = costValuesMatrix[i][j][s] + costF + costofAssignmentForConstraint;							
								}
								else {
									double costB = optimizationModel.objective().getCoefficient(optimizationModel.variables()[i*numTaskAug*maxNumPaths+j*maxNumPaths+s]);
									costofAssignment = Math.pow(linearWeight*costB, 2) + costofAssignment ;
									costofAssignmentForConstraint = costValuesMatrix[i][j][s]  + costofAssignmentForConstraint;
								}
	
							}
							matrixSolutionFound[iteration][i][j][s] =  newAssignmentMatrix[i][j][s];
						}
										
					}		
				}
				costSolutions[iteration] = costofAssignment;
			}else {
				costofAssignment =  costSolutions[indexSolutionFound];
				costSolutions[iteration] = costofAssignment;
			}
			
			//Compare actual solution and optimal solution finds so far
			if(iteration> 0) {
				
				double randPb= rand.nextDouble();
				double pb = probability(costSolutions[iteration-1],costofAssignment,iteration);
	
				if(randPb < pb) {
					
					for(int i=0; i< AssignmentMatrix.length;i ++) {
						for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
							for(int s = 0; s < maxNumPaths; s++) {
								AssignmentMatrix[i][j][s] = newAssignmentMatrix[i][j][s];
							}
						}
					}
				}
			}
			
			if (costofAssignment < objectiveOptimalValue ) {
				objectiveOptimalValue = costofAssignment;
				//optimalAssignmentMatrix = AssignmentMatrix;
				for(int i=0; i< AssignmentMatrix.length;i ++) {
					for(int j = 0 ; j <AssignmentMatrix[0].length; j++) {
						for(int s = 0; s < maxNumPaths; s++) {
							optimalAssignmentMatrix[i][j][s] = newAssignmentMatrix[i][j][s];
						}
					}
				}
			}	
		}	
		}
		//Return the Optimal Assignment Matrix 
		return  optimalAssignmentMatrix;    
	}



	@Override
	public int[][][] solveOptimizationProblem(ConstraintOptimizationProblem oap) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'solveOptimizationProblem'");
	}
	
	}

