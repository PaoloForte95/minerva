package se.oru.assignment.assignment_oru.delay;


import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import aima.core.util.datastructure.Pair;



public interface AbstractDelayEvaluator {


    public Pair<Double, Double> evaluatePathDelay(int robot1ID,PoseSteering[] pss1, int robot2ID,PoseSteering[] pss2);

    public Pair<Double, Double> evaluatePathDelay(int robot1ID,PoseSteering[] pss1,double[] curvs1, int robot2ID, PoseSteering[] pss2, double[] curvs2);
    
}
