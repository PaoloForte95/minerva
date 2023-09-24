package se.oru.assignment.assignment_oru.fleetmasterinterface;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import aima.core.util.datastructure.Pair;
import se.oru.assignment.assignment_oru.problems.AbstractDelayEvaluator;

public final class EclDelayEvaluator extends AbstractDelayEvaluator {
    protected AbstractFleetMasterInterface eclInterface;

    public EclDelayEvaluator(AbstractFleetMasterInterface eclInterface){
        super();
        this.eclInterface = eclInterface;
    }


    @Override
    public Pair<Double, Double> evaluatePathDelay(int robot1ID, PoseSteering[] pss1, int robot2ID, PoseSteering[] pss2) {
        if (this.eclInterface != null){
            return eclInterface.computeTimeDelayWPath(pss1, pss2, robot1ID, robot2ID);
        }
        return new Pair<Double, Double> (Double.NaN, Double.NaN);
    }
    
}
