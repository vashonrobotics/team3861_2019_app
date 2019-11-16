package org.firstinspires.ftc.teamcode;

public class ExponentialMovingAverage {
    private double alpha;
    private Double current;

    public ExponentialMovingAverage(double alpha) {
        this.alpha = alpha;
    }

    public double update(double value) {
        if(current != null) {
            current = alpha * value + (1 - alpha) * current;
        } else {
            current = value;
        }

        return current;
    }

    public Double getCurrent() {
        return current;
    }
}
