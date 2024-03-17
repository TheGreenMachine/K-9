package com.team254.lib.util;

import com.team1816.lib.util.logUtil.GreenLogger;

import java.util.ArrayList;
import java.util.DoubleSummaryStatistics;
import java.util.List;

/**
 * Finds the stats (mean, standard deviation, etc.) of a list
 * <p>
 * Example use case: finding out how long a planner takes from the average of 100 tries
 */
public class StatFinder {
    private final List<Double> numbers = new ArrayList<>();
    private final int numberToIgnore;
    private int numberIgnored = 0;

    private boolean stopped = false;

    /**
     * Creates a new StatFinder
     *
     * @param numberToIgnore the number of entries to ignore before logging (like burning the top card of a deck before dealing)
     */
    public StatFinder(int numberToIgnore) {
        this.numberToIgnore = numberToIgnore;
    }

    public boolean add(double number) {
        if (stopped) {
            return false;
        }

        if (numberIgnored < numberToIgnore) {
            numberIgnored++;
            return false;
        }

        numbers.add(number);
        return true;
    }

    public boolean addAndPrint(double number) {
        boolean success = add(number);

        if (success) {
            GreenLogger.log("added: " + number);
        }

        return success;
    }

    public DoubleSummaryStatistics getStats() {
        return numbers.stream().mapToDouble(x -> x).summaryStatistics();
    }

    public double getMean() {
        return getStats().getAverage();
    }

    public double getStandardDeviation() {
        double mean = getMean();
        return Math.sqrt(numbers.stream().mapToDouble(x -> Math.pow(x - mean, 2)).sum() / (getSize() - 1));
    }

    public double getSize() {
        return numbers.size();
    }

    public void printStats() {
        GreenLogger.log("mean: " + getMean());
        GreenLogger.log("standard deviation: " + getStandardDeviation());
        GreenLogger.log("min: " + getStats().getMin());
        GreenLogger.log("max: " + getStats().getMax());
        GreenLogger.log("size: " + getSize());
    }

    public void stop() {
        stopped = true;
    }

    public void stopAndPrint() {
        if (stopped) {
            return;
        }

        stop();
        printStats();
    }

}
