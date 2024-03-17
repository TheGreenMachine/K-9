package com.team1816.lib.util.logUtil;

import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;

import java.util.function.Supplier;

public class PeriodicLog {
    private final Supplier Supplier;
    private final DataLogEntry DataLogEntry;

    public PeriodicLog(DataLogEntry entry, Supplier supplier){
        DataLogEntry = entry;
        Supplier = supplier;
    }

    public void UpdateLog(){
        var value = Supplier.get();
        if(DataLogEntry instanceof DoubleLogEntry) {
            ((DoubleLogEntry)DataLogEntry).append((double) value);
        }
    }

}
