package com.team1816.lib.util;

import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.util.datalog.DataLogEntry;

public class LogTopic {
    public Publisher Publisher;
    public LogTopic(Publisher Publisher) {
        this.Publisher = Publisher;
    }
}
