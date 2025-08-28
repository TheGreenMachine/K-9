package com.team1816.lib.util;

public class FormatUtils {
    public static String GetDisplay(Double value) {
        if(value == null) value = 0.0;
        var str = String.format("%.10g", value);
        str = str.contains(".") ? str.replaceAll("0*$","").replaceAll("\\.$","") : str;
        return str;
    }
}
