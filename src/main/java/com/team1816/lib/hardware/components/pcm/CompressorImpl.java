package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class CompressorImpl extends Compressor implements ICompressor {

    public CompressorImpl(int module, PneumaticsModuleType moduleType) {
        super(module, moduleType);
    }

    public CompressorImpl(PneumaticsModuleType moduleType) {
        super(moduleType);
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    @Override
    public double getCompressorCurrent() {
        return 0;
    }
}
