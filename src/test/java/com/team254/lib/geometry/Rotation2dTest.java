package com.team254.lib.geometry;

import org.junit.Assert;
import org.junit.Test;

import static org.junit.Assert.*;

public class Rotation2dTest {

    @Test
    public void distance() {
        var a = Rotation2d.fromDegrees(60);
        var b = Rotation2d.fromDegrees(30);
        var result = Rotation2d.fromRadians(b.distance(a));
        var result2 = Rotation2d.fromDegrees(a.getDegrees() - b.getDegrees());
        System.out.println(result.getDegrees());
        System.out.println(result2.getDegrees());
    }

    @Test
    public void wrap() {
        var delta = .001;
        Assert.assertEquals( 90, Rotation2d.fromDegrees(90).getDegrees(), delta);
        Assert.assertEquals( -90, Rotation2d.fromDegrees(-90).getDegrees(), delta);
        Assert.assertEquals( 180, Rotation2d.fromDegrees(180).getDegrees(), delta);
        Assert.assertEquals( -180, Rotation2d.fromDegrees(-180).getDegrees(), delta);
        Assert.assertEquals( -170, Rotation2d.fromDegrees(190).getDegrees(), delta);
        Assert.assertEquals( 170, Rotation2d.fromDegrees(-190).getDegrees(), delta);
        Assert.assertEquals( 10, Rotation2d.fromDegrees(370).getDegrees(), delta);
        Assert.assertEquals( -10, Rotation2d.fromDegrees(-370).getDegrees(), delta);
        Assert.assertEquals( 0, Rotation2d.fromDegrees(-360).getDegrees(), delta);
        Assert.assertEquals( 0, Rotation2d.fromDegrees(360).getDegrees(), delta);
    }
}
