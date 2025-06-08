package com.team1816.lib;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SingletonTests {

    @Test
    void GetSingletonTest() {
        var test1 = Singleton.get(String.class);
        var test2 = Singleton.get(String.class);
        assertEquals(test1.hashCode(), test2.hashCode());
    }
}
