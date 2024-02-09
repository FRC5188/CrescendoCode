package frc.robot.subsystems.leds;

public class LEDColor {
    int _r;
    int _g;
    int _b;

    public LEDColor(int r, int g, int b) {
        _r = r;
        _g = g;
        _b = b;
    }

    public int getR() {
        return _r;
    }

    public int getG() {
        return _g;
    }

    public int getB() {
        return _b;
    }
}
