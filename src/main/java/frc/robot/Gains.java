package frc.robot;

public class Gains {

	public final double kP;
	public final double kI;
	public final double kD;
	public final double kFF;
	public final int kIzone;
	public final double kMinOutput;
	public final double kMaxOutput;

	public Gains(double _kP, double _kI, double _kD, double _kFF, int _kIzone, double _kMinOutput, double _kMaxOutput) {

		kP = _kP;
		kI = _kI;
		kD = _kD;
		kFF = _kFF;
		kIzone = _kIzone;
		kMinOutput = _kMinOutput;
		kMaxOutput = _kMaxOutput;

	}
}