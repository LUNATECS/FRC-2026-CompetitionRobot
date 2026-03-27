package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private final CANdle candle;

  private static final int START_IDX = 8;
  private static final int END_IDX = 200;
  private LEDState currentState = LEDState.IDLE;
  private LEDState lastState = null;

  private final RGBWColor neonGreen = new RGBWColor(57, 255, 20, 0);

  public LEDSubsystem() {
    candle = new CANdle(0, CANBus.roboRIO());

    var config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.RGB; // strip is rgb I think
    config.LED.BrightnessScalar = 1.0;
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    candle.getConfigurator().apply(config);

    // Clear all animation slots
    for (int i = 0; i < 8; i++) {
      candle.setControl(new EmptyAnimation(i));
    }

    setDefaultAnimation();
  }
  
  public enum LEDState {
    IDLE,
    INTAKING,
    AIMED,
    SHOOTING
  }

  public void setState(LEDState state) {
  currentState = state;
}


  public void setSolidColor(int r, int g, int b) {
    clearAnimations();
    candle.setControl(
        new SolidColor(START_IDX, END_IDX)
            .withColor(new RGBWColor(r, g, b, 0))
    );
  }

  public void setSolidColorHex(String hex) {
    RGBWColor color = RGBWColor.fromHex(hex).orElseThrow();
    setSolidColor(color.Red, color.Green, color.Blue);
  }


  public void setStrobe(int r, int g, int b, double speed) {
    candle.setControl(
        new StrobeAnimation(START_IDX, END_IDX)
            .withColor(new RGBWColor(r, g, b, 0))
            .withSlot(0)
            .withFrameRate(speed)
    );
  }

  public void setColorFlow(int r, int g, int b, double speed) {
    candle.setControl(
        new ColorFlowAnimation(START_IDX, END_IDX)
            .withColor(new RGBWColor(r, g, b, 0))
            .withSlot(0)
            .withFrameRate(speed)
    );
  }

  public void clearAnimations() {
    for (int i = 0; i < 8; i++) {
      candle.setControl(new EmptyAnimation(i));
    }
  }


  public void setDefaultAnimation() {
    setStrobe(neonGreen.Red, neonGreen.Green, neonGreen.Blue, 20);
  }

  @Override
public void periodic() {
  if (currentState == lastState) return;

  lastState = currentState;

  switch (currentState) {

    case IDLE:
      setStrobe(neonGreen.Red, neonGreen.Green, neonGreen.Blue, 20);
      break;

    case INTAKING:
      setColorFlow(0, 0, 255, 5); // blue
      break;

    case AIMED:
        setColorFlow(255, 185, 0, 5); // orange
        break;

    case SHOOTING:
        setColorFlow(255, 0, 0, 5); // red
        break;
  }
}
}