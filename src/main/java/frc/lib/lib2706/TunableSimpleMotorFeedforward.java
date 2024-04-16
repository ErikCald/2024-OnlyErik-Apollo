package frc.lib.lib2706;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;

import java.util.function.Consumer;

public class TunableSimpleMotorFeedforward {
    private final TunableDouble m_kS, m_kV, m_kA;
    private Consumer<SimpleMotorFeedforward> m_setFeedforward;

    /**
     * Create a UpdateFeedforward to update a {@link SimpleMotorFeedforward} object from networktables.
     *
     * @param setFeedforward A consumer to recieve an updated SimpleMotorFeedforward. Recommend: {@snippet (ff) -> m_topFF = ff}
     * @param tableName The table name to put the 3 entries in for kS, kV and kA
     * @param kS The default value for kS
     * @param kV The default value for kV
     * @param kA The default value for kA
     */
    public TunableSimpleMotorFeedforward(
            Consumer<SimpleMotorFeedforward> setFeedforward,
            NetworkTable table,
            double kS,
            double kV,
            double kA) {
        m_setFeedforward = setFeedforward;

        NetworkTable ffTable = table.getSubTable("TuneSimpleFF");
        m_kS = new TunableDouble("kS", ffTable, kS);
        m_kV = new TunableDouble("kV", ffTable, kV);
        m_kA = new TunableDouble("kA", ffTable, kA);

        // Set the initial feedforward to the defaults or to previously set values on networktables
        setFeedforward(m_kS.get(), m_kV.get(), m_kA.get());
    }

    /**
     * Sets the feedforward constants to the given values.
     *
     * @param kS The static gain of the feedforward.
     * @param kV The velocity gain of the feedforward.
     * @param kA The acceleration gain of the feedforward.
     */
    private void setFeedforward(double kS, double kV, double kA) {
        m_setFeedforward.accept(new SimpleMotorFeedforward(kS, kV, kA));
    }

    /**
     * Updates the feedforward values based on the current tunable doubles.
     */
    public void updateFeedforward() {
        TunableDouble.ifChanged(
                hashCode(),
                () -> this.setFeedforward(m_kS.get(), m_kV.get(), m_kA.get()),
                m_kS,
                m_kV,
                m_kA);
    }
}
