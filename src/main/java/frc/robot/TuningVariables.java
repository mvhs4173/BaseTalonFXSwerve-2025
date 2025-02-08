// Author: Bill Dunlap <williamwdunlap@gmail.com>, FRC 4173 (Team IMVERT), January 2023.
// This software is free to use by anyone with no restrictions.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

/** These things are like the variables in Constants,
 * but can be set via the Preferences class (e.g.,
 * via Shuffleboard) and so will be stored in [persistent]
 * flash memory on the roborio.  The TuningVariables class
 * also stores default values in Java (nonpersistent)
 * memory.
 * 
 * To add a new variable, say newVar, with default value newVarDefault,
 * to this scheme, add newVar(newVarDefault) to the comma-separated
 * list at the start of the enum below.
 * 
 * To get a tuning variable's value use the syntax
 *   TuningVariables.variableName.get()
 * 
 * A current limitation is that the turning variables
 * must be doubles.
 */
public enum TuningVariables {
    // To add a new value, just enter its name and default value to the following command-separated list
    // Semicolon ends the list.
    debugLevel(1.0), // scale of 0 to 10

    // The following bunch are to avoid CAN and other errors when testing the incomplete robot
    /** false means to not create swerve drive object */
    useSwerve(true),
    /** false means to create and use shooter motors */
    useShooter2(true),
    /** false means to not create wrist v.2 motor object */
    useWrist2(true),
    /** false means to not create shoulder motor objects */
    useShoulder(true),
    /** true means to use driver's Xbox controller object.  If so, it will be in port 0.
     *  false means to not create the controller object for swerve drive.
     */
    useDriveController(true),
    /** true means to use manipulator's Xbox controller.  
     *  If so, it will be lowest numbered available port.
     *  false means to not create that controller object.
     */
    useArmController(true),
    /**true means to try to use the collector roller */
    useCollectorRoller(true);

    private double m_defaultValueNumber;
    private boolean m_defaultValueBoolean;
    private String m_defaultValueString;
    enum Type { kNumber, kBoolean, kString; };
    private Type m_type;

    /** Users cannot call an enum constructor directly;
     * Java will call it for each variable listed above.
     * Note the constructor will not change pre-existing
     * values in flash memory.  Use setToDefaultValue or
     * setAllToDefaultValues for that.
     */
    private TuningVariables(double defaultValueNumber){
        m_defaultValueNumber = defaultValueNumber;
        m_type = Type.kNumber;
        if (!Preferences.containsKey(name())) {
          Preferences.setDouble(name(), m_defaultValueNumber);
        }
    }
    private TuningVariables(boolean defaultValueBoolean){
        m_defaultValueBoolean = defaultValueBoolean;
        m_type = Type.kBoolean;
        if (!Preferences.containsKey(name())){
            Preferences.setBoolean(name(), m_defaultValueBoolean);
        }
    }
    private TuningVariables(String defaultValueString){
        m_defaultValueString = defaultValueString;
        m_type = Type.kString;
        if (!Preferences.containsKey(name())){
            Preferences.setString(name(), m_defaultValueString);
        }
    }
    /** From flash memory, get the value of this tuning variable */
    private void checkType(Type type){
        if (m_type != type){
            remove();
            throw new Error("Requested number but TuningVariable." + name() + " is " + m_type);
        }
    }
    public double getNumber(){
        checkType(Type.kNumber);
        return Preferences.getDouble(name(), m_defaultValueNumber);
    }
    public boolean getBoolean(){
        checkType(Type.kBoolean);
        return Preferences.getBoolean(name(), m_defaultValueBoolean);
    }
    public String getString(){
        checkType(Type.kString);
        return Preferences.getString(name(), m_defaultValueString);
    }
    /** In flash memory, set this tuning variable to a value */
    public void set(double value){
        checkType(Type.kNumber);
        Preferences.setDouble(name(), value);
    }
    public void set(boolean value){
        checkType(Type.kBoolean);
        Preferences.setBoolean(name(), value);
    }
    public void set(String value){
        checkType(Type.kString);
        Preferences.setString(name(), value);
    }
    /** In flash memory, set this tuning variable to its default value */
    public void setToDefaultValue() {
        switch(m_type){
            case kNumber:
                set(m_defaultValueNumber);
                break;
            case kBoolean:
                set(m_defaultValueBoolean);
                break;
            case kString:
                set(m_defaultValueString);
                break;
        }
    }
    /** In flash memory, set all tuning variables to their default values */
    public static void setAllToDefaultValues() {
        for(TuningVariables tv: TuningVariables.values()) {
            tv.setToDefaultValue();
        }
    }
    /** Remove this tuning variable from flash memory */
    public void remove(){
        Preferences.remove(name());
    }
    /** Remove all tuning variables from flash memory */
    public static void removeAllKnown() {
        for(TuningVariables tv : TuningVariables.values()) {
            tv.remove();
        }
    }
    public static void removeAllPreferences(){
        removeAllKnown();
        Preferences.removeAll();
    }
}
