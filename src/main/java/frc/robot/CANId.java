// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

/** A class to hold the Ids of all devices on this robot's CAN bus.
 * At startup, Robot.java will call CANId.checkForDuplicates(),
 * which will abort the startup if there are any duplicate CAN Ids.
 */
public enum CANId {
  UNDECLARED_CAN_ID(-1)
  , FRONT_LEFT_DRIVE(31)
  , FRONT_LEFT_STEER(32)
  , FRONT_LEFT_CANCODER(41)
  , FRONT_RIGHT_DRIVE(33)
  , FRONT_RIGHT_STEER(34)
  , FRONT_RIGHT_CANCODER(43)
  , BACK_LEFT_DRIVE(38)
  , BACK_LEFT_STEER(37)
  , BACK_LEFT_CANCODER(47)
  , BACK_RIGHT_DRIVE(36)
  , BACK_RIGHT_STEER(35)
  , BACK_RIGHT_CANCODER(45)
  , LEFT_ELEVATOR(42)
  , RIGHT_ELEVATOR(62)
  , PIGEON(1)
  ;

  private int m_id;
  /**
   * Constructor is private, as enum's constructor must be.
   * @param id - the CAN bus identifier
   */
  private CANId(int id){
    m_id = id;
  }
  /**
   * Retrieve the CAN bus identifier
   * @return - the CAN bus identifier for this entry
   */
  public int getId(){
    return m_id;
  }
  /**
   * Format this entry for printing in form name(CAN id).
   */
  @Override
  public String toString(){
    return name() + "(CAN " + m_id + ")";
  }
  static HashMap<Integer, CANId> reverseMap ;
  /**
   * Report duplicate CAN bus identifiers and throw an error
   * if there are any.
   */
  public static void checkForDuplicates(){
    int nDups = 0;
    reverseMap = new HashMap<Integer, CANId>();
    for(CANId id: CANId.values()) {
      CANId prev = reverseMap.putIfAbsent(id.getId(), id);
      if (prev != null) {
        nDups++;
        System.out.println("duplicate ids: " + id + " and " + prev);
      }
    }
    if (nDups > 0){
      throw new Error("" + nDups + " duplicate CAN Id numbers in CANId.java");
    }
  }
  /**
   * Map a CAN bus identifier to the enum entry containing it.
   * @param id CAN bus identifier
   * @return CANId enum entry with this identifier or CANId.UNDECLARED_CAN_ID
   * if id is not listed above.
   */
  public static CANId idToCANId(int id){
    if (reverseMap == null) {
      checkForDuplicates();
    }
    CANId retVal = reverseMap.get(id);
    if (retVal == null){
      retVal = UNDECLARED_CAN_ID;
    }
    return retVal;
  } 
}
