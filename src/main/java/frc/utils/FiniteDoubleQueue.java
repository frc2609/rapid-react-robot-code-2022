// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Add your docs here.
 * 
 * @param <E>
 * 
 */
public class FiniteDoubleQueue extends FiniteQueue<Double> {
  int _max;

  /**
   * @param max_elements Precondition: max_elements has to be greater than 1!
   *
   */
  public FiniteDoubleQueue(int max_elements) {
    super(max_elements);
    _max = max_elements;
  }

  public Double getAverage() {
    Double sum = 0.0;
    if (this.size() == 0) {
      return 0.0;
    }
    for (int i = 0; i < this.size(); i++) {
      sum += this.get(i);
    }
    return sum / this.size();
  }

}