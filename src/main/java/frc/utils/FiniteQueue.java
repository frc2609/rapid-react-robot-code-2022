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
 * @param <E> Numeric
 */
public class FiniteQueue<E> extends LinkedList<E> {
  int _max;

  public FiniteQueue(int max_elements) {
    super();
    _max = max_elements;
  }

  @Override
  public void push(E e) {
    if (this.size() < _max) {
      super.push(e);
    } else {
      this.pop();
      super.push(e);
    }
  }

  public E pop() {
    E last = super.getLast();
    super.removeLast();
    return last;
  }

}