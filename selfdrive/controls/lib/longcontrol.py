from cereal import log
from common.numpy_fast import clip, interp
from selfdrive.controls.lib.pid import PIController2
from selfdrive.kegman_conf import kegman_conf
from selfdrive.debug.dataLogger import logData
from selfdrive.config import Conversions as CV

kegman = kegman_conf()
LongCtrlState = log.ControlsState.LongControlState

STOPPED_SPEED = .001

STOPPING_EGO_SPEED = 0.5
MIN_CAN_SPEED = 0.3  # TODO: parametrize this in car interface
STOPPING_TARGET_SPEED = MIN_CAN_SPEED + 0.01
STARTING_TARGET_SPEED = 0.5
BRAKE_THRESHOLD_TO_PID = 0.2

STOPPING_BRAKE_RATE = 0.2  # brake_travel/s while trying to stop
STARTING_BRAKE_RATE = 0.8  # brake_travel/s while releasing on restart
BRAKE_STOPPING_TARGET = float(kegman.conf['brakeStoppingTarget'])  # apply at least this amount of brake to maintain the vehicle stationary

_MAX_SPEED_ERROR_BP = [0., 30.]  # speed breakpoints
_MAX_SPEED_ERROR_V = [1.5, 1.5]  # max positive v_pid error VS actual speed; this avoids controls windup due to slow pedal resp

RATE = 100.0


def long_control_state_trans(long_plan, active, long_control_state, v_ego, v_target, v_pid,
                             output_gb, brake_pressed, cruise_standstill):
  """Update longitudinal control state machine"""

  stopping_condition = (v_ego < 2.0 and cruise_standstill) or \
                       (v_ego < STOPPING_EGO_SPEED and \
                        ((v_pid < STOPPING_TARGET_SPEED and v_target < STOPPING_TARGET_SPEED) or
                        brake_pressed))

  stopped_condition = (v_ego < STOPPED_SPEED)

  starting_condition = v_target > STARTING_TARGET_SPEED and not cruise_standstill


  # for now we are just working on the steady-state cruising state
  if not active:
    long_control_state = LongCtrlState.off
  else:
    logData(["haslead",long_plan.hasLead])
    logData(["leadTurnoff",long_plan.leadTurnoff])
    logData(["gotCutoff",long_plan.gotCutoff])
    logData(["prevXLead",long_plan.prevXLead])
    logData(["state",long_control_state])
    logData(["-----------------"])

    if (stopping_condition and stopped_condition):
      long_control_state = LongCtrlState.stopped
      return long_control_state

    if (stopping_condition and not stopped_condition):
      long_control_state = LongCtrlState.stopping
      return long_control_state

    if (starting_condition and long_plan.hasLead):
      long_control_state = LongCtrlState.startingWithLead
      return long_control_state

    if (starting_condition and not long_plan.hasLead):
      long_control_state = LongCtrlState.startingNoLead
      return long_control_state

    #long_control_state = LongCtrlState.steadyState

    if (long_control_state == LongCtrlState.off):
      # we've switched to active, in reality here do we need to
      # determine if we are in a starting condition? Not sure
      long_control_state = LongCtrlState.steadyState 

    elif (long_control_state == LongCtrlState.following):
      # goal is to fluctuate around desired react time, setting a
      # target speed (below our real target) that maintains this time.
      # a. lead pulls away -> bump up target speed -> following
      # b. lead gets closer -> bump down target speed -> coast
      # c. gaining too fast / getting too close -> slow
      long_control_state = LongCtrlState.steadyState

    elif (long_control_state == LongCtrlState.slowing):
      # here we are actively needing to brake. PID loop goal is to control decel
      # in a manner that is comfortable for the driver - achieve max decel well 
      # before the stopping point, then gradually ease up as we come upon the car.
      long_control_state = LongCtrlState.slowing

    elif (long_control_state == LongCtrlState.coasting):
      # J.R. determine following time
      long_control_state = LongCtrlState.coasting
      # if (no lead) -> steadyState
      # if (lead && within follow window) -> keep coasting
      # if (lead && vRel < 0) -> following

    elif (long_control_state == LongCtrlState.steadyState):
      if (long_plan.hasLead or long_plan.gotCutoff):
        long_control_state = LongCtrlState.following
      if (long_plan.leadTurnoff):
        long_control_state = LongCtrlState.startingNoLead

  return long_control_state


class LongControl():
  def __init__(self, AM, SM, CP, compute_gas, compute_brake):
    self.long_control_state = LongCtrlState.off  # initialized to off

    self.sm = SM
    self.am = AM # alert manager

    # J.R. first thing here, I'd config an additional PID and use one when accelerating, and a
    # different one when decelerating.
    self.pid = PIController2((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                            (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                            k_f=0,
                            rate=RATE,
                            sat_limit=0.8,
                            convert=compute_gas,
                            log_name="accel")

    self.decelPid = PIController2((CP.longitudinalBrakeTuning.kpBP, CP.longitudinalBrakeTuning.kpV),
                            (CP.longitudinalBrakeTuning.kiBP, CP.longitudinalBrakeTuning.kiV),
                            k_f=0,
                            rate=RATE,
                            sat_limit=0.8,
                            convert=compute_brake,
                            log_name="brake")
    self.v_pid = 0.0
    self.last_output_gb = 0.0

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update(self, frame, active, v_ego, brake_pressed, standstill, cruise_standstill, v_cruise, v_target, v_target_future, a_target, CP):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""

    # Actuation limits
    gas_max = interp(v_ego, CP.gasMaxBP, CP.gasMaxV)
    brake_max = interp(v_ego, CP.brakeMaxBP, CP.brakeMaxV)

    # Update state machine
    output_gb = self.last_output_gb
    last_state = self.long_control_state
    self.long_control_state = long_control_state_trans(self.sm['plan'], active, self.long_control_state, v_ego,
                                                       v_target_future, self.v_pid, output_gb,
                                                       brake_pressed, cruise_standstill)

    # based on potentially new state, choose our pid parameters
    #if (self.long_control_state != last_state and self.long_control_state != LongCtrlState.off):
    #  choosePidParams(last_state,self.long_control_state)

    # just a test see if I can pass an alert
    if (self.long_control_state != last_state and self.long_control_state != LongCtrlState.off):
      self.am.add(frame,"promptDriverDistracted")

    v_ego_pid = max(v_ego, MIN_CAN_SPEED)  # Without this we get jumps, CAN bus reports 0 when speed < 0.3

    if self.long_control_state == LongCtrlState.off:
      self.v_pid = v_ego_pid
      self.pid.reset()
      output_gb = 0.

    # tracking objects and driving
    elif (self.long_control_state == LongCtrlState.steadyState or self.long_control_state == LongCtrlState.following):
      # J.R. changed to v_cruise because we want the pid loop to do all the work
      # NOTE: v_cruise is in kph
      self.v_pid = v_cruise*CV.KPH_TO_MS
      self.pid.pos_limit = gas_max
      # set neg limit to zero to avoid braking while we are debugging
      self.pid.neg_limit = - brake_max
      self.pid.neg_limit = 0

      prevent_overshoot = False
      #deadzone = interp(v_ego_pid, CP.longitudinalTuning.deadzoneBP, CP.longitudinalTuning.deadzoneV)
      deadzone = 0

      # setpoint, measured, current speed, ....
      # v_pid is in m/s and so is v_ego_pid
      output_gb = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, feedforward=0)

      # v_cruise is in kph
      logData([v_cruise,self.v_pid,v_ego_pid,output_gb])

      # output_gb = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, deadzone=deadzone, feedforward=a_target, freeze_integrator=prevent_overshoot)
      # J.R. don't do any braking, just for testing. Actually here we are in steady-state cruising,
      # so we wouldn't do any braking anyway.
      output_gb = clip(output_gb,0,gas_max)

      # if prevent_overshoot:
      #  output_gb = min(output_gb, 0.0)

    # Intention is to stop, switch to a different brake control until we stop
    elif self.long_control_state == LongCtrlState.stopping:
      # Keep applying brakes until the car is stopped
      if not standstill or output_gb > -BRAKE_STOPPING_TARGET:
        # increase braking output because we haven't hit the stop target brake level yet, or
        # we haven't stopped
        output_gb -= STOPPING_BRAKE_RATE / RATE
      # clip forces output_bg to be between -brake_max and gas_max, clipping it into those range boundaries
      output_gb = clip(output_gb, -brake_max, gas_max)

      self.v_pid = v_ego
      self.pid.reset()

    # Intention is to move again, release brake fast before handing control to PID
    # J.R. or startingNoLead
    elif self.long_control_state == LongCtrlState.startingWithLead or self.long_control_state == LongCtrlState.startingNoLead:
      if output_gb < -0.2:
        output_gb += STARTING_BRAKE_RATE / RATE
      self.v_pid = v_ego
      self.pid.reset()

    self.last_output_gb = output_gb

    final_gas = clip(output_gb, 0., gas_max)
    final_brake = -clip(output_gb, -brake_max, 0.)

    return final_gas, final_brake
