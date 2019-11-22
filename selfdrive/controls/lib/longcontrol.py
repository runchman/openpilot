from cereal import log
from common.numpy_fast import clip, interp
from selfdrive.controls.lib.pid import PIController2
from selfdrive.kegman_conf import kegman_conf
from selfdrive.debug.dataLogger import logData
from selfdrive.config import Conversions as CV

kegman = kegman_conf()
LongCtrlState = log.ControlsState.LongControlState

TARGET_REACT_TIME = 2.5   # seconds following goal
FOLLOW_SPEED_BUMP = 2 * CV.MPH_TO_MS

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

#  enum LongControlState {
#    off @0;                  0
#    pidDEPRECATED @1;        1
#    stopping @2;             2
#    startingDEPRECATED @3;   3
#    startingNoLead @4;       4
#    startingWithLead @5;     5
#    following @6;            6
#    slowing @7;              7
#    coasting @8;             8
#    stopped @9;              9
#    steadyState @10;         10
#  }
def chooseAndResetPid(controlState,convert_gas,convert_brake):
  startingNoLead_Kp = 1.0
  startingNoLead_Ki = 0.05
  startingNoLead_Kf = 0.0

  startingWithLead_Kp = 1.0
  startingWithLead_Ki = 0.05
  startingWithLead_Kf = 0.0

  following_Kp = 1.5
  following_Ki = 0.05
  following_Kf = 0.0

  slowing_Kp = 0.2
  slowing_Ki = 0.1
  slowing_Kf = 0.0

  coasting_Kp = 0.0
  coasting_Ki = 0.0
  coasting_Kf = 0.0

  steadyState_Kp = 1.5
  steadyState_Ki = 0.05
  steadyState_Kf = 0.0

  # return a controller based on state
  if (controlState == LongCtrlState.startingNoLead):
    return PIController2(startingNoLead_Kp,startingNoLead_Ki,startingNoLead_Kf,convert=convert_gas, log_name="startNoLead")
  if (controlState == LongCtrlState.startingWithLead):
    return PIController2(startingWithLead_Kp,startingWithLead_Ki,startingWithLead_Kf,convert=convert_gas, log_name="startWithLead")
  if (controlState == LongCtrlState.following):
    return PIController2(following_Kp,following_Ki,following_Kf,convert=convert_gas, log_name="following")
  if (controlState == LongCtrlState.slowing):
    return PIController2(slowing_Kp,slowing_Ki,slowing_Kf,convert=convert_brake, log_name="slowing")
  if (controlState == LongCtrlState.steadyState):
    return PIController2(steadyState_Kp,steadyState_Ki,steadyState_Kf,convert=convert_gas, log_name="steadyState")
  # coasting pid won't actually get called, but we need it for controlsd to not shit the bed
  if (controlState == LongCtrlState.coasting):
    return PIController2(coasting_Kp,coasting_Ki,coasting_Kf,convert=convert_gas, log_name="coasting")

  return None


def long_control_state_trans(sm, active, long_control_state, v_ego, v_target, v_pid,
                             output_gb, brake_pressed, cruise_standstill, v_cruise):
  """Update longitudinal control state machine"""

  long_plan = sm['plan']
  vRel = long_plan.vRel
  deltaX = long_plan.prevXLead

  stopping_condition = (v_ego < 2.0 and cruise_standstill) or \
                       (v_ego < STOPPING_EGO_SPEED and \
                        ((v_pid < STOPPING_TARGET_SPEED and v_target < STOPPING_TARGET_SPEED) or
                        brake_pressed))

  stopped_condition = (v_ego < STOPPED_SPEED)

  starting_condition_with_lead = v_target > STARTING_TARGET_SPEED and not cruise_standstill
  starting_condition_no_lead = not cruise_standstill

  # assume we are golden unless there is a target
  react_time = TARGET_REACT_TIME
  if (long_plan.hasLead and v_ego > 0.2):
    react_time = deltaX / v_ego

  if not active:
    long_control_state = LongCtrlState.off
  else:
    logData(["haslead",long_plan.hasLead])
    logData(["leadTurnoff",long_plan.leadTurnoff])
    logData(["gotCutoff",long_plan.gotCutoff])
    logData(["prevXLead",long_plan.prevXLead])
    logData(["state",long_control_state])
    logData(["-----------------"])

    if (long_control_state == LongCtrlState.off):
      # we just became active, either while driving or sitting at a stop. 
      long_control_state = LongCtrlState.steadyState

    # slowing to a stop
    elif (long_control_state == LongCtrlState.stopping):
      long_control_state = LongCtrlState.stopping
      
    # starting from a stop no lead car
    elif (long_control_state == LongCtrlState.startingNoLead):
      long_control_state = LongCtrlState.startingNoLead

    # starting from a stop as first car
    elif (long_control_state == LongCtrlState.startingWithLead):
      long_control_state = LongCtrlState.startingWithLead

    # trailing behind a car, either at cruise speed or below
    elif (long_control_state == LongCtrlState.following):
      if (react_time < (.8 * TARGET_REACT_TIME)):
        long_control_state = LongCtrlState.coasting
      if (react_time < (.6 * TARGET_REACT_TIME)):
        long_control_state = LongCtrlState.slowing
      # no longer have a lead car, back to steadyState
      if (not long_plan.hasLead):
        long_control_state = LongCtrlState.steadyState

    # actively braking but not yet coming to a stop
    elif (long_control_state == LongCtrlState.slowing):
      long_control_state = LongCtrlState.slowing
      if (long_plan.hasLead and vRel > 0 and react_time > (1.0 * TARGET_REACT_TIME)):
        long_control_state = LongCtrlState.following
      elif ( not long_plan.hasLead):
        long_control_state = LongCtrlState.steadyState

    # bleeding off speed but not braking yet
    elif (long_control_state == LongCtrlState.coasting):
      if (long_plan.hasLead and react_time < (.6 * TARGET_REACT_TIME)):
        long_control_state = LongCtrlState.slowing
      elif (long_plan.hasLead and vRel > 0 and react_time > (1.0 * TARGET_REACT_TIME)):
        long_control_state = LongCtrlState.following
      elif (not long_plan.hasLead):
        long_control_state = LongCtrlState.steadyState

    elif (long_control_state == LongCtrlState.stopped):
      long_control_state = LongCtrlState.stopped
      # just sitting 

    elif (long_control_state == LongCtrlState.steadyState):
      if (long_plan.hasLead):
        long_control_state = LongCtrlState.following
        if (react_time < (.8 * TARGET_REACT_TIME)):
          long_control_state = LongCtrlState.coasting
        if (react_time < (.6 * TARGET_REACT_TIME)):
          long_control_state = LongCtrlState.slowing
    #  if (long_plan.gotCutoff):
    #    long_control_state = LongCtrlState.following
    #  elif (long_plan.leadTurnoff):
    #    long_control_state = LongCtrlState.startingNoLead
      # cruising at set speed, no lead car

    #if (long_control_state == LongCtrlState.stopped):
    #  if (starting_condition and long_plan.hasLead):
    #    long_control_state = LongCtrlState.startingWithLead
    #    return long_control_state

    #  if (starting_condition and not long_plan.hasLead):
    #    long_control_state = LongCtrlState.startingNoLead
    #    return long_control_state

    #if (stopping_condition and stopped_condition):
    #  long_control_state = LongCtrlState.stopped
    #  return long_control_state

    #if (stopping_condition and not stopped_condition):
    #  long_control_state = LongCtrlState.stopping
    #  return long_control_state

    #if (long_control_state == LongCtrlState.off):
    #  if (starting_condition and long_plan.hasLead):
    #    long_control_state = LongCtrlState.startingWithLead
    #    return long_control_state
    #  if (starting_condition and not long_plan.hasLead):
    #    long_control_state = LongCtrlState.startingNoLead
    #    return long_control_state

      # we just became active 
    #  long_control_state = LongCtrlState.steadyState 

    #elif (long_control_state == LongCtrlState.startingWithLead):
    #  if output_gb >= -BRAKE_THRESHOLD_TO_PID:
    #    long_control_state = LongCtrlState.steadyState

    #elif (long_control_state == LongCtrlState.startingNoLead):
    #  if output_gb >= -BRAKE_THRESHOLD_TO_PID:
    #    long_control_state = LongCtrlState.steadyState

    #elif (long_control_state == LongCtrlState.following):
      # goal is to fluctuate around desired react time, setting a
      # target speed (below our real target) that maintains this time.
      # a. lead pulls away -> bump up target speed -> following
      # b. lead gets closer -> bump down target speed -> coast
      # c. gaining too fast / getting too close -> slow
      #long_control_state = LongCtrlState.steadyState

    #elif (long_control_state == LongCtrlState.slowing):
      # here we are actively needing to brake. PID loop goal is to control decel
      # in a manner that is comfortable for the driver - achieve max decel well 
      # before the stopping point, then gradually ease up as we come upon the car.
    #  if (not long_plan.hasLead):
    #    long_control_state = LongCtrlState.steadyState
      # J.R. look at this value
    #  if (long_plan.leadTurnoff and v_ego < 2.5):
    #    long_control_state = LongCtrlState.startingNoLead
    #  if (long_plan.hasLead and react_time > (1.0 * TARGET_REACT_TIME)):
    #    long_control_state = LongCtrlState.following

    #elif (long_control_state == LongCtrlState.coasting):
    #  if (long_plan.hasLead and react_time < (.6 * TARGET_REACT_TIME)):
    #    long_control_state = LongCtrlState.slowing
    #  elif (long_plan.hasLead and vRel > 0 and react_time > (1.0 * TARGET_REACT_TIME)):
    #    long_control_state = LongCtrlState.following
    #  elif (not long_plan.hasLead):
    #    long_control_state = LongCtrlState.steadyState

    #elif (long_control_state == LongCtrlState.steadyState):
      # technically steadyState shouldn't have a target but we'll 
      # toss this in there anyway
    #  if (react_time < (.8 * TARGET_REACT_TIME)):
    #    long_control_state = LongCtrlState.coasting
    #  if (react_time < (.6 * TARGET_REACT_TIME)):
    #    long_control_state = LongCtrlState.slowing
    #  if (long_plan.gotCutoff):
    #    long_control_state = LongCtrlState.following
    #  elif (long_plan.leadTurnoff):
    #    long_control_state = LongCtrlState.startingNoLead

    #elif (long_control_state == LongCtrlState.following):
    #  if (react_time < (.8 * TARGET_REACT_TIME)):
    #    long_control_state = LongCtrlState.coasting
    #  if (react_time < (.6 * TARGET_REACT_TIME)):
    #    long_control_state = LongCtrlState.slowing

  return long_control_state


class LongControl():
  def __init__(self, AM, SM, CP, compute_gas, compute_brake):
    self.long_control_state = LongCtrlState.off  # initialized to off

    self.sm = SM
    self.am = AM # alert manager
    self.compute_gas = compute_gas
    self.compute_brake = compute_brake

    self.following_tick = 0
    self.braking_tick = 0

    #self.pid = PIController2((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
    #                        (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
    #                        k_f=0,
    #                        rate=RATE,
    #                        sat_limit=0.8,
    #                        convert=compute_gas,
    #                        log_name="accel")
    self.pid = chooseAndResetPid(LongCtrlState.steadyState,compute_gas,compute_brake)

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
    self.long_control_state = long_control_state_trans(self.sm, active, self.long_control_state, v_ego,
                                                       v_target_future, self.v_pid, output_gb,
                                                       brake_pressed, cruise_standstill, v_cruise)


    # based on new state, choose our pid parameters
    if (self.long_control_state != last_state and self.long_control_state != LongCtrlState.off):
      self.pid = chooseAndResetPid(self.long_control_state,self.compute_gas,self.compute_brake)
      if (self.pid is not None):
        self.pid.reset()
      if (self.long_control_state == LongCtrlState.steadyState):
        self.v_pid = v_cruise*CV.KPH_TO_MS
      if (self.long_control_state == LongCtrlState.following):
        # upon entrance to following mode, set target speed to current
        self.v_pid = v_ego
      

    # alert on state change
    if (self.long_control_state != last_state and self.long_control_state != LongCtrlState.off):
      if (self.long_control_state == LongCtrlState.steadyState):
        self.am.add(frame,"smooth")
      elif (self.long_control_state == LongCtrlState.startingNoLead):
        self.am.add(frame,"gunit")
      elif (self.long_control_state == LongCtrlState.startingWithLead):
        self.am.add(frame,"godude")
      elif (self.long_control_state == LongCtrlState.following):
        self.am.add(frame,"following")
      elif (self.long_control_state == LongCtrlState.slowing):
        self.am.add(frame,"brakes")
      elif (self.long_control_state == LongCtrlState.coasting):
        self.am.add(frame,"coast")
      elif (self.long_control_state == LongCtrlState.stopped):
        self.am.add(frame,"stopped")

    v_ego_pid = max(v_ego, MIN_CAN_SPEED)  # Without this we get jumps, CAN bus reports 0 when speed < 0.3

    # update appropriately 
    # slowing to a stop
    if (self.long_control_state == LongCtrlState.stopping):
      self.long_control_state = LongCtrlState.stopping
      
    # starting from a stop behind a car
    elif (self.long_control_state == LongCtrlState.startingNoLead):
      self.long_control_state = LongCtrlState.startingNoLead

    # starting from a stop as first car
    elif (self.long_control_state == LongCtrlState.startingWithLead):
      self.long_control_state = LongCtrlState.startingWithLead

    # trailing behind a car, either at cruise speed or below
    elif (self.long_control_state == LongCtrlState.following):
      if ((self.following_tick % 20) == 0):
        #update following speed based on reaction time. Don't go over the 
        #cruise setting.
        # J.R. rather than a bump amount, calc new speed based on calculated
        # lead car speed. Goal is drive vRel to zero.
        # lead car speed = vRel + v_ego
        #self.react_time = self.sm['plan'].prevXLead / v_ego
        #if (self.react_time > (1.0 * TARGET_REACT_TIME)):
        #  self.v_pid = min((self.v_pid + FOLLOW_SPEED_BUMP),(v_cruise*CV.KPH_TO_MS))
        self.v_pid = min(v_ego_pid + self.sm['plan'].vRel,(v_cruise*CV.KPH_TO_MS))
        # we don't drop speed target; we'll coast and then brake if required
        #elif (self.react_time < (.9 * TARGET_REACT_TIME)):
        #  self.v_pid = self.v_pid - FOLLOW_SPEED_BUMP 
        self.following_tick = 0
      # we update on every time thru the loop, not just when updating the speed
      output_gb = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, feedforward=0.0)
      self.following_tick += 1

    # actively braking but not yet coming to a stop
    elif (self.long_control_state == LongCtrlState.slowing):
      # update pid so we can log what's happening, for now no braking.
      # set target speed to lead speed
      self.v_pid = v_ego_pid + self.sm['plan'].vRel
      output_gb = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, feedforward=0.0)
      output_gb = 0.0

    # bleeding off speed but not braking yet
    elif (self.long_control_state == LongCtrlState.coasting):
      # update pid just so we can log that we are coasting
      output_gb = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, feedforward=0.0)
      output_gb = 0.0

    # just sitting 
    elif (self.long_control_state == LongCtrlState.stopped):
      self.long_control_state = LongCtrlState.stopped

    # cruisin' on a sunday afternoon
    elif (self.long_control_state == LongCtrlState.steadyState):
      self.v_pid = v_cruise*CV.KPH_TO_MS
      output_gb = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, feedforward=0.0)

    # J.R. no braking for now
    output_gb = clip(output_gb,0,gas_max)

    #if self.long_control_state == LongCtrlState.off:
    #  self.v_pid = v_ego_pid
    #  self.pid.reset()
    #  output_gb = 0.

    # tracking objects and driving
    #elif (self.long_control_state == LongCtrlState.steadyState):
      # J.R. changed to v_cruise because we want the pid loop to do all the work
      # NOTE: v_cruise is in kph
    #  self.v_pid = v_cruise*CV.KPH_TO_MS
    #  self.pid.pos_limit = gas_max
      # set neg limit to zero to avoid braking while we are debugging
    #  self.pid.neg_limit = - brake_max
    #  self.pid.neg_limit = 0

    #  prevent_overshoot = False
      #deadzone = interp(v_ego_pid, CP.longitudinalTuning.deadzoneBP, CP.longitudinalTuning.deadzoneV)
    #  deadzone = 0

      # setpoint, measured, current speed, ....
      # v_pid is in m/s and so is v_ego_pid
      #output_gb = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, feedforward=0)

      # v_cruise is in kph
      #logData([v_cruise,self.v_pid,v_ego_pid,output_gb])

      # output_gb = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, deadzone=deadzone, feedforward=a_target, freeze_integrator=prevent_overshoot)
     # J.R. don't do any braking, just for testing. Actually here we are in steady-state cruising,
      # so we wouldn't do any braking anyway.
      #output_gb = clip(output_gb,0,gas_max)

      # if prevent_overshoot:
      #  output_gb = min(output_gb, 0.0)

    #elif (self.long_control_state == LongCtrlState.following):
      # our present target is whatever is established due to our following reaction time
      #self.v_pid = v_ego
      # if we've fallen behind by a certain amount, bump target speed up, but not above our
      # current cruise setting
      #self.react_time = self.sm['plan'].prevXLead / v_ego
      #if (self.react_time > (1.1 * TARGET_REACT_TIME)):
      #  self.v_pid = min(self.v_pid * (FOLLOW_SPEED_BUMP / RATE),v_cruise)
      #  output_gb = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, feedforward=0)
      #  output_gb = clip(output_gb,0,gas_max)

    # Intention is to stop, switch to a different brake control until we stop
    #elif self.long_control_state == LongCtrlState.stopping:
      # Keep applying brakes until the car is stopped
      #if not standstill or output_gb > -BRAKE_STOPPING_TARGET:
        # increase braking output because we haven't hit the stop target brake level yet, or
        # we haven't stopped
      #  output_gb -= STOPPING_BRAKE_RATE / RATE
      # clip forces output_bg to be between -brake_max and gas_max, clipping it into those range boundaries
      #output_gb = clip(output_gb, -brake_max, gas_max)

      #self.v_pid = v_ego
      #self.pid.reset()

    #elif self.long_control_state == LongCtrlState.coasting:
      #output_gb = 0.0

    #elif self.long_control_state == LongCtrlState.slowing:
      # here we'd update the braking pid
     # output_gb = 0.0

    # Intention is to move again, release brake fast before handing control to PID
    # J.R. or startingNoLead
    #elif self.long_control_state == LongCtrlState.startingWithLead or self.long_control_state == LongCtrlState.startingNoLead:
    #  if output_gb < -0.2:
    #    output_gb += STARTING_BRAKE_RATE / RATE
    #  self.v_pid = v_ego
    #  self.pid.reset()

    self.last_output_gb = output_gb

    final_gas = clip(output_gb, 0., gas_max)
    final_brake = -clip(output_gb, -brake_max, 0.)

    return final_gas, final_brake
