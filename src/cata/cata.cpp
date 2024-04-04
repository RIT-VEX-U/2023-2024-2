#include "cata/cata.h"

bool intake_can_be_enabled(double cata_pos) {
  return (cata_pos == 0.0) || (cata_pos > inake_enable_lower_threshold && cata_pos < intake_enable_upper_threshold);
}
bool CataOnlySys::intaking_allowed() {
  double cata_pos = pot.angle(vex::deg);

  return (
    (cata_pos == 0.0) || (cata_pos > inake_enable_lower_threshold && cata_pos < intake_enable_upper_threshold) &&
                           !cata_watcher.isNearObject()
  );
}

class CataOff : public CataOnlySys::State {
public:
  void entry(CataOnlySys &sys) override { sys.mot.stop(vex::brakeType::coast); }
  CataOnlyState id() const override { return CataOnlyState::CataOff; }
  State *respond(CataOnlySys &sys, CataOnlyMessage m) override;
};

class WaitingForDrop : public CataOnlySys::State {
public:
  void entry(CataOnlySys &sys) override { drop_timer.reset(); }
  CataOnlySys::MaybeMessage work(CataOnlySys &sys) override {
    if (drop_timer.value() > intake_drop_seconds_until_enable) {
      return CataOnlyMessage::EnableCata;
    }
    return {};
  }
  CataOnlyState id() const override { return CataOnlyState::WaitingForDrop; }
  State *respond(CataOnlySys &sys, CataOnlyMessage m) override;

private:
  vex::timer drop_timer;
};

struct Reloading : public CataOnlySys::State {
  void entry(CataOnlySys &sys) override {

    sys.pid.update(sys.pot.angle(vex::deg));
    sys.pid.set_target(cata_target_charge);
    sys.l_endgame_sol.close();
    sys.r_endgame_sol.close();
    sys.cata_sol.open();
  }

  CataOnlySys::MaybeMessage work(CataOnlySys &sys) override {
    // work on motor
    double cata_deg = sys.pot.angle(vex::deg);
    if (cata_deg == 0.0) {
      // adc hasnt warmed up yet, we're getting silly results
      return {};
    }
    sys.pid.update(cata_deg);
    sys.mot.spin(vex::fwd, sys.pid.get(), vex::volt);

    // are we there yettt
    if (sys.pid.is_on_target()) {
      return CataOnlyMessage::DoneReloading;
    }
    // otherwise keep chugging
    return {};
  }

  void exit(CataOnlySys &sys) override {
  }

  CataOnlyState id() const override { return CataOnlyState::Reloading; }
  State *respond(CataOnlySys &sys, CataOnlyMessage m) override;
};

class Firing : public CataOnlySys::State {
public:
  void entry(CataOnlySys &sys) override {
    sys.mot.spin(vex::reverse, fire_voltage, vex::volt);
  }

  CataOnlySys::MaybeMessage work(CataOnlySys &sys) override {
    // started goin up again
    if (sys.pot.angle(vex::deg) > done_firing_angle) {
      return CataOnlyMessage::DoneFiring;
    }
    return {};
  }

  void exit(CataOnlySys &sys) override {
    
  }
  
  CataOnlyState id() const override { return CataOnlyState::Firing; }
  State *respond(CataOnlySys &sys, CataOnlyMessage m) override;
};

class ReadyToFire : public CataOnlySys::State {
public:
  CataOnlySys::MaybeMessage work(CataOnlySys &sys) override {
    double cata_deg = sys.pot.angle(vex::degrees);
    sys.pid.update(cata_deg);
    sys.mot.spin(vex::fwd, sys.pid.get(), vex::volt);

    // If we slipped, send message to go back to reload
    if (!intake_can_be_enabled(cata_deg)) {
      printf("Slipped\n");
      fflush(stdout);
      return CataOnlyMessage::Slipped;
    }

    // hold here until message comes from outside
    return {};
  }
  State *respond(CataOnlySys &sys, CataOnlyMessage m) override;
  CataOnlyState id() const override { return CataOnlyState::ReadyToFire; }
};

class PrimeClimb : public CataOnlySys::State {
  public:

  void entry(CataOnlySys &sys) override {
    sys.cata_sol.open();
    sys.mot.stop(vex::coast);
  };

  CataOnlySys::MaybeMessage work(CataOnlySys &sys) override {
    double ang = sys.pot.angle(vex::deg);

    if(ang < cata_target_extension)
      sys.mot.spin(vex::directionType::fwd, 12, vex::volt);
    else
      sys.mot.stop(vex::brakeType::hold);

    return {};
  }; 

  void exit(CataOnlySys &sys) override {
    
  };

  CataOnlyState id() const override {
    return CataOnlyState::PrimeClimb;
  };

  CataOnlySys::State *respond(CataOnlySys &s, CataOnlyMessage m) override;
};

class ClimbHold : public CataOnlySys::State {
  public:
  void entry(CataOnlySys &sys) override {
    sys.l_endgame_sol.open();
    sys.r_endgame_sol.open();
    sys.cata_sol.close();
  };

  CataOnlySys::MaybeMessage work(CataOnlySys &sys) override {
    double ang = sys.pot.angle(vex::deg);

    if(ang > cata_target_charge)
      sys.mot.spin(vex::directionType::rev, 12, vex::volt);
    else
      sys.mot.stop(vex::coast);

    return {};
  }; 

  void exit(CataOnlySys &sys) override {
  };

  CataOnlyState id() const override {
    return CataOnlyState::ClimbHold;
  };

  CataOnlySys::State *respond(CataOnlySys &s, CataOnlyMessage m) override;
};

CataOnlySys::State *CataOff::respond(CataOnlySys &sys, CataOnlyMessage m) {
  if (m == CataOnlyMessage::EnableCata) {
    return new Reloading();
  } else if (m == CataOnlyMessage::StartDrop) {
    return new WaitingForDrop();
  } else if (m == CataOnlyMessage::StartClimb) {
    return new PrimeClimb();
  }
  // Ignore other messages
  return this;
}

CataOnlySys::State *WaitingForDrop::respond(CataOnlySys &sys, CataOnlyMessage m) {
  if (m == CataOnlyMessage::EnableCata) {
    return new Reloading();
  }
  // Ignore other messages
  return this;
}
 
CataOnlySys::State *Reloading::respond(CataOnlySys &sys, CataOnlyMessage m) {
  if (m == CataOnlyMessage::DoneReloading) {
    return new ReadyToFire();
  } else if (m == CataOnlyMessage::DisableCata) {
    return new CataOff();
  } else if (m == CataOnlyMessage::Fire) {
    if (sys.cata_watcher.isNearObject()) {
      return new Firing();
    } else {
      return this;
    }
  } else if (m == CataOnlyMessage::StartClimb) {
    return new PrimeClimb();
  }
  // Ignore other messages
  return this;
}
CataOnlySys::State *ReadyToFire::respond(CataOnlySys &sys, CataOnlyMessage m) {
  if (m == CataOnlyMessage::Fire) {
    if (sys.cata_watcher.isNearObject()) {
      return new Firing();
    } else {
      return this;
    }

  } else if (m == CataOnlyMessage::Slipped) {
    return new Reloading();
  } else if (m == CataOnlyMessage::DisableCata) {
    return new CataOff();
  } else if (m == CataOnlyMessage::StartClimb) {
    return new PrimeClimb();
  }

  // Ignore other messages
  return this;
}

CataOnlySys::State *Firing::respond(CataOnlySys &sys, CataOnlyMessage m) {
  if (m == CataOnlyMessage::DoneFiring) {
    return new Reloading();
  } else if (m == CataOnlyMessage::DisableCata) {
    return new CataOff();
  }
  // Ignore other messages
  return this;
}

CataOnlySys::State *PrimeClimb::respond(CataOnlySys &s, CataOnlyMessage m) {
  switch(m) {
    case CataOnlyMessage::FinishClimb:
      return new ClimbHold();
    case CataOnlyMessage::DisableCata:
      return new CataOff();
    default:
      return this;
  }
};

CataOnlySys::State *ClimbHold::respond(CataOnlySys &s, CataOnlyMessage m) {
  switch(m) {
    case CataOnlyMessage::DisableCata:
      return new CataOff();
    default:
      return this;
  }
};

std::string to_string(CataOnlyState s) {
  switch (s) {
  case CataOnlyState::Firing:
    return "Firing";
  case CataOnlyState::Reloading:
    return "Reloading";
  case CataOnlyState::ReadyToFire:
    return "ReadyToFire";
  case CataOnlyState::CataOff:
    return "CataOff";
  case CataOnlyState::WaitingForDrop:
    return "WaitingForDrop";
  case CataOnlyState::PrimeClimb:
    return "PrimeClimb";
  case CataOnlyState::ClimbHold:
    return "ClimbHold";
  default:
    return "UNKNOWN CATA STATE";
  }
  return "UNHANDLED CATA STATE";
}

std::string to_string(CataOnlyMessage m) {
  switch (m) {
  case CataOnlyMessage::Slipped:
    return "Slipped";
  case CataOnlyMessage::DoneReloading:
    return "Done Reloading";
  case CataOnlyMessage::DoneFiring:
    return "Done Firing";
  case CataOnlyMessage::Fire:
    return "Fire";
  case CataOnlyMessage::EnableCata:
    return "EnableCata";
  case CataOnlyMessage::DisableCata:
    return "DisableCata";
  case CataOnlyMessage::StartDrop:
    return "StartDrop";
  case CataOnlyMessage::StartClimb:
    return "StartClimb";
  case CataOnlyMessage::FinishClimb:
    return "FinishClimb";
  }
  return "UNHANDLED CATA MESSAGE";
}

CataOnlySys::CataOnlySys(
  vex::pot &cata_pot, vex::optical &cata_watcher, vex::motor_group &cata_motor, PIDFF &cata_pid, DropMode drop, vex::pneumatics &l_endgame_sol, vex::pneumatics &r_endgame_sol, vex::pneumatics &cata_sol
)
    : StateMachine(
        (drop == DropMode::Required) ? (CataOnlySys::State *)(new CataOff()) : (CataOnlySys::State *)(new Reloading())
      ),
      pot(cata_pot), cata_watcher(cata_watcher), mot(cata_motor), pid(cata_pid), l_endgame_sol(l_endgame_sol), r_endgame_sol(r_endgame_sol), cata_sol(cata_sol) {}
