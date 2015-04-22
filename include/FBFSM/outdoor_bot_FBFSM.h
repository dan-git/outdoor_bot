#ifndef _OUTDOOR_BOT_FBFSM__H_
#define _OUTDOOR_BOT_FBFSM__H_

#include "FBFSM/FBFSM.h"

class botFSM
{
 public:
  /**
   * An FSM that has two states.  It switches between the states every limit counts.
   */
  botFSM(int limit);
  void update();

 private:
  void setupFSM();

  void on_enter_state1();
  int on_update_state1();
  void on_exit_state1();

  void on_enter_state2();
  int on_update_state2();

  FBFSM m_fsm;

  int m_state1;
  int m_state2;

  int m_limit;
  int m_counter;
};

#endif  // _OUTDOOR_BOT_FBFSM__H__
