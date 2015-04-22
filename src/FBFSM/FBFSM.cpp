#include "FBFSM/FBFSM.h"

int FBFSM::add_state(const std::string& name)
{
  int new_state = m_entry_functions.size();
  m_entry_functions.push_back(OptionalEntryFunction());
  m_exit_functions.push_back(OptionalExitFunction());
  m_update_functions.push_back(OptionalUpdateFunction(new_state));
  m_names.push_back(name);
  return new_state;
}

bool FBFSM::set_entry_function(int state, const EntryFunction& entry_function)
{
  if (state < 0 || state >= static_cast<int>(m_entry_functions.size()))
  {
    ROS_ERROR("Unable to set entry function for state %d because it does not exist.", state);
    return false;
  }
  m_entry_functions[state].set_function(entry_function);
  return true;
}

bool FBFSM::set_exit_function(int state, const ExitFunction& exit_function)
{
  if (state < 0 || state >= static_cast<int>(m_exit_functions.size()))
  {
    ROS_ERROR("Unable to set exit function for state %d because it does not exist.", state);
    return false;
  }
  m_exit_functions[state].set_function(exit_function);
  return true;
}

bool FBFSM::set_update_function(int state, const UpdateFunction& update_function)
{
  if (state < 0 || state >= static_cast<int>(m_update_functions.size()))
  {
    ROS_ERROR("Unable to set update function for state %d because it does not exist.", state);
    return false;
  }
  m_update_functions[state].set_function(update_function);
  return true;
}

void FBFSM::set_state(int state)
{
  if (state < 0 || state >= static_cast<int>(m_entry_functions.size()))
  {
    ROS_WARN("FSM state %d does not exist when setting state.  There are only %zu states.",
             state, m_entry_functions.size());
  }
  // Call the exit function on the current state if applicable.
  finish();
  m_current_state = state;
  m_last_state = -1;
}

void FBFSM::finish()
{
  m_current_state = -1;
  update();
}

void FBFSM::update()
{
  if (m_last_state != m_current_state &&
      m_current_state >= 0 && m_current_state < static_cast<int>(m_entry_functions.size()))
  {
    // We just entered the current state.  Call its entry function.
    m_entry_functions[m_current_state]();
  }

  m_last_state = m_current_state;
  if (m_current_state >= 0 && m_current_state < static_cast<int>(m_update_functions.size()))
  {
    // Update the current state.
    m_current_state = m_update_functions[m_current_state]();
  }

  if (m_last_state != m_current_state)
  {
    m_transition_function(m_last_state, m_current_state);
    if (m_current_state >= 0 && m_current_state < static_cast<int>(m_exit_functions.size()))
    {
      // We just left the last state.  Call its exit function.
      m_exit_functions[m_last_state]();
    }
  }
}

std::string FBFSM::state_name(int state) const
{
  if (state < 0 || state >= static_cast<int>(m_names.size()))
  {
    ROS_WARN("Request for name of invalid state %d.", state);
    return "Invalid";
  }
  return m_names[state];
}
