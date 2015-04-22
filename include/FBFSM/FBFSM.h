#ifndef __FBFSM__H__
#define __FBFSM__H__

#include <string>
#include <vector>

#include <boost/function.hpp>

#include <ros/console.h>

class FBFSM
{
 public:
  typedef boost::function<void()> EntryFunction;
  typedef boost::function<void()> ExitFunction;
  typedef boost::function<int()>  UpdateFunction;
  typedef boost::function<void(int, int)> TransitionFunction;
 public:
  FBFSM() {}
  int add_state(const std::string& name);
  int add_state() {return add_state("Unnamed");}
  bool set_entry_function(int state, const EntryFunction& entry_function);
  bool set_update_function(int state, const UpdateFunction& update_function);
  bool set_exit_function(int state, const ExitFunction& exit_function);
  void set_transition_function(const TransitionFunction& transition_function)
  {m_transition_function.set_function(transition_function);}

  void set_state(int state);
  void finish();
  void update();

  int current_state() const {return m_current_state;}
  int last_state() const {return m_last_state;}
  std::string state_name(int state) const;
  std::string current_state_name() const {return state_name(m_current_state);}

 private:
  class OptionalEntryFunction
  {
   public:
    OptionalEntryFunction() : m_has_function(false) {}
    void set_function(const EntryFunction& function) {m_function = function; m_has_function = true;}
    void operator()()
    {
      if (m_has_function)
      {
        return m_function();
      }
      ROS_WARN("Entering state with no entry function..");
    }
   private:
    bool m_has_function;
    EntryFunction m_function;
  };

  class OptionalExitFunction
  {
   public:
    OptionalExitFunction() : m_has_function(false) {}
    void set_function(const ExitFunction& function) {m_function = function; m_has_function = true;}
    void operator()()
    {
      if (m_has_function)
      {
        return m_function();
      }
    }
   private:
    bool m_has_function;
    ExitFunction m_function;
  };

  class OptionalUpdateFunction
  {
   public:
    explicit OptionalUpdateFunction(int default_val) : m_has_function(false), m_default_val(default_val) {}
    void set_function(const UpdateFunction& function) {m_function = function; m_has_function = true;}
    int operator()()
    {
      if (m_has_function)
      {
        return m_function();
      }
      return m_default_val;
    }
   private:
    bool m_has_function;
    int m_default_val;
    UpdateFunction m_function;
  };

  class OptionalTransitionFunction
  {
   public:
    OptionalTransitionFunction() : m_has_function(false) {}
    void set_function(const TransitionFunction& function) {m_function = function; m_has_function = true;}
    void operator()(int from_state, int to_state)
    {
      if (m_has_function)
      {
        return m_function(from_state, to_state);
      }
    }
   private:
    bool m_has_function;
    TransitionFunction m_function;
  };

  int m_current_state;
  int m_last_state;

  std::vector<OptionalEntryFunction>  m_entry_functions;
  std::vector<OptionalExitFunction>   m_exit_functions;
  std::vector<OptionalUpdateFunction> m_update_functions;
  OptionalTransitionFunction          m_transition_function;
  std::vector<std::string>            m_names;
};

#endif  //  __FBFSM__H__
