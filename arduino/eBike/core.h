/* 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**************************************************************************
 *
 * The core module defines core classes used by all other modules.
 *
 * Modules can subclass ModuleSetup to implement setup behavior and
 * ModuleLoop to execute periodic instruction with each iteration of the
 * main loop.
 *
 * Instructions that can be invoked via a serial connection, must subclass
 * the Instruction class.
 **************************************************************************/

#ifndef CORE_H_
#define CORE_H_


#define null NULL

namespace rcs {

// Logging level
#define LOG_NONE 0
#define LOG_ERROR 1
#define LOG_WARNING 2
#define LOG_INFO 3
#define LOG_DEBUG 4
int log_level;

// Active serial port.
// Instructions are currently processed from this serial port and replies should
// be returned on this serial port.
// HardwareSerial* serial;

// Returns +1 if x is positive, -1 if x is negative, and 0 if x is 0.
int sign(float x) {
  if (x > 0.0f) return 1;
  else if (x < 0.0f) return -1;
  else return 0;
}

// Abstract base class for all instructions.
// All instructions must derive from this class and implement run(String);
// Subclasses are also encouraged to overload operator() with instruction specific arguments.
// In order to register the instruction, an instance of the subclass must be created, ideally
// with the same name as the instruction.
// For example:
//
// class MyInstruction : public Instruction {
//   public:
//     MyInstruction() : Instruction("myInstruction") {}
//
//     virtual boolean run(String* args) {
//        // instruction code
//        // return true if successful
//     }
// } myInstruction;

class Instruction {
  public:
    // Creates an instruction with the specified name.
    explicit Instruction(const String& name) : name_(name) // making single parameter constructor explicit
    {                                                      // is a good idea to avoid confusion when called
        registerInstruction(name, this);                   // use const String since we never want to change the name internally
    }                                                      // and pass as reference just to avoid making an extra copy

    virtual ~Instruction() {
    }

    // Returns the name of the instruction.
    const String& name() const {
      return name_;
    }

    // Derived classes must implement this method.
    // args is a comma separated list of function arguments. The pop* methods should be used by
    // derived classes to obtain individual typed arguments.
    // Returns true if execution was successful.
    virtual boolean run(String* args) = 0;

    // The functions below are static so  they can be called on the class itself, no instance of the class necessary.
    // This saves memory, as we get access to the functions without needing to create objects.
    
    
    // Pops the next argument from the argument list and returns it as a string in result.
    // Returns the empty string if there are no more arguments left.
    // The popped argument is removed from args.
    static void popToken(String* args, String* result) {
      int index = args->indexOf(',');
      if (index < 0) {
        *result = *args;
        *args = "";
      } else {
        *result = args->substring(0, index);
        *args = args->substring(index + 1);
      }
    }

    // Pops the next argument from the argument list and returns it as an integer.
    // Returns 0 on error.
    static void popInteger(String* args, int* result) {
      String str;
      popToken(args, &str);
      *result = str.toInt();
    }

    // Pops the next argument from the argument list and returns it as a float.
    // Returns 0.0 on error.
    static void popFloat(String* args, float* result) {
      String str;
      popToken(args, &str);
      char buf[9];
      str.toCharArray(buf, 8);
      buf[8] = 0;
      *result = atof(buf);
    }

    // Returns a pointer to a registered Instruction with the specified name.
    // Returns null if there is no such Instruction.
    static Instruction* find(const String& name) {
      for (int i = 0; i < num_instructions; ++i) {
        if (instructions[i]->name() == name) {
          return instructions[i];
        }
      }
      return null;
    }

    // Runs the Instruction with the specified name and the comma separated list of arguments.
    static boolean run(const String& name, String* args) {
      Instruction* instruction = find(name);
      if (instruction == null) return false;
      return instruction->run(args);
    }

    // Registers an instruction. Only MAX_NUM_INSTRUCTIONS many Instructions can be registered.
    // Returns true if the instruction has been registered.
    // Does not take ownership of the instruction.
    static boolean registerInstruction(const String& name, Instruction* instruction) {
      if (num_instructions >= MAX_NUM_INSTRUCTIONS) return false;
      instructions[num_instructions] = instruction;
      num_instructions++;
      return true;
    }

  private:
    String name_;

    static Instruction* instructions[MAX_NUM_INSTRUCTIONS];
    static int num_instructions;
};
Instruction* Instruction::instructions[MAX_NUM_INSTRUCTIONS];
int Instruction::num_instructions(0);

// Base class for module specific setup method.
// A module must define a subclass of this class and overwrite the setup() method.
// In addition, the module must create an instance of the subclass by declaring
// a variable for the subclass, for example:
//
// class MyModuleSetup : public ModuleSetup {
//   public:
//     virtual void setup() {
//        // module setup code
//     }
// } my_setup;
class ModuleSetup {
  public:
    ModuleSetup() {
      registerModuleSetup(this);
    }
    
    virtual ~ModuleSetup() {
    }

    // In subclasses defines module specific setup behavior.
    virtual void setup() = 0;

    // Runs all registered module setups. Called by setup().
    static void runAll() {
        for (int i = 0; i < num_modules; ++i) {
        if (DEBUG)
        {
          DEBUG_SERIAL_PORT.print("Setting up module ");
          DEBUG_SERIAL_PORT.println(i);
        }
        module_setups[i]->setup();
      }
    }

    // Registers a module setup. Only MAX_NUM_MODULES many ModuleSetups can be registered.
    // Returns true if the ModuleSetup has been registered.
    // Does not take ownership of the ModuleSetup.
    static boolean registerModuleSetup(ModuleSetup* module_setup) {
      if (num_modules >= MAX_NUM_MODULES) return false;
      module_setups[num_modules] = module_setup;
      num_modules++;
      return true;
    }

  private:
    static ModuleSetup* module_setups[MAX_NUM_MODULES];
    static int num_modules;
};
ModuleSetup* ModuleSetup::module_setups[MAX_NUM_MODULES];
int ModuleSetup::num_modules;

// Base class for module specific loop method.
// A module must define a subclass of this class and overwrite the loop() method.
// In addition, the module must create an instance of the subclass by declaring
// a variable for the subclass, for example:
//
// class MyModuleLoop : public ModuleLoop {
//   public:
//     virtual void loop() {
//        // module loop code
//     }
// } my_loop;
class ModuleLoop {
  public:
    ModuleLoop() {
      registerModuleLoop(this);
    }
    
    virtual ~ModuleLoop() {
    }
    
    // In subclasses defines module specific loop behavior.
    virtual void loop() = 0;
    
    // Runs all registered module loops. Called by loop().
    static void runAll() {
      for (int i = 0; i < num_modules; ++i) {       
        module_loops[i]->loop();
      }
    }
  
    // Registers a module loop. Only MAX_NUM_MODULES many ModuleLoops can be registered.
    // Returns true if the ModuleLoop has been registered.
    // Does not take ownership of the ModuleLoop.
    static boolean registerModuleLoop(ModuleLoop* module_loop) {
      if (num_modules >= MAX_NUM_MODULES) return false;
      module_loops[num_modules] = module_loop;
      num_modules++;
      return true;
    }
  
  private:
    static ModuleLoop* module_loops[MAX_NUM_MODULES];
    static int num_modules;
};
ModuleLoop* ModuleLoop::module_loops[MAX_NUM_MODULES];
int ModuleLoop::num_modules;

// ping();
// ping() returns the robotController version.
class Ping : public Instruction {
  public:
    Ping()
      : Instruction("ping"),
        last_ping_time_(millis()) {
    }

    virtual boolean run(String* args) {
      return (*this)();
    }

    boolean operator()() {
      last_ping_time_ = millis();
      return true;
    }
    
    boolean timeout() const {
      #if TIMEOUT
      unsigned long current_time_ = millis();
      if (current_time_ < last_ping_time_)
        return false;  // overflow
      return (current_time_ - last_ping_time_) > TIMEOUT_MILLIS;
      #else
      return false;
      #endif
    }
    
    void resetTimeout() {
      #if TIMEOUT
      last_ping_time_ = millis();
      #endif
    }
    
  private:
    // Time of last ping.
    unsigned long last_ping_time_;
} ping;



}  // namespace rcs

#endif  // CORE_H_

