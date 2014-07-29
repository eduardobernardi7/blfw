//Header
// 17/06/2014

#include <stdint.h>

// Event Signal Typedef
typedef uint8_t FSM_EventSignal;

// Event structure. Extend parameters by inheriting from this structure. 
typedef struct FSM_Event_Tag
{
    FSM_EventSignal signal;     // Event signal
} FSM_Event;

// Reserved Event Signals Enumeration
enum FSM_ReservedEventSignals
{
    FSM_NULL_SIGNAL = 0,    // Reserved signal for init purposes   
    FSM_ENTRY_SIGNAL = 1,   // Reserved signal for entry actions
    FSM_EXIT_SIGNAL,        // Reserved signal for exit actions
    FSM_INIT_SIGNAL,        // Reserved signal for the initial transition
    FSM_USER_SIGNAL         // First Signal available to user applications
};

// State Variable Type
typedef uint8_t FSM_State;

// State Handler Type, returns State
typedef FSM_State (*FSM_StateHandler)(void *me, FSM_Event const *e);

// State Machine Type
typedef struct FSM_Tag
{
    FSM_StateHandler state; 
} FSM; 

// Constructor definition. This is usually unnecessary, because it obfuscates
// the FSM structure initialization. For now I'm leaving it here, but I'm not
// sure if this will stay. 
#define FSM_Ctor(me_, initial_) (((FSM*)me_)->state=(FSM_StateHandler)(initial_))

// Init and dispatch functions
void FSM_Init (FSM* me);
void FSM_Dispatch (FSM* me, FSM_Event const *e);

// Return values for FSM event handling
#define FSM_RET_HANDLED ((FSM_State)0)
#define FSM_RET_IGNORED ((FSM_State)1)
#define FSM_RET_TRAN    ((FSM_State)2)


// Return macros for FSM event handling
#define FSM_HANDLED()   (FSM_RET_HANDLED)
#define FSM_IGNORED()   (FSM_RET_IGNORED)

// State transition macro is defined by a comma expression. 
// The operation is: ((FSM*)me_)-> (FSM_StateHandler)(target_)
// The type and value are defined by the last member of the expression, in this
// particular case is type FSM_State, with a value of 2.

#define FSM_TRAN(me_,target_) (((FSM*)me_)->state=(FSM_StateHandler)(target_), FSM_RET_TRAN)


