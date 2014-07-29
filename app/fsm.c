// Source file for my implementation of the UML FSM
// 17/06/2014

#include "fsm.h"

// Reserved Events array. This is strictly internal of this class and should
// never be used globally. If you think you need it, you're probably doing
// something unorthodox. 
static FSM_Event FSM_ReservedEvts[] = {{FSM_NULL_SIGNAL },
                                       {FSM_ENTRY_SIGNAL},
                                       {FSM_EXIT_SIGNAL },
                                       {FSM_INIT_SIGNAL }};

// FSM Init function. The original QEP implementation passes an event to this
// which I find unnecessary and confusing. I eliminated this to ensure clarity
void FSM_Init(FSM* me)
{
    // Take the initial transition, which MUST end with a Transition
    (*me->state)(me, &FSM_ReservedEvts[FSM_NULL_SIGNAL]);
    
    // Enters the target first state, performing any entry actions specified
    (void) (*me->state)(me,&FSM_ReservedEvts[FSM_ENTRY_SIGNAL]);
}

// FSM Event Dispatcher
void FSM_Dispatch (FSM* me, FSM_Event const *e)
{
    // Saves the current state
    FSM_StateHandler current_state = me->state;
    
    // Executes the current state with the incoming event
    FSM_State handler_return = (*current_state)(me,e);
    
    // If execution resulted in transition beeing taken, takes care of
    // exiting the current state and transitioning to the next. 
    if(handler_return == FSM_RET_TRAN)
    {
        // Executes the exit action of the current state
        (void)(*current_state)(me,&FSM_ReservedEvts[FSM_EXIT_SIGNAL]);
        
        // Executes the entry action of the next state
        (void)(me->state)(me,&FSM_ReservedEvts[FSM_ENTRY_SIGNAL]);
    }
}

