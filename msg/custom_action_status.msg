uint64 timestamp                        # time since system start (microseconds)

uint8 STATUS_ACTION_ACCEPTED     = 0
uint8 STATUS_ACTION_STARTED      = 1
uint8 STATUS_ACTION_SUCCEEDED    = 2
uint8 STATUS_ACTION_CANCELLED    = 3
uint8 STATUS_ACTION_FAILED       = 4
uint8 STATUS_ACTION_TIMED_OUT    = 5

int8 REACTION_IGNORE             = 0
int8 REACTION_LOITER             = 1
int8 REACTION_RTL                = 2

int8 action_id                          # ID of current action. -1 for none.
uint8 status                            # Current status of custom actions
uint8 failure_reaction                  # Reaction performed because of failed custom action
