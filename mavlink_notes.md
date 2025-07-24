## Mavlink MAIN Loop
- Some nonsense at the top for iridium mode
- Calculate rate multiplier (look at impl)

## These are kinda BS
- Configure SiK radio (move!!!)
- handleStatus() --> sets HIL mode or handles Iridium nonsense ... move!!!
- handleCommands() --> iridium nonsense and gimbal v1 ... remove!
- handleAndGetCurrentCommandAck() --> sends out ACKS, needs to be simplified
- handleMavlinkShellOutput() --> sends out mavlink shell output if enabled
- check_requested_subscriptions() --> configures streams that have been request, could be simplified

## Not BS
- Iterate over all streams and send
- Send ulog data
- check and send events
- forward messages in message_buffer
- update stats and publish telemetry status
- BACK TO THE TOP
