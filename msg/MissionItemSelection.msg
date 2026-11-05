# Records an item selection, landing abort, or mission replacement.
# A separate topic keeps fence, rally and automatic mission updates from overwriting the selection.
# It also leaves the saved Mission message layout unchanged.

uint64 timestamp # Selection/replacement time since system start (microseconds)
uint32 mission_id # Mission checksum identifying the selected mission
int32 selected_seq # Explicitly selected item, or -1 when the mission was replaced/cleared
uint8 mission_dataman_id # Dataman storage containing the selected mission
