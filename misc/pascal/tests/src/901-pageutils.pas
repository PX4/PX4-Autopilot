UNIT pageutls;

INTERFACE

CONST
emp_logonid = '$own$';
flag_on = 'y';
flag_off = 'n';
pcs_phys_termtype = 'PCS';
min_integer = -2100000000;
max_integer = 2100000000;
invalid_integer = 2147483647;
invalid_timestamp = invalid_integer;
invalid_saldo = invalid_integer;
invalid_short = 32767;
invalid_duration = invalid_short;
undefined_sreal = -32767;
invalid_taris_time = -9999;
no_time = -9998;
no_allowance = no_time;
min_year = 25;
no_year_high = 2100;
only_valid_blockid = 6309;
sign_offset = 128;
seconds_per_day = 86400;
max_allowance = 2880;
empty_integer = -2147483647;
empty_short = -32767;
no_variable = empty_integer;
keydata_len = 60;
break_add_on = 64;
first_search_char = 32;
last_search_char = 255;
type_char = 256;
type_unsigned_byte = 257;
type_short = 512;
type_mod = 513;
type_duration = 514;
type_dduration = 515;
type_allowance = 516;
type_long = 768;
type_saldo = 769;
type_dsaldo = 770;
type_balance = 771;
type_date = 1024;
type_time = 1025;
type_emplno = -32512;
type_deptno = -32511;
type_costid = -32510;
tyoe_employcat = -32509;
type_commtype = -32508;
emplno_len = 10;
deptno_len = 10;
employcat_len = 4;
adjustcat_len = 4;
firstname_len = 20;
surname_len = 30;
badgeno_len = 10;
long_badgeno_len = 20;
objectid_len = 8;
logonid_len = 8;
chapter_len = 10;
objecttype_len = 8;
phys_filename_len = 32;
processname_len = 32;
code_len = 4;
long_record_len = 32000;
exit_code_len = 6;
error_text_len = 60;
perror_text_len = 80;
part_of_error_text_len = 57;
vos_modulename_len = 32;
time_zone_len = 3;
tmon_command_line_len = 300;
input_line_len = 1024;
comm_buffer_len = 4096;
double_comm_buffer_len = 8192;
cci_header_len = 4;
long_text_len = 30;
time_array_len = 5;
rpoolid_len = 12;
max_no_of_attributes = 8;
attribid_len = 12;
parproid_len = 12;
projectid_len = 12;
taskid_len = 12;
versionid_len = 4;
export_data_len = 249;
pcs_request_len = 200;
costid_type_len = 20;
costid_ext_type_len = 22;
search_set_len = 20;
cust_conv_table_common_type_len = 16001;
tstat_tab_len = 36;
max_repvar_len = 16;
userid_len = 16;
recid_len = 8;
fieldid_len = 16;
queryid_len = 8;
maxaccesses = 400;
maxallowances = 5;
maxalt_wpats = 8;
maxbreaks = 6;
maxdaytypes = 10;
maxdiffs_from_plan = 12;
maxseverity_codes = 8;
maxshifts_per_cycle = 64;
maxacmasks = 96;
maxaccodes = 48;
maxtermcats = 4;
max_menue_lines = 12;
max_select_options = 16;
maxacsel = 48;
maxclients = 5;
maxmethods = 26;
maxdaysloaded = 5;
max_search_sets_in_list = 24;
max_prcon2_printers = 20;
max_costno_grupps = 178;
maxtp = 50;
maxtimepairs = 16;
maxtimepairs_per_record = 4;
maxtimepair_records = 4;
maxtimes_booked = 32;
max_daily_inc_corrections = 40;
maxtimes = 64;
maxcodes = 32;
maxrecords = 8;
maxcorr_incomes = 10;
maxvacations = 4;
maxzones = 64;
max_no_of_balances = 24;
max_no_of_oem_balances = 8;
max_no_of_inc_groups = 8;
max_inc_per_group = 5;
first_no_basic_bal_ids = 33;
max_no_basic_bal_ids = 48;
max_sel_inc_per_group = 10;
max_no_sel_inc_bal_ids = 98;
max_empl_corr_values = 4;
max_comp_balances = 40;
adjust_rule_tab_len = 72;
mcconf_rule_tab_len = 5;
first_memory = 224;
last_memory = 233;
rule_applies = 'y';
rule_applies_not = 'n';
rule_applies_possibly = ' ';
max_query_records = 15;
query_def_len = 200;
queryt_def_len = 3000;
max_fields_per_record = 234;
max_selected_fields = 500;
selection_line_len = 2000;
max_winc_incomes = 36;
max_winc_codeids = 70;
max_winc2_codeids = 109;
max_winc_dayincs = 10;
max_standard_cols = 10;
no_value = chr (3);
total_value = chr (4);
planned_days_value = chr (5);
current_value = chr (6);
target_value = chr (7);
created_until_value = chr (8);
vl_max_cols = 32;
vl_dinfo_string_len = 30;
vl_output_line_len = 78;
vl_index_output_line_len = 75;
vl_filler_var_len = 3000;
dslang_key_len = 64;
dslang_year_offset = 2000;
dbg_option_default = 0;
dbg_append = 1;
dbg_timestamp = 2;
dbg_header = 4;
dbg_force_flush = 8;
dbg_indent = 16;
dbg_lineheader = 32;
dbg_max_sessions = 100;
dbg_max_levels = 10;
dbg_max_progs = 100;
dbg_default = 1;
dbg_debug = 2;
dbg_dump = 3;
dbg_deep_dump = 4;
dbg_special = 5;
maxprogs = 250;
awtime_max_projects_per_day = 69;
select_emplno = 'e';
select_deptno = 'd';
select_employcat = 'E';
select_name = 'n';
select_factory = 'f';
select_costid = 'c';
select_no_sort = 'q';
select_mach_job = 'm';
max_installations = 32;
max_zones = 200;
tps_per_access_range = 3;
startup_string_len = 512;
parm_tab_len = 24;
parm_tab_line_len = 60;
no_export = 'n';
pids = 'y';
siport = 's';
siport_badgeno_len = 14;
CreateTimeZone = 'c';
DeleteTimeZone = 'd';
CreatePers = 'p';
DeletePers = 'l';
SetPersInfo = 'i';
SetDayType = 's';
AccessPers = 'a';
RefreshAllTimeZones = 'r';
RefreshAllDayType = 't';
RefreshAllPers = 'f';
NoAction = ' ';
maxdefault_time_incomes = 4;
maxcostbookings = 32;
maxcostbookingst = 64;
center = 'c';
right = 'r';
left = 'l';
justify_center = 'c';
justify_right = 'r';
justify_left = 'l';
prt_messages = 'p';
prt_errors = 'q';
ret_errors = 'r';
lm_normal = 'n';
lm_change = 'c';
lm_update = 'u';
lm_insert = 'i';
lm_old = ' ';
lm_lsg = 'd';
orderno_len = 16;
fault_reasonid_len = 4;
prod_operationid_len = 4;
piece_work_type_len = 4;
fkey_maxpages = 3;
fkey_maxlines = 6;
fkey_maxcols = 4;
fkey_max_per_page = 8;
unused_return_value = 0;
request_ok = 1;
request_error = 2;
invalid_badgeno = 3;
invalid_costid = 4;
booking_in_core_time = 5;
last_going_automatic = 6;
invalid_pin_code = 7;
invalid_terminal = 8;
unsufficient_allowance = 9;
invalid_emplno = 10;
rc_dummy5 = 11;
rc_dummy6 = 12;
rc_dummy7 = 13;
rc_dummy8 = 14;
rc_dummy9 = 15;
rc_dummy10 = 16;
unknown_error = 17;
not_valid_input = 18;
not_valid_master_data = 19;
no_sfc_error = 20;
more_employees_than_planned = 21;
more_prod_steps_than_planned = 22;
no_next_pstep_found = 23;
not_in_tolerances = 24;
unexpected_machine = 25;
booking_forced = 26;
less_employees_than_planned = 27;
prod_step_overlap = 28;
cost_booking_failed = 29;
employee_already_registrated = 30;
not_all_employees_have_completed = 31;
not_interrupted = 32;
no_timestamp_found = 33;
prod_step_not_active = 34;
prod_step_already_registrated = 35;
prod_step_already_completed = 36;
prod_step_not_registrated = 37;
sequence_error = 38;
tolerance_exceeded = 39;
employee_has_already_completed = 40;
too_many_prod_steps = 41;
too_many_employees = 42;
employee_not_on_prod_step = 43;
unknown_machine = 44;
unknown_message_type = 45;
unknown_request_type = 46;
wrong_machine = 47;
unknown_sfc_badgeno = 48;
unknown_prod_step = 49;
unknown_order = 50;
too_less_employees = 51;
prod_status_is_hold = 52;
sfc_queue_error = 53;
employee_not_registrated = 54;
sfc_file_error = 55;
already_interrupted = 56;
already_resumed = 57;
sfc_booking_ignored = 58;
unknown_fault_reason = 59;
too_many_messages_per_second = 60;
undefined_sfc_code = 61;
employee_on_too_many_machines = 62;
no_sfc_permission = 63;
created_employee_completion = 64;
auto_empl_completion_failed = 65;
machine_overlap = 66;
sfc_booking_while_absent = 67;
employee_on_too_many_psteps = 68;
special_interruption = 69;
no_sfc_info_error = 70;
inherit_quantity_active = 71;
predecessors_not_completed = 72;
predecessors_completed = 73;
unknown_machine_order = 74;
undefined_parallelity = 75;
quantity_changed = 76;
no_space_for_splitting = 77;
msg_type_not_allowed = 78;
employee_not_valid = 79;
auto_empl_registration_failed = 80;
employee_not_startup_registrated = 81;
employee_already_startup_registrated = 82;
quantity_change_failed = 83;
quantity_too_small = 84;
more_costtypes = 85;
other_msg_type_expected = 86;
offline_booking = 87;
no_answer_from_terminal = 88;
responsibility_change_request = 89;
responsibility_change_successful = 90;
responsibility_table_error = 91;
sfc_custom_error = 100;
check_balance_base = 2140000000;
ts_req_msgno = 1;
ts_rpl_msgno = 2;
read_reqno = 1;
more_reqno = 2;
change_reqno = 3;
print_reqno = 4;
demp_download_reqno = 5;
delete_reqno = 6;
get_codeid_reqno = 10;
get_wpatid_reqno = 11;
get_special_service_reqno = 12;
load_abbr_list_reqno = 13;
get_vemp_reqno = 14;
get_mach_job_reqno = 15;
get_cycleid_reqno = 16;
fill_timepair_info_reqno = 17;
update_dscalen_tab_reqno = 18;
read_abilities_reqno = 19;
read_requirements_reqno = 20;
load_empl_data_reqno = 21;
load_external_empl_reqno = 22;
load_empl_list_reqno = 23;
load_reassigned_empls_reqno = 24;
write_schedule_reqno = 25;
get_accesses_reqno = 26;
load_staff_list_reqno = 27;
get_wpatid_from_cycle_reqno = 28;
load_empl_djob_data_reqno = 29;
write_djob_tab_reqno = 30;
load_squal_list_reqno = 31;
nvvlist_reqno = 32;
get_caschid_reqno = 33;
write_staff_tab_reqno = 34;
save_cascw_tab_reqno = 35;
save_cascj_tab_reqno = 36;
get_cascjid_reqno = 37;
get_cascwid_reqno = 38;
get_dscols_reqno = 39;
get_ruleid_reqno = 40;
get_casceid_reqno = 41;
read_dscalen_reqno = 42;
read_dssgnoff_reqno = 43;
write_dssgnoff_reqno = 44;
update_comms_reqno = 46;
dkeys_abbr_tab_len = 500;
function_code_len = 1;
change_insert = 'i';
change_update = 'u';
change_delete = 'd';
change_copy = 'c';
change_rename = 'r';
change_copy_new = 'n';
change_insert_new = 'w';
variants_current = 'c';
variants_newest = 'n';
variants_future = 'f';
variants_all = 'a';
variants_oneday = 'h';
variants_onlyone = 'o';
acalen_msg_ct = 80;
acalen_max_lines = 160;
accemp_rpl_ct = 30;
anlbal_rpl_ct = 221;
atcomp_rpl_ct = 39;
awtime_rpl_ct = 31;
awtime_comment_rpl_ct = 44;
awtime_task_rpl_ct = 78;
bookmsg_rpl_ct = 16;
bmsg_rpl_ct = bookmsg_rpl_ct;
ccomp_rpl_ct = 45;
comm_rpl_ct = 36;
comm_update_ct = 50;
corder_rpl_ct = 11;
cols_rpl_ct = 18;
cplan_rpl_ct = 27;
cplane_rpl_ct = cplan_rpl_ct;
cplanm_rpl_ct = cplan_rpl_ct;
cplanp_rpl_ct = 28;
cusers_rpl_ct = 11;
dabsemp_rpl_ct = 14;
dabsgrp_rpl_ct = 12;
dacce_rpl_ct = 100;
daccv_rpl_ct = 30;
daccz_rpl_ct = 49;
dcbal_rpl_ct = 22;
dcost_rpl_ct = 45;
demp_rpl_ct = 18;
demp_download_ct = 225;
derrors_rpl_ct = 16;
dfrsn_rpl_ct = 49;
dkeys_rpl_ct = 20;
dmonth_rpl_ct = 40;
dmsg_rpl_ct = 26;
dorder_rpl_ct = 44;
dperror_rpl_ct = 39;
dpres_rpl_ct = 19;
dprint_rpl_ct = 12;
dprint2_rpl_ct = 3540;
dpsopn_rpl_ct = 60;
dpstep_rpl_ct = 66;
dpwarn_rpl_ct = 22;
dscom_rpl_ct = 50;
dscom_update_ct = 50;
dsplan_rpl_ct = 28;
dstatus_rpl_ct = 58;
dswarn_rpl_ct = 12;
dvisit_rpl_ct = 34;
dwarn_rpl_ct = 12;
dzone_rpl_ct = 45;
empvar_rpl_ct = 59;
emplac_rpl_ct = 60;
lmsg_rpl_ct = bookmsg_rpl_ct;
lpstep_rpl_ct = 48;
ltime_rpl_ct = 17;
ltime_update_ct = 60;
mcbal_rpl_ct = 15;
menue_rpl_ct = 40;
monov_rpl_ct = 40;
msg_rpl_ct = 8;
order_rpl_ct = 50;
pbar_rpl_ct = 30;
plan_rpl_ct = 13;
plan_tab_max = 27;
pconf_rpl_ct = 12;
pconf_update_ct = 24;
pnet_rpl_ct = pbar_rpl_ct;
ppbook_line_ct = 130;
ppbook2_line_ct = 59;
ppord_rpl_ct = 64;
prstat_rpl_ct = 17;
prstat_dsp_rpl_ct = 3660;
pscap_rpl_ct = 80;
psprod_rpl_ct = 80;
pstruc_rpl_ct = 23;
pwbal_rpl_ct = 12;
query_rpl_ct = 25;
queryd_rpl_ct = 36;
dscost_rpl_ct = 18;
tstat_rpl_ct = 36;
upstep_rpl_ct = 50;
vds_rpl_ct = 49;
recobh_rpl_ct = 58;
smsg_rpl_ct = 75;
time_rpl_ct = 1;
ttline_rpl_ct = 13;
wcycst_rpl_ct = 30;
chlog_rpl_ct = 70;
rcalen_rpl_ct = 12;
mplace_rpl_ct = 2;
comm_text_buffer_len = 1080;
max_fields_per_mask = 200;
term_ct = 10;
job_rpl_ct = 20;
pemploy_rpl_ct = 10;
reass_rpl_ct = 24;
reass_rpl3_ct = 10;
reass_upd_ct = 15;
reass_cqg_ct = 199;
aemploy_rpl_ct = 10;
avemp_rpl_ct = 60;
res_rpl_ct = 57;
profil_rpl_ct = 59;
max_pmreq_tab_len = 68;
statusid_len = 8;
schedu_rpl_ct = 2;
opspla_rpl_ct = 2;
emsche_rpl_ct = 10;
ts_schedu_load_abbr_list_ct = 25;
ts_schedu_read_able_ct = 40;
ts_schedu_read_reqs_ct = 10;
ts_schedu_load_empl_list_ct = 8;
ts_schedu_load_emplno_list_ct = 200;
ts_schedu_max_empls_ct = 250;
ts_schedu_max_jobs_ct = 64;
ts_schedu_load_reass_empls_ct = 7;
ts_schedu_load_empl_data_ct = 20;
ts_schedu_write_schedule_ct = 10;
schedu_max_conflicts = 20;
ts_schedu_max_staff_ct = 10;
schedu_sums_tab_len = 403;
ts_schedu_load_djob_ct = 140;
ts_scheds_max = 96;
max_comms_for_opspla = 18;
casch_rpl_ct = 50;
wasc_rpl_ct = 19;
wasc_page_length = 50;
cascj_tab_len = 200;
cascw_tab_len = 300;
NUMBERBALANCE = 8;
numbalance = 8;
numb_breaks = 3;
MAX_ADP_VALUE = 1000;
ADP_ERROR_BASIC_VALUE = 600;
ADP_MESSAGE_BASIC_VALUE = 600;
ADP_WARNING_BASIC_VALUE = 69;
MAX_CHAR_REASON_OF_ABSENCE = 2;
djob_rpl_ct = 0;
ts_schedu_write_djob_tab_ct = 100;
squal_rpl_ct = 10;
nvvlist_rpl_ct = 128;
nvvlist_req_ct = 875;
mode_dkeys = 'k';
mode_prog = 'p';
action_getinfo = 'g';
action_dinfo = '0';
action_new = '1';
action_unselect = 'u';
action_dkeys = 'k';
action_saved = 'a';
action_quit = 'v';
action_select = 's';
action_update = '2';
action_copy = '3';
action_delete = '4';
action_info = '5';
action_print = '6';
action_rename = '7';
action_modify = '8';
action_mpdata = 'm';
action_change_all = '9';
action_plan = 'p';
action_cancel = 'q';
action_smach = 'S';
action_restore = 'r';
action_copy_order = 'z';
action_booktime = 't';
action_msg = 'i';
action_dpstep = 'd';
action_add_pstep = 'e';
action_remove_pstep = 'f';
action_start = 'w';
action_ppbook = 'P';
action_startterm = 'b';
action_stopterm = 'c';
action_download = 'h';
action_balload = 'j';
action_starttrace = 'l';
action_stoptrace = 'n';
action_timesync = 'o';
action_dmsg = 'D';
action_line_len = 76;
attribute_normal = 'n';
attribute_hide = 'h';
attribute_grey = 'g';
attribute_selected = 's';
attribute_gridlines = 'q';
attribute_colored = 'c';
attribute_noedit = 'e';
invert_flag = 'i';
cit_yesterday = 0;
cit_today = 1;
modify_deleted = 0;
modify_original = 1;
modify_inserted = 2;
modify_by_empl = 3;
modify_changed = 4;
modify_by_job = 10;
modify_by_timepairs = 11;
deptno_error_index = 32766;
employcat_error_index = 32765;
costid_error_index = 32764;
select_options_error_index = 32763;
short_table_len = 64000;
index_table_len = 4000;
dscode_key_primary = 'p';
dscode_key_scodeid_holiday = 'h';
dscode_key_scodeid_day_off = 'd';
dscode_key_scodeid_workday = 'w';
dscode_key_holiday_keys = 'H';
dscode_key_day_off_keys = 'D';
dscode_key_workday_keys = 'W';
dscode_key_all_keys = 'A';
max_cwrp_days = 400;
cusers_name_len = 30;
max_cols = 4096;
max_bytes_per_table = 5000;
clientno_tab_len = 256;
language_tab_len = 32;
codepage_tab_len = 256;
local_printername = 'LPT';
ul_block_len = 4096;
ul_read_first = 1;
ul_read_last = 2;
ul_read_next = 3;
ul_read_prev = 4;
ul_read_new_pos = 5;
ul_append = 6;
ul_delete_first = 7;
ul_delete_last = 8;
ul_change = 9;
ul_change_new_pos = 10;
ul_create = 11;
ul_destroy = 12;
ix_keys_in_block = 10;
ix_hashtab_mod = 11;
ix_hashtab_max = 10;
ix_position = 1;
ix_read_next = 2;
ix_insert = 3;
ix_create = 4;
ix_destroy = 5;
ix_delete = 6;
open_file_dc = 01;
close_file_dc = 02;
seq_read_dc = 10;
seq_write_dc = 11;
seq_position_dc = 12;
seq_delete_dc = 13;
keyed_read_dc = 20;
keyed_write_dc = 21;
keyed_position_dc = 22;
keyed_delete_dc = 23;
keyed_rewrite_dc = 24;
maxfiles = 200;
minkey_short = -9999;
input = 1;
output = 2;
value_undefined_char = 128;
taris_time_array_len = 8;
no_year_low = 1900;
no_month_low = 1;
no_day_low = 1;
root_year = 1980;
seconds_per_year = 31536000;
seconds_per_hour = 3600;
sysname_len = 10;
min_chars_for_move = 0;
eend_of_file = 1025;
erecord_too_long = 1026;
eobject_not_found = 1032;
einvalid_io_operation = 1040;
eusage_given = 1044;
einvalid_io_type = 1070;
etimeout = 1081;
einvalid_duplicate_key = 1111;
erecord_not_found = 1112;
ealready_locked = 1206;
ecaller_must_wait = 1277;
einvalid_arg = 1371;
eform_aborted = 1453;
ebeginning_of_file = 1773;
erecord_in_use = 2408;
DiskReadError = 100;
DiskWriteError = 101;
FileNotAssignedError = 102;
FileNotOpenError = 103;
FileNotOpenForInputError = 104;
FileNotOpenForOutputError = 105;
InvalidNumericFormatError = 106;
tm_CopyLine = 101;
tm_AppendWin = 102;
tm_RemoveWin = 103;
tm_DKeysReturn = 104;
tm_DeleteRecord = 105;
tm_ReReadList = 106;
tm_Keyboard = 107;
tm_Escape = 108;
tm_Arrows = 109;
tm_GetCurEmplno = 110;
tm_GetPrevEmplno = 111;
tm_GetNextEmplno = 112;
tm_Read = 113;
tm_setFocus = 114;
tm_FkeyPressed = 115;
tm_UpdateData = 116;
tm_SetWinProgno = 117;
tm_WindowsOpen = 118;
tm_MultipleSel = 119;
tm_UpdateMask = 120;
tm_UnselectList = 121;
tm_NewLogon = 122;
tm_SqualReturn = 123;
tm_GetPrognoHandle = 124;
tm_WriteStatic = 125;
tm_SetColor = 126;
tm_GetColor = 127;
tm_OtherKeys = 128;
tm_GetProgno = 129;
tm_ReadStatic = 130;
tm_CloseDemp = 131;
tm_CallDemp = 132;
tm_MsgCustLeitz = 133;
tm_PingPong = 199;
OkId = 101;
MoreId = 102;
CancelId = 103;
LessId = 104;
ExecuteId = 104;
DuplicateId = 105;
SkipId = 105;
DeleteId = 106;
SelectId = 107;
NewId = 108;
UpdateId = 109;
AuswahlId = 110;
FarbId = 111;
AktualisierenId = 112;
ReadId = 113;
PrinterId = 114;
AllOkId = 115;
AllSkipId = 116;
tarisAllok = 101;
tarisAllSkip = 102;
PrevEmplnoId = 801;
CurEmplnoId = 802;
NextEmplnoId = 803;
error_text_fname = 'errtxt.dat';
error_index_fname = 'errtxt.ind';
max_message_code = 21999;
index_step = 10;
MCTEntriesPerTable = 21000;
MaxMCTEntries = 80000;
user_opts_versionno = 2;
user_opts_filename = 'Useropt.dat';
bt_alpha = 'a';
bt_short = 'n';
bt_mod_type = 't';
bt_duration = 'p';
bt_dduration = 'v';
bt_long = 'N';
bt_saldo = 's';
bt_dsaldo = 'V';
bt_date = 'd';
bt_time = 'u';
bt_costid = 'c';
bt_flag_type = 'f';
bt_cycle = 'F';
bt_allowance = 'w';
bt_char = 'C';
bt_unsigned_byte = 'b';
bt_balance = 'B';
bt_mptime = 'm';
bt_dbalance = 'D';
bt_sreal = 'S';
bt_ireal = 'I';
bt_prod_stepno = 'P';
bt_timearray = '[';
bt_ogpdraw = ']';
bt_dcbal = 'l';
bt_owndraw = 'o';
bt_color = 'r';
bt_percentage = 'G';
bt_emplcorrvalue = 'H';
bt_specialsaldo = 'J';
bt_effort = 'K';
bt_pduration = 'L';
max_intervals = 80;
nii = -10001;
LptDev = 1;
ScreenDev = 2;
FileDev = 3;
NoDongle = 1000;
time_apply = 240;
first_allowance_apply = 241;
last_allowance_apply = 245;
max_gacce_emplnos = 240;
matchcode_table_size = 4;
matchcode_table_text_len = 100;
max_no_of_bal_rec = 13;
tree_plus = '+';
tree_minus = '-';
tree_blank = ' ';
indentation_const = 12;
benz_did_len = 200;
general_base_module = 1;
taris_base_module = 2;
tse_base_module = 3;
general_income_module = 11;
taris_income_module = 12;
tse_income_module = 13;
general_access_module = 21;
taris_access_module = 22;
tse_access_module = 23;
general_pop_module = 31;
taris_pop_module = 32;
tse_pop_module = 33;
general_cost_module = 41;
taris_cost_module = 42;
tse_cost_module = 43;
general_ttbal_module = 51;
taris_ttbal_module = 52;
tse_ttbal_module = 53;
general_statistik_module = 61;
taris_statistik_module = 62;
tse_statistik_module = 63;
general_bde1_module = 71;
taris_bde1_module = 72;
tse_bde1_module = 73;
general_bde2_module = 81;
taris_bde2_module = 82;
tse_bde2_module = 83;
general_canteen_module = 91;
taris_canteen_module = 92;
tse_canteen_module = 93;
general_bde3_module = 101;
taris_bde3_module = 102;
tse_bde3_module = 103;
general_win_module = 111;
taris_win_module = 112;
tse_win_module = 113;
general_time_acc_module = 121;
taris_time_acc_module = 122;
tse_time_acc_module = 123;
general_screen_module = 131;
taris_screen_module = 132;
tse_screen_module = 133;
general_online_module = 141;
taris_online_module = 142;
tse_online_module = 143;
general_web_module = 151;
taris_web_module = 152;
tse_web_module = 153;
general_telefon_module = 161;
taris_telefon_module = 162;
tse_telefon_module = 163;
general_time_acc2_module = 171;
taris_time_acc2_module = 172;
tse_time_acc2_module = 173;
general_planner_module = 181;
taris_planner_module = 182;
tse_planner_module = 183;
general_analysis_module = 191;
taris_analysis_module = 192;
tse_analysis_module = 193;
general_mobile_module = 201;
taris_mobile_module = 202;
tse_mobile_module = 203;
general_aeneis_module = 211;
taris_aeneis_module = 212;
tse_aeneis_module = 213;
tse_multi_clients_addon = 223;
tse_cycle_addon = 233;
tse_income_addon = 243;
tse_adjust_addon = 253;
tse_apply_and_grant_addon = 263;
taris_installation = 271;
ase_installation = 272;
test_installation = 273;
oem_installation = 274;
botime_installation = 280;
botime_standard_installation = 281;
botime_extended_installation = 282;
botime_enterprise_installation = 283;
palmOk = 0;
palmGeneralError = -1;
palmNoSequence = -2;
palmWrongSequence = -3;
palmNotConnected = -4;
palmUnknownCommand = -5;
palmNoCommand = -6;
palmLoginWithoutUser = -7;
palmLoginConnectionFailed = -8;
palmLogoutDisconnectFailed = -9;
palmBookingFailed = -10;
palmWrongAnswerFromServer = -11;
palmTimeout = -12;
palmFatalError = -13;
palmUnknownEntity = -14;
palm_str_array_len = 16;
palm_act_array_len = 8;
palm_id_array_len = 1024;
palm_flag_del = 1;
palm_flag_sync = 2;
palm_flag_mode = 4;
palm_flag_bearb = 8;
module_table_len = 30;
moduleid_text_len = 40;
start_moduleid_taris231 = 1;
end_moduleid_taris231 = 25;
start_moduleid_ase231 = 32;
end_moduleid_ase231 = 50;
pin_flag_on = 'l';
pin_flag_off = 'a';
NO_SAP = 'n';
KK1 = 'y';
HR_PDC = 'h';
SAP_BC = HR_PDC;
UNKNOWN_DB = 0;
AIX_DB2 = 1;
AIX_ORACLE = 2;
AS400_DB2 = 10;
AS400_DB2_ESQL = 11;
LINUX_DB2 = 20;
LINUX_ORACLE = 21;
NT_DB2 = 30;
NT_ORACLE = 31;
NT_SQLSERVER = 32;
participant_array_len = 32;
dsmdlptp_code_len = 1;
dsmdsum_code_len = 17;
hex_block_len = 16;
byte_block_len = 8;
minutes_per_day = 1440;
tslicence_read_dsmdsum = 1;
tslicence_read_key = 2;
tslicence_read_timer = 3;
tslicence_get_debug = 4;
tslicence_set_debug = 5;
tslicence_kill_server = 6;
tslicence_for_javaproxy = 7;
no_of_readers = 24;
benz_readers = 16;
max_conv_tab_value = 16364;

max_conv_tab_value_short = 8182;
maxctcustbuffersize = 32000;


TYPE

{pspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspsp}
   {short.inc}
short = -32768..32767;
ulong = longword;

{pspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspsp}
   {types.inc}
long_text_type = ARRAY [1..long_text_len] OF char;
very_long_text_type = ARRAY [1..60] OF char;
objectid_type = ARRAY [1..objectid_len] OF char;
objecttype_type = ARRAY [1..objecttype_len] OF char;
flag_type = char;
indexname_type = ARRAY [1..32] OF char;
sqlstate_type = ARRAY [1..6] OF char;
error_text_type = ARRAY [1..error_text_len] OF char;
perror_text_type = ARRAY [1..perror_text_len] OF char;
repvar_type = ARRAY [1..max_repvar_len] OF char;
timer_text_type = ARRAY [1..80] OF char;
languageid_type = ARRAY [1..2] OF char;
packed_1_type = ARRAY [1..1] OF char;
packed_2_type = ARRAY [1..2] OF char;
packed_3_type = ARRAY [1..3] OF char;
packed_4_type = ARRAY [1..4] OF char;
packed_5_type = ARRAY [1..5] OF char;
packed_6_type = ARRAY [1..6] OF char;
packed_7_type = ARRAY [1..7] OF char;
packed_8_type = ARRAY [1..8] OF char;
packed_9_type = ARRAY [1..9] OF char;
packed_10_type = ARRAY [1..10] OF char;
packed_11_type = ARRAY [1..11] OF char;
packed_12_type = ARRAY [1..12] OF char;
packed_13_type = ARRAY [1..13] OF char;
packed_14_type = ARRAY [1..14] OF char;
packed_15_type = ARRAY [1..15] OF char;
packed_16_type = ARRAY [1..16] OF char;
packed_20_type = ARRAY [1..20] OF char;
packed_21_type = ARRAY [1..21] OF char;
packed_22_type = ARRAY [1..22] OF char;
packed_23_type = ARRAY [1..23] OF char;
packed_24_type = ARRAY [1..24] OF char;
packed_25_type = ARRAY [1..25] OF char;
packed_26_type = ARRAY [1..26] OF char;
packed_27_type = ARRAY [1..27] OF char;
packed_28_type = ARRAY [1..28] OF char;
packed_29_type = ARRAY [1..29] OF char;
packed_30_type = ARRAY [1..30] OF char;
packed_32_type = ARRAY [1..32] OF char;
packed_40_type = ARRAY [1..40] OF char;
packed_42_type = ARRAY [1..42] OF char;
packed_48_type = ARRAY [1..48] OF char;
packed_50_type = ARRAY [1..50] OF char;
packed_52_type = ARRAY [1..52] OF char;
packed_60_type = ARRAY [1..60] OF char;
packed_64_type = ARRAY [1..64] OF char;
packed_75_type = ARRAY [1..75] OF char;
packed_80_type = ARRAY [1..80] OF char;
packed_120_type = ARRAY [1..120] OF char;
packed_252_type = ARRAY [1..252] OF char;
packed_256_type = ARRAY [1..256] OF char;
packed_32000_type = ARRAY [1..32000] OF char;
pc_date_time_type = ARRAY [1..17] OF char;
a_char = char;
wpatid_type = ARRAY [1..4] OF char;
codeid_type = packed_2_type;
long = LongInt;
SYSNAME = ARRAY [1..sysname_len] OF char;
dummy_type = short;
fkey_mask_type = ARRAY [1..32] OF char;
message_type = ARRAY [1..80] OF char;
dcindexname_type = indexname_type;
function_type = RECORD
{    1}  operation_id: short;
{    3}  position_id: short;
{    5}  read_type: char
{=   5}  END;
qcode_type = ARRAY [1..2] OF char;
status_type = ARRAY [1..76] OF char;
index_array = ARRAY [0..2199] OF RECORD
{    1}  first: short;
{    3}  pos: short
{=8800}  END;
PCopyLineRec = ^TCopyLineRec;
TCopyLineRec = RECORD
{    1}  Id: short;
{    3}  FormatNo: short;
{    5}  RecAddr: long
{=   8}  END;
mct_pt = ^mct_type;
mct_entry = RECORD
{    1}  n: short;
{    3}  t: char
{=   3}  END;
mct_type = ARRAY [1..MCTEntriesPerTable] OF mct_entry;
ccb_type = RECORD
{    1}  timeout_used: long;
{    5}  rc: short;
{    7}  time_array: ARRAY [1..8] OF char;
{   15}  alignment1: ARRAY [1..2] OF char;
{   17}  timeout_planned: long;
{   21}  portid: short;
{   23}  event_flag: flag_type;
{   24}  commtype: char;
{   25}  used_protocol: char;
{   26}  charset_conv_flag: flag_type;
{   27}  buflen: short;
{   29}  hostname: ARRAY [1..32] OF char;
{   61}  programname: ARRAY [1..32] OF char;
{   93}  conversationid: long;
{   97}  rc1: long;
{  101}  rc2: long;
{  105}  timeout_std: long;
{  109}  {event_handle: hWnd;}
{  110}  cci_status: char;
{  111}  use_cci_header_flag: flag_type;
{  112}  clientname: ARRAY [1..32] OF char;
{  144}{  svc_stop_evt: THandle;}
{  145}{  cci_stop_evt: THandle;}
{  146}{  cci_write_evt: THandle;}
{  147}{  cci_read_pending_evt: THandle;}
{  148}{  read_thread: THandle;}
{  149}  cci_id: long
{= 152}  END;
user_opts_type = RECORD
{    1}  version: short;
{    3}  debug_flag: flag_type;
{    4}  mask_names_flag: flag_type;
{    5}  old_menu_flag: flag_type;
{    6}  alignment1: char;
{    7}  debug_level: short
{=   8}  END;
objectid_s_type = string;
objecttype_s_type = string;
long_text_s_type = string;
indexname_s_type = string;
repvar_s_type = string;
error_text_s_type = string;
wpatid_s_type = string;
codeid_s_type = string;
frsn_index_type = STRING [20];
search_set_type = STRING [search_set_len];
pcb_type = RECORD
{    1}  line_cnt: long;
{    5}  page_cnt: long;
{    9}  page_len: short;
{   11}  after: short;
{   13}  f: file
{=  13}  END;
order_by_type = ARRAY [1..256] OF char;
interval_range = INTEGER;
interval_type = ARRAY [1..2] OF interval_range;
TIntervalData = ARRAY [0..max_intervals] OF interval_type;
TColorData = ARRAY [1..max_intervals] OF long;
processid_type = ulong;
buffer_type = ARRAY [1..2] OF char;
timestamp = long;
rec_header_type = RECORD
{    1}  byte: buffer_type;
{    3}  reference_count: short;
{    5}  last_modified: timestamp
{=   8}  END;
fcb_type = RECORD
{    1}  accessno: short;
{    3}  last_error: short;
{    5}  last_msgid: long;
{    9}  io_type: short;
{   11}  filename: objectid_type;
{   19}  filler: short
{=  20}  END;
schedu_abbr_type = ARRAY [1..2] OF char;
vl_cols_dinfo_tab_type = RECORD
{    1}  pos: short;
{    3}  len: short;
{    5}  col_dist: short;
{    7}  dinfo: short
{=   8}  END;
vl_cols_dinfo_type = RECORD
{    1}  tab: ARRAY [1..vl_max_cols] OF vl_cols_dinfo_tab_type;
{  257}  no_cols: short
{= 258}  END;
vl_output_line_type = ARRAY [1..vl_output_line_len] OF char;
vl_index_output_line_type = ARRAY [1..vl_index_output_line_len] OF char;
vl_rec_type_tab_record_type = RECORD
{    1}  rec_type: char;
{    2}  justification: char
{=   2}  END;
vl_rec_type_tab_type = ARRAY [1..vl_max_cols] OF vl_rec_type_tab_record_type;
vl_filler_var = ARRAY [1..vl_filler_var_len] OF char;
vl_colsusage_type = ARRAY [1..vl_max_cols] OF boolean;
unsigned_byte = char;
signed_byte = char;
filler_type = unsigned_byte;
color_type = long;
color_attr_type = RECORD
{    1}  color: color_type;
{    5}  attr: unsigned_byte;
{    6}  filler_ca: ARRAY [1..3] OF filler_type
{=   8}  END;
one_masks_colors_type = ARRAY [1..10] OF color_attr_type;
special_colors_type = ARRAY [1..16] OF color_attr_type;
emplcorrvalue_type = long;
specialsaldo_type = short;
percentage_type = short;
phoneno_type = ARRAY [1..24] OF char;
url_type = ARRAY [1..254] OF char;
alias_type = ARRAY [1..15] OF char;
commtype_type = ARRAY [1..4] OF char;
firstname_type = ARRAY [1..firstname_len] OF char;
surname_type = ARRAY [1..surname_len] OF char;
email_address_type = ARRAY [1..32] OF char;
m_timestamp = long;
mod_type = short;
duration_type = short;
dduration_type = short;
dbalance_type = duration_type;
allowance_type = duration_type;
saldo_type = long;
dsaldo_type = long;
balance_type = saldo_type;
chapter_type = ARRAY [1..chapter_len] OF char;
phys_filename_type = ARRAY [1..phys_filename_len] OF char;
print_queue_name_type = ARRAY [1..32] OF char;
expanded_filename_type = ARRAY [1..256] OF char;
processname_type = ARRAY [1..processname_len] OF char;
code_type = ARRAY [1..code_len] OF char;
exit_code_type = ARRAY [1..exit_code_len] OF char;
vos_modulename_type = ARRAY [1..vos_modulename_len] OF char;
userid_type = ARRAY [1..userid_len] OF char;
time_zone_type = ARRAY [1..time_zone_len] OF char;
fkey_mask_array = ARRAY [1..32] OF char;
fkey_set_type = SET OF 1..32;
fkey_subst_type = ARRAY [1..32] OF short;
fkey_tab_type = ARRAY [-1..32] OF short;
char_set_type = SET OF char;
operator_type = (add, subtract);
encode_decode_type = (encode, decode);
return_value_type = (undefined, good_value, bad_value);
pcs_request_type = ARRAY [1..pcs_request_len] OF char;
codetype_type = char;
termcat_type = packed_8_type;
logonid_type = ARRAY [1..logonid_len] OF char;
username_type = ARRAY [1..32] OF char;
menue_title_type = ARRAY [1..46] OF char;
menue_line_type = ARRAY [1..50] OF char;
menue_info_type = ARRAY [1..10] OF char;
operationid_type = ARRAY [1..8] OF char;
deptno_type = ARRAY [1..deptno_len] OF char;
emplno_type = ARRAY [1..emplno_len] OF char;
badgeno_type = ARRAY [1..badgeno_len] OF char;
long_badgeno_type = ARRAY [1..long_badgeno_len] OF char;
select_options_type = ARRAY [1..max_select_options] OF char;
input_line_type = ARRAY [1..input_line_len] OF char;
startup_type = ARRAY [1..startup_string_len] OF char;
cb_state_type = (cb_uninitialized, cb_initialized, cb_open, cb_connected);
object_state_type = short;
object_enum_type = short;
conv_table_type_pointer = ^conv_table_type;
conv_table_type = RECORD
{    1}  int_len: short;
{    3}  ext_len: short;
{    5}  no_of_values: short;
{    7}  cust_start_pos: short;
{    9}  CASE dummy: short OF
{+   2}     0: (value_tab: ARRAY [1..max_conv_tab_value] OF char);
{+   2}     1: (short_tab: ARRAY [1..max_conv_tab_value_short] OF short);
{=16374}  END;
conv_table_common_type = RECORD
{    1}  no_of_tables: short;
{    3}  CASE max_no_of_tables: short OF
{+   2}     0: (index_table: ARRAY [1..index_table_len] OF RECORD
{+   2}           table_name: long_text_type;
{+  32}           filler: short;
{+  34}           start_pos: long
{=14400}           END;);
{+   2}     1: (short_table: ARRAY [1..short_table_len] OF short);
{=14400}  END;
half_conv_table_common_type = RECORD
{    1}  info: ARRAY [1..16000] OF short;
{32001}  conv_key: short
{=32002}  END;
whole_conv_table_common_type = RECORD
{    1}  CASE dummy: long OF
{+   4}     0: (split: ARRAY [1..2] OF half_conv_table_common_type);
{+   4}     1: (normal: conv_table_common_type);
{=14400}  END;
cust_conv_table_common_type = ARRAY [1..cust_conv_table_common_type_len] OF short;
cust_conv_table_type = RECORD
{    1}  no_of_cust_values: short;
{    3}  CASE switch: short OF
{+   2}     0: (value_tab: ARRAY [1..max_conv_tab_value] OF char);
{+   2}     1: (short_tab: ARRAY [1..max_conv_tab_value_short] OF short);
{=16368}  END;
short_pointer = ^short;
convert_pointer_type = RECORD
{    1}  CASE useless: short OF
{+   2}     0: (p1: short_pointer);
{+   2}     1: (p2: conv_table_type_pointer);
{=   8}  END;
cust_conv_table_type_ptr = ^cust_conv_table_type;
cust_convert_pointer_type = RECORD
{    1}  CASE useless: short OF
{+   2}     0: (p1: short_pointer);
{+   2}     1: (p2: cust_conv_table_type_ptr);
{=   8}  END;
video_attributes_type = RECORD
{    1}  mask_title_color: ARRAY [1..1] OF char;
{    2}  title_color: ARRAY [1..1] OF char;
{    3}  input_color: ARRAY [1..1] OF char;
{    4}  help_text_color: ARRAY [1..1] OF char;
{    5}  background_color: ARRAY [1..1] OF char;
{    6}  mask_title_inv_flag: flag_type;
{    7}  mask_title_ul_flag: flag_type;
{    8}  title_ul_flag: flag_type;
{    9}  input_high_flag: flag_type;
{   10}  error_inv_flag: flag_type;
{   11}  filler: ARRAY [1..6] OF char
{=  16}  END;
codes_table_record_type = RECORD
{    1}  color: long;
{    5}  codeid: codeid_type;
{    7}  attr: unsigned_byte;
{    8}  filler: char
{=   8}  END;
codes_table_type = ARRAY [1..31] OF codes_table_record_type;
acccatid_type = ARRAY [1..4] OF char;
accpatid_type = ARRAY [1..4] OF char;
jobid_type = objectid_type;
qualificationid_type = objectid_type;
mach_job_type = RECORD
{    1}  machineid: objectid_type;
{    9}  jobid: objectid_type
{=  16}  END;
abbr_type = ARRAY [1..2] OF char;
abbr_key_type = RECORD
{    1}  deptno: deptno_type;
{   11}  abbr: abbr_type
{=  12}  END;
importance_type = ARRAY [1..1] OF char;
costid_type = ARRAY [1..costid_type_len] OF char;
costid_ext_type = ARRAY [1..costid_ext_type_len] OF char;
cost_bookings_record_type = RECORD
{    1}  switch_time: mod_type;
{    3}  costid: costid_type
{=  22}  END;
cost_bookings_type = ARRAY [1..maxcostbookings] OF cost_bookings_record_type;
cycleid_type = ARRAY [1..4] OF char;
shiftid_type = ARRAY [1..4] OF char;
ruleid_type = ARRAY [1..2] OF char;
employcat_type = ARRAY [1..employcat_len] OF char;
rcalenid_type = ARRAY [1..4] OF char;
event_type = ARRAY [1..2] OF char;
tariffarea_type = ARRAY [1..4] OF char;
costno_type = ARRAY [1..10] OF char;
adjustcat_type = ARRAY [1..adjustcat_len] OF char;
chain_balance_tab_type = ARRAY [1..max_no_of_balances] OF short;
continued_pay_len_type = short;
msg_header_type = RECORD
{    1}  byte: buffer_type;
{    3}  reqno: short;
{    5}  error_code: code_type;
{    9}  vos_error: short;
{   11}  queue_name: ARRAY [1..10] OF char;
{   21}  callerno: short;
{   23}  notify_flag: flag_type;
{   24}  filler1: char;
{   25}  queue_timestamp: timestamp;
{   29}  reserved: short
{=  30}  END;
switch_msg = RECORD
{    1}  taskid: short;
{    3}  objectid: objectid_type;
{   11}  objecttype: objecttype_type
{=  18}  END;
long_msg_header_type = RECORD
{    1}  byte: buffer_type;
{    3}  reqno: short;
{    5}  error_code: code_type;
{    9}  vos_error: short;
{   11}  reserved: ARRAY [1..10] OF short;
{   31}  msgtype: short;
{   33}  switchmsg: switch_msg
{=  50}  END;
file_param_type = RECORD
{    1}  phys_filename: phys_filename_type;
{   33}  organization: short;
{   35}  maxreclen: short;
{   37}  logging_flag: flag_type;
{   38}  logging_align: char;
{   39}  length_in_pages: short;
{   41}  starting_recordno: short;
{   43}  time_limit: long
{=  46}  END;
open_param_type = RECORD
{    1}  io_type: short;
{    3}  locking_mode: short;
{    5}  access_mode: short
{=   6}  END;
string_date_type = ARRAY [1..8] OF char;
string_time_type = ARRAY [1..5] OF char;
date_type = RECORD
{    1}  yy: short;
{    3}  mo: short;
{    5}  dd: short
{=   6}  END;
time_type = RECORD
{    1}  hh: short;
{    3}  mi: short;
{    5}  ss: short
{=   6}  END;
date_and_time_type = RECORD
{    1}  date: date_type;
{    7}  time: time_type
{=  12}  END;
task_context_type = RECORD
{    1}  byte: ARRAY [1..100] OF char
{= 100}  END;
income_type = ARRAY [1..4] OF char;
wrk_code_type = RECORD
{    1}  codeid: codeid_type;
{    3}  codetype: codetype_type
{=   3}  END;
error_log_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  date_and_time: date_and_time_type;
{   21}  millisec: short;
{   23}  error_code: code_type;
{   27}  severity_code: char;
{   28}  filler1: ARRAY [1..1] OF filler_type;
{   29}  userid: userid_type;
{   45}  processname: processname_type;
{   77}  versionno: short;
{   79}  vos_error: short;
{   81}  fileid: objectid_type;
{   89}  error_text: error_text_type;
{  149}  exit_code: exit_code_type;
{  155}  filler: ARRAY [1..2] OF filler_type;
{  157}  counter: long;
{  161}  emplno: emplno_type;
{  171}  filler2: ARRAY [1..20] OF filler_type
{= 190}  END;
error_text_rec = RECORD
{    1}  error_code: code_type;
{    5}  severity_code: char;
{    6}  error_text: error_text_type
{=  65}  END;
recid_type = ARRAY [1..recid_len] OF char;
fieldid_type = ARRAY [1..fieldid_len] OF char;
fieldtype_type = short;
queryid_type = ARRAY [1..queryid_len] OF char;
selection_line_type = ARRAY [1..selection_line_len] OF char;
selected_fields_type = ARRAY [1..max_selected_fields] OF fieldid_type;
month_unsigned_byte_type = ARRAY [1..31] OF unsigned_byte;
month_char_type = ARRAY [1..31] OF char;
month_codes_type = ARRAY [1..31] OF codeid_type;
month_short_type = ARRAY [1..31] OF short;
abs_table_pointer = ^abs_table_type;
abs_table_type = RECORD
{    1}  tab_start: date_type;
{    7}  tab_months: short;
{    9}  offduty_reasons: ARRAY [1..24] OF month_codes_type;
{ 1497}  ondutytypes: ARRAY [1..24] OF month_char_type
{=2240}  END;
ccbal_rule_display_type = RECORD
{    1}  d_method: unsigned_byte;
{    2}  d_corr_value_1: packed_8_type;
{   10}  d_transbalanceid_1: unsigned_byte;
{   11}  d_corr_value_2: packed_8_type;
{   19}  d_transbalanceid_2: unsigned_byte
{=  19}  END;
adjust_date_tab_type = ARRAY [1..adjust_rule_tab_len] OF date_type;
chain_step_tab_type = ARRAY [1..max_no_of_balances] OF short;
memory_tab_type = ARRAY [1..10] OF balance_type;
balance_index_tab_type = ARRAY [1..max_no_of_balances] OF short;
balance_set_type = SET OF 1..max_no_of_balances;
mcbal_sections_type = RECORD
{    1}  first: balance_index_tab_type;
{   49}  last: balance_index_tab_type;
{   97}  mcbal_balances: balance_set_type
{=  97}  END;
rule_apply_type = flag_type;
rule_apply_tab_type = ARRAY [1..adjust_rule_tab_len] OF rule_apply_type;
return_code_type = short;
record_no_type = short;
empl_corr_values_type = ARRAY [1..max_empl_corr_values] OF saldo_type;
cwrp_result_tab_type = RECORD
{    1}  wpatid: wpatid_type;
{    5}  codeid: codeid_type;
{    7}  ruleid: ruleid_type
{=   8}  END;
cwrp_period_result_tab_type = ARRAY [1..max_cwrp_days] OF cwrp_result_tab_type;
attribid_type = ARRAY [1..attribid_len] OF char;
attributes_type = ARRAY [1..max_no_of_attributes] OF attribid_type;
parproid_type = ARRAY [1..parproid_len] OF char;
projectid_type = ARRAY [1..projectid_len] OF char;
rpoolid_type = ARRAY [1..rpoolid_len] OF char;
taskid_type = ARRAY [1..taskid_len] OF char;
versionid_type = ARRAY [1..versionid_len] OF char;
balancetype_array_type = ARRAY [1..4] OF unsigned_byte;
search_list_type = RECORD
{    1}  length: short;
{    3}  search_sets: ARRAY [1..max_search_sets_in_list] OF search_set_type
{=  26}  END;
scb_type = RECORD
{    1}  dsaempfcb: fcb_type;
{   21}  dsvempfcb: fcb_type;
{   41}  last_error: short;
{   43}  best_search_list_length: short;
{   45}  current_search_set: short;
{   47}  key_len: short;
{   49}  last_invalid_from: date_type;
{   55}  last_key: ARRAY [1..30] OF char;
{   85}  last_emplno: emplno_type;
{   95}  use_last_key: flag_type;
{   96}  first_pattern: ARRAY [1..30] OF char;
{  126}  last_pattern: ARRAY [1..30] OF char;
{  156}  best_key: char;
{  157}  sorted_by_name: flag_type;
{  158}  subkey_used: flag_type;
{  159}  filler: ARRAY [1..14] OF char
{= 172}  END;
semp_type = RECORD
{    1}  variant_no: short;
{    3}  from_date: date_type;
{    9}  to_date: date_type;
{   15}  deptno: deptno_type;
{   25}  employcat: employcat_type;
{   29}  costid: costid_type;
{   49}  badgeno: long_badgeno_type;
{   69}  emplno: emplno_type;
{   79}  surname: surname_type;
{  109}  select_options: select_options_type;
{  125}  get_all_selected_variants: flag_type;
{  126}  check_permission: flag_type;
{  127}  employcat_filled: char;
{  128}  check_interval: flag_type;
{  129}  factory: packed_12_type;
{  141}  mach_job: mach_job_type;
{  157}  adjustcat: adjustcat_type;
{  161}  tariffarea: tariffarea_type;
{  165}  cycleid: cycleid_type;
{  169}  cycle_start: short;
{  171}  rpoolid: rpoolid_type;
{  183}  accesscat: acccatid_type;
{  187}  filler: ARRAY [1..14] OF char
{= 200}  END;
sreal_type = short;
ireal_type = long;
prod_stepno_type = long;
orderno_type = ARRAY [1..orderno_len] OF char;
fault_reasonid_type = ARRAY [1..fault_reasonid_len] OF char;
prod_status_type = char;
prod_operationid_type = ARRAY [1..prod_operationid_len] OF char;
piece_work_type_type = ARRAY [1..piece_work_type_len] OF char;
prod_step_key_type = RECORD
{    1}  orderno: orderno_type;
{   17}  prod_stepno: prod_stepno_type;
{   21}  prod_stepno_var: prod_stepno_type
{=  24}  END;
pwtype_key_type = RECORD
{    1}  piece_work_type: piece_work_type_type;
{    5}  invalid_from: date_type
{=  10}  END;
dsinst_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  flags: RECORD
{+   0}     customer_code: char;
{+   1}     costid_to_next_day: flag_type;
{+   2}     update_dsttraf_flag: flag_type;
{+   3}     pcsstat_flag: flag_type;
{+   4}     saldo_ztariff_flag: flag_type;
{+   5}     adj_income_flag: flag_type;
{+   6}     dabsemp_calculation: char;
{+   7}     monthly_schedu: flag_type;
{+   8}     more_costbookings: flag_type;
{+   9}     dabs_change_past: flag_type;
{+  10}     use_3hour_guaranty: flag_type;
{+  11}     scodeid_mtbal: flag_type;
{+  12}     dsaldo_two_digits: flag_type;
{+  13}     unique_badgeno_over_all_clients: flag_type;
{+  14}     plan_info: char;
{+  15}     holiday_single: flag_type;
{+  16}     schedu_info: char;
{+  17}     export_available: char;
{+  18}     zdel_quick: flag_type;
{+  19}     SAP_interface: char;
{+  20}     flag1: flag_type;
{+  21}     flag2: flag_type;
{+  22}     flag3: flag_type;
{+  23}     filler: ARRAY [1..9] OF filler_type
{=  32}     END;
{   41}  country: char;
{   42}  overtime_1_2_method: char;
{   43}  auto_badgeno: long_badgeno_type;
{   63}  sfc_installed: flag_type;
{   64}  prw_installed: flag_type;
{   65}  medical_solution_installed: flag_type;
{   66}  canteen_installed: flag_type;
{   67}  sfm_installed: flag_type;
{   68}  client_installed: flag_type;
{   69}  proj_management_installed: flag_type;
{   70}  timeclt_installed: flag_type;
{   71}  type_of_pps: char;
{   72}  sfc_costid_usage: char;
{   73}  absence_balance: short;
{   75}  time_lost_balance: short;
{   77}  missing_coretime_balance: short;
{   79}  fkey_tab: fkey_tab_type;
{  147}  companyname: ARRAY [1..70] OF char;
{  217}  basis_module_installed: flag_type;
{  218}  income_calc_installed: flag_type;
{  219}  access_control_installed: flag_type;
{  220}  pop_installed: flag_type;
{  221}  cost_acc_installed: flag_type;
{  222}  ttbal_installed: flag_type;
{  223}  p3_installed: flag_type;
{  224}  statistic_installed: flag_type;
{  225}  severity_codes: ARRAY [1..maxseverity_codes] OF char;
{  233}  beep_severity_codes: ARRAY [1..maxseverity_codes] OF char;
{  241}  timer: ARRAY [1..8] OF char;
{  249}  balance_type_tab: ARRAY [1..max_no_of_balances] OF char;
{  273}  sfc_auto_registration: char;
{  274}  sfc_auto_completion: char;
{  275}  install_flag17: flag_type;
{  276}  install_flag18: flag_type;
{  277}  install_flag19: flag_type;
{  278}  install_flag20: flag_type;
{  279}  keep_time: short;
{  281}  aemp_time: short;
{  283}  past_income_change_flag: flag_type;
{  284}  ztariff_v130_mode: char;
{  285}  today_date: date_type;
{  291}  dabsgrp_display_mode: char;
{  292}  ztariff_optimization: char;
{  293}  cost_pos_to_no: ARRAY [1..3] OF short;
{  299}  cost_no_to_pos: ARRAY [1..3] OF short;
{  305}  cost_maxlen: ARRAY [1..3] OF short;
{  311}  cost_prestype: ARRAY [1..3] OF char;
{  314}  sfc_variable_registration_mode: char;
{  315}  sfc_auto_completion_mode: char;
{  316}  flag4: flag_type;
{  317}  cost_delimiter: ARRAY [1..2] OF char;
{  319}  cost_alt_delimiter: ARRAY [1..2] OF char;
{  321}  install_flag21: flag_type;
{  322}  install_flag22: flag_type;
{  323}  install_flag23: flag_type;
{  324}  valid_for_overtime: flag_type;
{  325}  real_worktime: flag_type;
{  326}  sickness_codeid2: codeid_type;
{  328}  install_flag24: flag_type;
{  329}  sfc_apply_pwtype_daily: flag_type;
{  330}  cost_checks: ARRAY [1..3] OF flag_type;
{  333}  menue_mode: char;
{  334}  codeid_worked: codeid_type;
{  336}  codeid_day_off: codeid_type;
{  338}  day_filler: codeid_type;
{  340}  day_invalid: codeid_type;
{  342}  real_delimiter: char;
{  343}  video_attributes: video_attributes_type;
{  359}  week_first_day_ztariff: char;
{  360}  codeid_no_permission: codeid_type;
{  362}  sickness_codeid: codeid_type;
{  364}  sfc_parallelity_mode: char;
{  365}  sfc_cost_priority: flag_type;
{  366}  sfc_send_time_to_pps: char;
{  367}  sfc_calc_partial_ps: flag_type;
{  368}  change_from_date_text: packed_16_type;
{  384}  tm_change_from_date_text: packed_16_type;
{  400}  tarislib1: ARRAY [1..10] OF char;
{  410}  tarislib2: ARRAY [1..10] OF char;
{  420}  tarislib3: ARRAY [1..10] OF char;
{  430}  use_prtempday: flag_type;
{  431}  digits_from_orderno: short;
{  433}  digits_from_prod_stepno: short;
{  435}  pos_from_orderno: short;
{  437}  basic_income: income_type;
{  441}  prw_costtype: costid_type;
{  461}  sfc_time: short;
{  463}  v180_inst_date: date_type;
{  469}  v220_inst_stamp: timestamp;
{  473}  v230_inst_stamp: timestamp;
{  477}  v240_inst_stamp: timestamp;
{  481}  bankholtype: ARRAY [3..maxdaytypes] OF char;
{  489}  balance_calc_tab: ARRAY [1..max_no_of_balances] OF char;
{  513}  plan_time: short;
{  515}  warn_time: short;
{  517}  acclog_time: short;
{  519}  errlog_time: short;
{  521}  daily_delimiter: char;
{  522}  unit_continued_payment: char;
{  523}  duration_continued_payment: short;
{  525}  codes_continued_payment: ARRAY [1..5] OF codeid_type;
{  535}  vacation_balance: short;
{  537}  negative_time_border: mod_type;
{  539}  primary_languageid: languageid_type;
{  541}  balance_order_tab: ARRAY [1..max_no_of_balances] OF short;
{  589}  empl_corr_values_type_tab: ARRAY [1..8] OF char;
{  597}  lang_timer: packed_8_type;
{  605}  custdata: long;
{  609}  custdata1: ARRAY [1..96] OF unsigned_byte;
{  705}  min_pwd_len: short;
{  707}  max_logon_tries: short;
{  709}  email_on_request: flag_type;
{  710}  email_on_approval: flag_type;
{  711}  use_break_for_schedu: flag_type;
{  712}  use_break_for_schedud: flag_type;
{  713}  picture_url: url_type;
{  967}  filler3: ARRAY [1..1] OF filler_type;
{  968}  proj_time_acc_installed: flag_type;
{  969}  dsmtbix_projno_flag: flag_type;
{  970}  dsmtbix_costno_flag: flag_type;
{  971}  dsmtbix_cttype_flag: flag_type;
{  972}  dsmtbix_costid_flag: flag_type;
{  973}  regular_worktime: duration_type;
{  975}  empl_cost_factor: sreal_type;
{  977}  commtype_wasc: commtype_type;
{  981}  schedu_balance: short;
{  983}  dsabs_future_months: short;
{  985}  emplnotime_to_pps: char;
{  986}  new_timer: timer_text_type;
{ 1066}  filler1: ARRAY [1..1] OF filler_type;
{ 1067}  pin_valid_len: short;
{ 1069}  max_pin_try: short;
{ 1071}  duration_form: char;
{ 1072}  filler2: ARRAY [1..79] OF filler_type
{=1150}  END;
allowances_type = ARRAY [1..maxallowances] OF allowance_type;
timepairs_record_type = RECORD
{    1}  time_in: mod_type;
{    3}  time_out: mod_type;
{    5}  timepair_code: codeid_type
{=   6}  END;
timepairs_type = ARRAY [1..maxtimepairs] OF timepairs_record_type;
balance_rounding_type = RECORD
{    1}  min_unit: short;
{    3}  round_up: short;
{    5}  minimum: short;
{    7}  filler: short
{=   8}  END;
l_form_type = ARRAY [1..7] OF char;
l_data_type = ARRAY [1..1910] OF char;
l_modes_type = ARRAY [1..500] OF short;
l_message_type = ARRAY [1..78] OF char;
day_emplno_type = RECORD
{    1}  date: date_type;
{    7}  emplno: emplno_type
{=  16}  END;
emplno_day_seq_type = RECORD
{    1}  emplno: emplno_type;
{   11}  date: date_type;
{   17}  sequenceno: short
{=  18}  END;
emplno_day_type = RECORD
{    1}  emplno: emplno_type;
{   11}  date: date_type
{=  16}  END;
emplno_year_type = RECORD
{    1}  emplno: emplno_type;
{   11}  year: short
{=  12}  END;
breakusage_ext_type = ARRAY [1..maxbreaks] OF boolean;
filter_result_type = (yes, no, abort);
progname_type = short;
parm_tab_line_type = ARRAY [1..parm_tab_line_len] OF char;
parm_tab_type = ARRAY [1..parm_tab_len] OF parm_tab_line_type;
pin_code_type = ARRAY [1..4] OF char;
break_pattern_type = RECORD
{    1}  time_from: mod_type;
{    3}  time_to: mod_type;
{    5}  paid_breaklen: duration_type;
{    7}  unpaid_breaklen: duration_type
{=   8}  END;
fkey_line_type = ARRAY [1..78] OF char;
fkcb_type = RECORD
{    1}  pageno: short;
{    3}  no_of_pages: short;
{    5}  fkey_line: ARRAY [1..fkey_maxlines] OF fkey_line_type;
{  473}  line1: fkey_line_type;
{  551}  line2: fkey_line_type
{= 628}  END;
long_record_type = ARRAY [1..long_record_len] OF char;
long_record_ptr_type = ^long_record_type;
mode_type = char;
action_type = char;
action_ctab_type = ARRAY [1..32] OF char;
action_subst_type = ARRAY [1..32] OF short;
action_set_type = SET OF char;
action_line_type = ARRAY [1..action_line_len] OF char;
action_mask_type = RECORD
{    1}  line1: action_line_type;
{   77}  line2: action_line_type
{= 152}  END;
keydata_type = ARRAY [1..keydata_len] OF char;
prog_param_type = RECORD
{    1}  CASE action: action_type OF
{+   1}     action_getinfo: (fileid: ARRAY [1..10] OF char;
{+  11}        title_ctab: long_text_type;
{+  41}        keylen: short;
{+  43}        total_keylen: short;
{+  45}        progno: short;
{+  47}        variants: flag_type;
{+  48}        org_value: keydata_type;
{+ 108}        alignment1: char);
{+   1}     action_update: (keydata: keydata_type;
{+  61}        itemid: short);
{+   1}     action_rename: (old_keydata: keydata_type;
{+  61}        new_keydata: keydata_type);
{= 122}  END;
buffer_pointer = ^buffer_type;
comm_buffer_pointer = ^comm_buffer_type;
comm_buffer_type = ARRAY [1..comm_buffer_len] OF char;
time_array_type = ARRAY [1..time_array_len] OF char;
date_array_type = ARRAY [1..8] OF char;
taris_time_array_type = ARRAY [1..taris_time_array_len] OF char;
cci_header_type = RECORD
{    1}  data_len: long
{=   4}  END;
cusers_name_type = ARRAY [1..cusers_name_len] OF char;
mcb_type = RECORD
{    1}  reserved: short;
{    3}  getcursorpos: short;
{    5}  key_code: short;
{    7}  f4_window: short;
{    9}  redisplay_flag: boolean
{=   9}  END;
mc_param_type = RECORD
{    1}  status: short;
{    3}  message: message_type;
{   83}  beep_flag: boolean
{=  83}  END;
dschlog_data_type = ARRAY [1..4000] OF unsigned_byte;
dschlog_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  date_and_time: date_and_time_type;
{   21}  millisec: short;
{   23}  emplno: emplno_type;
{   33}  recno: record_no_type;
{   35}  first_reclen: short;
{   37}  second_reclen: short;
{   39}  logonid: logonid_type;
{   47}  os_logonid: packed_16_type;
{   63}  terminalname: objectid_type;
{   71}  updatetype: char;
{   72}  filler: ARRAY [1..27] OF char;
{   99}  rec_data_len: short;
{  101}  rec_data: dschlog_data_type
{=4100}  END;
tstat_tabline_type = RECORD
{    1}  termid: objectid_type;
{    9}  termno: short;
{   11}  state: unsigned_byte;
{   12}  term_code: char;
{   13}  phys_termtype: objectid_type;
{   21}  location: ARRAY [1..16] OF char;
{   37}  error: code_type;
{   41}  number_in: long;
{   45}  error_in: long;
{   49}  number_out: long;
{   53}  error_out: long
{=  56}  END;
tstat_tab_type = ARRAY [1..tstat_tab_len] OF tstat_tabline_type;
terminal_state_type = ARRAY [1..2] OF char;
pcs_term_state_type = RECORD
{    1}  serialnumber: ARRAY [1..11] OF char;
{   12}  align1: char;
{   13}  driver: ARRAY [1..4] OF char;
{   17}  bb: ARRAY [1..4] OF char;
{   21}  dispcontrol: ARRAY [1..4] OF char;
{   25}  rtk: ARRAY [1..4] OF char;
{   29}  tcl: ARRAY [1..4] OF char;
{   33}  sterminal: terminal_state_type;
{   35}  sbb: terminal_state_type;
{   37}  sdispcontrol: terminal_state_type;
{   39}  sbbm: ARRAY [1..8] OF terminal_state_type;
{   55}  display: ARRAY [1..80] OF char
{= 134}  END;
print_msg_type = RECORD
{    1}  msg_header: msg_header_type;
{   31}  msgtype: short;
{   33}  command: ARRAY [1..512] OF char;
{  545}  logonid: logonid_type;
{  553}  start_datetime: date_and_time_type;
{  565}  priority: packed_2_type;
{  567}  printerid: objectid_type;
{  575}  printerfile: phys_filename_type;
{  607}  outfile: phys_filename_type;
{  639}  clientno: buffer_type;
{  641}  languageid: languageid_type
{= 642}  END;
staff_key_type = RECORD
{    1}  mach_job: mach_job_type;
{   17}  time_from: mod_type;
{   19}  time_to: mod_type;
{   21}  whole_day: flag_type;
{   22}  link: alias_type;
{   37}  invalid_from: date_type
{=  42}  END;
djob_key_type = RECORD
{    1}  emplno: emplno_type;
{   11}  date: date_type;
{   17}  time: mod_type
{=  18}  END;
table_col_record_type = RECORD
{    1}  datatype: char;
{    2}  alignment1: ARRAY [1..3] OF char;
{    5}  len: long
{=   8}  END;
table_col_type = ARRAY [1..MAX_COLS] OF table_col_record_type;
clientno_tab_type = ARRAY [1..clientno_tab_len] OF buffer_type;
language_tab_type = ARRAY [1..language_tab_len] OF languageid_type;
codepage_tab_type = ARRAY [0..255] OF char;
ul_block_ptr_type = ^ul_block_type;
ul_block_type = RECORD
{    1}  chars: ARRAY [1..ul_block_len] OF char;
{ 4097}  nextpt: ul_block_ptr_type;
{ 4101}  prevpt: ul_block_ptr_type
{=4104}  END;
ul_pos_type = RECORD
{    1}  blockpt: ul_block_ptr_type;
{    5}  recordno: short
{=   6}  END;
ulcb_type = RECORD
{    1}  last_error: short;
{    3}  reclen: short;
{    5}  new_cur_pos: ul_pos_type;
{   11}  filler1: ARRAY [1..2] OF char;
{   13}  no_of_records: long;
{   17}  first_pos: ul_pos_type;
{   23}  filler2: ARRAY [1..2] OF char;
{   25}  last_pos: ul_pos_type;
{   31}  filler3: ARRAY [1..2] OF char;
{   33}  cur_pos: ul_pos_type
{=  38}  END;
ix_block_ptr_type = ^ix_block_type;
ix_pos_type = RECORD
{    1}  blockpt: ix_block_ptr_type;
{    5}  recordno: short
{=   6}  END;
ix_block_type_keytab = RECORD
{    1}  keyno: long;
{    5}  next_pos: ix_pos_type;
{   11}  alignment1: ARRAY [1..2] OF char;
{   13}  data_pos: ul_pos_type
{=  18}  END;
ix_block_type = RECORD
{    1}  keytab: ARRAY [1..ix_keys_in_block] OF ix_block_type_keytab;
{  181}  nextpt: ix_block_ptr_type;
{  185}  prevpt: ix_block_ptr_type
{= 188}  END;
ixcb_type = RECORD
{    1}  last_error: short;
{    3}  alignment1: ARRAY [1..2] OF char;
{    5}  first_pos: ix_pos_type;
{   11}  alignment2: ARRAY [1..2] OF char;
{   13}  last_pos: ix_pos_type;
{   19}  alignment3: ARRAY [1..2] OF char;
{   21}  cur_pos: ix_pos_type;
{   27}  alignment6: ARRAY [1..2] OF char;
{   29}  hashtab: ARRAY [0..ix_hashtab_max] OF RECORD
{+   0}     h_first_pos: ix_pos_type;
{+   6}     alignment4: ARRAY [1..2] OF char;
{+   8}     h_last_pos: ix_pos_type;
{+  14}     alignment5: ARRAY [1..2] OF char
{= 176}     END;
{  205}  free_first_pos: ix_pos_type;
{  211}  alignment7: ARRAY [1..2] OF char;
{  213}  free_last_pos: ix_pos_type
{= 218}  END;
matchcode_table_element = RECORD
{    1}  input_code: ARRAY [1..matchcode_table_text_len] OF char;
{  101}  is_blank: boolean;
{  102}  has_matchcode: boolean;
{  103}  matchcode_pos: short;
{  105}  matchcode_char: char;
{  106}  index: indexname_type;
{  138}  alignment1: char;
{  139}  priority: short
{= 140}  END;
matchcode_table_type = ARRAY [1..matchcode_table_size] OF matchcode_table_element;
benzing_req = RECORD
{    1}  dbgid: char;
{    2}  dbdid: char;
{    3}  dbbzus: char;
{    4}  dbsaum: ARRAY [1..2] OF char;
{    6}  dbuhrk: char;
{    7}  dbyear: ARRAY [1..2] OF char;
{    9}  dbmon: ARRAY [1..2] OF char;
{   11}  dbday: ARRAY [1..2] OF char;
{   13}  dbhh: ARRAY [1..2] OF char;
{   15}  dbmm: ARRAY [1..2] OF char;
{   17}  dbss: ARRAY [1..2] OF char;
{   19}  dbfiller: char;
{   20}  dbid: ARRAY [1..benz_did_len] OF char
{= 219}  END;
benzing_reply = RECORD
{    1}  awsaum: ARRAY [1..2] OF char;
{    3}  awinat: ARRAY [1..42] OF char
{=  44}  END;
var_balance_type = RECORD
{    1}  start_pos: short;
{    3}  max_len: short;
{    5}  no_of_bal: short;
{    7}  offset: short
{=   8}  END;
var_balance_tab_type = ARRAY [1..max_no_of_bal_rec] OF var_balance_type;
var_balance_type_tab = ARRAY [1..max_no_of_balances] OF char;
tse_rec = RECORD
{    1}  tse_installed: boolean;
{    2}  tse_multi_clients_installed: boolean;
{    3}  tse_cycle_installed: boolean;
{    4}  tse_income_installed: boolean;
{    5}  tse_adjust_installed: boolean;
{    6}  tse_apply_and_grant_installed: boolean;
{    7}  tse_cost_installed: boolean;
{    8}  tse_proj_installed: boolean;
{    9}  tse_pop_installed: boolean;
{   10}  tse_access_installed: boolean
{=  10}  END;
pntoln_badgeno_type = ARRAY [1..20] OF char;
pntoln_key_type = char;
token_type = short;
Ptree_node_type = ^tree_node_type;
tree_node_type = RECORD
{    1}  next_child_left: Ptree_node_type;
{    5}  next_child_right: Ptree_node_type;
{    9}  token: token_type;
{   11}  fieldid: fieldid_type;
{   27}  condition_value: ARRAY [1..32] OF char
{=  58}  END;
moduleid_text_type = ARRAY [1..moduleid_text_len] OF char;
module_table_elem_type = RECORD
{    1}  moduleid: short;
{    3}  moduleid_text: moduleid_text_type;
{   43}  alignment1: ARRAY [1..2] OF char;
{   45}  maxemplno: long;
{   49}  db_size: long;
{   53}  db_all_size: long;
{   57}  participant: char
{=  57}  END;
project_time_type = RECORD
{    1}  costid: costid_type;
{   21}  duration: duration_type;
{   23}  comment: long_text_type
{=  52}  END;
project_tab_type = ARRAY [1..awtime_max_projects_per_day] OF project_time_type;
licence_elem_type = RECORD
{    1}  moduleid: short;
{    3}  filler: ARRAY [1..2] OF filler_type;
{    5}  used: long;
{    9}  in_clientno: long;
{   13}  in_all: long
{=  16}  END;
licence_table_type = ARRAY [1..participant_array_len] OF licence_elem_type;
short_block_type = ARRAY [1..8] OF short;
hex_block_type = ARRAY [1..16] OF char;
flag_block_type = ARRAY [1..64] OF flag_type;
dsmdlptp_code_type = ARRAY [1..dsmdlptp_code_len] OF hex_block_type;
dsmdsum_code_type = ARRAY [1..dsmdsum_code_len] OF hex_block_type;
participant_flag_array_type = ARRAY [1..participant_array_len] OF flag_type;
participant_long_array_type = ARRAY [1..participant_array_len] OF long;
uemploy_table_elem_type = RECORD
{    1}  ind_module: char;
{    2}  alignment1: char;
{    3}  old_moduleid: short;
{    5}  old_moduleid_text: moduleid_text_type;
{   45}  new_moduleid: short;
{   47}  new_moduleid_text: moduleid_text_type
{=  86}  END;
tsuemploy_table_elem_type = RECORD
{    1}  ind_module: char;
{    2}  filler: filler_type;
{    3}  old_moduleid: short;
{    5}  new_moduleid: short
{=   6}  END;
tsuemploy_table_type = ARRAY [1..module_table_len] OF tsuemploy_table_elem_type;
eload_table_elem_type = RECORD
{    1}  moduleid: short;
{    3}  moduleid_text: moduleid_text_type;
{   43}  position: short
{=  44}  END;
tseload_table_elem_type = RECORD
{    1}  moduleid: short;
{    3}  position: short
{=   4}  END;
eload_table_type = ARRAY [1..module_table_len] OF eload_table_elem_type;
tseload_table_type = ARRAY [1..module_table_len] OF tseload_table_elem_type;
input_pduration_type = ARRAY [1..9] OF char;
pduration_type = long;
byte_8_type = ARRAY [1..8] OF byte;
byte_28_type = ARRAY [1..28] OF byte;
byte_32_type = ARRAY [1..32] OF byte;
byte_48_type = ARRAY [1..48] OF byte;
byte_56_type = ARRAY [1..56] OF byte;
byte_64_type = ARRAY [1..64] OF byte;
licence_module_type = RECORD
{    1}  installed: boolean;
{    2}  ase: boolean;
{    3}  number: byte;
{    4}  alignment1: char;
{    5}  size: long
{=   8}  END;
licencerec_type = RECORD
{    1}  clientno: buffer_type;
{    3}  ok: boolean;
{    4}  client_depend: boolean;
{    5}  test_timer: boolean;
{    6}  alignment1: char;
{    7}  number: short;
{    9}  max_clientnos: short;
{   11}  test_end_date: date_type;
{   17}  test_max: long;
{   21}  ASE_timer: boolean;
{   22}  timer: timer_text_type;
{  102}  ckey: packed_8_type;
{  110}  as400bib: packed_10_type;
{  120}  alignment2: char;
{  121}  module_table: ARRAY [1..26] OF licence_module_type
{= 328}  END;
pduration_positions_type = RECORD
{    1}  total_no_of_positions: short;
{    3}  positions: ARRAY [1..10] OF short
{=  22}  END;
awtime_text_type = ARRAY [1..26] OF char;
ccsid_tab_type = RECORD
{    1}  visible_on_as400: char;
{    2}  ord_ebcdic: packed_3_type;
{    5}  ord_uppercase: packed_3_type;
{    8}  ord_unmodified: packed_3_type;
{   11}  ord_ansi: packed_3_type;
{   14}  ord_pcsterminal: packed_3_type;
{   17}  ord_ascii: packed_3_type;
{   20}  special1_ord: packed_3_type;
{   23}  special2_ord: packed_3_type;
{   26}  special3_ord: packed_3_type;
{   29}  special4_ord: packed_3_type;
{   32}  special5_ord: packed_3_type
{=  34}  END;
clientvalues_elem_ptr = ^clientvalues_elem_type;
clientvalues_elem_type = RECORD
{    1}  check: boolean;
{    2}  clientno: buffer_type;
{    4}  alignment1: char;
{    5}  dsinstrec: dsinst_rec;
{ 1155}  alignment2: ARRAY [1..2] OF char;
{ 1157}  licencerec: licencerec_type;
{ 1485}  next: clientvalues_elem_ptr
{=1488}  END;
siport_badgeno_type = ARRAY [1..siport_badgeno_len] OF char;
zone_range_tab_type = RECORD
{    1}  clientno: buffer_type;
{    3}  termid: objectid_type;
{   11}  from_zone: short;
{   13}  to_zone: short;
{   15}  tariffarea: tariffarea_type;
{   19}  zexport_termid: objectid_type
{=  26}  END;
zone_range_pointer = ^zone_range_list_type;
zone_range_list_type = RECORD
{    1}  next_pt: zone_range_pointer;
{    5}  zone_range_tab: zone_range_tab_type
{=  30}  END;
mask_cost_tab_rec_type = RECORD
{    1}  costid: costid_type;
{   21}  switch_time: mod_type;
{   23}  termno: short;
{   25}  modify_code: short;
{   27}  to_time: mod_type;
{   29}  filler1: ARRAY [1..4] OF filler_type
{=  32}  END;
projno_array_type = RECORD
{    1}  projno: ARRAY [1..20] OF char
{=  20}  END;
projno_tab_type = ARRAY [1..max_costno_grupps] OF projno_array_type;
statusid_type = ARRAY [1..statusid_len] OF char;
statustype_type = char;
wrk_status_type = RECORD
{    1}  statusid: statusid_type;
{    9}  statustype: statustype_type
{=   9}  END;

{comdecl2} {CONST lens12.inc ???}
   {dbrecs.inc  was?}
accat_type = ARRAY [1..8] OF char;
pensum_type = ARRAY [1..4] OF char;
emplno_qual_type = RECORD
{    1}  emplno: emplno_type;
{   11}  qualificationid: qualificationid_type
{=  18}  END;
alt_wpatids_type = ARRAY [1..maxalt_wpats] OF wpatid_type;
alt_wpatids_flag_type = ARRAY [1..maxalt_wpats] OF flag_type;
zone_perm_tab_type = ARRAY [1..max_zones] OF char;
calenid_type = RECORD
{    1}  tariffarea: tariffarea_type;
{    5}  year: short
{=   6}  END;
dtimeid_type = RECORD
{    1}  day_emplno: day_emplno_type;
{   17}  sequenceno: char
{=  17}  END;
income_group_type = ARRAY [1..max_inc_per_group] OF income_type;
income_group_tab_type = ARRAY [1..max_no_of_inc_groups] OF income_group_type;
adjust_key_type = RECORD
{    1}  adjustcat: adjustcat_type;
{    5}  invalid_from: date_type;
{   11}  sequenceno: short
{=  12}  END;
selopt_key_type = RECORD
{    1}  seloptno: char;
{    2}  selopt: char
{=   2}  END;
balance_adj_type = RECORD
{    1}  balance: unsigned_byte;
{    2}  bal_unit: char;
{    3}  count: short;
{    5}  adjust_day: date_type;
{   11}  method: short;
{   13}  transbalanceid_1: short;
{   15}  transbalanceid_2: short;
{   17}  corr_value_1: saldo_type;
{   21}  corr_value_2: saldo_type
{=  24}  END;
balance_cbal_type = RECORD
{    1}  balance: balance_type;
{    5}  cycle_terminated: flag_type;
{    6}  alignment1: filler_type;
{    7}  sum_of_positive: short
{=   8}  END;
adjust_rule_tab_type = ARRAY [1..adjust_rule_tab_len] OF balance_adj_type;
cbal_balance_tab_type = ARRAY [1..max_no_of_balances] OF balance_cbal_type;
add_value_tab_type = ARRAY [1..max_no_of_balances] OF balance_type;
plan_key_type = RECORD
{    1}  emplno: emplno_type;
{   11}  date_to: date_type
{=  16}  END;
rule_key_type = RECORD
{    1}  ruleid: ruleid_type;
{    3}  invalid_from: date_type
{=   8}  END;
timepair_type = RECORD
{    1}  codeid: codeid_type;
{    3}  time_in: mod_type;
{    5}  terminal_in: short;
{    7}  modify_in: short;
{    9}  time_out: mod_type;
{   11}  terminal_out: short;
{   13}  modify_out: short
{=  14}  END;
wpat_key_type = RECORD
{    1}  wpatid: wpatid_type;
{    5}  invalid_from: date_type
{=  10}  END;
dsabs_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  absid: emplno_day_type;
{   25}  codeid_tab: month_codes_type;
{   87}  last_filled_day: short;
{   89}  ondutytype_tab: month_char_type;
{  120}  filler1: ARRAY [1..1] OF filler_type;
{  121}  ondutylen_tab: month_short_type;
{  183}  filler: ARRAY [1..18] OF filler_type
{= 200}  END;
dsaccat_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  catid: accat_type;
{   17}  default_perm: char;
{   18}  cat: ARRAY [1..24] OF RECORD
{+   0}     employcat: employcat_type;
{+   4}     perm: char
{= 120}     END;
{  138}  filler: ARRAY [1..43] OF char
{= 180}  END;
dsaccomm_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  catid: accat_type;
{   17}  default_perm: char;
{   18}  comm: ARRAY [1..24] OF RECORD
{+   0}     commtype: commtype_type;
{+   4}     perm: char
{= 120}     END;
{  138}  filler: ARRAY [1..23] OF unsigned_byte
{= 160}  END;
dsaccop_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  catid: accat_type;
{   17}  default_perm: char;
{   18}  cop: ARRAY [1..24] OF RECORD
{+   0}     operationid: operationid_type;
{+   8}     perm: char
{= 216}     END;
{  234}  filler: ARRAY [1..27] OF unsigned_byte
{= 260}  END;
dsaccost_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  catid: accat_type;
{   17}  default_perm: char;
{   18}  cost: ARRAY [1..24] OF RECORD
{+   0}     costid: costid_type;
{+  20}     perm: char
{= 504}     END;
{  522}  filler: ARRAY [1..28] OF char
{= 549}  END;
deptno_permission_type = RECORD
{    1}  deptno: deptno_type;
{   11}  perm: char
{=  11}  END;
dsacdept_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  catid: accat_type;
{   17}  default_perm: char;
{   18}  dept: ARRAY [1..48] OF deptno_permission_type;
{  546}  filler: ARRAY [1..15] OF char
{= 560}  END;
accode_perm_type = RECORD
{    1}  codeid: codeid_type;
{    3}  codetype: codetype_type;
{    4}  perm: char
{=   4}  END;
dsaccode_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  catid: accat_type;
{   17}  sequenceno: short;
{   19}  default_perm: char;
{   20}  codeid: codeid_type;
{   22}  codetype: codetype_type;
{   23}  perm: char;
{   24}  filler: ARRAY [1..1] OF char
{=  24}  END;
dsacinc_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  catid: accat_type;
{   17}  default_perm: char;
{   18}  inc: ARRAY [1..24] OF RECORD
{+   0}     income: income_type;
{+   4}     perm: char
{= 120}     END;
{  138}  filler: ARRAY [1..39] OF char
{= 176}  END;
acmask_perm_type = RECORD
{    1}  maskid: objectid_type;
{    9}  perm: char;
{   10}  permission_code: char
{=  10}  END;
dsacmask_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  catid: accat_type;
{   17}  sequenceno: short;
{   19}  default_perm: char;
{   20}  mask: acmask_perm_type;
{   30}  filler: ARRAY [1..3] OF char
{=  32}  END;
dsacprog_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  catid: accat_type;
{   17}  default_perm: char;
{   18}  filler1: ARRAY [1..3] OF a_char;
{   21}  prog: ARRAY [1..24] OF RECORD
{+   0}     progid: objectid_type;
{+   8}     perm: char;
{+   9}     filler: char
{= 240}     END;
{  261}  filler: ARRAY [1..40] OF char
{= 300}  END;
dsacsel_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  catid: accat_type;
{   17}  sequenceno: short;
{   19}  default_perm: char;
{   20}  selopt_key: selopt_key_type;
{   22}  perm: char
{=  22}  END;
acsel_perm_type = RECORD
{    1}  seloptno: char;
{    2}  selopt: char;
{    3}  perm: char
{=   3}  END;
dsacterm_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  catid: accat_type;
{   17}  default_perm: char;
{   18}  term: ARRAY [1..24] OF RECORD
{+   0}     termid: objectid_type;
{+   8}     perm: char
{= 216}     END;
{  234}  filler: ARRAY [1..67] OF char
{= 300}  END;
dsaczone_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  catid: accat_type;
{   17}  default_perm: char;
{   18}  zone_perm_tab: zone_perm_tab_type;
{  218}  filler: ARRAY [1..23] OF filler_type
{= 240}  END;
dsadjusr_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  adjust_key: adjust_key_type;
{   21}  balance_adj: balance_adj_type;
{   45}  filler: ARRAY [1..16] OF filler_type
{=  60}  END;
dsadjust_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  adjust_key: adjust_key_type;
{   21}  valid_from: date_type;
{   27}  adjust_text: long_text_type;
{   57}  chain_balance_tab: chain_balance_tab_type;
{  105}  income_group: income_group_tab_type;
{  265}  useable: flag_type;
{  266}  filler: ARRAY [1..35] OF filler_type
{= 300}  END;
dsadjustt_rec = RECORD
{    1}  dsadjustrec: dsadjust_rec;
{  301}  rule_tab: adjust_rule_tab_type
{=2028}  END;
dsccbal_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  ccbalid: emplno_day_type;
{   25}  add_value_tab: add_value_tab_type;
{  121}  filler: ARRAY [1..40] OF filler_type
{= 160}  END;
dsccbalr_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  ccbalrid: emplno_day_seq_type;
{   27}  filler1: ARRAY [1..2] OF filler_type;
{   29}  balance_adj: balance_adj_type;
{   53}  filler: ARRAY [1..28] OF filler_type
{=  80}  END;
dsccbalt_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  ccbalid: emplno_day_type;
{   25}  add_value_tab: add_value_tab_type;
{  121}  rule_tab: adjust_rule_tab_type;
{ 1849}  use_method_tab: rule_apply_tab_type;
{ 1921}  balance_numbers: ARRAY [1..max_no_of_balances] OF unsigned_byte
{=1944}  END;
dsadmbno_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  date_and_time: date_and_time_type;
{   21}  millisec: short;
{   23}  badgeno: badgeno_type;
{   33}  physical_key: ARRAY [1..20] OF char;
{   53}  key_type: char;
{   54}  key_state: char;
{   55}  logonid: logonid_type;
{   63}  owner: ARRAY [1..10] OF char;
{   73}  filler: ARRAY [1..18] OF char
{=  90}  END;
dsaemp_const_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  emplno: emplno_type;
{   19}  deptno: deptno_type;
{   29}  employcat: employcat_type;
{   33}  filler3: ARRAY [1..1] OF filler_type;
{   34}  factory: packed_12_type;
{   46}  tariffarea: tariffarea_type;
{   50}  pin_code: pin_code_type;
{   54}  manager_emplno: emplno_type;
{   64}  firstname: firstname_type;
{   84}  surname: surname_type;
{  114}  vacation_method: unsigned_byte;
{  115}  vacation: ARRAY [1..maxvacations] OF dbalance_type;
{  123}  allowances_workdays: allowances_type;
{  133}  allowances_freedays: allowances_type;
{  143}  accesscat: packed_4_type;
{  147}  hometerm: objectid_type;
{  155}  termcats: ARRAY [1..maxtermcats] OF termcat_type;
{  187}  dist_deduct_on_call: ARRAY [1..2] OF duration_type;
{  191}  v_invalid_from: date_type;
{  197}  v_valid_from: date_type;
{  203}  start_date: date_type;
{  209}  ref_period_work_time: saldo_type;
{  213}  ref_period_len: short;
{  215}  phoneno: packed_14_type;
{  229}  auto_cost_switch: flag_type;
{  230}  individual_distance: flag_type;
{  231}  valid_from: date_type;
{  237}  valid_to: date_type;
{  243}  cycleid: cycleid_type;
{  247}  cycle_start: short;
{  249}  std_effect_rate: sreal_type;
{  251}  dist_deduct: ARRAY [1..2] OF duration_type;
{  255}  adjustcat: adjustcat_type;
{  259}  mean_std_worktime: duration_type;
{  261}  mean_std_worktime_week: duration_type;
{  263}  continued_pay_len: continued_pay_len_type;
{  265}  filler4: ARRAY [1..7] OF filler_type;
{  272}  working_location: unsigned_byte;
{  273}  costid: costid_type;
{  293}  sfc_participant: flag_type;
{  294}  multi_machine_user: char;
{  295}  birthdate: date_type;
{  301}  mach_job: mach_job_type;
{  317}  badgeno: long_badgeno_type;
{  337}  select_options: select_options_type;
{  353}  empl_corr_values: empl_corr_values_type;
{  369}  empl_corr_values_add: empl_corr_values_type;
{  385}  filler5: packed_16_type
{= 400}  END;
dsaemp_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  emplno: emplno_type;
{   19}  deptno: deptno_type;
{   29}  employcat: employcat_type;
{   33}  filler3: ARRAY [1..1] OF filler_type;
{   34}  factory: packed_12_type;
{   46}  tariffarea: tariffarea_type;
{   50}  pin_code: pin_code_type;
{   54}  manager_emplno: emplno_type;
{   64}  firstname: firstname_type;
{   84}  surname: surname_type;
{  114}  vacation_method: unsigned_byte;
{  115}  vacation: ARRAY [1..maxvacations] OF dbalance_type;
{  123}  allowances_workdays: allowances_type;
{  133}  allowances_freedays: allowances_type;
{  143}  accesscat: acccatid_type;
{  147}  hometerm: objectid_type;
{  155}  termcats: ARRAY [1..maxtermcats] OF termcat_type;
{  187}  dist_deduct_on_call: ARRAY [1..2] OF duration_type;
{  191}  v_invalid_from: date_type;
{  197}  v_valid_from: date_type;
{  203}  start_date: date_type;
{  209}  ref_period_work_time: saldo_type;
{  213}  ref_period_len: short;
{  215}  phoneno: packed_14_type;
{  229}  auto_cost_switch: flag_type;
{  230}  individual_distance: flag_type;
{  231}  valid_from: date_type;
{  237}  valid_to: date_type;
{  243}  cycleid: cycleid_type;
{  247}  cycle_start: short;
{  249}  std_effect_rate: sreal_type;
{  251}  dist_deduct: ARRAY [1..2] OF duration_type;
{  255}  adjustcat: adjustcat_type;
{  259}  mean_std_worktime: duration_type;
{  261}  mean_std_worktime_week: duration_type;
{  263}  continued_pay_len: continued_pay_len_type;
{  265}  filler4: ARRAY [1..7] OF filler_type;
{  272}  working_location: unsigned_byte;
{  273}  costid: costid_type;
{  293}  sfc_participant: flag_type;
{  294}  multi_machine_user: char;
{  295}  birthdate: date_type;
{  301}  mach_job: mach_job_type;
{  317}  badgeno: long_badgeno_type;
{  337}  select_options: select_options_type;
{  353}  empl_corr_values: empl_corr_values_type;
{  369}  email_address: email_address_type;
{  401}  postal_code: ARRAY [1..8] OF char;
{  409}  home_town: ARRAY [1..32] OF char;
{  441}  street: ARRAY [1..40] OF char;
{  481}  phoneno1: phoneno_type;
{  505}  handyno1: phoneno_type;
{  529}  phoneno2: phoneno_type;
{  553}  handyno2: phoneno_type;
{  577}  photo_location: url_type;
{  831}  filler6: ARRAY [1..1] OF filler_type;
{  832}  user_template: logonid_type;
{  840}  password: ARRAY [1..8] OF unsigned_byte;
{  848}  alias: alias_type;
{  863}  no_of_blank_lines: short;
{  865}  position_number: short;
{  867}  rpoolid: rpoolid_type;
{  879}  hourly_wage: sreal_type;
{  881}  last_change_pin: date_type;
{  887}  last_change_pwd: date_type;
{  893}  pin_locked: flag_type;
{  894}  pwd_locked: flag_type;
{  895}  filler5: ARRAY [1..106] OF filler_type
{=1000}  END;
dsaltpat_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  emplno: emplno_type;
{   19}  first_day: date_type;
{   25}  altpat_tab: ARRAY [1..7] OF alt_wpatids_type;
{  249}  warn_altpat_tab: ARRAY [1..7] OF alt_wpatids_flag_type;
{  305}  filler: ARRAY [1..36] OF filler_type
{= 340}  END;
dsacalen_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  tariffarea: tariffarea_type;
{   13}  special_day: date_type;
{   19}  special_day_text: long_text_type;
{   49}  cal_function: char;
{   50}  daytype: unsigned_byte;
{   51}  date_offset: short;
{   53}  time_difference: mod_type;
{   55}  filler: ARRAY [1..26] OF filler_type
{=  80}  END;
special_day_daytype_type = ARRAY [1..31] OF unsigned_byte;
dscalen_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  calenid: calenid_type;
{   15}  calen_text: long_text_type;
{   45}  industry_day_1: short;
{   47}  special_day_daytype: ARRAY [1..12] OF special_day_daytype_type;
{  419}  time_difference: mod_type;
{  421}  filler: ARRAY [1..30] OF char
{= 450}  END;
dscant_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  emplno: emplno_type;
{   19}  datetime: date_and_time_type;
{   31}  modify_id: short;
{   33}  termid: objectid_type;
{   41}  amount: long;
{   45}  termno: short;
{   47}  articleno: objectid_type;
{   55}  quantity: short;
{   57}  filler: ARRAY [1..44] OF char
{= 100}  END;
dscbal_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  cbalid: emplno_day_type;
{   25}  prev_section_date: date_type;
{   31}  next_section_date: date_type;
{   37}  balance_tab: cbal_balance_tab_type;
{  229}  filler: ARRAY [1..12] OF filler_type
{= 240}  END;
dsccsid_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  ccsid_tab: ccsid_tab_type
{=  42}  END;
dsccalen_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  tariffarea: tariffarea_type;
{   13}  date: date_type;
{   19}  sequenceno: short;
{   21}  commt_text: long_text_type
{=  50}  END;
dscemp_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  emplno: emplno_type;
{   19}  working_location: unsigned_byte;
{   20}  filler1: ARRAY [1..1] OF filler_type;
{   21}  manager_emplno: emplno_type;
{   31}  email_address: email_address_type;
{   63}  filler2: ARRAY [1..26] OF filler_type;
{   89}  rpoolid: rpoolid_type;
{  101}  postal_code: ARRAY [1..8] OF char;
{  109}  home_town: ARRAY [1..32] OF char;
{  141}  street: ARRAY [1..40] OF char;
{  181}  phoneno1: phoneno_type;
{  205}  handyno1: phoneno_type;
{  229}  phoneno2: phoneno_type;
{  253}  handyno2: phoneno_type;
{  277}  photo_location: url_type;
{  531}  filler5: ARRAY [1..1] OF filler_type;
{  532}  alias: alias_type;
{  547}  no_of_blank_lines: short;
{  549}  position_number: short;
{  551}  hourly_wage: sreal_type;
{  553}  filler3: ARRAY [1..54] OF filler_type;
{  607}  text_1: ARRAY [1..60] OF char;
{  667}  text_2: ARRAY [1..60] OF char;
{  727}  last_date_auto_schedu: date_type;
{  733}  last_closing: date_type;
{  739}  qual_for_job: flag_type;
{  740}  reserved01: char;
{  741}  planned_until: date_type;
{  747}  filler4: ARRAY [1..54] OF filler_type;
{  801}  last_change_pin: date_type;
{  807}  pin_locked: flag_type;
{  808}  pwd_locked: flag_type;
{  809}  last_change_pwd: date_type;
{  815}  user_template: logonid_type;
{  823}  filler: ARRAY [1..1178] OF filler_type
{=2000}  END;
dscitem_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  articleno: objectid_type;
{   17}  price: ARRAY [1..3] OF long;
{   29}  article_text: long_text_type;
{   59}  kind: char;
{   60}  incomeid: income_type;
{   64}  filler1: ARRAY [1..1] OF char;
{   65}  invalid_from: date_type;
{   71}  valid_from: date_type;
{   77}  filler: ARRAY [1..4] OF char
{=  80}  END;
dscode_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  wrk_code: wrk_code_type;
{   12}  wrk_codetype_long: char;
{   13}  wrk_codetext: long_text_type;
{   43}  vacation_code: flag_type;
{   44}  warning_if_absence: flag_type;
{   45}  balanceid: short;
{   47}  free_shift: flag_type;
{   48}  worktime: char;
{   49}  valid_for_overtime: flag_type;
{   50}  overtime_allowance_possible: flag_type;
{   51}  allowance_possible: flag_type;
{   52}  real_worktime: flag_type;
{   53}  valid_on_free_day: flag_type;
{   54}  continue_code: char;
{   55}  combination_code: char;
{   56}  sv_day_flag: flag_type;
{   57}  u_day_flag: flag_type;
{   58}  debit_time_deduct_flag: flag_type;
{   59}  note_break: flag_type;
{   60}  flextime_allowed_flag: flag_type;
{   61}  incomeid: income_type;
{   65}  priority: short;
{   67}  scodeid_day_off: codeid_type;
{   69}  scodeid_holiday: codeid_type;
{   71}  invalid_from: date_type;
{   77}  valid_from: date_type;
{   83}  scodeid_workday: codeid_type;
{   85}  costid: costid_type;
{  105}  color: color_attr_type;
{  113}  wizard_mark: char;
{  114}  standby_as_planned: flag_type;
{  115}  useable: flag_type;
{  116}  inside_normaltime: flag_type;
{  117}  filler: ARRAY [1..24] OF filler_type
{= 140}  END;
cols_key_type = RECORD
{    1}  filetype: ARRAY [1..10] OF char;
{   11}  logonid: logonid_type;
{   19}  formatno: short;
{   21}  col_no: short
{=  22}  END;
dscols_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  cols_key: cols_key_type;
{   31}  rec_pos: short;
{   33}  rec_type: ARRAY [1..1] OF char;
{   34}  title_one_line_flag: flag_type;
{   35}  col_len: short;
{   37}  title1: ARRAY [1..15] OF char;
{   52}  title2: ARRAY [1..15] OF char;
{   67}  col_dist: short;
{   69}  conv_table_name: long_text_type;
{   99}  justify: char;
{  100}  fieldid: fieldid_type;
{  116}  filler: ARRAY [1..21] OF char
{= 136}  END;
commid_type = RECORD
{    1}  emplno: emplno_type;
{   11}  date: date_type;
{   17}  commtype: commtype_type;
{   21}  sequenceno: short
{=  22}  END;
dscomm_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  commid: commid_type;
{   31}  modify_code: short;
{   33}  comment: ARRAY [1..60] OF char;
{   93}  following_line: flag_type;
{   94}  filler: ARRAY [1..7] OF unsigned_byte
{= 100}  END;
dscommt_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  commtype: commtype_type;
{   13}  commt_text: long_text_type;
{   43}  filler: ARRAY [1..18] OF unsigned_byte
{=  60}  END;
dscusers_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  jobid: cusers_name_type;
{   39}  logonid: logonid_type;
{   47}  os_username: cusers_name_type;
{   77}  login_time: date_and_time_type;
{   89}  clientname: cusers_name_type;
{  119}  servername: cusers_name_type;
{  149}  clientadr: cusers_name_type;
{  179}  serveradr: cusers_name_type;
{  209}  filler1: ARRAY [1..2] OF filler_type;
{  211}  clientport: short;
{  213}  serverport: short;
{  215}  last_planner: char;
{  216}  tcp_serverjob: cusers_name_type;
{  246}  filler: ARRAY [1..35] OF unsigned_byte
{= 280}  END;
dscycle_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  cycleid: cycleid_type;
{   13}  cycle_text: long_text_type;
{   43}  begin_date: date_type;
{   49}  filler1: ARRAY [1..1] OF filler_type;
{   50}  wizard_mark: char;
{   51}  length: short;
{   53}  shiftid: ARRAY [1..maxshifts_per_cycle] OF shiftid_type;
{  309}  filler: ARRAY [1..42] OF filler_type
{= 350}  END;
dsdbal_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  dbalid: emplno_day_type;
{   25}  dbal: ARRAY [1..max_no_of_balances] OF dbalance_type;
{   73}  auto_break: duration_type;
{   75}  filler: ARRAY [1..6] OF filler_type
{=  80}  END;
dsdept_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  deptno: deptno_type;
{   19}  dept_text: long_text_type;
{   49}  filler: ARRAY [1..12] OF char
{=  60}  END;
dsdic_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  recid: recid_type;
{   17}  fieldid: fieldid_type;
{   33}  dic_text: long_text_type;
{   63}  keyfield: short;
{   65}  pos: short;
{   67}  fieldtype: fieldtype_type;
{   69}  len: short;
{   71}  conv_table_name: long_text_type;
{  101}  filler: ARRAY [1..40] OF filler_type
{= 140}  END;
record_definition_type = ARRAY [1..max_fields_per_record] OF dsdic_rec;
dist_deduct_tab_type = ARRAY [1..11] OF short;
dsdist_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  dist_deduct_tabid: ARRAY [1..4] OF char;
{   13}  dist_deduct_tab: ARRAY [1..10] OF dist_deduct_tab_type;
{  233}  filler: ARRAY [1..168] OF char
{= 400}  END;
dsdtime_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  dtimeid: dtimeid_type;
{   26}  record_locked: char;
{   27}  no_of_timepairs: short;
{   29}  no_of_del_timepairs: short;
{   31}  timepair_tab: ARRAY [1..maxtimepairs_per_record] OF timepair_type;
{   87}  filler: ARRAY [1..14] OF char
{= 100}  END;
all_timepairs_tab_type = ARRAY [1..maxtimepairs] OF timepair_type;
dsdtimet_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  dtimet_key: day_emplno_type;
{   25}  filler1: char;
{   26}  record_locked: char;
{   27}  no_of_timepairs: short;
{   29}  no_of_del_timepairs: short;
{   31}  timepair_tab: all_timepairs_tab_type;
{  255}  del_timepair_tab: all_timepairs_tab_type;
{  479}  filler: ARRAY [1..22] OF char
{= 500}  END;
dsecalen_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  mach_job: mach_job_type;
{   25}  date: date_type;
{   31}  event: event_type;
{   33}  del_flag: flag_type;
{   34}  filler1: filler_type;
{   35}  from_time: mod_type;
{   37}  to_time: mod_type;
{   39}  filler2: ARRAY [1..12] OF filler_type
{=  50}  END;
dseload_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  parametername: ARRAY [1..8] OF char;
{   17}  eload_text: long_text_type;
{   47}  emplno_pos: short;
{   49}  input_emplno_len: short;
{   51}  manager_emplno_pos: short;
{   53}  input_manager_emplno_len: short;
{   55}  badgeno_pos: short;
{   57}  input_badgeno_len: short;
{   59}  costid_pos: short;
{   61}  input_costid_len: short;
{   63}  v_valid_from_yy_pos: short;
{   65}  v_valid_from_mo_pos: short;
{   67}  v_valid_from_dd_pos: short;
{   69}  valid_from_yy_pos: short;
{   71}  valid_from_mo_pos: short;
{   73}  valid_from_dd_pos: short;
{   75}  valid_to_yy_pos: short;
{   77}  valid_to_mo_pos: short;
{   79}  valid_to_dd_pos: short;
{   81}  start_date_pos: ARRAY [1..3] OF short;
{   87}  birthdate_pos: ARRAY [1..3] OF short;
{   93}  select_options_pos: ARRAY [1..max_select_options] OF short;
{  125}  empl_corr_values_pos: ARRAY [1..max_empl_corr_values] OF short;
{  133}  input_empl_corr_values_len: short;
{  135}  vacation_pos: ARRAY [1..maxvacations] OF short;
{  143}  input_vacation_len: short;
{  145}  allowances_pos: ARRAY [1..10] OF short;
{  165}  input_allowances_len: short;
{  167}  name_pos: short;
{  169}  input_name_len: short;
{  171}  surname_pos: short;
{  173}  input_surname_len: short;
{  175}  firstname_pos: short;
{  177}  input_firstname_len: short;
{  179}  deptno_pos: short;
{  181}  input_deptno_len: short;
{  183}  hometerm_pos: short;
{  185}  input_hometerm_len: short;
{  187}  tariffarea_pos: short;
{  189}  input_tariffarea_len: short;
{  191}  termcats_pos: short;
{  193}  input_termcats_len: short;
{  195}  pincode_pos: short;
{  197}  input_pincode_len: short;
{  199}  employcat_pos: short;
{  201}  input_employcat_len: short;
{  203}  phoneno_pos: short;
{  205}  input_phoneno_len: short;
{  207}  factory_pos: short;
{  209}  input_factory_len: short;
{  211}  accesscat_pos: short;
{  213}  input_accesscat_len: short;
{  215}  cycleid_pos: short;
{  217}  input_cycleid_len: short;
{  219}  cycle_start_pos: short;
{  221}  ref_p_len_pos: short;
{  223}  ref_p_work_time_pos: short;
{  225}  adjustcat_pos: short;
{  227}  input_adjustcat_len: short;
{  229}  mean_std_w_pos: short;
{  231}  input_mean_std_w_len: short;
{  233}  mean_std_w_w_pos: short;
{  235}  inp_mean_std_w_w_len: short;
{  237}  cont_pay_len_pos: short;
{  239}  inp_cont_pay_len_len: short;
{  241}  machineid_pos: short;
{  243}  input_machineid_len: short;
{  245}  jobid_pos: short;
{  247}  input_jobid_len: short;
{  249}  working_location_pos: short;
{  251}  input_working_location_len: short;
{  253}  email_address_pos: short;
{  255}  input_email_address_len: short;
{  257}  position_number_pos: short;
{  259}  input_position_number_len: short;
{  261}  no_of_blank_lines_pos: short;
{  263}  input_no_of_blank_lines_len: short;
{  265}  alias_pos: short;
{  267}  input_alias_len: short;
{  269}  text_1_pos: short;
{  271}  input_text_1_len: short;
{  273}  text_2_pos: short;
{  275}  input_text_2_len: short;
{  277}  street_pos: short;
{  279}  input_street_len: short;
{  281}  postal_code_pos: short;
{  283}  input_postal_code_len: short;
{  285}  home_town_pos: short;
{  287}  input_home_town_len: short;
{  289}  telephoneno_pos: short;
{  291}  input_telephoneno_len: short;
{  293}  auto_cost_switch_pos: short;
{  295}  dist_from_terminal_pos: short;
{  297}  sfc_participant_pos: short;
{  299}  multi_machine_user_pos: short;
{  301}  vacation_method_pos: short;
{  303}  input_vacation_method_len: short;
{  305}  phoneno1_pos: short;
{  307}  input_phoneno1_len: short;
{  309}  phoneno2_pos: short;
{  311}  input_phoneno2_len: short;
{  313}  handyno1_pos: short;
{  315}  input_handyno1_len: short;
{  317}  handyno2_pos: short;
{  319}  input_handyno2_len: short;
{  321}  photo_location_pos: short;
{  323}  input_photo_location_len: short;
{  325}  user_template_pos: short;
{  327}  input_user_template_len: short;
{  329}  password_pos: short;
{  331}  input_password_len: short;
{  333}  rpoolid_pos: short;
{  335}  input_rpoolid_len: short;
{  337}  hourly_wage_pos: short;
{  339}  module: ARRAY [1..32] OF tseload_table_elem_type;
{  467}  v_valid_to_yy_pos: short;
{  469}  v_valid_to_mo_pos: short;
{  471}  v_valid_to_dd_pos: short;
{  473}  filler: ARRAY [1..54] OF filler_type
{= 526}  END;
dsevent_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  event: event_type;
{   11}  event_text: long_text_type;
{   41}  invalid_from: date_type;
{   47}  valid_from: date_type;
{   53}  event_percent: long;
{   57}  event_absolute: long;
{   61}  event_delay: short;
{   63}  no_of_days: short;
{   65}  allocation: char;
{   66}  filler1: ARRAY [1..3] OF filler_type;
{   69}  manual_allocation: ARRAY [1..7] OF long;
{   97}  color: color_attr_type;
{  105}  filler2: ARRAY [1..6] OF filler_type
{= 110}  END;
dsfbadge_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  badgeno: long_badgeno_type;
{   29}  hostno: short;
{   31}  host_clientno: buffer_type;
{   33}  host_termid: objectid_type;
{   41}  host_termno: short;
{   43}  host_termcat: termcat_type;
{   51}  filler: ARRAY [1..30] OF filler_type
{=  80}  END;
dsff10_rec = RECORD
{    1}  key10: ARRAY [1..10] OF unsigned_byte;
{   11}  data90: ARRAY [1..90] OF unsigned_byte
{= 100}  END;
dsff16_rec = RECORD
{    1}  key16: ARRAY [1..16] OF unsigned_byte;
{   17}  data64: ARRAY [1..64] OF unsigned_byte
{=  80}  END;
export_interface_type = ARRAY [1..2] OF char;
export_line_type = ARRAY [1..2] OF char;
export_status_type = ARRAY [1..2] OF char;
export_data_type = ARRAY [1..export_data_len] OF char;
export_key_type = RECORD
{    1}  export_interface: export_interface_type;
{    3}  line_type: export_line_type;
{    5}  date_time_written: date_and_time_type;
{   17}  millisec: short
{=  18}  END;
dsexport_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  export_key: export_key_type;
{   27}  export_status: export_status_type;
{   29}  export_data: export_data_type;
{  278}  filler: ARRAY [1..123] OF char
{= 400}  END;
dsfsick_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  emplno: emplno_type;
{   19}  date_to: date_type;
{   25}  date_from: date_type;
{   31}  codeid: codeid_type;
{   33}  ref_date: date_type;
{   39}  filler: ARRAY [1..22] OF char
{=  60}  END;
dsjobflg_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  jobflag: ARRAY [1..3] OF char;
{   12}  jobflg_txt: ARRAY [1..50] OF char;
{   62}  filler: ARRAY [1..9] OF filler_type
{=  70}  END;
dslang_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  tableno: record_no_type;
{   11}  languageid: languageid_type;
{   13}  foreign_key: ARRAY [1..dslang_key_len] OF char;
{   77}  long_key1: long;
{   81}  long_key2: long;
{   85}  unsigned_key1: ARRAY [1..8] OF unsigned_byte;
{   93}  date: date_type;
{   99}  foreign_text: long_text_type;
{  129}  filler: ARRAY [1..12] OF filler_type
{= 140}  END;
lodist_deduct_tab_type = ARRAY [1..10] OF short;
dslodist_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  lodist_deduct_tabid: ARRAY [1..4] OF char;
{   13}  lodist_deduct_tab: ARRAY [1..50] OF lodist_deduct_tab_type;
{ 1013}  filler: ARRAY [1..188] OF filler_type
{=1200}  END;
dslogatt_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  userid: packed_20_type;
{   29}  login_time: date_and_time_type;
{   41}  jobid: cusers_name_type;
{   71}  os_username: cusers_name_type;
{  101}  clientname: cusers_name_type;
{  131}  access_granted: flag_type
{= 131}  END;
dsltime_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  ltimeid: date_and_time_type;
{   21}  termid: objectid_type;
{   29}  termno: short;
{   31}  req_code: char;
{   32}  cost_req_code: char;
{   33}  costid: costid_type;
{   53}  badgeno: long_badgeno_type;
{   73}  codeid: codeid_type;
{   75}  book_date_time: date_and_time_type;
{   87}  ignore_time: short;
{   89}  reason: short;
{   91}  codeidtype: char;
{   92}  filler1: filler_type;
{   93}  amount: long;
{   97}  quantity: short;
{   99}  citemno: objectid_type;
{  107}  millisec: short;
{  109}  emplno: emplno_type;
{  119}  filler: ARRAY [1..2] OF filler_type
{= 120}  END;
booklog_rec = dsltime_rec;
dsmasks_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  menuename: objectid_type;
{   17}  menue_title: menue_title_type;
{   63}  languageid: languageid_type;
{   65}  prog_tab: ARRAY [1..max_menue_lines] OF RECORD
{+   0}     menue_line: menue_line_type;
{+  50}     progname: objectid_type;
{+  58}     info_field: menue_info_type
{= 816}     END;
{  881}  filler: ARRAY [1..70] OF filler_type
{= 950}  END;
short_adjust_rule_type = RECORD
{    1}  method: short;
{    3}  transbalanceid_1: short;
{    5}  transbalanceid_2: short;
{    7}  filler: ARRAY [1..2] OF filler_type;
{    9}  corr_value_1: saldo_type;
{   13}  corr_value_2: saldo_type
{=  16}  END;
dsmcconf_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  operationid: operationid_type;
{   17}  operation_text: long_text_type;
{   47}  balanceid: short;
{   49}  add_value: saldo_type;
{   53}  rule_tab: ARRAY [1..mcconf_rule_tab_len] OF short_adjust_rule_type;
{  133}  value_suggestion: saldo_type;
{  137}  show_cond_min: saldo_type;
{  141}  show_cond_max: saldo_type;
{  145}  check_cond_min: saldo_type;
{  149}  check_cond_max: saldo_type;
{  153}  filler: ARRAY [1..48] OF char
{= 200}  END;
dsperror_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  logonid: logonid_type;
{   17}  start_datetime: date_and_time_type;
{   29}  rowno: short;
{   31}  date_time_written: date_and_time_type;
{   43}  error_code: code_type;
{   47}  vos_error: short;
{   49}  error_text: error_text_type;
{  109}  severity_code: char;
{  110}  emplno: emplno_type;
{  120}  exit_code: exit_code_type;
{  126}  filler: ARRAY [1..25] OF unsigned_byte
{= 150}  END;
dsplan_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  plan_key: plan_key_type;
{   25}  date_from: date_type;
{   31}  allowances: allowances_type;
{   41}  wpatid: wpatid_type;
{   45}  ruleid: ruleid_type;
{   47}  codeid: codeid_type;
{   49}  employcat: employcat_type;
{   53}  tariffarea: tariffarea_type;
{   57}  costid: costid_type;
{   77}  cycleid: cycleid_type;
{   81}  cycle_start: short;
{   83}  special_service: codeid_type;
{   85}  from_time: mod_type;
{   87}  to_time: mod_type;
{   89}  ondutylen_from_cur_wpat: char;
{   90}  filler: ARRAY [1..31] OF unsigned_byte
{= 120}  END;
apply_key_type = RECORD
{    1}  emplno: emplno_type;
{   11}  date_to: date_type;
{   17}  apptype: short;
{   19}  time: mod_type
{=  20}  END;
dsapply_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  apply_key: apply_key_type;
{   29}  date_from: date_type;
{   35}  allowance: allowance_type;
{   37}  allowance_remainder: allowance_type;
{   39}  date_applied: date_type;
{   45}  date_granted: date_type;
{   51}  state: char;
{   52}  granted_by_head: flag_type;
{   53}  granted_by_pd: flag_type;
{   54}  filler: ARRAY [1..15] OF filler_type
{=  68}  END;
dsprstat_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  username: username_type;
{   41}  logonid: logonid_type;
{   49}  jobname: packed_10_type;
{   59}  filler1: ARRAY [1..2] OF char;
{   61}  processno: ulong;
{   65}  start_datetime: date_and_time_type;
{   77}  end_datetime: date_and_time_type;
{   89}  print_file: phys_filename_type;
{  121}  printerid: objectid_type;
{  129}  jobstatus: char;
{  130}  filler: ARRAY [1..31] OF unsigned_byte
{= 160}  END;
dsquery_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  queryid: queryid_type;
{   17}  sequenceno: short;
{   19}  query_text: long_text_type;
{   49}  root_recid: recid_type;
{   57}  query_definition: ARRAY [1..query_def_len] OF char;
{  257}  filler: ARRAY [1..44] OF filler_type
{= 300}  END;
dsqueryt_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  queryid: queryid_type;
{   17}  query_text: long_text_type;
{   47}  root_recid: recid_type;
{   55}  query_definition: ARRAY [1..queryt_def_len] OF char
{=3054}  END;
dsrcalen_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  rcalenid: rcalenid_type;
{   13}  rcalen_text: long_text_type;
{   43}  tariffarea: tariffarea_type;
{   47}  date: date_type;
{   53}  date_last_year: date_type;
{   59}  modified: char;
{   60}  first_day: flag_type;
{   61}  filler: ARRAY [1..20] OF filler_type
{=  80}  END;
dsrcalls_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  mach_job: mach_job_type;
{   25}  date: date_type;
{   31}  time_from: mod_type;
{   33}  no_calls: long;
{   37}  time_to: mod_type;
{   39}  average_time: short;
{   41}  no_lost_calls: long;
{   45}  filler: ARRAY [1..18] OF filler_type
{=  62}  END;
dsrooms_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  roomnumber: ARRAY [1..4] OF char;
{   13}  mpoint: char;
{   14}  room_text: ARRAY [1..100] OF char;
{  114}  filler: ARRAY [1..7] OF filler_type
{= 120}  END;
dsrule_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  rule_key: rule_key_type;
{   17}  valid_from: date_type;
{   23}  rule_text: long_text_type;
{   53}  offduty_code: codeid_type;
{   55}  auto_time_come: char;
{   56}  break_code: codeid_type;
{   58}  auto_time_go: char;
{   59}  calculation_rule: short;
{   61}  code_come_auto_time: char;
{   62}  code_come_warn_flag: flag_type;
{   63}  code_go_auto_time: char;
{   64}  code_go_warn_flag: flag_type;
{   65}  balance_less_employment: char;
{   66}  allowance_1_absolute: flag_type;
{   67}  interval_go: short;
{   69}  balance_priority: ARRAY [1..3] OF char;
{   72}  round_point_of_time: char;
{   73}  balance_rounding_tab: ARRAY [1..3] OF balance_rounding_type;
{   97}  interval_come: short;
{   99}  round_up_come: short;
{  101}  round_up_go: short;
{  103}  late_arrival: short;
{  105}  early_leave: short;
{  107}  late_arrive_code: codeid_type;
{  109}  early_leave_code: codeid_type;
{  111}  abs_warn_flag: flag_type;
{  112}  closed_model_flag: flag_type;
{  113}  time_of_rest: duration_type;
{  115}  oncall_overtime_min: duration_type;
{  117}  ondutylen_from_cur_wpat: flag_type;
{  118}  wizard_mark: char;
{  119}  useable: flag_type;
{  120}  filler: ARRAY [1..21] OF filler_type
{= 140}  END;
dsselopt_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  selopt_key: selopt_key_type;
{   11}  selopt_text: long_text_type;
{   41}  filler: ARRAY [1..20] OF char
{=  60}  END;
dsshift_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  shiftid: shiftid_type;
{   13}  shift_text: long_text_type;
{   43}  ruleid: ruleid_type;
{   45}  shift_unit: char;
{   46}  alt_wpatid_for_type: flag_type;
{   47}  count: short;
{   49}  wpatid_dayno: ARRAY [1..maxdaytypes] OF wpatid_type;
{   89}  wpatid_daytype: ARRAY [1..maxdaytypes] OF wpatid_type;
{  129}  alt_wpatid: ARRAY [1..maxdaytypes] OF alt_wpatids_type;
{  449}  warn_alt_wpatid: ARRAY [1..maxdaytypes] OF alt_wpatids_flag_type;
{  529}  codeid: ARRAY [1..maxdaytypes] OF codeid_type;
{  549}  wizard_mark: char;
{  550}  filler: ARRAY [1..91] OF filler_type
{= 640}  END;
ttraf_key_type = RECORD
{    1}  termno: short;
{    3}  dd: short
{=   4}  END;
dsttraf_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  ttraf_key: ttraf_key_type;
{   13}  number_in: long;
{   17}  error_in: long;
{   21}  number_out: long;
{   25}  error_out: long;
{   29}  filler: ARRAY [1..12] OF char
{=  40}  END;
dsuser_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  logonid: logonid_type;
{   17}  emplno: emplno_type;
{   27}  password: objectid_type;
{   35}  menuename: objectid_type;
{   43}  acterm_cat: accat_type;
{   51}  acmask_cat: accat_type;
{   59}  acdept_cat: accat_type;
{   67}  accat_cat: accat_type;
{   75}  acinc_cat: accat_type;
{   83}  accode_cat: accat_type;
{   91}  username: username_type;
{  123}  password_check_flag: flag_type;
{  124}  change_old_data_flag: flag_type;
{  125}  acprog_cat: accat_type;
{  133}  own_data_perm: char;
{  134}  video_attributes: video_attributes_type;
{  150}  aczone_cat: accat_type;
{  158}  accost_cat: accat_type;
{  166}  time_manager: flag_type;
{  167}  accop_cat: accat_type;
{  175}  accomm_cat: accat_type;
{  183}  printerid: objectid_type;
{  191}  password_locked: flag_type;
{  192}  password_overall: flag_type;
{  193}  password_timestamp: long;
{  197}  password_validity: short;
{  199}  acsel_cat: accat_type;
{  207}  accost_enter_cat: accat_type;
{  215}  project_planer: flag_type;
{  216}  master_proj_perm: char;
{  217}  emplno_pattern: flag_type;
{  218}  filler1: ARRAY [1..3] OF filler_type;
{  221}  special_colors: special_colors_type;
{  349}  mask_colors: ARRAY [1..20] OF one_masks_colors_type;
{ 1949}  filler: ARRAY [1..12] OF filler_type
{=1960}  END;
dsusers_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  logonid: logonid_type;
{   17}  prog_init: ARRAY [1..300] OF unsigned_byte;
{  317}  emplno: emplno_type;
{  327}  filler1: ARRAY [1..2] OF filler_type;
{  329}  dmonth_date: date_type;
{  335}  dmonth_balanceid: ARRAY [1..4] OF short;
{  343}  dmonth_printerid: objectid_type;
{  351}  dmonth_display_daily_incomes: flag_type;
{  352}  filler2: ARRAY [1..1] OF filler_type;
{  353}  dpres_date: date_type;
{  359}  dpres_time: mod_type;
{  361}  dpres_deptno: costid_type;
{  381}  dpres_wpatid: ARRAY [1..3] OF wpatid_type;
{  393}  dpres_select_options: select_options_type;
{  409}  dpres_printerid: objectid_type;
{  417}  dpres_selection: char;
{  418}  dpres_org_type: char;
{  419}  dpres_codeid: ARRAY [1..5] OF codeid_type;
{  429}  dabsemp_date: date_type;
{  435}  dabsemp_no_of_months: short;
{  437}  dabsemp_fix_code: ARRAY [1..4] OF codeid_type;
{  445}  dabsemp_printerid: objectid_type;
{  453}  dabsgrp_date: date_type;
{  459}  dabsgrp_deptno: deptno_type;
{  469}  dabsgrp_costid: costid_type;
{  489}  dabsgrp_select_options: select_options_type;
{  505}  dabsgrp_printerid: objectid_type;
{  513}  dabsgrp_mark_warning_flags: flag_type;
{  514}  dabsgrp_mark_presence_flags: flag_type;
{  515}  dabsgrp_offduty_char: char;
{  516}  dabsgrp_codeid: ARRAY [1..5] OF codeid_type;
{  526}  filler3: ARRAY [1..3] OF filler_type;
{  529}  dcbal_deptno: deptno_type;
{  539}  dcbal_balanceid: ARRAY [1..4] OF short;
{  547}  dcbal_costid: costid_type;
{  567}  dcbal_select_options: select_options_type;
{  583}  dcbal_bal_type: char;
{  584}  dcbal_sort: char;
{  585}  dcbal_balance_min: long;
{  589}  dcbal_balance_max: long;
{  593}  dcbal_target_date: date_type;
{  599}  dcbal_created_until_date: date_type;
{  605}  dcbal_printerid: objectid_type;
{  613}  dcbal_employcat: employcat_type;
{  617}  dcbal_cols_extension: char;
{  618}  dcbal_adj_balance_flags: ARRAY [1..4] OF flag_type;
{  622}  filler4: ARRAY [1..3] OF filler_type;
{  625}  derrors_date_time: date_and_time_type;
{  637}  derrors_severity_codes: ARRAY [1..8] OF char;
{  645}  derrors_jobnames: ARRAY [1..60] OF char;
{  705}  derrors_username: userid_type;
{  721}  derrors_no_of_days: short;
{  723}  derrors_printerid: objectid_type;
{  731}  filler5: ARRAY [1..2] OF filler_type;
{  733}  tstat_date: date_type;
{  739}  tstat_terminalid: objectid_type;
{  747}  filler6: ARRAY [1..2] OF filler_type;
{  749}  dwarn_date_time: date_and_time_type;
{  761}  dwarn_no_of_days: short;
{  763}  dwarn_warningtype: char;
{  764}  dwarn_status: char;
{  765}  dwarn_emplno: emplno_type;
{  775}  dwarn_deptno: deptno_type;
{  785}  dwarn_employcat: employcat_type;
{  789}  dwarn_badgeno: long_badgeno_type;
{  809}  dwarn_costid: costid_type;
{  829}  dwarn_select_options: select_options_type;
{  845}  dwarn_printerid: objectid_type;
{  853}  demp_printerid: objectid_type;
{  861}  dcost_date_from: date_type;
{  867}  dcost_date_to: date_type;
{  873}  dcost_sum_1: char;
{  874}  dcost_sum_2: char;
{  875}  dcost_deptno: deptno_type;
{  885}  dcost_employcat: employcat_type;
{  889}  dcost_select_options: select_options_type;
{  905}  dcost_costid: costid_type;
{  925}  dcost_income: income_type;
{  929}  dcost_printerid: objectid_type;
{  937}  dcost_regular_costid: costid_type;
{  957}  schedu_deptno: deptno_type;
{  967}  schedu_start_date: date_type;
{  973}  schedu_no_of_weeks: short;
{  975}  schedu_printerid: objectid_type;
{  983}  schedu_quote: short;
{  985}  schedu_unit: char;
{  986}  filler7: ARRAY [1..3] OF filler_type;
{  989}  daccz_zoneid: short;
{  991}  daccz_from_date_and_time: date_and_time_type;
{ 1003}  daccz_to_date_and_time: date_and_time_type;
{ 1015}  daccz_deptno: deptno_type;
{ 1025}  daccz_printerid: objectid_type;
{ 1033}  dacce_zoneid: short;
{ 1035}  dacce_from_date_and_time: date_and_time_type;
{ 1047}  dacce_to_date_and_time: date_and_time_type;
{ 1059}  dacce_printerid: objectid_type;
{ 1067}  filler8: ARRAY [1..2] OF filler_type;
{ 1069}  dzone_zoneid: short;
{ 1071}  dzone_start_dat: date_and_time_type;
{ 1083}  dzone_deptno: deptno_type;
{ 1093}  dzone_pres_time: duration_type;
{ 1095}  dzone_duration: duration_type;
{ 1097}  dzone_printerid: objectid_type;
{ 1105}  dvisit_update_table: flag_type;
{ 1106}  filler9: ARRAY [1..3] OF filler_type;
{ 1109}  pstep_orderno: orderno_type;
{ 1125}  pstep_prod_stepno: ARRAY [1..5] OF char;
{ 1130}  pstep_prod_stepno_var: ARRAY [1..5] OF char;
{ 1135}  pstep_formno: ARRAY [1..10] OF char;
{ 1145}  pstep_machid: objectid_type;
{ 1153}  pstep_coll_orderid: orderno_type;
{ 1169}  dmsg_orderno: orderno_type;
{ 1185}  dmsg_date_time: date_and_time_type;
{ 1197}  dmsg_prod_stepno: prod_stepno_type;
{ 1201}  dmsg_prod_stepno_var: prod_stepno_type;
{ 1205}  dmsg_emplno: emplno_type;
{ 1215}  dmsg_machid: objectid_type;
{ 1223}  filler10: ARRAY [1..2] OF filler_type;
{ 1225}  dstatus_orderno: orderno_type;
{ 1241}  dstatus_prod_stepno: prod_stepno_type;
{ 1245}  dstatus_prod_stepno_var: prod_stepno_type;
{ 1249}  dstatus_prod_operationid: prod_operationid_type;
{ 1253}  dstatus_emplno: emplno_type;
{ 1263}  dstatus_date_time: date_and_time_type;
{ 1275}  dstatus_machid: objectid_type;
{ 1283}  dstatus_fault_reasonid: fault_reasonid_type;
{ 1287}  dstatus_status: char;
{ 1288}  filler11: ARRAY [1..1] OF filler_type;
{ 1289}  dswarn_prod_stepno: prod_stepno_type;
{ 1293}  dswarn_prod_stepno_var: prod_stepno_type;
{ 1297}  dswarn_date_time: date_and_time_type;
{ 1309}  dswarn_no_of_days: short;
{ 1311}  dswarn_state: char;
{ 1312}  dswarn_warning_type: char;
{ 1313}  dswarn_emplno: emplno_type;
{ 1323}  dswarn_orderno: orderno_type;
{ 1339}  dswarn_machid: objectid_type;
{ 1347}  dswarn_printerid: objectid_type;
{ 1355}  filler12: ARRAY [1..2] OF filler_type;
{ 1357}  dpsopn_begin_date_time: date_and_time_type;
{ 1369}  dpsopn_end_date_time: date_and_time_type;
{ 1381}  dpsopn_machid: objectid_type;
{ 1389}  dpsopn_prod_operationid: prod_operationid_type;
{ 1393}  dpsopn_only_ready: flag_type;
{ 1394}  dpsopn_printerid: objectid_type;
{ 1402}  filler13: ARRAY [1..3] OF filler_type;
{ 1405}  lpstep_orderno: orderno_type;
{ 1421}  lpstep_machid: objectid_type;
{ 1429}  lpstep_prod_operationid: prod_operationid_type;
{ 1433}  lpstep_begin_date_time: date_and_time_type;
{ 1445}  lpstep_end_date_time: date_and_time_type;
{ 1457}  lpstep_pstep_status: char;
{ 1458}  lpstep_printerid: objectid_type;
{ 1466}  filler14: ARRAY [1..3] OF filler_type;
{ 1469}  dorder_orderno: orderno_type;
{ 1485}  dorder_text: long_text_type;
{ 1515}  dorder_articleno: ARRAY [1..16] OF char;
{ 1531}  dorder_order_state: char;
{ 1532}  dorder_state: char;
{ 1533}  dorder_planned_begin: date_and_time_type;
{ 1545}  dorder_planned_end: date_and_time_type;
{ 1557}  dorder_current_data: char;
{ 1558}  filler15: ARRAY [1..3] OF filler_type;
{ 1561}  dfrsn_from_date: date_type;
{ 1567}  dfrsn_to_date: date_type;
{ 1573}  dfrsn_machid: objectid_type;
{ 1581}  dfrsn_fault_reasonid: fault_reasonid_type;
{ 1585}  dfrsn_emplno: emplno_type;
{ 1595}  dfrsn_sum1: char;
{ 1596}  dfrsn_sum2: char;
{ 1597}  dfrsn_printerid: objectid_type;
{ 1605}  reass_date: date_type;
{ 1611}  reass_printerid: objectid_type;
{ 1619}  filler16: ARRAY [1..2] OF filler_type;
{ 1621}  corder_orderno: orderno_type;
{ 1637}  corder_machid: objectid_type;
{ 1645}  corder_prod_operationid: prod_operationid_type;
{ 1649}  corder_pstep_state: char;
{ 1650}  corder_additional_psteps: char;
{ 1651}  filler17: ARRAY [1..2] OF filler_type;
{ 1653}  ccomp_orderno: orderno_type;
{ 1669}  ccomp_machid: objectid_type;
{ 1677}  ccomp_prod_operationid: prod_operationid_type;
{ 1681}  pwbal_date: date_type;
{ 1687}  pwbal_emplno: emplno_type;
{ 1697}  upstep_orderno: orderno_type;
{ 1713}  upstep_machid: objectid_type;
{ 1721}  upstep_pw_type: piece_work_type_type;
{ 1725}  upstep_state: char;
{ 1726}  upstep_prod_operationid: prod_operationid_type;
{ 1730}  filler18: ARRAY [1..3] OF filler_type;
{ 1733}  ltime_begin_date_time: date_and_time_type;
{ 1745}  ltime_end_date_time: date_and_time_type;
{ 1757}  ltime_badgeno: long_badgeno_type;
{ 1777}  ltime_function_code: unsigned_byte;
{ 1778}  filler19: ARRAY [1..3] OF filler_type;
{ 1781}  anlbal_balanceid: short;
{ 1783}  anlbal_target_date: date_type;
{ 1789}  anlbal_created_until_date: date_type;
{ 1795}  filler20: ARRAY [1..2] OF filler_type;
{ 1797}  cplanm_deptno: deptno_type;
{ 1807}  cplanm_costid: costid_type;
{ 1827}  cplanm_select_options: select_options_type;
{ 1843}  cplanm_state: char;
{ 1844}  cplanm_sort_char: char;
{ 1845}  comm_commtype: commtype_type;
{ 1849}  comm_printerid: objectid_type;
{ 1857}  comm_cover: flag_type;
{ 1858}  filler21: ARRAY [1..3] OF filler_type;
{ 1861}  monov_date: date_type;
{ 1867}  monov_balanceid: ARRAY [1..3] OF short;
{ 1873}  filler22: ARRAY [1..4] OF filler_type;
{ 1877}  dscost_emplno: emplno_type;
{ 1887}  dscost_date_from: date_type;
{ 1893}  dscost_date_to: date_type;
{ 1899}  dscost_deptno: deptno_type;
{ 1909}  dscost_select_options: select_options_type;
{ 1925}  dscost_costid: costid_type;
{ 1945}  dscost_machid: objectid_type;
{ 1953}  dscost_orderno: orderno_type;
{ 1969}  dscost_prod_stepno: ARRAY [1..5] OF char;
{ 1974}  dscost_prod_stepno_var: ARRAY [1..5] OF char;
{ 1979}  dscost_articleno: ARRAY [1..16] OF char;
{ 1995}  dscost_summation1: char;
{ 1996}  dscost_summation2: char;
{ 1997}  dscost_summation3: char;
{ 1998}  dscost_printerid: objectid_type;
{ 2006}  filler23: ARRAY [1..3] OF filler_type;
{ 2009}  pscap_machid: objectid_type;
{ 2017}  pscap_costid: costid_type;
{ 2037}  pscap_date_from: date_type;
{ 2043}  pscap_date_to: date_type;
{ 2049}  pscap_cw: ARRAY [1..2] OF char;
{ 2051}  filler24: ARRAY [1..2] OF filler_type;
{ 2053}  psprod_machid: objectid_type;
{ 2061}  psprod_costid: costid_type;
{ 2081}  psprod_date_from: date_type;
{ 2087}  psprod_date_to: date_type;
{ 2093}  psprod_cw: ARRAY [1..2] OF char;
{ 2095}  pspord_orderno: orderno_type;
{ 2111}  psprod_printerid: objectid_type;
{ 2119}  filler25: ARRAY [1..2] OF filler_type;
{ 2121}  accemp_emplno: emplno_type;
{ 2131}  accemp_zoneid: short;
{ 2133}  accemp_printerid: objectid_type;
{ 2141}  schedu_schedsid: objectid_type;
{ 2149}  schedu_select_options: select_options_type;
{ 2165}  schedu_use_prtempday: flag_type;
{ 2166}  schedu_filter_sort_char1: char;
{ 2167}  schedu_filter_sort_char2: char;
{ 2168}  schedu_filter_sort_char3: char;
{ 2169}  schedu_filter_sort_char4: char;
{ 2170}  schedu_ascid: objectid_type;
{ 2178}  filler26: ARRAY [1..3] OF filler_type;
{ 2181}  plan_printerid: objectid_type;
{ 2189}  stpgm_printerid: objectid_type;
{ 2197}  ppord_printerid: objectid_type;
{ 2205}  pscap_printerid: objectid_type;
{ 2213}  pstruc_projectid: projectid_type;
{ 2225}  pstruc_versionid: versionid_type;
{ 2229}  pstruc_parproid: parproid_type;
{ 2241}  pnet_projectid: projectid_type;
{ 2253}  pnet_versionid: versionid_type;
{ 2257}  pnet_taskid: taskid_type;
{ 2269}  pnet_attributes: attributes_type;
{ 2365}  pnet_rpool_assigned: rpoolid_type;
{ 2377}  pnet_emplno_assigned: emplno_type;
{ 2387}  pnet_processing_rate_min: sreal_type;
{ 2389}  pnet_processing_rate_max: sreal_type;
{ 2391}  filler27: ARRAY [1..2] OF filler_type;
{ 2393}  pbar_projectid: projectid_type;
{ 2405}  pbar_versionid: versionid_type;
{ 2409}  pbar_taskid: taskid_type;
{ 2421}  pbar_attributes: attributes_type;
{ 2517}  pbar_rpool_assigned: rpoolid_type;
{ 2529}  pbar_emplno_assigned: emplno_type;
{ 2539}  pbar_processing_rate_min: sreal_type;
{ 2541}  pbar_processing_rate_max: sreal_type;
{ 2543}  filler28: ARRAY [1..6] OF filler_type;
{ 2549}  res_start_date: date_type;
{ 2555}  res_unitlen: short;
{ 2557}  res_unittype: char;
{ 2558}  res_rpoolid: rpoolid_type;
{ 2570}  res_emplno: emplno_type;
{ 2580}  res_projectid: projectid_type;
{ 2592}  res_versionid: versionid_type;
{ 2596}  filler29: ARRAY [1..7] OF filler_type;
{ 2603}  dpwarn_start_date: date_type;
{ 2609}  dpwarn_start_time: time_type;
{ 2615}  dpwarn_projectid: projectid_type;
{ 2627}  dpwarn_versionid: versionid_type;
{ 2631}  dpwarn_parproid: parproid_type;
{ 2643}  dpwarn_taskid: taskid_type;
{ 2655}  dpwarn_rpoolid: rpoolid_type;
{ 2667}  dpwarn_emplno: emplno_type;
{ 2677}  dpwarn_state: char;
{ 2678}  dpwarn_pmwarn_char: char;
{ 2679}  filler30: ARRAY [1..2] OF filler_type;
{ 2681}  dsplan_printerid: objectid_type;
{ 2689}  dsplan_from: date_type;
{ 2695}  dsplan_to: date_type;
{ 2701}  cplane_outlook: flag_type;
{ 2702}  filler31: ARRAY [1..3] OF filler_type;
{ 2705}  awtime_pm_kontierungen: flag_type;
{ 2706}  filler32: ARRAY [1..3] OF filler_type;
{ 2709}  filler: ARRAY [1..792] OF filler_type
{=3500}  END;
dsvisit_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  emplno: emplno_type;
{   19}  surname: surname_type;
{   49}  firstname: firstname_type;
{   69}  company: long_text_type;
{   99}  badgeno: long_badgeno_type;
{  119}  accesscat: acccatid_type;
{  123}  filler1: ARRAY [1..2] OF filler_type;
{  125}  valid_from_date: date_type;
{  131}  valid_from_time: mod_type;
{  133}  valid_to_date: date_type;
{  139}  valid_to_time: mod_type;
{  141}  carlicid: ARRAY [1..12] OF char;
{  153}  contact: long_text_type;
{  183}  pin_code: pin_code_type;
{  187}  factory: packed_12_type;
{  199}  filler: ARRAY [1..62] OF filler_type
{= 260}  END;
warn_log_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  req_code: char;
{   10}  book_code: char;
{   11}  emplno: emplno_type;
{   21}  old_badgeno: badgeno_type;
{   31}  state: char;
{   32}  seqno: unsigned_byte;
{   33}  date_time_written: date_and_time_type;
{   45}  work_day: date_type;
{   51}  time: mod_type;
{   53}  termid: objectid_type;
{   61}  logonid: logonid_type;
{   69}  warntype: short;
{   71}  warnparameter: ARRAY [1..20] OF char;
{   91}  badgeno: long_badgeno_type;
{  111}  filler: ARRAY [1..20] OF filler_type
{= 130}  END;
special_service_type = RECORD
{    1}  special_code: codeid_type;
{    3}  from_time: mod_type;
{    5}  to_time: mod_type
{=   6}  END;
dswopen_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  wopenid: emplno_day_type;
{   25}  wpatid: wpatid_type;
{   29}  ruleid: ruleid_type;
{   31}  allowances: allowances_type;
{   41}  daily_codeid: codeid_type;
{   43}  reass_flag: flag_type;
{   44}  ondutylen_from_cur_wpat: flag_type;
{   45}  alt_wpatids_flag: flag_type;
{   46}  tariffarea: tariffarea_type;
{   50}  employcat: employcat_type;
{   54}  breakusage: unsigned_byte;
{   55}  auto_break_tot_max: allowance_type;
{   57}  cycle_wpatid: wpatid_type;
{   61}  original_wpatid: wpatid_type;
{   65}  costid: costid_type;
{   85}  special_service: ARRAY [1..3] OF special_service_type;
{  103}  filler: ARRAY [1..8] OF filler_type
{= 110}  END;
dswpat_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  wpat_key: wpat_key_type;
{   19}  valid_from: date_type;
{   25}  wpat_text: long_text_type;
{   55}  wpat: ARRAY [1..3] OF RECORD
{+   0}     time_from: mod_type;
{+   2}     time_to: mod_type;
{+   4}     ondutylen: duration_type
{=  18}     END;
{   73}  freeshift: duration_type;
{   75}  break_pattern: ARRAY [1..maxbreaks] OF break_pattern_type;
{  123}  break_tot_min: duration_type;
{  125}  break_tot_max: duration_type;
{  127}  break_time: mod_type;
{  129}  work_min_break: duration_type;
{  131}  onduty_inc: duration_type;
{  133}  break_inc: duration_type;
{  135}  break_percent: short;
{  137}  break_interval: RECORD
{+   0}     time_come: mod_type;
{+   2}     time_go: mod_type
{=   4}     END;
{  141}  day_allowance: RECORD
{+   0}     time_from: mod_type;
{+   2}     time_to: mod_type
{=   4}     END;
{  145}  daybreak: mod_type;
{  147}  daybreak_tariff: mod_type;
{  149}  open_checkout: mod_type;
{  151}  work_mean_break: duration_type;
{  153}  break_tot_mean: duration_type;
{  155}  interval_ind_come: char;
{  156}  interval_ind_go: char;
{  157}  charge_break_flag: flag_type;
{  158}  break_limit_on_worktime: flag_type;
{  159}  sliding_break: flag_type;
{  160}  wizard_mark: char;
{  161}  min_work_length: duration_type;
{  163}  selid: ARRAY [1..2] OF char;
{  165}  block_break_time: duration_type;
{  167}  break_tot_block: duration_type;
{  169}  useable: flag_type;
{  170}  filler: ARRAY [1..11] OF filler_type
{= 180}  END;
placeno_type = ARRAY [1..2] OF char;
roomnumber_type = ARRAY [1..4] OF char;
mpoint_type = char;
jobflag_type = ARRAY [1..3] OF char;
dswplace_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  emplno: emplno_type;
{   19}  work_day: date_type;
{   25}  placeno: placeno_type;
{   27}  climno: char;
{   28}  mpoint: char;
{   29}  jobflag: jobflag_type;
{   32}  roomnumber: roomnumber_type;
{   36}  district_payment: ARRAY [1..3] OF char;
{   39}  pay_method: char;
{   40}  gedingeno: ARRAY [1..2] OF char;
{   42}  bpu: char;
{   43}  worker_rank: ARRAY [1..4] OF char;
{   47}  journey: char;
{   48}  bonus1: ARRAY [1..3] OF char;
{   51}  filler1: ARRAY [1..2] OF filler_type;
{   53}  amount1: long;
{   57}  bonus2: ARRAY [1..3] OF char;
{   60}  filler2: filler_type;
{   61}  amount2: long;
{   65}  bonus3: ARRAY [1..3] OF char;
{   68}  filler3: filler_type;
{   69}  amount3: long;
{   73}  Info1: ARRAY [1..10] OF char;
{   83}  Info2: ARRAY [1..10] OF char;
{   93}  Info3: ARRAY [1..10] OF char;
{  103}  Info4: ARRAY [1..10] OF char;
{  113}  Info5: ARRAY [1..10] OF char
{= 122}  END;
fmsglog_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  date_time_written: date_and_time_type;
{   21}  millisec: short;
{   23}  termid: objectid_type;
{   31}  termno: short;
{   33}  badgeno: badgeno_type;
{   43}  emplno: emplno_type;
{   53}  filler: ARRAY [1..18] OF unsigned_byte;
{   71}  data: ARRAY [1..40] OF char
{= 110}  END;
fmsglog2_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  date_time_written: date_and_time_type;
{   21}  millisec: short;
{   23}  termid: objectid_type;
{   31}  termno: short;
{   33}  badgeno: long_badgeno_type;
{   53}  emplno: emplno_type;
{   63}  filler: ARRAY [1..8] OF filler_type;
{   71}  data: ARRAY [1..100] OF char
{= 170}  END;
help_page_line_rec = ARRAY [1..78] OF char;
help_page_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  chapter: chapter_type;
{   19}  sequenceno: short;
{   21}  line_tab: ARRAY [1..18] OF help_page_line_rec
{=1424}  END;
helptable_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  maskname: objectid_type;
{   17}  chapter: chapter_type
{=  26}  END;
spiinf_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  pgmname: packed_10_type;
{   19}  std_spi: packed_10_type;
{   29}  subpgm01: packed_10_type;
{   39}  subpgm02: packed_10_type;
{   49}  subpgm03: packed_10_type;
{   59}  subpgm04: packed_10_type;
{   69}  subpgm05: packed_10_type;
{   79}  subpgm06: packed_10_type;
{   89}  subpgm07: packed_10_type;
{   99}  subpgm08: packed_10_type;
{  109}  subpgm09: packed_10_type;
{  119}  subpgm10: packed_10_type;
{  129}  subpgm11: packed_10_type;
{  139}  subpgm12: packed_10_type;
{  149}  subpgm13: packed_10_type;
{  159}  subpgm14: packed_10_type;
{  169}  subpgm15: packed_10_type;
{  179}  subpgm16: packed_10_type;
{  189}  subpgm17: packed_10_type;
{  199}  subpgm18: packed_10_type;
{  209}  subpgm19: packed_10_type;
{  219}  subpgm20: packed_10_type;
{  229}  subpgm21: packed_10_type;
{  239}  subpgm22: packed_10_type;
{  249}  subpgm23: packed_10_type;
{  259}  subpgm24: packed_10_type;
{  269}  subpgm25: packed_10_type;
{  279}  subpgm26: packed_10_type;
{  289}  subpgm27: packed_10_type;
{  299}  subpgm28: packed_10_type;
{  309}  subpgm29: packed_10_type;
{  319}  subpgm30: packed_10_type;
{  329}  subpgm31: packed_10_type;
{  339}  subpgm32: packed_10_type;
{  349}  subpgm33: packed_10_type;
{  359}  subpgm34: packed_10_type;
{  369}  subpgm35: packed_10_type;
{  379}  subpgm36: packed_10_type;
{  389}  subpgm37: packed_10_type;
{  399}  subpgm38: packed_10_type;
{  409}  subpgm39: packed_10_type;
{  419}  subpgm40: packed_10_type;
{  429}  subpgm41: packed_10_type;
{  439}  subpgm42: packed_10_type;
{  449}  subpgm43: packed_10_type;
{  459}  subpgm44: packed_10_type;
{  469}  subpgm45: packed_10_type;
{  479}  subpgm46: packed_10_type;
{  489}  subpgm47: packed_10_type;
{  499}  subpgm48: packed_10_type;
{  509}  subpgm49: packed_10_type;
{  519}  subpgm50: packed_10_type;
{  529}  subpgm51: packed_10_type;
{  539}  subpgm52: packed_10_type;
{  549}  subpgm53: packed_10_type;
{  559}  subpgm54: packed_10_type;
{  569}  subpgm55: packed_10_type;
{  579}  subpgm56: packed_10_type;
{  589}  subpgm57: packed_10_type;
{  599}  subpgm58: packed_10_type;
{  609}  subpgm59: packed_10_type;
{  619}  subpgm60: packed_10_type;
{  629}  subpgm61: packed_10_type;
{  639}  subpgm62: packed_10_type;
{  649}  subpgm63: packed_10_type;
{  659}  subpgm64: packed_10_type;
{  669}  subpgm65: packed_10_type;
{  679}  subpgm66: packed_10_type;
{  689}  subpgm67: packed_10_type;
{  699}  subpgm68: packed_10_type;
{  709}  subpgm69: packed_10_type;
{  719}  subpgm70: packed_10_type;
{  729}  subpgm71: packed_10_type;
{  739}  subpgm72: packed_10_type;
{  749}  subpgm73: packed_10_type;
{  759}  subpgm74: packed_10_type;
{  769}  subpgm75: packed_10_type;
{  779}  subpgm76: packed_10_type;
{  789}  subpgm77: packed_10_type;
{  799}  subpgm78: packed_10_type;
{  809}  subpgm79: packed_10_type;
{  819}  subpgm80: packed_10_type;
{  829}  subpgm81: packed_10_type;
{  839}  subpgm82: packed_10_type;
{  849}  subpgm83: packed_10_type;
{  859}  subpgm84: packed_10_type;
{  869}  subpgm85: packed_10_type;
{  879}  subpgm86: packed_10_type;
{  889}  subpgm87: packed_10_type;
{  899}  subpgm88: packed_10_type;
{  909}  subpgm89: packed_10_type;
{  919}  subpgm90: packed_10_type;
{  929}  subpgm91: packed_10_type;
{  939}  subpgm92: packed_10_type;
{  949}  subpgm93: packed_10_type;
{  959}  subpgm94: packed_10_type;
{  969}  subpgm95: packed_10_type;
{  979}  subpgm96: packed_10_type;
{  989}  subpgm97: packed_10_type;
{  999}  subpgm98: packed_10_type;
{ 1009}  subpgm99: packed_10_type;
{ 1019}  subpgm100: packed_10_type;
{ 1029}  subpgm101: packed_10_type;
{ 1039}  subpgm102: packed_10_type;
{ 1049}  subpgm103: packed_10_type;
{ 1059}  subpgm104: packed_10_type;
{ 1069}  subpgm105: packed_10_type;
{ 1079}  subpgm106: packed_10_type;
{ 1089}  subpgm107: packed_10_type;
{ 1099}  subpgm108: packed_10_type;
{ 1109}  subpgm109: packed_10_type;
{ 1119}  subpgm110: packed_10_type;
{ 1129}  subpgm111: packed_10_type;
{ 1139}  subpgm112: packed_10_type;
{ 1149}  subpgm113: packed_10_type;
{ 1159}  subpgm114: packed_10_type;
{ 1169}  subpgm115: packed_10_type;
{ 1179}  subpgm116: packed_10_type;
{ 1189}  subpgm117: packed_10_type;
{ 1199}  subpgm118: packed_10_type;
{ 1209}  subpgm119: packed_10_type;
{ 1219}  subpgm120: packed_10_type;
{ 1229}  heapsize_first: long;
{ 1233}  heapsize_max: long;
{ 1237}  stacksize: long;
{ 1241}  cobolcaps: packed_10_type;
{ 1251}  cmdname: packed_10_type;
{ 1261}  originalname: packed_30_type;
{ 1291}  qtcpneeded: flag_type;
{ 1292}  reserved_01: packed_10_type;
{ 1302}  reserved_02: packed_10_type;
{ 1312}  reserved_03: packed_10_type;
{ 1322}  reserved_04: packed_10_type;
{ 1332}  reserved_05: packed_10_type;
{ 1342}  reserved: ARRAY [1..100] OF char
{=1441}  END;
ipcb_type = RECORD
{    1}  dsabsfcb: fcb_type;
{   21}  dsadjustfcb: fcb_type;
{   41}  dscalenfcb: fcb_type;
{   61}  dscodefcb: fcb_type;
{   81}  dsdtimefcb: fcb_type;
{  101}  dsplanfcb: fcb_type;
{  121}  dsshiftfcb: fcb_type;
{  141}  dsvempfcb: fcb_type;
{  161}  dswopenfcb: fcb_type;
{  181}  dswpatfcb: fcb_type;
{  201}  dsrulefcb: fcb_type;
{  221}  logonid: logonid_type;
{  229}  dsinstrec: dsinst_rec;
{ 1379}  ask_user: boolean;
{ 1380}  codes_only: boolean;
{ 1381}  is_plan: boolean;
{ 1382}  consider_code_priorities: boolean;
{ 1383}  dsabsrec: dsabs_rec;
{ 1583}  dsabsrec_read: boolean;
{ 1584}  dsabsrec_changed: boolean;
{ 1585}  no_new: short;
{ 1587}  no_changed: short;
{ 1589}  dsvemprec: dsaemp_rec;
{ 2589}  version_number: short
{=2590}  END;
chosen_day_type = cit_yesterday..cit_today;
chosen_day_set = SET OF chosen_day_type;
info_rec_type = (dswopen, dsdtime, dswpat);
info_rec_set = SET OF info_rec_type;
common_info_pointer = ^common_info_type;
common_info_type = RECORD
{    1}  dswopen_read: boolean;
{    2}  dsdtime_read: boolean;
{    3}  dswpat_read: boolean;
{    4}  write_dswopen: boolean;
{    5}  org_dswopenrec: dswopen_rec;
{  115}  dswopenrec: dswopen_rec;
{  225}  org_dsdtimetrec: dsdtimet_rec;
{  725}  dsdtimetrec: dsdtimet_rec;
{ 1225}  dswpatrec: dswpat_rec;
{ 1405}  old_no_of_timepairs: short;
{ 1407}  old_no_of_del_timepairs: short;
{ 1409}  current_index: short;
{ 1411}  current_is_coming: boolean;
{ 1412}  alt_wpat_chosen: boolean
{=1412}  END;
common_info_tab = ARRAY [cit_yesterday..cit_today] OF common_info_pointer;
vl_dscolsrec_tab_tab_type = RECORD
{    1}  dscolsrec: dscols_rec;
{  137}  conv_table_pos: short;
{  139}  filler: ARRAY [1..2] OF char
{= 140}  END;
vl_dscolsrec_tab_type = RECORD
{    1}  cols: ARRAY [1..vl_max_cols] OF vl_dscolsrec_tab_tab_type;
{ 4481}  no_cols: short
{=4482}  END;
dsdocno_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  emplno: emplno_type;
{   19}  date_from: date_type;
{   25}  documentno: long;
{   29}  codeid: codeid_type;
{   31}  time_from: mod_type;
{   33}  time_to: mod_type;
{   35}  income: income_type;
{   39}  costid: costid_type;
{   59}  filler: ARRAY [1..2] OF filler_type
{=  60}  END;
info_text_type = ARRAY [1..13] OF char;
sapid_type = RECORD
{    1}  badgeno: badgeno_type;
{   11}  valid_from: date_type
{=  16}  END;
dssap_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  sapid: sapid_type;
{   25}  valid_to: date_type;
{   31}  info: ARRAY [1..10] OF info_text_type;
{  161}  filler: ARRAY [1..30] OF char
{= 190}  END;
booklog_sap_rec = RECORD
{    1}  satza: ARRAY [1..3] OF char;
{    4}  terid: ARRAY [1..4] OF char;
{    8}  ldate: ARRAY [1..8] OF char;
{   16}  ltime: ARRAY [1..6] OF char;
{   22}  erdat: ARRAY [1..8] OF char;
{   30}  ertim: ARRAY [1..6] OF char;
{   36}  zausw: ARRAY [1..8] OF char;
{   44}  abwgr: ARRAY [1..4] OF char;
{   48}  exlga: ARRAY [1..4] OF char;
{   52}  hrazl: ARRAY [1..9] OF char;
{   61}  zeinh: ARRAY [1..3] OF char;
{   64}  hrbet: ARRAY [1..9] OF char;
{   73}  cr: char;
{   74}  lf: char
{=  74}  END;
lhzzma_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  emplno: emplno_type;
{   19}  date: date_type;
{   25}  worked_time: short;
{   27}  balance_3: short;
{   29}  balance_9: short;
{   31}  filler: ARRAY [1..10] OF char
{=  40}  END;
taris_rec = RECORD
{    1}  clientno: buffer_type;
{    3}  termid: objectid_type;
{   11}  year: packed_2_type;
{   13}  month: packed_2_type;
{   15}  day: packed_2_type;
{   17}  hour: packed_2_type;
{   19}  minute: packed_2_type;
{   21}  second: packed_2_type;
{   23}  badgeno: badgeno_type;
{   33}  req_code: char;
{   34}  access_req_code: char;
{   35}  codeidtype: char;
{   36}  codeid: codeid_type;
{   38}  citemno: objectid_type;
{   46}  amount: packed_11_type;
{   57}  quantity: packed_6_type;
{   63}  costid: costid_type;
{   83}  long_badgeno: long_badgeno_type
{= 102}  END;
dspntoln_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  physical_key: pntoln_badgeno_type;
{   29}  logical_key: pntoln_badgeno_type;
{   49}  key_type: pntoln_key_type;
{   50}  filler: ARRAY [1..10] OF filler_type
{=  59}  END;
dsqual_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  qualificationid: qualificationid_type;
{   17}  qualification_text: long_text_type;
{   47}  delete_period: short;
{   49}  qual_refresh: short;
{   51}  qual_warn: short;
{   53}  filler: ARRAY [1..18] OF filler_type
{=  70}  END;
dsable_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  able_key: emplno_qual_type;
{   27}  first_date: date_type;
{   33}  last_date: date_type;
{   39}  no_of_days: short;
{   41}  no_of_days_corr: short;
{   43}  filler: ARRAY [1..28] OF char
{=  70}  END;
dsmdlptp_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  emplno: emplno_type;
{   19}  badgeno: long_badgeno_type;
{   39}  code: dsmdlptp_code_type;
{   55}  filler: ARRAY [1..40] OF filler_type
{=  94}  END;
dsmdsum_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  code: dsmdsum_code_type;
{  281}  filler: ARRAY [1..40] OF filler_type
{= 320}  END;
dscostgr_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  costid_grp: costid_type;
{   29}  costid_grp_text: long_text_type;
{   59}  seq_no: short;
{   61}  costno: costid_type;
{   81}  projno: costid_type;
{  101}  cttype: costid_type;
{  121}  filler: ARRAY [1..20] OF filler_type
{= 140}  END;
dspmstat_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  wrk_status: wrk_status_type;
{   18}  status_committed: wrk_status_type;
{   27}  status_rejected: wrk_status_type;
{   36}  status_text: long_text_type;
{   66}  catid: accat_type;
{   74}  wrk_code: wrk_code_type;
{   77}  filler: ARRAY [1..24] OF filler_type
{= 100}  END;


{pspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspsp}
   {dbconf.inc}
mod_objecttype_type = RECORD
{    1}  moduleid: objectid_type;
{    9}  objecttype: objecttype_type
{=  16}  END;
dsfconf_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  fileid: objectid_type;
{   17}  file_text: long_text_type;
{   47}  logging_flag: flag_type;
{   48}  standard_file: flag_type;
{   49}  maskid: objectid_type;
{   57}  record_length: short;
{   59}  fileno: short;
{   61}  transaction_flag: flag_type;
{   62}  filler: ARRAY [1..39] OF filler_type
{= 100}  END;
dspconf_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  programid: objectid_type;
{   17}  programname: phys_filename_type;
{   49}  print_file: phys_filename_type;
{   81}  priority: short;
{   83}  no_of_parameters: short;
{   85}  privileged_flag: flag_type;
{   86}  pconf_text: long_text_type;
{  116}  filler: ARRAY [1..35] OF unsigned_byte
{= 150}  END;
parameter_type = RECORD
{    1}  parm_text: ARRAY [1..16] OF char;
{   17}  parmname: ARRAY [1..16] OF char;
{   33}  parmtype: char;
{   34}  form_flag: flag_type;
{   35}  minimum: short;
{   37}  maximum: short;
{   39}  max_length: short;
{   41}  max_occurrences: short;
{   43}  required_flag: flag_type;
{   44}  parm_default: parm_tab_line_type;
{  104}  conv_table_name: long_text_type;
{  134}  filler: ARRAY [1..3] OF filler_type
{= 136}  END;
dspconfp_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  programid: objectid_type;
{   17}  sequenceno: short;
{   19}  parameter: parameter_type;
{  155}  filler: ARRAY [1..26] OF filler_type
{= 180}  END;
dspconft_rec = RECORD
{    1}  dspconfrec: dspconf_rec;
{  151}  parameter_tab: ARRAY [1..parm_tab_len] OF parameter_type
{=3414}  END;
dsprconf_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  printerid: objectid_type;
{   17}  print_queue_name: print_queue_name_type;
{   49}  printer_text: long_text_type;
{   79}  printerfile: phys_filename_type;
{  111}  filler: ARRAY [1..90] OF char
{= 200}  END;
dstconf_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  termid: objectid_type;
{   17}  termno: short;
{   19}  termtype: objecttype_type;
{   27}  term_code: char;
{   28}  term_address: objecttype_type;
{   36}  phys_termtype: objecttype_type;
{   44}  download_fileid: objectid_type;
{   52}  moduleid: objectid_type;
{   60}  filler3: filler_type;
{   61}  dist_deduct_grp: short;
{   63}  termname_in: phys_filename_type;
{   95}  termname_out: phys_filename_type;
{  127}  filler1: ARRAY [1..1] OF filler_type;
{  128}  location: long_text_type;
{  158}  telno: ARRAY [1..16] OF char;
{  174}  deptno: deptno_type;
{  184}  user: long_text_type;
{  214}  termcat: termcat_type;
{  222}  perm_download: flag_type;
{  223}  telno_modem: ARRAY [1..14] OF char;
{  237}  info_path: objectid_type;
{  245}  entry_pointname: phys_filename_type;
{  277}  ignore_time: short;
{  279}  forms_input: flag_type;
{  280}  printerid: objectid_type;
{  288}  costid: costid_type;
{  308}  exit_allowed: flag_type;
{  309}  tariffarea: tariffarea_type;
{  313}  life_check_interval: short;
{  315}  life_check_timeout: short;
{  317}  time_sync_interval: short;
{  319}  from_zoneid: short;
{  321}  to_zoneid: short;
{  323}  door_open_time: short;
{  325}  door_signal_time: short;
{  327}  log_entry: flag_type;
{  328}  log_exit: flag_type;
{  329}  log_rejected: flag_type;
{  330}  count_entry: flag_type;
{  331}  count_exit: flag_type;
{  332}  auto_booking: char;
{  333}  log_storno_entry: flag_type;
{  334}  log_storno_exit: flag_type;
{  335}  pin_code: flag_type;
{  336}  off_line_mode: char;
{  337}  entry_from_any_zone: flag_type;
{  338}  filler2: ARRAY [1..11] OF filler_type;
{  349}  dist_deduct_come: duration_type;
{  351}  dist_deduct_go: duration_type;
{  353}  filler: ARRAY [1..8] OF filler_type
{= 360}  END;
dstmasks_rec = RECORD
{    1}  rec_header: rec_header_type;
{    9}  progno: short;
{   11}  maskname: objectid_type;
{   19}  masktitle: menue_line_type
{=  68}  END;


{pspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspsp}
   {tsvrmsg.inc}
ts_msg_header_type = RECORD
{    1}  byte: buffer_type;
{    3}  msgno: short;
{    5}  queue_name: ARRAY [1..1] OF char;
{    6}  filler: ARRAY [1..1] OF unsigned_byte;
{    7}  logonid: logonid_type;
{   15}  sessionid: short;
{   17}  seqno: long;
{   21}  reqno: short;
{   23}  progno: short;
{   25}  old_key: keydata_type
{=  84}  END;
line_flags_type = RECORD
{    1}  selected: char;
{    2}  attribute: char
{=   2}  END;
mask_perm_array = ARRAY [1..maxacmasks] OF acmask_perm_type;
mask_accesses_type = RECORD
{    1}  catid: accat_type;
{    9}  default_perm: char;
{   10}  filler: ARRAY [1..3] OF char;
{   13}  mask: mask_perm_array
{= 972}  END;
selopt_perm_array = ARRAY [1..maxacsel] OF acsel_perm_type;
selopt_accesses_type = RECORD
{    1}  catid: accat_type;
{    9}  default_perm: char;
{   10}  selopt_tab: selopt_perm_array
{= 153}  END;
common_type = RECORD
{    1}  menuename: objectid_type;
{    9}  termno: short;
{   11}  termname: phys_filename_type;
{   43}  own_data_perm: char;
{   44}  default_perm: char;
{   45}  dept_accesses: dsacdept_rec;
{  605}  no_dept_exceptions: short;
{  607}  no_mask_exceptions: short;
{  609}  mask_accesses: mask_accesses_type;
{ 1581}  dsusersrec: dsusers_rec;
{ 5081}  logonid_emplno: emplno_type;
{ 5091}  logonid_surname: surname_type;
{ 5121}  logonid_firstname: firstname_type;
{ 5141}  logonid_email: email_address_type;
{ 5173}  emplno: emplno_type;
{ 5183}  date: date_type;
{ 5189}  project_planer: flag_type;
{ 5190}  filler1: ARRAY [1..1] OF char;
{ 5191}  alignment1: ARRAY [1..2] OF char;
{ 5193}  mask_colors: ARRAY [1..20] OF one_masks_colors_type;
{ 6793}  special_colors: special_colors_type;
{ 6921}  color_masks: ARRAY [1..20] OF menue_line_type;
{ 7921}  printer_name: objectid_type;
{ 7929}  logonid: logonid_type;
{ 7937}  deptno: deptno_type;
{ 7947}  employcat: employcat_type;
{ 7951}  no_selopt_exceptions: short;
{ 7953}  selopt_accesses: selopt_accesses_type;
{ 8106}  programname: ARRAY [1..20] OF objectid_type
{=8265}  END;
code_perm_array = ARRAY [1..maxaccodes] OF accode_perm_type;
code_accesses_type = RECORD
{    1}  catid: accat_type;
{    9}  default_perm: char;
{   10}  code: code_perm_array
{= 201}  END;
accesses_type = RECORD
{    1}  logonid: logonid_type;
{    9}  logonid_emplno: emplno_type;
{   19}  logonid_surname: surname_type;
{   49}  logonid_firstname: firstname_type;
{   69}  logonid_email: email_address_type;
{  101}  ownid: objectid_type;
{  109}  own_taskid: short;
{  111}  own_data_perm: char;
{  112}  default_perm: char;
{  113}  time_manager: flag_type;
{  114}  change_old_data: flag_type;
{  115}  master_proj_perm: char;
{  116}  filler1: ARRAY [1..1] OF char;
{  117}  no_cat_exceptions: short;
{  119}  cat_accesses: dsaccat_rec;
{  299}  code_accesses: code_accesses_type;
{  500}  filler5: ARRAY [1..1] OF char;
{  501}  no_code_exceptions: short;
{  503}  comm_accesses: dsaccomm_rec;
{  663}  no_comm_exceptions: short;
{  665}  no_cop_exceptions: short;
{  667}  cop_accesses: dsaccop_rec;
{  927}  cost_accesses: dsaccost_rec;
{ 1476}  filler2: ARRAY [1..1] OF char;
{ 1477}  no_cost_exceptions: short;
{ 1479}  dept_accesses: dsacdept_rec;
{ 2039}  no_dept_exceptions: short;
{ 2041}  no_inc_exceptions: short;
{ 2043}  inc_accesses: dsacinc_rec;
{ 2219}  filler3: ARRAY [1..2] OF char;
{ 2221}  no_mask_exceptions: short;
{ 2223}  mask_accesses: mask_accesses_type;
{ 3195}  prog_accesses: dsacprog_rec;
{ 3495}  no_prog_exceptions: short;
{ 3497}  no_zone_exceptions: short;
{ 3499}  zone_accesses: dsaczone_rec;
{ 3739}  search_list_acdept: search_list_type;
{ 3765}  search_list_accat: search_list_type;
{ 3791}  search_list_accost: search_list_type;
{ 3817}  search_list_acsel: search_list_type;
{ 3843}  no_selopt_exceptions: short;
{ 3845}  selopt_accesses: selopt_accesses_type;
{ 3998}  filler4: ARRAY [1..1] OF char;
{ 3999}  no_cost_enter_exceptions: short;
{ 4001}  cost_enter_accesses: dsaccost_rec
{=4549}  END;
ts_req_msg_pointer = ^ts_req_msg_type;
ts_req_msg_type = RECORD
{    1}  msg_header: ts_msg_header_type;
{   85}  indexname: indexname_type;
{  117}  function_code: ARRAY [1..function_code_len] OF char;
{  118}  filler: ARRAY [1..15] OF char;
{  133}  reqdata: buffer_type
{= 134}  END;
ts_copy_pointer = ^ts_copy_type;
ts_copy_type = RECORD
{    1}  old_key: keydata_type;
{   61}  new_key: keydata_type
{= 120}  END;
ts_print_pointer = ^ts_print_type;
ts_print_type = RECORD
{    1}  byte: buffer_type;
{    3}  programid: objectid_type;
{   11}  printerid: objectid_type;
{   19}  parm_tab: parm_tab_type;
{ 1459}  prstatkey: packed_30_type
{=1488}  END;
prt_zone_type = short;
mask_timepair_tab_type = ARRAY [1..64] OF RECORD
{    1}  timepair: timepair_type;
{   15}  modify_in: char;
{   16}  modify_out: char;
{   17}  time_in_hrec: ARRAY [1..7] OF char;
{   24}  time_out_hrec: ARRAY [1..7] OF char
{=1920}  END;
ts_cplan_pointer = ^ts_cplan_type;
ts_cplan_type = RECORD
{    1}  calling_client: short;
{    3}  emplno: emplno_type;
{   13}  deptno: deptno_type;
{   23}  costid: costid_type;
{   43}  select_options: select_options_type;
{   59}  state: unsigned_byte;
{   60}  sort_char: char;
{   61}  indexname: indexname_type;
{   93}  cur_deptno: deptno_type;
{  103}  cur_costid: costid_type;
{  123}  cur_firstname: firstname_type;
{  143}  cur_surname: surname_type;
{  173}  cur_emplno: emplno_type;
{  183}  cur_date: date_type;
{  189}  cur_date_to: date_type;
{  195}  cur_time: mod_type;
{  197}  cur_apptype: short;
{  199}  last_deptno: deptno_type;
{  209}  last_costid: costid_type;
{  229}  emplno_apply_read: emplno_type;
{  239}  filler: ARRAY [1..2] OF filler_type;
{  241}  scb: scb_type
{= 412}  END;
ts_next_page_type = ARRAY [1..8] OF unsigned_byte;
ts_rpl_msg_pointer = ^ts_rpl_msg_type;
ts_rpl_msg_type = RECORD
{    1}  msg_header: ts_msg_header_type;
{   85}  error_code: code_type;
{   89}  sys_error: short;
{   91}  error_index: short;
{   93}  error_text: error_text_type;
{  153}  no_of_lines: short;
{  155}  more_flag: flag_type;
{  156}  filler: ARRAY [1..1] OF char;
{  157}  next_page: ts_next_page_type;
{  165}  rpldata: buffer_type
{= 166}  END;
ts_rpl_line_pointer = ^ts_rpl_line_type;
ts_rpl_line_type = RECORD
{    1}  formatno: long;
{    5}  line_flags: line_flags_type;
{    7}  line_data: buffer_type
{=   8}  END;
ts_cplan_lpt = ^ts_cplan_rpl_line;
ts_cplan_rpl_line = RECORD
{    1}  formatno: long;
{    5}  line_flags: line_flags_type;
{    7}  emplno: emplno_type;
{   17}  name: ARRAY [1..15] OF char;
{   32}  filler: ARRAY [1..3] OF char;
{   35}  deptno: deptno_type;
{   45}  plantype: unsigned_byte;
{   46}  state: char;
{   47}  date_from: date_type;
{   53}  date_to: date_type;
{   59}  days: short;
{   61}  time: mod_type;
{   63}  allowance_days: short;
{   65}  allowance: allowance_type;
{   67}  allowance_remainder: allowance_type;
{   69}  date_applied: date_type;
{   75}  date_granted: date_type;
{   81}  email_employee: email_address_type
{= 112}  END;
ts_cplan_rpl_pointer = ^ts_cplan_rpl_type;
ts_cplan_rpl_type = RECORD
{    1}  vacation_claim: balance_type;
{    5}  old_vacation: balance_type;
{    9}  vacation_taken: balance_type;
{   13}  vacation_planned: balance_type;
{   17}  vacation_to_plan: balance_type;
{   21}  balance1: balance_type;
{   25}  balance2: balance_type;
{   29}  balance3: balance_type;
{   33}  balance_date: date_type;
{   39}  indexname: indexname_type;
{   71}  cur_deptno: deptno_type;
{   81}  cur_costid: costid_type;
{  101}  cur_firstname: firstname_type;
{  121}  cur_surname: surname_type;
{  151}  cur_emplno: emplno_type;
{  161}  cur_date: date_type;
{  167}  cur_date_to: date_type;
{  173}  cur_apptype: short;
{  175}  cur_time: mod_type;
{  177}  email_self: email_address_type;
{  209}  email_manager: email_address_type;
{  241}  firstname_manager: firstname_type;
{  261}  surname_manager: surname_type;
{  291}  emplno_apply_read: emplno_type;
{  301}  last_deptno: deptno_type;
{  311}  last_costid: costid_type;
{  331}  self_inquiry_flag: flag_type;
{  332}  manager_flag: flag_type;
{  333}  read_all_flag: flag_type;
{  334}  low_vacation: flag_type;
{  335}  scb: scb_type;
{  507}  alignment1: ARRAY [1..2] OF char;
{  509}  line_tab: ARRAY [1..cplan_rpl_ct] OF ts_cplan_rpl_line
{=3532}  END;
ts_cplan_update_pointer = ^ts_cplan_update_type;
ts_cplan_update_type = RECORD
{    1}  old_line: ts_cplan_rpl_line;
{  113}  new_line: ts_cplan_rpl_line;
{  225}  vacation_claim: balance_type;
{  229}  old_vacation: balance_type;
{  233}  vacation_taken: balance_type;
{  237}  vacation_planned: balance_type;
{  241}  vacation_to_plan: balance_type;
{  245}  balance1: balance_type;
{  249}  balance2: balance_type;
{  253}  balance3: balance_type;
{  257}  balance_date: date_type;
{  263}  calling_client: short;
{  265}  self_inquiry_flag: flag_type;
{  266}  update_apply_type: char;
{  267}  read_new_flag: flag_type;
{  268}  low_vacation: flag_type
{= 268}  END;
comm_buffer_conv_pointer = ^comm_buffer_conv_type;
comm_buffer_conv_type = RECORD
{    1}  CASE cb_type: long OF
{+   4}     1: (byte: buffer_type);
{+   4}     2: (buffer: comm_buffer_type);
{+   4}     3: (req: ts_req_msg_type;
{+ 138}        alignment1: ARRAY [1..2] OF char);
{+   4}     4: (rpl: ts_rpl_msg_type;
{+ 170}        alignment2: ARRAY [1..2] OF char);
{=4100}  END;
page_pointer = ^page_type;
page_type = RECORD
{    1}  prev_page: page_pointer;
{    5}  next_page: page_pointer;
{    9}  page: comm_buffer_conv_type
{=4108}  END;
list_cb_pointer = ^list_cb_type;
list_cb_type = RECORD
{    1}  current_line: short;
{    3}  selected: short;
{    5}  no_of_lines: short;
{    7}  max_nol_in_page: short;
{    9}  first_page: page_pointer;
{   13}  last_page: page_pointer;
{   17}  no_of_top_lines: short;
{   19}  no_of_bottom_lines: short;
{   21}  const_page: page_pointer;
{   25}  use_attributes: boolean;
{   26}  filler1: ARRAY [1..1] OF char;
{   27}  selected_col: short;
{   29}  user_cursor: short;
{   31}  reserved_cursor: short
{=  32}  END;


{pspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspsp}
   {ts_demp-inc}
ts_demp_pointer = ^ts_demp_type;
ts_demp_type = RECORD
{    1}  byte: buffer_type;
{    3}  emplno: emplno_type;
{   13}  start_emplno: emplno_type;
{   23}  deptno: deptno_type;
{   33}  start_deptno: deptno_type;
{   43}  surname: surname_type;
{   73}  start_surname: surname_type;
{  103}  badgeno: long_badgeno_type;
{  123}  start_badgeno: long_badgeno_type;
{  143}  costid: costid_type;
{  163}  start_costid: costid_type;
{  183}  select_options: select_options_type;
{  199}  accesscat: acccatid_type;
{  203}  employcat: employcat_type;
{  207}  start_employcat: employcat_type;
{  211}  jobid: jobid_type;
{  219}  start_jobid: jobid_type;
{  227}  machineid: jobid_type;
{  235}  start_machineid: jobid_type;
{  243}  adjustcat: adjustcat_type;
{  247}  tariffarea: tariffarea_type;
{  251}  cycleid: cycleid_type;
{  255}  cycle_start: short;
{  257}  variants: char;
{  258}  fileid: ARRAY [1..10] OF char;
{  268}  first_page: flag_type;
{  269}  date_from: date_type;
{  275}  date_to: date_type;
{  281}  start_date: date_type;
{  287}  is_first_variant: flag_type;
{  288}  employ_flag: flag_type;
{  289}  visit_flag: flag_type;
{  290}  femp_flag: flag_type;
{  291}  company: long_text_type;
{  321}  carlicid: ARRAY [1..12] OF char;
{  333}  factory: packed_12_type;
{  345}  foreign_clientno: buffer_type;
{  347}  foreign_emplno: emplno_type;
{  357}  rpoolid: rpoolid_type;
{  369}  scb: scb_type
{= 540}  END;
ts_demp_download_lpt = ^ts_demp_download_line;
ts_demp_download_line = RECORD
{    1}  emplno: emplno_type;
{   11}  clientno: buffer_type;
{   13}  sign: char;
{   14}  filler: ARRAY [1..3] OF char
{=  16}  END;
ts_demp_download_pointer = ^ts_demp_download_type;
ts_demp_download_type = RECORD
{    1}  no_of_lines: short;
{    3}  no_of_ok_lines: short;
{    5}  more_flag: flag_type;
{    6}  first_call: flag_type;
{    7}  filler: ARRAY [1..2] OF char;
{    9}  line_tab: ARRAY [1..demp_download_ct] OF ts_demp_download_line
{=3608}  END;
ts_demp_lpt = ^ts_demp_rpl_line;
ts_demp_rpl_line = RECORD
{    1}  formatno: long;
{    5}  line: line_flags_type;
{    7}  emplno: emplno_type;
{   17}  pseudo_date: date_type;
{   23}  deptno: deptno_type;
{   33}  line_data: ARRAY [1..75] OF char;
{  108}  sfc_participant: flag_type;
{  109}  v_valid_from: date_type;
{  115}  employee: ARRAY [1..34] OF char;
{  149}  clientno: buffer_type;
{  151}  sign: char;
{  152}  position: signed_byte
{= 152}  END;
ts_demp_rpl_pointer = ^ts_demp_rpl_type;
ts_demp_rpl_type = RECORD
{    1}  byte: buffer_type;
{    3}  last_emplno: emplno_type;
{   13}  last_deptno: deptno_type;
{   23}  last_surname: surname_type;
{   53}  last_badgeno: long_badgeno_type;
{   73}  last_costid: costid_type;
{   93}  last_employcat: employcat_type;
{   97}  last_machineid: jobid_type;
{  105}  last_jobid: jobid_type;
{  113}  indexname: indexname_type;
{  145}  scb: scb_type;
{  317}  next_date: date_type;
{  323}  is_first_variant: flag_type;
{  324}  title1: vl_index_output_line_type;
{  399}  title2: vl_index_output_line_type;
{  474}  filler: ARRAY [1..9] OF char;
{  483}  cols_dinfo_tab: vl_cols_dinfo_type;
{  741}  vl_rec_type_tab: vl_rec_type_tab_type;
{  805}  line_tab: ARRAY [1..demp_rpl_ct] OF ts_demp_rpl_line
{=3540}  END;


{pspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspsp}
   get_line_pointer_type = FUNCTION (page: page_pointer; ind: short):
      ts_rpl_line_pointer;
   copy_line_to_page_type = PROCEDURE (line: ts_rpl_line_pointer;
      page: page_pointer; ind: short);

VAR
{pspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspsp}
demp_list       : list_cb_type;
message_code : code_type;

FUNCTION get_demp_line (page: page_pointer; line: short): ts_rpl_line_pointer;

{$I pageutlsH.inc}

IMPLEMENTATION

CONST
   versionno = 230;

{ *********************** init_list *************************************** }

PROCEDURE init_list  (VAR list: list_cb_type; max_nol: short) ;

{ initialisiert 'list_cb_type' }

BEGIN WITH list DO BEGIN
current_line := 1;
no_of_lines := 0;
first_page := nil;
last_page := nil;
max_nol_in_page := max_nol;
no_of_top_lines := 0;
no_of_bottom_lines := 0;
const_page := nil;
END; END; { init_list }

{ *********************** dispose_list ************************************ }

PROCEDURE dispose_list  (VAR list: list_cb_type) ;

{ gibt den Speicher der Liste frei und initialisiert 'list_cb_type' }

VAR
   page_temp: page_pointer;

BEGIN WITH list DO BEGIN
WHILE first_page <> nil DO
   BEGIN
   page_temp := first_page;
   first_page := first_page^.next_page;
   DISPOSE (page_temp);
   END;
WHILE const_page <> nil DO
   BEGIN
   page_temp := const_page;
   const_page := const_page^.next_page;
   DISPOSE (page_temp);
   END;
init_list (list, max_nol_in_page);
END; END; { dispose_list }

{ *********************** copy_list *************************************** }

PROCEDURE copy_list  (source: list_cb_type; VAR dest: list_cb_type) ;

{ erstellt eine identische Kopie der Liste source }

VAR
   p, source_ptr, dest_ptr: page_pointer;

BEGIN

dispose_list (dest);
dest := source;
dest.first_page := nil;
dest.last_page := nil;
dest.const_page := nil;

source_ptr := source.first_page;
dest_ptr := nil;
WHILE source_ptr <> nil DO
   BEGIN
   new (p);
   IF p = nil THEN
     EXIT(**); { memory allocation failure }
   p^.prev_page := dest_ptr;
   p^.next_page := nil;
   p^.page := source_ptr^.page;
   IF dest_ptr = nil THEN
      dest.first_page := p
   ELSE
      dest_ptr^.next_page := p;
   dest.last_page := p;
   dest_ptr := p;
   source_ptr := source_ptr^.next_page;
   END;

source_ptr := source.const_page;
dest_ptr := nil;
WHILE source_ptr <> nil DO
   BEGIN
   new (p);
   IF p = nil THEN
     EXIT(**); { memory allocation failure }
   p^.prev_page := dest_ptr;
   p^.next_page := nil;
   p^.page := source_ptr^.page;
   IF dest_ptr = nil THEN
      dest.const_page := p
   ELSE
      dest_ptr^.next_page := p;
   dest_ptr := p;
   source_ptr := source_ptr^.next_page;
   END;

END; { copy_list }

{ ************************ get_line *************************************** }

PROCEDURE get_line  (list: list_cb_type; line: short; VAR page: page_pointer;
   VAR ind: short) ;

{ setzt page/ind auf Zeile Nummer 'line' (fortlaufend)  }

BEGIN
page := list.first_page;
ind := line;
IF line < 1 THEN
   page := nil;
WHILE page <> nil DO
   BEGIN
   IF ind <= page^.page.rpl.no_of_lines THEN
      EXIT(**);
   ind := ind - page^.page.rpl.no_of_lines;
   page := page^.next_page;
   END;
END; { get_line }

{ ************************ get_const_line ********************************* }

PROCEDURE get_const_line  (list: list_cb_type; line: short; VAR page: page_pointer;
   VAR ind: short) ;

{ setzt page/ind auf Zeile Nummer 'line' (fortlaufend) in den festen Zeilen }

BEGIN
page := list.const_page;
ind := line;
IF line < 1 THEN
   page := nil;
WHILE page <> nil DO
   BEGIN
   IF ind <= page^.page.rpl.no_of_lines THEN
      EXIT(**);
   ind := ind - page^.page.rpl.no_of_lines;
   page := page^.next_page;
   END;
END; { get_const_line }

{ ************************ get_next_line ********************************** }

PROCEDURE get_next_line  (list: list_cb_type; VAR page: page_pointer;
   VAR ind: short) ;

{ holt den Nachfolger von page/ind }
{ Parameter 'list' obsolete }

VAR
   cur_last_line: short;

BEGIN
IF page = nil THEN
   EXIT(**);
ind := ind + 1;
cur_last_line := page^.page.rpl.no_of_lines;
WHILE (page <> nil) AND (ind > cur_last_line) DO
   BEGIN
   page := page^.next_page;
   ind := 1;
   IF page <> nil THEN
      cur_last_line := page^.page.rpl.no_of_lines;
   END;
END; { get_next_line }

{ ************************ get_prev_line ********************************** }

PROCEDURE get_prev_line  (list: list_cb_type; VAR page: page_pointer;
   VAR ind: short) ;

{ holt den Vorgaenger von page/ind }
{ Parameter 'list' obsolete }

BEGIN
IF page = nil THEN
   EXIT(**);
ind := ind - 1;
WHILE (page <> nil) AND (ind < 1) DO
   BEGIN
   page := page^.prev_page;
   IF page <> nil THEN
      ind := page^.page.rpl.no_of_lines;
   END;
END; { get_prev_line }

{ ************************ get_next_visible_line ************************** }

PROCEDURE get_next_visible_line  (list: list_cb_type;
   VAR page: page_pointer; VAR ind: short; VAR line: short;
    get_line_pointer:  get_line_pointer_type) ;

{ holt den naechsten sichtbaren Nachfolger von page/ind;
  erhoeht 'line' um die bis dahin ueberlesenen versteckten Zeilen }

VAR
   attribute: char;
   selection: char;

BEGIN
REPEAT
   get_next_line (list, page, ind);
   get_line_flags (page, ind, selection, attribute, get_line_pointer);
   IF attribute = attribute_hide THEN
      line := line + 1;
   UNTIL (page = nil) OR (attribute <> attribute_hide);
END; { get_next_visible_line }


{ ************************ get_first_visible_line ************************** }
PROCEDURE get_first_visible_line  (list: list_cb_type;
   VAR cur: short; VAR page: page_pointer; VAR ind: short;
   VAR line: short;  get_line_pointer:  get_line_pointer_type);


VAR
   attribute: char;
   selection: char;
   hidden: short;

BEGIN
hidden := 0;
get_line (list, cur, page, ind);
get_line_flags (page, ind, selection, attribute, get_line_pointer);
IF attribute = attribute_hide THEN
   BEGIN
   hidden := 1;
   get_next_visible_line (list, page, ind, hidden, get_line_pointer);
   END;
line := line + hidden;
cur := cur + hidden;
END;


{ ************************ get_prev_visible_line ************************** }

PROCEDURE get_prev_visible_line  (list: list_cb_type;
   VAR page: page_pointer; VAR ind: short; VAR line: short;
    get_line_pointer:  get_line_pointer_type) ;

{ holt den naechsten sichtbaren Vorgaenger von page/ind;
  erniedrigt 'line' um die bis dahin ueberlesenen versteckten Zeilen }

VAR
   attribute: char;
   selection: char;

BEGIN
REPEAT
   get_prev_line (list, page, ind);
   get_line_flags (page, ind, selection, attribute, get_line_pointer);
   IF attribute = attribute_hide THEN
      line := line - 1;
   UNTIL (page = nil) OR (attribute <> attribute_hide);
END; { get_prev_visible_line }

{ ************************* append_page *********************************** }

PROCEDURE append_page  (VAR list: list_cb_type;
   buffer_ptr: ts_rpl_msg_pointer; to_page: page_pointer) ;

{ haengt die Reply 'buffer_ptr' als Seite an 'to_page' an }

VAR
   p: page_pointer;

BEGIN
NEW (p);
IF p = nil THEN
   EXIT(**); { memory allocation failure }
MOVE (buffer_ptr^.msg_header.byte[1], p^.page.byte[1], comm_buffer_len);
list.no_of_lines := list.no_of_lines + buffer_ptr^.no_of_lines;

IF to_page = nil THEN
   BEGIN
   p^.prev_page := nil;
   p^.next_page := nil;
   END
ELSE
   BEGIN
   p^.prev_page := to_page;
   p^.next_page := to_page^.next_page;
   to_page^.next_page := p;
   IF p^.next_page <> nil THEN
      p^.next_page^.prev_page := p;
   END;

IF list.first_page = nil THEN
   BEGIN
   list.first_page := p;
   list.last_page := p;
   END
ELSE
   IF list.last_page = to_page THEN
      list.last_page := p;
END; { append_page }

{ ************************* append_const_page ***************************** }

PROCEDURE append_const_page  (VAR list: list_cb_type;
   buffer_ptr: ts_rpl_msg_pointer; to_page: page_pointer) ;

{ haengt die Reply 'buffer_ptr' als Seite an 'to_page' der festen Zeilen an }

VAR
   p: page_pointer;

BEGIN
NEW (p);
IF p = nil THEN
   EXIT(**); { memory allocation failure }
MOVE (buffer_ptr^.msg_header.byte[1], p^.page.byte[1], comm_buffer_len);
list.no_of_bottom_lines := list.no_of_bottom_lines + buffer_ptr^.no_of_lines;

IF to_page = nil THEN
   BEGIN
   p^.prev_page := nil;
   p^.next_page := nil;
   END
ELSE
   BEGIN
   p^.prev_page := to_page;
   p^.next_page := to_page^.next_page;
   to_page^.next_page := p;
   IF p^.next_page <> nil THEN
      p^.next_page^.prev_page := p;
   END;

IF list.const_page = nil THEN
   BEGIN
   list.const_page := p;
   list.const_page := p;
   END;
END; { append_const_page }

{ ************************* create_const_page ***************************** }

PROCEDURE create_const_page  (VAR list: list_cb_type; notl, nobl: short;
   buffer_ptr: ts_rpl_msg_pointer) ;

{ erzeugt die Seite fuer Anzeige konstanter Zeilen in der display_logic;
  die Seite wird mit 'notl' (no_of_top_lines), 'nobl' (no_of_bottom_lines)
  und 'buffer_ptr' initialisiert; eine Aufruf mit 0,0,nil ist erlaubt }

BEGIN
{ erzeuge konstante Seite }
IF list.const_page <> nil THEN
   EXIT(**);
NEW (list.const_page);
IF list.const_page = nil THEN
   EXIT(**); { memory allocation failure }

{ fuelle Seite }
IF buffer_ptr <> nil THEN
   MOVE (buffer_ptr^.msg_header.byte[1], list.const_page^.page.byte[1],
      comm_buffer_len);
list.const_page^.prev_page := nil;
list.const_page^.next_page := nil;
list.no_of_top_lines := notl;
list.no_of_bottom_lines := nobl;
END; { create_const_page }

{ ************************* more ****************************************** }

FUNCTION more  (list: list_cb_type): boolean ;

{ gibt das more_flag der Liste zurueck }

BEGIN
IF list.last_page = nil THEN
   more := false
ELSE
   more := (list.last_page^.page.rpl.more_flag = flag_on);
END; { more }

{ ************************* nol_in_page *********************************** }

FUNCTION nol_in_page  (page: page_pointer): short ;

{ no of lines in page }

BEGIN
IF page = nil THEN
   nol_in_page := 0
ELSE
   nol_in_page := page^.page.rpl.no_of_lines;
END; { nol_in_page }

{ ************************* set_nol_in_list ******************************* }

PROCEDURE set_nol_in_page  (page: page_pointer; nol: short) ;

{ Setze 'no of lines in page' auf einen neuen Wert }

BEGIN
IF page <> nil THEN
   page^.page.rpl.no_of_lines := nol;
END; { set_nol_in_page }

{ ************************* change_line *********************************** }

PROCEDURE change_line  (VAR list: list_cb_type; mode: char;
   lpt: ts_rpl_line_pointer; dest_page: page_pointer; dest_ind: short;
    get_line_pointer:  get_line_pointer_type;
    copy_line_to_page:  copy_line_to_page_type
   ) ;

{ Kopieren, Loeschen und Einfuegen einer Zeile in eine Seite:
  Kopieren :  kopiere lpt nach dest
     change_line (list, change_copy, $ADDR (line), dest_page, dest_ind);
  Loeschen :  loesche dest
     change_line (list, change_delete, nil, dest_page, dest_ind);
  Einfuegen:  fuege lpt vor dest ein
     change_line (list, change_insert, $ADDR (line), dest_page, dest_ind); }

VAR
   i: short;
   nol   : short;
   buffer: comm_buffer_conv_type;

BEGIN

CASE mode OF

   change_delete:
      { loescht eine Zeile }
      BEGIN
      FOR i := dest_ind + 1 TO nol_in_page (dest_page) DO
         copy_line_to_page (get_line_pointer (dest_page, i), dest_page, i - 1);
      list.no_of_lines := list.no_of_lines - 1;
      set_nol_in_page (dest_page, nol_in_page (dest_page) - 1);
      END;

   change_insert:
      { fuegt eine Zeile vor 'dest' ein;
        falls die Zeile auf der geforderten Seite keinen Platz hat, wird
        eine neue Seite eingefuegt }
      BEGIN
      IF dest_page = nil THEN
         IF (list.first_page <> nil) AND (list.no_of_lines = 0) THEN
            BEGIN
            { Liste enthaelt nur leere Seiten }
            dest_page := list.first_page;
            dest_ind := 1;
            END
         ELSE
            BEGIN
            { die angesprochene Seite existiert nicht: fuege leere Seite an;
              falls bereits eine existiert, dann verwende diese bereits
              initialisierte Seite }
            IF list.first_page <> nil THEN
               buffer.buffer := list.first_page^.page.buffer;
            buffer.rpl.no_of_lines := 0;
            buffer.rpl.more_flag := flag_off;
            append_page (list, @buffer.rpl , list.last_page);
            dest_page := list.last_page;
            dest_ind := 1;
            IF dest_page = nil THEN
               EXIT(**);
            END;

      IF nol_in_page (dest_page) = list.max_nol_in_page THEN
         { es musz eine neue Seite eingefuegt werden }
         BEGIN
         nol := list.no_of_lines; { merke Anzahl Zeilen }

         { dupliziere dest_page und fuege sie hinter dest_page ein }
         append_page (list, @dest_page^.page.rpl , dest_page);

         { ueberschreibe dest_ind mit neuer Zeile }
         copy_line_to_page (lpt, dest_page, dest_ind);

         { loesche alle weiteren Zeilen in dieser Seite }
         set_nol_in_page (dest_page, dest_ind);

         { loesche die Zeilen 1 bis dest_ind - 1 in zweiter Seite }
         FOR i := 1 TO dest_ind - 1 DO
            change_line (list, change_delete, nil, dest_page^.next_page, 1,
               get_line_pointer, copy_line_to_page);

         list.no_of_lines := nol + 1; { restauriere Anzahl Zeilen }
         END
      ELSE
         BEGIN
         { verschiebe nachfolgende Zeilen }
         FOR i := nol_in_page (dest_page) DOWNTO dest_ind DO
            copy_line_to_page (get_line_pointer (dest_page, i), dest_page, i + 1);

         { kopiere Zeile in freigewordenen Platz }
         copy_line_to_page (lpt, dest_page, dest_ind);

         { erhoehe Anzahl Zeilen }
         list.no_of_lines := list.no_of_lines + 1;
         set_nol_in_page (dest_page, nol_in_page (dest_page) + 1);
         END;
      END;

   change_copy:
      { kopiert die Zeile lpt nach dest }
      copy_line_to_page (lpt, dest_page, dest_ind);

   ELSE(**)
   END;

END; { change_line }

{ ************************ get_last_const_page **************************** }

FUNCTION get_last_const_page (list: list_cb_type): page_pointer;

{ holt letzte Seite in den 'festen' Zeilen }

VAR
   page: page_pointer;

BEGIN
page := list.const_page;
IF page <> nil THEN
   WHILE page^.next_page <> nil DO
      page := page^.next_page;
get_last_const_page := page;
END; { get_last_const_page }

{ ************************* change_const_line ***************************** }

PROCEDURE change_const_line  (VAR list: list_cb_type; mode: char;
   lpt: ts_rpl_line_pointer; dest_page: page_pointer; dest_ind: short;
    get_line_pointer:  get_line_pointer_type;
    copy_line_to_page:  copy_line_to_page_type
   ) ;

{ Version fuer 'feste Zeilen' }

{ Kopieren, Loeschen und Einfuegen einer Zeile in eine Seite:
  Kopieren :  kopiere lpt nach dest
     change_line (list, change_copy, $ADDR (line), dest_page, dest_ind);
  Loeschen :  loesche dest
     change_line (list, change_delete, nil, dest_page, dest_ind);
  Einfuegen:  fuege lpt vor dest ein
     change_line (list, change_insert, $ADDR (line), dest_page, dest_ind); }

VAR
   i: short;
   nol   : short;
   buffer: comm_buffer_conv_type;

BEGIN

CASE mode OF

   change_delete:
      { loescht eine Zeile }
      BEGIN
      FOR i := dest_ind + 1 TO nol_in_page (dest_page) DO
         copy_line_to_page (get_line_pointer (dest_page, i), dest_page, i - 1);
      list.no_of_bottom_lines := list.no_of_bottom_lines - 1;
      set_nol_in_page (dest_page, nol_in_page (dest_page) - 1);
      END;

   change_insert:
      { fuegt eine Zeile vor 'dest' ein;
        falls die Zeile auf der geforderten Seite keinen Platz hat, wird
        eine neue Seite eingefuegt }
      BEGIN
      IF dest_page = nil THEN
         IF (list.const_page <> nil) AND (list.no_of_bottom_lines = 0) THEN
            BEGIN
            { Liste enthaelt nur leere Seiten }
            dest_page := list.const_page;
            dest_ind := 1;
            END
         ELSE
            BEGIN
            { die angesprochene Seite existiert nicht: fuege leere Seite an;
              falls bereits eine existiert, dann verwende diese bereits
              initialisierte Seite }
            IF list.const_page <> nil THEN
               buffer.buffer := list.const_page^.page.buffer;
            buffer.rpl.no_of_lines := 0;
            buffer.rpl.more_flag := flag_off;
            append_const_page (list, @buffer.rpl , get_last_const_page (list));
            dest_page := get_last_const_page (list);
            dest_ind := 1;
            IF dest_page = nil THEN
               EXIT(**);
            END;

      IF nol_in_page (dest_page) = list.max_nol_in_page THEN
         { es musz eine neue Seite eingefuegt werden }
         BEGIN
         nol := list.no_of_bottom_lines; { merke Anzahl Zeilen }

         { dupliziere dest_page und fuege sie hinter dest_page ein }
         append_const_page (list, @dest_page^.page.rpl , dest_page);

         { ueberschreibe dest_ind mit neuer Zeile }
         copy_line_to_page (lpt, dest_page, dest_ind);

         { loesche alle weiteren Zeilen in dieser Seite }
         set_nol_in_page (dest_page, dest_ind);

         { loesche die Zeilen 1 bis dest_ind - 1 in zweiter Seite }
         FOR i := 1 TO dest_ind - 1 DO
            change_const_line (list, change_delete, nil, dest_page^.next_page, 1,
               get_line_pointer, copy_line_to_page);

         list.no_of_bottom_lines := nol + 1; { restauriere Anzahl Zeilen }
         END
      ELSE
         BEGIN
         { verschiebe nachfolgende Zeilen }
         FOR i := nol_in_page (dest_page) DOWNTO dest_ind DO
            copy_line_to_page (get_line_pointer (dest_page, i), dest_page, i + 1);

         { kopiere Zeile in freigewordenen Platz }
         copy_line_to_page (lpt, dest_page, dest_ind);

         { erhoehe Anzahl Zeilen }
         list.no_of_bottom_lines := list.no_of_bottom_lines + 1;
         set_nol_in_page (dest_page, nol_in_page (dest_page) + 1);
         END;
      END;

   change_copy:
      { kopiert die Zeile lpt nach dest }
      copy_line_to_page (lpt, dest_page, dest_ind);

   ELSE(**)
   END;

END; { change_const_line }

{ ************************* set_line_flags ******************************** }

PROCEDURE set_line_flags 
   (page: page_pointer; ind: short; select, attribute: char;
    get_line_pointer:  get_line_pointer_type) ;

{ Setze Zeilenattribute der Zeile page/ind auf select/attribute:
     select    = ' ' keine Aenderung
     select    = invert_flag, dann invertiere 'selected'
     attribute = ' ' keine Aenderung }

VAR
   lpt: ts_rpl_line_pointer;

BEGIN
lpt := get_line_pointer (page, ind);
IF lpt <> nil THEN
   BEGIN
   IF select <> ' ' THEN
      IF select = invert_flag THEN
         IF (lpt^.line_flags.selected = flag_on) OR
            (lpt^.line_flags.selected = action_select) OR
            (lpt^.line_flags.selected = chr(1)) THEN
            lpt^.line_flags.selected := flag_off
         ELSE
            lpt^.line_flags.selected := flag_on
      ELSE
         lpt^.line_flags.selected := select;
   IF attribute <> ' ' THEN
      lpt^.line_flags.attribute := attribute;
   END;
END; { set_line_flags }

{ ************************* get_line_flags ******************************** }

PROCEDURE get_line_flags 
   (page: page_pointer; ind: short; VAR select, attribute: char;
    get_line_pointer:  get_line_pointer_type) ;

{ holt die Attribute der Zeile }

VAR
   lpt: ts_rpl_line_pointer;

BEGIN
lpt := get_line_pointer (page, ind);
IF lpt = nil THEN
   BEGIN
   select := flag_off;
   attribute := attribute_normal;
   END
ELSE
   BEGIN
   select := lpt^.line_flags.selected;
   attribute := lpt^.line_flags.attribute;
   END;
END; { get_line_flags }

{ ************************* is_selected *********************************** }

FUNCTION is_selected 
   (list: list_cb_type; page: page_pointer; ind: short;
    get_line_pointer:  get_line_pointer_type): boolean ;

{ prueft auf Selektion einer Zeile:
     eine Zeile ist selektiert, wenn
          'line_flags.selected' = flag_on oder action_selected oder chr(1)
     oder 'liste.selected' auf dieser Zeile steht }

VAR
   selpage  : page_pointer;
   selind   : short;
   selected : char;
   attribute: char;

BEGIN
get_line (list, list.selected, selpage, selind);
IF (page = nil) OR (selpage = nil) THEN
   BEGIN
   is_selected := false;
   EXIT(**)
   END;
IF (page = selpage) AND (ind = selind) THEN
   is_selected := true
ELSE
   BEGIN
   get_line_flags (page, ind, selected, attribute, get_line_pointer);
   is_selected := (selected = flag_on) OR (selected = action_select) OR
      (selected = chr(1));
   END;
END; { is_selected }

{ ************************* set_list_line_flags *************************** }

PROCEDURE set_list_line_flags 
   (VAR list: list_cb_type; select, attribute: char;
    get_line_pointer:  get_line_pointer_type
   ) ;

{ Setze alle Zeilenattribute der Liste auf die neuen Werte
  (siehe 'set_line_flags') }

VAR
   page: page_pointer;
   ind : short;

BEGIN
page := list.first_page;
ind := 1;
WHILE page <> nil DO
   BEGIN
   set_line_flags (page, ind, select, attribute, get_line_pointer);
   get_next_line (list, page, ind);
   END;
END; { set_list_line_flags }

{ ************************* get_next_selected_line ************************ }

PROCEDURE get_next_selected_line 
   (VAR list: list_cb_type; VAR page: page_pointer; VAR ind: short;
    get_line_pointer:  get_line_pointer_type) ;

{ holt den naechsten selektierten Nachfolger von page/ind }

VAR
   selected : char;
   attribute: char;

BEGIN
REPEAT
   get_next_line (list, page, ind);
   list.selected := list.selected + 1;
   IF page = nil THEN
      EXIT(**);
   get_line_flags (page, ind, selected, attribute, get_line_pointer);
   IF (selected = flag_on) OR (selected = action_select) OR (selected = chr(1)) THEN
      EXIT(**);
   UNTIL false;
END; { get_next_selected_line }

{ ************************* get_prev_selected_line ************************ }

PROCEDURE get_prev_selected_line 
   (VAR list: list_cb_type; VAR page: page_pointer; VAR ind: short;
    get_line_pointer:  get_line_pointer_type) ;

{ holt den naechsten selektierten Vorgaenger von page/ind }

VAR
   selected : char;
   attribute: char;

BEGIN
REPEAT
   get_prev_line (list, page, ind);
   list.selected := list.selected - 1;
   IF page = nil THEN
      EXIT(**);
   get_line_flags (page, ind, selected, attribute, get_line_pointer);
   IF (selected = flag_on) OR (selected = action_select) OR (selected = chr(1)) THEN
      EXIT(**);
   UNTIL false;
END; { get_prev_selected_line }

{ ************************* get_first_selected_line_m *********************** }

PROCEDURE get_first_selected_line_m 
   (VAR list: list_cb_type; VAR page: page_pointer; VAR ind: short;
    get_line_pointer:  get_line_pointer_type) ;

{ holt die erste selektierte Zeile der Liste }

VAR
   selected : char;
   attribute: char;

BEGIN
list.selected := 1;
get_line (list, 1, page, ind);
get_line_flags (page, ind, selected, attribute, get_line_pointer);
IF (selected = flag_on) OR (selected = action_select) OR (selected = chr(1)) THEN
   EXIT(**);
get_next_selected_line (list, page, ind, get_line_pointer);
END; { get_first_selected_line_m }

{ ************************* get_first_selected_line ************************ }

PROCEDURE get_first_selected_line 
   (VAR list: list_cb_type; VAR page: page_pointer; VAR ind: short;
    get_line_pointer:  get_line_pointer_type) ;

{ holt die erste selektierte Zeile der Liste }

BEGIN
get_line (list, 1, page, ind);
IF page = nil THEN
   EXIT(**);
IF is_selected (list, page, ind, get_line_pointer) THEN
   EXIT(**);
list.selected := 1;
get_next_selected_line (list, page, ind, get_line_pointer);
END; { get_first_selected_line }

{ ************************* get_selected_emplno *************************** }

CONST
   cur_emplno = 1;
   next_emplno = 2;
   prev_emplno = 3;

TYPE
{hie war $I tsdemp.inc}

   conv_demp_type = RECORD
      CASE ptr_type: integer OF
         1: (buffer_ptr: buffer_pointer);
         2: (req_ptr   : ts_demp_pointer);
         3: (rpl_ptr   : ts_demp_rpl_pointer);
         END;
   lpt_demp_ptr_type = RECORD
      CASE ptr_type: integer OF
         1: (lpt     : ts_demp_lpt);
         2: (rpl_lpt : ts_rpl_line_pointer);
         END;

FUNCTION get_demp_line (page: page_pointer; line: short): ts_rpl_line_pointer;

VAR
   c1: conv_demp_type;
   c2: lpt_demp_ptr_type;

BEGIN
get_demp_line := nil;
IF (line < 1) OR (line > demp_rpl_ct) THEN
   page := nil;
IF page = nil THEN
   EXIT(**);
c1.buffer_ptr := @page^.page.rpl.rpldata ;
c2.lpt := @c1.rpl_ptr^.line_tab(.line.) ;
get_demp_line := c2.rpl_lpt;
END; { get_demp_line }

FUNCTION get_selected_emplno (which_emplno: short): string;

VAR
   c1: lpt_demp_ptr_type;
   page: page_pointer;
   line: short;
   old_selected : short;

BEGIN

get_selected_emplno := '';
old_selected := demp_list.selected;

get_line (demp_list, demp_list.selected, page, line);
IF page = nil THEN
   BEGIN
   message_code := '6601';
   EXIT(**)
   END;

IF which_emplno = next_emplno THEN
   get_next_selected_line (demp_list, page, line, get_demp_line);
IF which_emplno = prev_emplno THEN
   get_prev_selected_line (demp_list, page, line, get_demp_line);

IF page = nil THEN
   BEGIN
   demp_list.selected := old_selected;
   IF which_emplno = prev_emplno THEN
      message_code := '6602';
   IF which_emplno = next_emplno THEN
      message_code := '6603';
   END
ELSE
   BEGIN
   c1.rpl_lpt := get_demp_line (page, line);

{pspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspspsp}
{   get_selected_emplno := STRI (@c1.lpt^.emplno, SizeOf (c1.lpt^.emplno)) ;}
   END;

END; { get_selected_emplno }

{ ************************* get_next_selected_emplno *********************** }

FUNCTION get_next_selected_emplno  : string ;

BEGIN
get_next_selected_emplno := get_selected_emplno (next_emplno);
END; { get_next_selected_emplno }

{ ************************* get_prev_selected_emplno *********************** }

FUNCTION get_prev_selected_emplno  : string ;

BEGIN
get_prev_selected_emplno := get_selected_emplno (prev_emplno);
END; { get_prev_selected_emplno }

BEGIN
END;

PROCEDURE timestamp_to_date
   (at_timestamp : long; VAR date: date_type)
   ;

{ converts a number of days (where 1.1.1901 is 1) into a date }

LABEL 1;

{ number of days passed, when the i+1-st month starts }


(*
CONST month_days  : ARRAY [0..11] of short =
      (0,31,59,90,120,151,181,212,243,273,304,334);
*)



VAR
   i, no_of_leap_years: short;

BEGIN

IF at_timestamp <= 0 THEN WITH date DO
   BEGIN
   yy := 0; mo := 0; dd := 0;
   EXIT(**)
   END;

date.yy := (at_timestamp DIV 365) + 1901;
no_of_leap_years := (date.yy - 1901) DIV 4;
IF no_of_leap_years >= (at_timestamp MOD 365) THEN
   BEGIN
   date.yy := date.yy - 1;
   at_timestamp := 365 - (no_of_leap_years - (at_timestamp MOD 365));
   IF ((date.yy MOD 4) = 0) THEN
      at_timestamp := at_timestamp + 1; {leap year is not yet completed, but
                                    one day has already been subtracted before}
   END
ELSE
   at_timestamp := (at_timestamp MOD 365) - no_of_leap_years;

IF ((date.yy MOD 4) = 0) AND (at_timestamp = 60) THEN
   BEGIN
   date.mo := 2;
   date.dd := 29;
   EXIT(**)
   END
ELSE
   BEGIN
   FOR i := 1 TO 11 DO
      IF at_timestamp <= month_days[i] THEN
         BEGIN
         date.mo := i;
         date.dd := at_timestamp - month_days[i-1];
         GOTO 1;
         END;
   date.mo := 12;
   date.dd := at_timestamp - month_days[11];
   END;

1:
IF ((date.yy MOD 4) = 0) AND (date.mo > 2) THEN
   {subtract one day}
   IF date.dd = 1 THEN
      BEGIN
      date.mo := date.mo - 1;
      {last day of date.mo}
      date.dd := month_days[date.mo] - month_days[date.mo - 1];
      END
   ELSE
      date.dd := date.dd - 1;

END; { timestamp_to_date }

(* end of location *)

BEGIN
END.
