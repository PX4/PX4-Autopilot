
#include "battery.h"

Battery1::Battery1()
{
	updateParams();

	migrateParam(_param_old_bat_v_empty, _param_bat_v_empty, "V_EMPTY", 3.5f);
	migrateParam(_param_old_bat_v_charged, _param_bat_v_charged, "V_CHARGED", 4.05f);
	migrateParam(_param_old_bat_v_load_drop, _param_bat_v_load_drop, "V_LOAD_DROP", 0.3f);
	migrateParam(_param_old_bat_r_internal, _param_bat_r_internal, "R_INTERNAL", -1.0f);
	migrateParam(_param_old_bat_n_cells, _param_bat_n_cells, "N_CELLS", 0);
	migrateParam(_param_old_bat_capacity, _param_bat_capacity, "CAPACITY", -1.0f);
}
