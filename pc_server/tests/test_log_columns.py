"""ログ列契約: 100 列(77 + v2 末尾 17 列 + v3 末尾 6 列、順序は LOG_STRUCTURE)。"""

from __future__ import annotations

from core.logger import COLUMNS

V2_TAIL_COLUMNS = (
    "cmd_yaw_ref_rad", "cmd_yaw_ref_deg", "yaw_ctrl_on",
    "tlm_yaw_est_rad", "tlm_yaw_gyro_int_rad", "tlm_yaw_ref_rad",
    "tlm_current_a", "tlm_db_hat_x_ut", "tlm_db_hat_y_ut",
    "tlm_bm_x_ut", "tlm_bm_y_ut",
    "tlm_nis", "tlm_ffg", "tlm_ff_status",
    "mocap_yaw_deg", "traj_mode", "traj_phase_rad",
)

V3_TAIL_COLUMNS = (
    "xy_cmd_mode",
    "cmd_err_x_m", "cmd_err_y_m", "cmd_xy_valid", "cmd_mocap_yaw_deg",
    "mocap_heading_deg",
)


def test_column_count_is_100():
    assert len(COLUMNS) == 100


def test_v3_columns_appended_in_contract_order():
    assert COLUMNS[-6:] == V3_TAIL_COLUMNS


def test_v2_columns_precede_v3_in_contract_order():
    assert COLUMNS[-23:-6] == V2_TAIL_COLUMNS


def test_v1_prefix_unchanged():
    # v1 の 77 列は先頭からそのまま(末尾追加のみの契約)
    assert COLUMNS[0] == "timestamp"
    assert COLUMNS[76] == "tlm_loop_dt_us"
