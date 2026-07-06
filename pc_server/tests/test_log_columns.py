"""v2 ログ列契約: 94 列(77 + 末尾 17 列、順序は LOG_STRUCTURE v2)。"""

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


def test_column_count_is_94():
    assert len(COLUMNS) == 94


def test_v2_columns_appended_in_contract_order():
    assert COLUMNS[-17:] == V2_TAIL_COLUMNS


def test_v1_prefix_unchanged():
    # v1 の 77 列は先頭からそのまま(末尾追加のみの契約)
    assert COLUMNS[0] == "timestamp"
    assert COLUMNS[76] == "tlm_loop_dt_us"
