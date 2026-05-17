#include "VOFA.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "globals.hpp"

/* -------------------------------------------------------------------------- */
/*  VOFA+ JustFloat 上行 (TX)                                                 */
/* -------------------------------------------------------------------------- */

void VOFA_Prepare_Package(float x[], Vofa_TxFrame &tx_buffer, int len) {
  for (int i = 0; i < len; i++) {
    tx_buffer.x[i] = x[i];
  }
}

void VOFA_Send_JustFloat_DMA(UART_HandleTypeDef *huart, Vofa_TxFrame &tx_buffer) {
  if (huart->gState == HAL_UART_STATE_READY) {
    HAL_UART_Transmit_DMA(huart, reinterpret_cast<const uint8_t *>(&tx_buffer), sizeof(Vofa_TxFrame));
  }
}

__attribute__((section(".sram4"))) Vofa_TxFrame tx_package;

void VOFASend() {
  float buffer[10];
  buffer[0] = globals->shooter_motor_1.rpm();
  buffer[1] = globals->shooter_motor_2.rpm();
  buffer[2] = globals->shooter_motor_3.rpm();
  buffer[3] = globals->shooter_motor_4.rpm();
  buffer[4] = globals->shooter_motor_5.rpm();
  buffer[5] = globals->shooter_motor_6.rpm();
  buffer[6] = -globals->ahrs.yaw();
  buffer[7] = -globals->ahrs.pitch();
  buffer[8] = globals->ahrs.roll();
  buffer[9] = 1;
  VOFA_Prepare_Package(buffer, tx_package, 10);
  // .sram4 NOLOAD: 帧尾需要每次显式写入, POR 上电时为随机值
  tx_package.tail[0] = 0x00;
  tx_package.tail[1] = 0x00;
  tx_package.tail[2] = 0x80;
  tx_package.tail[3] = 0x7F;
  VOFA_Send_JustFloat_DMA(&huart7, tx_package);
}

/* -------------------------------------------------------------------------- */
/*  PID 调参表 — 函数局部静态延迟初始化 (globals 在 AppMain 中才创建)               */
/* -------------------------------------------------------------------------- */

static const ParamEntry *GetParamTable(int &count) {
  // 首次调用时初始化 (此时 globals 已有效)
  static ParamEntry table[] = {
      // clang-format off

      // ── 底盘速度环 (x4, 统一修改) ──────────────────────────
      {"kp_cv",  "Chassis vel Kp",      globals->velocity_pid, 4, 'p', 10.f},
      {"ki_cv",  "Chassis vel Ki",      globals->velocity_pid, 4, 'i', 0.5f},
      {"kd_cv",  "Chassis vel Kd",      globals->velocity_pid, 4, 'd', 0.f},

      // ── 底盘随动 ──────────────────────────────────────────
      {"kp_cf_pos", "Chassis follow pos Kp", &globals->pid_chassis_follow_pos, 1, 'p', 300.f},
      {"ki_cf_pos", "Chassis follow pos Ki", &globals->pid_chassis_follow_pos, 1, 'i', 0.f},
      {"kd_cf_pos", "Chassis follow pos Kd", &globals->pid_chassis_follow_pos, 1, 'd', 3.f},
      {"kp_cf_vel", "Chassis follow vel Kp", &globals->pid_chassis_follow_vel, 1, 'p', 36.f},
      {"ki_cf_vel", "Chassis follow vel Ki", &globals->pid_chassis_follow_vel, 1, 'i', 0.f},
      {"kd_cf_vel", "Chassis follow vel Kd", &globals->pid_chassis_follow_vel, 1, 'd', 0.2f},

      // ── 云台 yaw ──────────────────────────────────────────
      {"kp_yaw_pos", "Yaw pos Kp",  &globals->pid_yaw_position, 1, 'p', 20.f},
      {"ki_yaw_pos", "Yaw pos Ki",  &globals->pid_yaw_position, 1, 'i', 0.f},
      {"kd_yaw_pos", "Yaw pos Kd",  &globals->pid_yaw_position, 1, 'd', 0.f},
      {"kp_yaw_vel", "Yaw vel Kp",  &globals->pid_yaw_velocity, 1, 'p', 4.f},
      {"ki_yaw_vel", "Yaw vel Ki",  &globals->pid_yaw_velocity, 1, 'i', 0.f},
      {"kd_yaw_vel", "Yaw vel Kd",  &globals->pid_yaw_velocity, 1, 'd', 0.0001f},

      // ── 云台 pitch ────────────────────────────────────────
      {"kp_pt_pos", "Pitch pos Kp",  &globals->pid_pitch_position, 1, 'p', 60.f},
      {"ki_pt_pos", "Pitch pos Ki",  &globals->pid_pitch_position, 1, 'i', 0.f},
      {"kd_pt_pos", "Pitch pos Kd",  &globals->pid_pitch_position, 1, 'd', 0.f},
      {"kp_pt_vel", "Pitch vel Kp",  &globals->pid_pitch_velocity, 1, 'p', 5000.f},
      {"ki_pt_vel", "Pitch vel Ki",  &globals->pid_pitch_velocity, 1, 'i', 20.f},
      {"kd_pt_vel", "Pitch vel Kd",  &globals->pid_pitch_velocity, 1, 'd', 40.f},

      // ── 拨盘 ──────────────────────────────────────────────
      {"kp_mg_pos", "Mag pos Kp", &globals->pid_magz_position, 1, 'p', 42.f},
      {"ki_mg_pos", "Mag pos Ki", &globals->pid_magz_position, 1, 'i', 0.001f},
      {"kd_mg_pos", "Mag pos Kd", &globals->pid_magz_position, 1, 'd', 0.56f},
      {"kp_mg_vel", "Mag vel Kp", &globals->pid_magz_velocity, 1, 'p', 0.35f},
      {"ki_mg_vel", "Mag vel Ki", &globals->pid_magz_velocity, 1, 'i', 0.f},
      {"kd_mg_vel", "Mag vel Kd", &globals->pid_magz_velocity, 1, 'd', 0.001f},

      // ── 摩擦轮 ────────────────────────────────────────────
      {"kp_sh1", "Shooter1 Kp", &globals->pid_shooter_1, 1, 'p', 30.f},
      {"ki_sh1", "Shooter1 Ki", &globals->pid_shooter_1, 1, 'i', 0.0000007f},
      {"kd_sh1", "Shooter1 Kd", &globals->pid_shooter_1, 1, 'd', 0.f},
      {"kp_sh2", "Shooter2 Kp", &globals->pid_shooter_2, 1, 'p', 30.f},
      {"ki_sh2", "Shooter2 Ki", &globals->pid_shooter_2, 1, 'i', 0.0000007f},
      {"kd_sh2", "Shooter2 Kd", &globals->pid_shooter_2, 1, 'd', 0.f},
      {"kp_sh3", "Shooter3 Kp", &globals->pid_shooter_3, 1, 'p', 30.f},
      {"ki_sh3", "Shooter3 Ki", &globals->pid_shooter_3, 1, 'i', 0.0000007f},
      {"kd_sh3", "Shooter3 Kd", &globals->pid_shooter_3, 1, 'd', 0.f},
      {"kp_sh4", "Shooter4 Kp", &globals->pid_shooter_4, 1, 'p', 30.f},
      {"ki_sh4", "Shooter4 Ki", &globals->pid_shooter_4, 1, 'i', 0.0000007f},
      {"kd_sh4", "Shooter4 Kd", &globals->pid_shooter_4, 1, 'd', 0.f},
      {"kp_sh5", "Shooter5 Kp", &globals->pid_shooter_5, 1, 'p', 30.f},
      {"ki_sh5", "Shooter5 Ki", &globals->pid_shooter_5, 1, 'i', 0.0000007f},
      {"kd_sh5", "Shooter5 Kd", &globals->pid_shooter_5, 1, 'd', 0.f},
      {"kp_sh6", "Shooter6 Kp", &globals->pid_shooter_6, 1, 'p', 30.f},
      {"ki_sh6", "Shooter6 Ki", &globals->pid_shooter_6, 1, 'i', 0.0000007f},
      {"kd_sh6", "Shooter6 Kd", &globals->pid_shooter_6, 1, 'd', 0.f},

      // clang-format on
  };
  count = sizeof(table) / sizeof(table[0]);
  return table;
}

/* -------------------------------------------------------------------------- */
/*  PidTuner 实现                                                             */
/* -------------------------------------------------------------------------- */

// DMA-accessible buffers (UART7 DMA 无法访问 DTCM)
__attribute__((section(".sram4"))) static char g_line_buf[256];
__attribute__((section(".sram4"))) static volatile int g_line_len = 0;
__attribute__((section(".sram4"))) static volatile bool g_cmd_ready = false;

// 响应缓冲区与发送状态
__attribute__((section(".sram4"))) static char g_resp_buf[512];
__attribute__((section(".sram4"))) static volatile int g_resp_len = 0;

/**
 * @brief UART7 RX 回调 — 累积字符直到 \\n
 */
void PidTuner::RxCallback(const etl::span<const uint8_t> &data) {
  for (size_t i = 0; i < data.size(); i++) {
    char c = static_cast<char>(data[i]);
    if (c == '\r') continue;  // 忽略 \\r

    if (c == '\n') {
      g_line_buf[g_line_len] = '\0';
      g_cmd_ready = true;
      return;  // 一行完成, 等待 Process() 处理
    }

    if (g_line_len < static_cast<int>(sizeof(g_line_buf)) - 1) {
      g_line_buf[g_line_len++] = c;
    }
  }
}

/**
 * @brief 注册 RX 回调到 uart7_debug
 */
void PidTuner::Init() {
  // .sram4 段为 NOLOAD, POR 上电时未初始化, 此处显式清零所有 POD 状态变量
  g_line_len = 0;
  g_cmd_ready = false;
  g_resp_len = 0;
  globals_no_dtcm.uart7_debug.AttachRxCallback(PidTuner::RxCallback);
}

/**
 * @brief 发送响应文本 (非阻塞 DMA, huart7 忙时跳过, 下次重试)
 */
void PidTuner::SendResponse(const char *msg) {
  if (huart7.gState == HAL_UART_STATE_READY) {
    size_t len = strlen(msg);
    if (len > 0) {
      g_resp_len = 0;  // 已发出, 清除待发
      HAL_UART_Transmit_DMA(&huart7, reinterpret_cast<const uint8_t *>(msg), len);
    }
  } else {
    // UART 忙 (VOFA 正在上行), 缓存待发
    snprintf(g_resp_buf, sizeof(g_resp_buf), "%s", msg);
    g_resp_len = static_cast<int>(strlen(g_resp_buf));
  }
}

/**
 * @brief 主循环驱动 — 处理待决命令并尝试发送缓存响应
 */
void PidTuner::Process() {
  // 1. 尝试发送上次缓存但未发出的响应
  if (g_resp_len > 0 && huart7.gState == HAL_UART_STATE_READY) {
    HAL_UART_Transmit_DMA(&huart7, reinterpret_cast<const uint8_t *>(g_resp_buf), g_resp_len);
    g_resp_len = 0;
  }

  // 2. 处理新到的命令
  if (!g_cmd_ready) return;

  char line[256];
  strncpy(line, g_line_buf, sizeof(line) - 1);
  line[sizeof(line) - 1] = '\0';
  g_line_len = 0;
  g_cmd_ready = false;

  ExecuteCommand(line);
}

/* -------------------------------------------------------------------------- */
/*  命令执行                                                                   */
/* -------------------------------------------------------------------------- */

void PidTuner::ExecuteCommand(char *line) {
  // 跳过前导空格
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  // 分词
  char *token = strtok(line, " \t");
  if (!token) return;

  if (strcmp(token, "set") == 0) {
    const char *name = strtok(nullptr, " \t");
    const char *val = strtok(nullptr, " \t");
    if (!name || !val) {
      SendResponse("ERR: usage: set <param> <value>\r\n");
      return;
    }
    const ParamEntry *e = FindEntry(name);
    if (!e) {
      SendResponse("ERR: unknown param\r\n");
      return;
    }
    CmdSet(e, strtof(val, nullptr));
    return;
  }

  if (strcmp(token, "get") == 0) {
    const char *arg = strtok(nullptr, " \t");
    if (!arg) {
      SendResponse("ERR: usage: get <param> | get all\r\n");
      return;
    }
    if (strcmp(arg, "all") == 0) {
      CmdGetAll();
      return;
    }
    const ParamEntry *e = FindEntry(arg);
    if (!e) {
      SendResponse("ERR: unknown param\r\n");
      return;
    }
    CmdGet(e);
    return;
  }

  if (strcmp(token, "fuzzy") == 0) {
    const char *name = strtok(nullptr, " \t");
    const char *val = strtok(nullptr, " \t");
    if (!name || !val) {
      SendResponse("ERR: usage: fuzzy <param> <0|1>\r\n");
      return;
    }
    const ParamEntry *e = FindEntry(name);
    if (!e) {
      SendResponse("ERR: unknown param\r\n");
      return;
    }
    CmdFuzzy(e, strtol(val, nullptr, 10) != 0);
    return;
  }

  if (strcmp(token, "fscale") == 0) {
    const char *name = strtok(nullptr, " \t");
    const char *es = strtok(nullptr, " \t");
    const char *ds = strtok(nullptr, " \t");
    if (!name || !es || !ds) {
      SendResponse("ERR: usage: fscale <param> <err_scale> <derr_scale>\r\n");
      return;
    }
    const ParamEntry *e = FindEntry(name);
    if (!e) {
      SendResponse("ERR: unknown param\r\n");
      return;
    }
    CmdFuzzyScale(e, strtof(es, nullptr), strtof(ds, nullptr));
    return;
  }

  if (strcmp(token, "save") == 0) {
    CmdSave();
    return;
  }

  if (strcmp(token, "help") == 0) {
    CmdHelp(line);
    return;
  }

  SendResponse("ERR: unknown command. try: set get fuzzy fscale save help\r\n");
}

/* -------------------------------------------------------------------------- */
/*  辅助函数                                                                   */
/* -------------------------------------------------------------------------- */

const ParamEntry *PidTuner::FindEntry(const char *name) {
  int count = 0;
  const ParamEntry *table = GetParamTable(count);
  for (int i = 0; i < count; i++) {
    if (strcmp(table[i].name, name) == 0) {
      return &table[i];
    }
  }
  return nullptr;
}

/**
 * @brief 读取一个 PID (可能是数组) 的指定参数值
 */
static float ReadPidParam(const ParamEntry &e) {
  switch (e.param) {
    case 'p':
      return e.pid_first->kp();
    case 'i':
      return e.pid_first->ki();
    case 'd':
      return e.pid_first->kd();
    default:
      return 0;
  }
}

/**
 * @brief 写入 pid_count 个连续 PID 的指定参数
 */
static void WritePidParam(const ParamEntry &e, float value) {
  for (uint8_t i = 0; i < e.pid_count; i++) {
    switch (e.param) {
      case 'p':
        e.pid_first[i].SetKp(value);
        break;
      case 'i':
        e.pid_first[i].SetKi(value);
        break;
      case 'd':
        e.pid_first[i].SetKd(value);
        break;
    }
  }
}

/* -------------------------------------------------------------------------- */
/*  子命令实现                                                                 */
/* -------------------------------------------------------------------------- */

void PidTuner::CmdSet(const ParamEntry *e, float value) {
  WritePidParam(*e, value);
  char buf[128];
  if (e->pid_count > 1) {
    snprintf(buf, sizeof(buf), "OK: %s[0..%d] = %.6g\r\n", e->name, e->pid_count - 1, (double)value);
  } else {
    snprintf(buf, sizeof(buf), "OK: %s = %.6g\r\n", e->name, (double)value);
  }
  SendResponse(buf);
}

void PidTuner::CmdGet(const ParamEntry *e) {
  float v = ReadPidParam(*e);
  char buf[128];
  snprintf(buf, sizeof(buf), "%s = %.6g  (%s)\r\n", e->name, (double)v, e->desc);
  SendResponse(buf);
}

void PidTuner::CmdGetAll() {
  int count = 0;
  const ParamEntry *table = GetParamTable(count);

  // 分批发送避免单帧过大
  char buf[256];
  int pos = 0;
  for (int i = 0; i < count; i++) {
    float v = ReadPidParam(table[i]);
    pos += snprintf(buf + pos, sizeof(buf) - pos, "%-16s %-10.6g  %s\r\n", table[i].name, (double)v, table[i].desc);

    // 每 8 行发一批
    if ((i + 1) % 8 == 0 || i == count - 1) {
      SendResponse(buf);
      buf[0] = '\0';
      pos = 0;
    }
  }
}

void PidTuner::CmdFuzzy(const ParamEntry *e, bool enable) {
  for (uint8_t i = 0; i < e->pid_count; i++) {
    e->pid_first[i].SetFuzzy(enable);
  }
  char buf[96];
  snprintf(buf, sizeof(buf), "OK: fuzzy %s = %d\r\n", e->name, enable ? 1 : 0);
  SendResponse(buf);
}

void PidTuner::CmdFuzzyScale(const ParamEntry *e, float e_scale, float de_scale) {
  for (uint8_t i = 0; i < e->pid_count; i++) {
    e->pid_first[i].SetFuzzyErrorScale(e_scale).SetFuzzyDErrorScale(de_scale);
  }
  char buf[128];
  snprintf(buf, sizeof(buf), "OK: %s fuzzy error_scale=%.6g d_error_scale=%.6g\r\n", e->name, (double)e_scale,
           (double)de_scale);
  SendResponse(buf);
}

/**
 * @brief 输出所有 PID 参数的初始化代码 (可直接复制粘贴到 globals.hpp)
 */
void PidTuner::CmdSave() {
  char buf[384];
  int pos = 0;

  pos += snprintf(buf + pos, sizeof(buf) - pos, "// ── PID save ──\r\n");

  // 底盘速度环
  for (int i = 0; i < 4; i++) {
    pos += snprintf(buf + pos, sizeof(buf) - pos, "  {%-5.6gf, %-5.6gf, %-5.6gf, %-5.0ff, %-5.0ff}%s\r\n",
                    (double)globals->velocity_pid[i].kp(), (double)globals->velocity_pid[i].ki(),
                    (double)globals->velocity_pid[i].kd(), (double)globals->velocity_pid[i].max_out(),
                    (double)globals->velocity_pid[i].max_iout(), i < 3 ? "," : "");
  }

  // 其他 PID 用一个宏风格的格式
#define SAVE_ONE(pid_var)                                                                                       \
  pos += snprintf(buf + pos, sizeof(buf) - pos,                                                                 \
                  "  {%-5.6gf, %-5.6gf, %-5.6gf, %-5.0ff, %-5.0ff}  // " #pid_var "\r\n", (double)pid_var.kp(), \
                  (double)pid_var.ki(), (double)pid_var.kd(), (double)pid_var.max_out(), (double)pid_var.max_iout())

  SAVE_ONE(globals->pid_chassis_follow_pos);
  SAVE_ONE(globals->pid_chassis_follow_vel);
  SAVE_ONE(globals->pid_yaw_position);
  SAVE_ONE(globals->pid_yaw_velocity);
  SAVE_ONE(globals->pid_pitch_position);
  SAVE_ONE(globals->pid_pitch_velocity);
  SAVE_ONE(globals->pid_magz_position);
  SAVE_ONE(globals->pid_magz_velocity);
  SAVE_ONE(globals->pid_shooter_1);
  SAVE_ONE(globals->pid_shooter_2);
  SAVE_ONE(globals->pid_shooter_3);
  SAVE_ONE(globals->pid_shooter_4);
  SAVE_ONE(globals->pid_shooter_5);
  SAVE_ONE(globals->pid_shooter_6);

#undef SAVE_ONE

  pos += snprintf(buf + pos, sizeof(buf) - pos, "// ── end ──\r\n");
  SendResponse(buf);
}

void PidTuner::CmdHelp(char * /*unused*/) {
  SendResponse(
      "\r\n"
      "=== PID Tuning Commands ===\r\n"
      "  set <param> <value>    e.g. set kp_yaw_pos 25.0\r\n"
      "  get <param>            e.g. get kd_sh1\r\n"
      "  get all                list all params\r\n"
      "  fuzzy <param> <0|1>    e.g. fuzzy yaw_pos 1\r\n"
      "  fscale <param> <e> <de> e.g. fscale yaw_pos 0.5 5.0\r\n"
      "  save                   dump all as init code\r\n"
      "  help                   this message\r\n"
      "\r\n"
      "Params: kp/ki/kd _cv | _cf_(pos|vel) | _yaw_(pos|vel) | _pt_(pos|vel)\r\n"
      "             | _mg_(pos|vel) | _sh1.._sh6\r\n"
      "===========================\r\n");
}
