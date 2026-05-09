#pragma once

#include "librm.hpp"
#include "usart.h"

#pragma pack(1)
struct Vofa_TxFrame {
  float x[10];
  uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
};
#pragma pack()

void VOFA_Prepare_Package(float x[], Vofa_TxFrame &tx_buffer, int len);
void VOFA_Send_JustFloat_DMA(UART_HandleTypeDef *huart, Vofa_TxFrame &tx_buffer);
void VOFASend();

/**
 * @brief PID 参数表条目 — 将命令名映射到具体 PID 对象和参数类型
 */
struct ParamEntry {
  const char *name;             ///< 命令名 e.g. "kp_yaw_pos"
  const char *desc;             ///< 人类可读描述
  rm::modules::PID *pid_first;  ///< PID 对象指针 (数组则为第一个元素)
  uint8_t pid_count;            ///< 1=单个PID, 4=底盘速度环数组
  char param;                   ///< 'p'=Kp, 'i'=Ki, 'd'=Kd
  float default_val;            ///< 默认值 (save 时输出)
};

/**
 * @brief PID 动态调参器
 *
 * 通过 UART7 RX 接收上位机文本指令，实时修改 PID 参数。
 * Init() 注册 RX 回调到 uart7_debug。
 * Process() 在主循环 Monitor() 中调用以处理待决命令并发送响应。
 *
 * 协议格式 (纯文本, \\n 结尾):
 *   set <param> <value>    - 修改参数
 *   get <param>            - 查询参数
 *   get all                - 列出所有参数
 *   fuzzy <param> <0|1>    - 开关模糊 PID
 *   fscale <param> <e> <de> - 设置模糊尺度因子
 *   save                   - 输出所有参数初始值代码
 *   ident <start|stop|status> - Pitch 轴动力学辨识
 *   help                   - 帮助
 */
class PidTuner {
 public:
  static void Init();
  static void Process();

 private:
  static void RxCallback(const etl::span<const uint8_t> &data);
  static void ExecuteCommand(char *line);
  static void SendResponse(const char *msg);
  static const ParamEntry *FindEntry(const char *name);

  static void CmdSet(const ParamEntry *e, float value);
  static void CmdGet(const ParamEntry *e);
  static void CmdGetAll();
  static void CmdFuzzy(const ParamEntry *e, bool enable);
  static void CmdFuzzyScale(const ParamEntry *e, float e_scale, float de_scale);
  static void CmdSave();
  static void CmdIdent(const char *arg);
  static void CmdHelp(char *line);
};
