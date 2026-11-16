/**
  ******************************************************************************
  * @file    arm_uart.c
  * @brief   Serial layer.  See arm_uart.h for the protocol contract.
  *
  *          Receive path
  *          ------------
  *          The DMA runs in circular mode and is never stopped, so no byte can
  *          be lost in a stop/restart window.  On every USART IDLE interrupt
  *          the number of bytes that have arrived is derived from the DMA
  *          counter, and the new bytes are appended to a line assembler.  The
  *          assembler splits on '\r' and ignores '\n', exactly like
  *          robotArm.ino:18-20.
  *
  *          The IDLE flag is NOT cleared by HAL_UART_IRQHandler(), so the hook
  *          must run inside the generated USARTx_IRQHandler(); see the
  *          USER CODE blocks in Core/Src/stm32f4xx_it.c.
  *
  *          Transmit path
  *          -------------
  *          Answers go out as "ok\r\n" / "rs\r\n" / "!!\r\n" / "// ...\r\n".
  *          CR LF is mandatory: the Delphi host splits on #13#10
  *          (UnitRobotArm.pas:182) and only accepts a line as an "ok" when it
  *          is the first line of the received chunk (UnitRobotArm.pas:164).
  ******************************************************************************
  */

#include "arm_uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

/* The UART handles are defined in main.c and are not declared in main.h. */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/*============================== Private types ===============================*/

typedef struct
{
  UART_HandleTypeDef *huart;
  DMA_HandleTypeDef  *hdma;
  uint8_t             ring[ARM_UART_RX_RING];
  volatile uint16_t   head;             /**< DMA progress at the last IDLE      */
  uint16_t            tail;             /**< bytes already consumed             */
  char                line[ARM_LINE_MAX];
  uint16_t            line_len;
  bool                overflow;         /**< line was longer than the buffer    */
} arm_uart_rx_t;

/*============================= Private state ===============================*/

static arm_uart_rx_t s_gcode;
static arm_uart_rx_t s_debug;

static osMutexId_t s_tx_mutex = NULL;

static arm_line_cb_t s_line_cb = NULL;

static char s_txbuf[ARM_LINE_MAX + 8u];

#if (ARM_DEBUG_UART_ENABLE == 1)
#define ARM_DEBUG_RING 256u
static char             s_dbg_ring[ARM_DEBUG_RING];
static volatile uint16_t s_dbg_head = 0u;
static uint16_t          s_dbg_tail = 0u;
#endif

/*=========================== Internal prototypes ===========================*/

static void arm_uart_rx_start(arm_uart_rx_t *rx, UART_HandleTypeDef *huart);
static void arm_uart_rx_idle_isr(arm_uart_rx_t *rx);
static void arm_uart_rx_drain(arm_uart_rx_t *rx, arm_line_cb_t cb);
static uint16_t arm_uart_available(const arm_uart_rx_t *rx);
#if (ARM_DEBUG_UART_ENABLE == 1)
static void arm_uart_debug_flush(void);
#endif

/*=============================== Helpers ===================================*/

/** Number of bytes the DMA has delivered but the software has not consumed. */
static uint16_t arm_uart_available(const arm_uart_rx_t *rx)
{
  uint16_t head = rx->head;
  uint16_t tail = rx->tail;

  if (head >= tail)
  {
    return (uint16_t)(head - tail);
  }
  /* The circular buffer wrapped between the producer and the consumer. */
  return (uint16_t)(ARM_UART_RX_RING - tail + head);
}

/*============================= Initialisation ==============================*/

/**
  * @brief  [学习标注] 初始化两路串口的 DMA 循环接收、创建发送互斥量、清空调试环。
  */
void arm_uart_init(void)
{
  const osMutexAttr_t mtx_attr = {.name = "uart_tx_mtx"};

  s_tx_mutex = osMutexNew(&mtx_attr);

  arm_uart_rx_start(&s_gcode, &huart2);
  arm_uart_rx_start(&s_debug, &huart3);

#if (ARM_DEBUG_UART_ENABLE == 1)
  s_dbg_head = 0u;
  s_dbg_tail = 0u;
#endif
}

/**
  * @brief  [学习标注] 启动一路串口的 DMA 循环接收，并手动使能 IDLE 中断（HAL 不会开）。
  * @note   漏掉最后那句 → head 永不更新 → 收到的命令全被丢弃（案例 2）。
  */
static void arm_uart_rx_start(arm_uart_rx_t *rx, UART_HandleTypeDef *huart)
{
  rx->huart    = huart;
  rx->hdma     = huart->hdmarx;
  rx->head     = 0u;
  rx->tail     = 0u;
  rx->line_len = 0u;
  rx->overflow = false;
  memset(rx->ring, 0, sizeof(rx->ring));
  memset(rx->line, 0, sizeof(rx->line));

  if (rx->hdma == NULL)
  {
    /* The CubeMX project must provide the DMA handle; without it the link
       would be dead, so fail loudly instead of silently dropping input. */
    Error_Handler();
  }

  if (HAL_UART_Receive_DMA(rx->huart, rx->ring, (uint16_t)ARM_UART_RX_RING) != HAL_OK)
  {
    Error_Handler();
  }

  /* HAL_UART_Receive_DMA() only unmasks the parity and error interrupts
     (stm32f4xx_hal_uart.c, UART_Start_Receive_DMA).  It never enables the IDLE
     line detection interrupt, and the whole receive path of this firmware is
     driven by that interrupt:
         IDLE -> arm_uart_gcode_idle_isr() -> rx->head -> arm_uart_rx_drain()
     Without the line below the head index never moves, the drain always sees
     zero available bytes, and every command sent by the host is silently
     ignored - the board answers nothing and looks completely dead on the
     G-code link while the DMA happily fills the ring in the background. */
  __HAL_UART_ENABLE_IT(rx->huart, UART_IT_IDLE);
}

/*============================== IDLE hooks =================================*/

/**
  * @brief  [学习标注] G-code 口 IDLE 钩子：由 USART2_IRQHandler 调用，只做 NDTR→head 快照。
  */
void arm_uart_gcode_idle_isr(void)
{
  arm_uart_rx_idle_isr(&s_gcode);
}

/**
  * @brief  [学习标注] 调试口 IDLE 钩子：同上，供 USART3_IRQHandler 调用。
  */
void arm_uart_debug_idle_isr(void)
{
  arm_uart_rx_idle_isr(&s_debug);
}

/**
  * @brief  Snapshot the DMA progress after an IDLE condition.
  * @note   Interrupt context.  Kept to a handful of register accesses: no
  *         float, no printing, no blocking call.
  */
static void arm_uart_rx_idle_isr(arm_uart_rx_t *rx)
{
  uint32_t remaining;

  remaining = __HAL_DMA_GET_COUNTER(rx->hdma);

  if (remaining > ARM_UART_RX_RING)
  {
    remaining = ARM_UART_RX_RING;   /* defensive: never trust the register */
  }

  rx->head = (uint16_t)(ARM_UART_RX_RING - remaining);
}

/*============================= Line assembly ===============================*/

/**
  * @brief  [学习标注] CommandTask 每 1 ms 调用：把 G-code 口字节拆成整行并回调；顺带清调试口接
  *         收、flush 日志。
  */
void arm_uart_poll(void)
{
  arm_uart_rx_drain(&s_gcode, s_line_cb);

#if (ARM_DEBUG_UART_ENABLE == 1)
  /* Drain the debug receive ring (its lines are discarded) and then push
     whatever the log has accumulated out of the debug transmit ring. */
  arm_uart_rx_drain(&s_debug, NULL);
  arm_uart_debug_flush();
#endif
}

/**
  * @brief  Consume freshly received bytes and emit every complete line.
  * @param  cb  NULL to silently discard the lines (debug link).
  */
static void arm_uart_rx_drain(arm_uart_rx_t *rx, arm_line_cb_t cb)
{
  uint16_t avail;

  while ((avail = arm_uart_available(rx)) > 0u)
  {
    uint16_t chunk = avail;

    /* Never cross the end of the circular buffer in one memcpy. */
    if ((uint16_t)(rx->tail + chunk) > (uint16_t)ARM_UART_RX_RING)
    {
      chunk = (uint16_t)(ARM_UART_RX_RING - rx->tail);
    }

    /* Safety bound: never consume more than half the ring in one pass.  Should
       the task ever fall so far behind that the DMA laps the consumer, this
       keeps the "head - tail" arithmetic meaningful at the cost of dropping
       the oldest bytes instead of reading garbage. */
    if (chunk > (uint16_t)(ARM_UART_RX_RING / 2u))
    {
      chunk = (uint16_t)(ARM_UART_RX_RING / 2u);
    }

    for (uint16_t i = 0u; i < chunk; i++)
    {
      char c = (char)rx->ring[rx->tail];
      rx->tail = (uint16_t)((rx->tail + 1u) % ARM_UART_RX_RING);

#if (ARM_ACCEPT_LF_TERMINATOR == 1)
      bool is_terminator = ((c == '\r') || (c == '\n'));
#else
      bool is_terminator = (c == '\r');
#endif

      if (is_terminator)
      {
        rx->line[rx->line_len] = '\0';
        if (rx->overflow == false)
        {
          if ((rx->line_len > 0u) && (cb != NULL))
          {
            (void)cb(rx->line);
          }
        }
        rx->line_len = 0u;
        rx->overflow = false;
        continue;
      }

      if (c == '\n')
      {
        /* The original firmware ignores LF entirely (robotArm.ino:18-20).
           Reachable only when LF is not itself a terminator, see
           ARM_ACCEPT_LF_TERMINATOR. */
        continue;
      }

      if (rx->line_len < (uint16_t)(ARM_LINE_MAX - 1u))
      {
        rx->line[rx->line_len++] = c;
      }
      else
      {
        /* Line too long: mark it and swallow the rest of the line.  It will
           be reported as a parse error once the '\r' arrives. */
        rx->overflow = true;
        rx->line_len = 0u;
      }
    }
  }
}

/** Register the consumer of complete command lines. */
void arm_uart_set_line_callback(arm_line_cb_t cb)
{
  s_line_cb = cb;
}

/*================================ Transmit =================================*/

/**
  * @brief  [学习标注] 在 USART2 上发一行：自动补 CRLF，用互斥量串行化，阻塞发送（超时 100 ms）
  *         。
  */
void arm_uart_send_line(const char *text)
{
  size_t len = strlen(text);

  if (len > (size_t)ARM_LINE_MAX)
  {
    len = (size_t)ARM_LINE_MAX;
  }

  memcpy(s_txbuf, text, len);
  s_txbuf[len]      = '\r';
  s_txbuf[len + 1u] = '\n';
  s_txbuf[len + 2u] = '\0';

  if (s_tx_mutex != NULL)
  {
    (void)osMutexAcquire(s_tx_mutex, osWaitForever);
  }

  (void)HAL_UART_Transmit(&huart2, (uint8_t *)s_txbuf, (uint16_t)(len + 2u),
                          ARM_UART_TX_TIMEOUT_MS);

  if (s_tx_mutex != NULL)
  {
    (void)osMutexRelease(s_tx_mutex);
  }
}

/**
  * @brief  [学习标注] 应答 "ok"（入队成功时立刻发，与执行解耦）。
  */
void arm_uart_print_ok(void)
{
  arm_uart_send_line("ok");
}

/**
  * @brief  [学习标注] 应答 "rs"（解析失败，要求上位机重发）。
  */
void arm_uart_print_resend(void)
{
  arm_uart_send_line("rs");
}

/**
  * @brief  [学习标注] 应答 "!!"（执行失败，配合前面的 // 注释行）。
  */
void arm_uart_print_fault(void)
{
  arm_uart_send_line("!!");
}

/**
  * @brief  [学习标注] 用 snprintf 组一行 "// 文本" 并发送（占用较多栈，调用它的任务要留余量）。
  */
void arm_uart_print_comment(const char *text)
{
  char buf[ARM_LINE_MAX];

  /* "// " prefix, matching printComment() in command.cpp:118-126. */
  (void)snprintf(buf, sizeof(buf), "// %s", text);
  arm_uart_send_line(buf);
}

/**
  * @brief  [学习标注] 只发“延迟应答”部分：WARN→注释行；FAULT→注释行 + "!!"；OK 不发（ok 已
  *         在入队时发过）。
  */
void arm_uart_send_deferred(arm_ack_t ack, const char *warn)
{
  switch (ack)
  {
    case ARM_ACK_OK:
      /* Nothing deferred: "ok" was already sent when the command was queued. */
      break;

    case ARM_ACK_RESEND:
      arm_uart_print_resend();
      break;

    case ARM_ACK_WARN:
      if (warn != NULL)
      {
        arm_uart_print_comment(warn);
      }
      break;

    case ARM_ACK_FAULT:
      /* Comment first, then "!!" - section 4.10 rule 3. */
      if (warn != NULL)
      {
        arm_uart_print_comment(warn);
      }
      arm_uart_print_fault();
      break;

    default:
      arm_uart_print_resend();
      break;
  }
}

/*=============================== Debug link ================================*/

#if (ARM_DEBUG_UART_ENABLE == 1)

/** Push up to one chunk of the debug log out of USART3.  Task context only. */
static void arm_uart_debug_flush(void)
{
  char chunk[32];
  uint16_t n = 0u;

  while ((n < (uint16_t)(sizeof(chunk) - 1u)) && (s_dbg_tail != s_dbg_head))
  {
    chunk[n++] = s_dbg_ring[s_dbg_tail];
    s_dbg_tail = (uint16_t)((s_dbg_tail + 1u) % ARM_DEBUG_RING);
  }

  if (n > 0u)
  {
    chunk[n] = '\0';
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)chunk, n, ARM_UART_TX_TIMEOUT_MS);
  }
}

/**
  * @brief  [学习标注] 把文本塞进调试发送环；环满就丢，绝不阻塞控制任务。
  */
void arm_uart_debug(const char *text)
{
  size_t len = strlen(text);

  for (size_t i = 0u; i < len; i++)
  {
    uint16_t next = (uint16_t)((s_dbg_head + 1u) % ARM_DEBUG_RING);

    if (next == s_dbg_tail)
    {
      break;                       /* ring full, drop rather than block */
    }

    s_dbg_ring[s_dbg_head] = text[i];
    s_dbg_head             = next;
  }
}

/**
  * @brief  [学习标注] printf 风格调试输出：内部 char buf[128] + vsnprintf，再塞进调试环。
  * @note   正是它把 defaultTask 的 512 B 栈撑爆（案例 6）。
  */
void arm_uart_debugf(const char *fmt, ...)
{
  char buf[128];
  va_list ap;

  va_start(ap, fmt);
  (void)vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  arm_uart_debug(buf);
}

#else  /* ARM_DEBUG_UART_ENABLE == 0 */

/**
  * @brief  [学习标注] 调试口被编译关掉（ARM_DEBUG_UART_ENABLE=0）时的空实现。
  */
void arm_uart_debug(const char *text)
{
  (void)text;
}

/**
  * @brief  [学习标注] 调试口关闭时的空实现（不产生任何代码）。
  */
void arm_uart_debugf(const char *fmt, ...)
{
  (void)fmt;
}

#endif /* ARM_DEBUG_UART_ENABLE */

/*============================== Boot report ================================*/

/**
  * @brief  [学习标注] 上电一次性横幅：把固定说明文本塞进调试环，由 CommandTask 真正发出。
  */
void arm_uart_boot_report(void)
{
#if (ARM_DEBUG_UART_ENABLE == 1) && (ARM_DEBUG_BOOT_REPORT == 1)
  /* Built as one static string so the report cannot be interleaved. */
  static char report[192];

  (void)snprintf(report, sizeof(report),
                 "STM32F407 robotArm: TIM2/3+DMA step engine, USART2 %u 8N1, "
                 "USART3 %u 8N1, FreeRTOS 1 kHz\r\n",
                 (unsigned)ARM_GCODE_UART_BAUD, (unsigned)ARM_DEBUG_UART_BAUD);

  arm_uart_debug(report);
#endif
}
