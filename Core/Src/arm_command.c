/**
  ******************************************************************************
  * @file    arm_command.c
  * @brief   G-code parser and executor.
  *
  *          The parser reproduces command.cpp:32-90 token by token, including
  *          the parts that look like bugs, because the Delphi host depends on
  *          the exact acceptance behaviour:
  *
  *            - the line is upper cased and a trailing space is appended, which
  *              is what makes the simple "find the next space" tokenizer work
  *            - the first character must be 'G' or 'M', otherwise "rs"
  *            - an unknown parameter letter stops the parameter loop without
  *              failing the command, so "G1 X10 Q5" still moves to X10
  *            - up to five parameters are read
  *            - a value that does not parse as a number silently becomes 0,
  *              again like String::toFloat() plus the original's handling
  *
  *          The answer order is fixed by section 4.10 rule 3 and by the host
  *          implementation:
  *
  *            "ok" is sent when the command is queued, and "// WARN" plus "!!"
  *            follow later, when the command is actually executed.  The host
  *            only releases its stop-and-wait lock on a line that starts with
  *            "ok" (UnitRobotArm.pas:164), so an unsupported command must still
  *            answer "ok" or the PC would hang forever.
  ******************************************************************************
  */

#include "arm_command.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "main.h"
#include "cmsis_os.h"

#include "arm_config.h"
#include "arm_geometry.h"
#include "arm_interpolation.h"
#include "arm_limits.h"
#include "arm_stepper.h"
#include "arm_uart.h"

/*============================================================================*/
/* Tokenizer                                                                  */
/*============================================================================*/

/** Uppercase a copy of the line and append the space the tokenizer needs. */
static void arm_command_normalise(const char *line, char *out, size_t out_size)
{
  size_t n = 0u;

  while ((line[n] != '\0') && (n < (out_size - 2u)))
  {
    char c = line[n];

    if ((c >= 'a') && (c <= 'z'))
    {
      c = (char)(c - 'a' + 'A');
    }

    out[n] = c;
    n++;
  }

  /* command.cpp:33 - msg += ' '; "helps parsing" */
  out[n]       = ' ';
  out[n + 1u]  = '\0';
}

/** Position of the next occurrence of c at or after start, -1 when absent. */
static int arm_command_find(const char *s, char c, int start)
{
  for (int i = start; s[i] != '\0'; i++)
  {
    if (s[i] == c)
    {
      return i;
    }
  }

  return -1;
}

/*============================================================================*/
/* Parser                                                                     */
/*============================================================================*/

/**
  * @brief  [学习标注] 把一行文本解析成 arm_cmd_t：转大写、末尾补空格、手写整数/浮点，未给的坐标
  *         置 NaN。
  * @note   刻意保留原版 command.cpp 的容错：未知字母结束循环、坏数字变 0。
  */
bool arm_command_parse(const char *line, arm_cmd_t *out)
{
  char buf[ARM_LINE_MAX + 4u];
  int  first;
  int  last;
  int  parse_pos;
  int  i;

  if ((line == NULL) || (out == NULL))
  {
    return false;
  }

  arm_command_normalise(line, buf, sizeof(buf));

  memset(out, 0, sizeof(*out));

  /* Defaults from Command::Command() and Command::processMessage(). */
  out->valueX = NAN;
  out->valueY = NAN;
  out->valueZ = NAN;
  out->valueE = NAN;
  out->valueT = 0.0f;
  out->valueF = 0.0f;

  out->id = buf[0];

  /* command.cpp:37-40 - anything that is not G or M is a resend request. */
  if ((out->id != 'G') && (out->id != 'M'))
  {
    return false;
  }

  /* command.cpp:42-47 - the command number runs to the first space. */
  first = 1;
  last  = arm_command_find(buf, ' ', first);
  if (last < 0)
  {
    return false;
  }

  /* Parse the number by hand so no locale or float formatting is involved. */
  {
    int   value = 0;
    bool  any   = false;
    bool  neg   = false;

    for (int k = first; k < last; k++)
    {
      char c = buf[k];

      if ((c == '-') && (any == false))
      {
        neg = true;
      }
      else if ((c >= '0') && (c <= '9'))
      {
        value = (value * 10) + (c - '0');
        any   = true;
      }
      else
      {
        break;      /* String::toInt() stops at the first non digit */
      }
    }

    out->num = (int16_t)(neg ? -value : value);
  }

  /* command.cpp:61-87 - up to five "LETTERvalue" pairs separated by spaces.
     An unknown letter ends the loop, the command itself stays valid. */
  parse_pos = last + 1;
  i = 0;

  while (i < (int)ARM_CMD_MAX_VALUES)
  {
    char id = buf[parse_pos++];

    if (id != ' ')
    {
      first = parse_pos;
      last  = arm_command_find(buf, ' ', parse_pos);

      if ((last < first) || (last < 0))
      {
        break;      /* no terminating space: malformed tail */
      }

      /* Parse the value as a float without strtof() so that no locale or
         library dependency is pulled in. */
      {
        float value = 0.0f;
        float scale = 0.1f;
        bool  neg   = false;
        bool  frac  = false;
        int   k     = first;

        if (buf[k] == '-')
        {
          neg = true;
          k++;
        }
        else if (buf[k] == '+')
        {
          k++;
        }

        for (; k < last; k++)
        {
          char c = buf[k];

          if ((c >= '0') && (c <= '9'))
          {
            if (frac)
            {
              value += (float)(c - '0') * scale;
              scale *= 0.1f;
            }
            else
            {
              value = (value * 10.0f) + (float)(c - '0');
            }
          }
          else if (c == '.')
          {
            frac = true;
          }
          else
          {
            break;
          }
        }

        if (neg)
        {
          value = -value;
        }

        switch (id)
        {
          case 'X': out->valueX = value; break;
          case 'Y': out->valueY = value; break;
          case 'Z': out->valueZ = value; break;
          case 'E': out->valueE = value; break;
          case 'F': out->valueF = value; break;
          case 'T': out->valueT = value; break;
          default:  i = (int)ARM_CMD_MAX_VALUES; break;   /* command.cpp:79 */
        }
      }

      parse_pos = last + 1;
    }
    else
    {
      break;
    }

    i++;
  }

  return true;
}

/*============================================================================*/
/* Execution                                                                  */
/*============================================================================*/

/** Fill the missing coordinates from the current virtual position. */
static void arm_command_default_coordinates(arm_cmd_t *cmd)
{
  if (cmd->valueX != cmd->valueX)   /* NaN test, command.cpp:167-178 */
  {
    cmd->valueX = arm_interp_get_x();
  }

  if (cmd->valueY != cmd->valueY)
  {
    cmd->valueY = arm_interp_get_y();
  }

  if (cmd->valueZ != cmd->valueZ)
  {
    cmd->valueZ = arm_interp_get_z();
  }

  if (cmd->valueE != cmd->valueE)
  {
    cmd->valueE = 0.0f;
  }
}

/** G0/G1: start a straight line move, refused when it leaves the soft limits. */
static arm_ack_t arm_command_move(const arm_cmd_t *cmd, const char **warn_out)
{
  arm_joints_t joints;
  arm_limit_verdict_t verdict;

  if (arm_limits_is_faulted())
  {
    *warn_out = "machine faulted - send M18 then M17";
    return ARM_ACK_FAULT;
  }

  arm_geometry_solve(cmd->valueX, cmd->valueY, cmd->valueZ, &joints);

  verdict = arm_limits_check_pose(joints.rot, joints.low, joints.high);

  if (verdict != ARM_LIMIT_OK)
  {
    /* Section 4.9: refuse the move and report it.  The axis is not moved, so
       the virtual position stays where it was and the next command is planned
       from the real pose. */
    switch (verdict)
    {
      case ARM_LIMIT_LOW:   *warn_out = "soft limit: lower arm"; break;
      case ARM_LIMIT_HIGH:  *warn_out = "soft limit: upper arm"; break;
      case ARM_LIMIT_ELBOW: *warn_out = "soft limit: elbow (arm-to-arm angle)"; break;
      default:              *warn_out = "soft limit: rotation";  break;
    }

    arm_limits_flash_warn();
    return ARM_ACK_FAULT;
  }

  /* `F` is a real feed rate, so a single wrong digit asks the motors for ten
     times the step rate they can start from.  arm_interp_start() clamps it; say
     so on the debug link instead of silently running slower than commanded. */
  if (cmd->valueF > ARM_MAX_SPEED_MM_S)
  {
    arm_uart_debugf("F%ld clamped to %ld mm/s (ARM_MAX_SPEED_MM_S)\r\n",
                    (long)cmd->valueF, (long)ARM_MAX_SPEED_MM_S);
  }

  arm_interp_start(arm_interp_get_x(), arm_interp_get_y(), arm_interp_get_z(),
                   cmd->valueX, cmd->valueY, cmd->valueZ, cmd->valueF);

  return ARM_ACK_OK;
}

/** G4: busy wait in the motion task, so the arm keeps interpolating. */
static arm_ack_t arm_command_dwell(const arm_cmd_t *cmd, uint32_t *dwell_ms_out)
{
  float seconds = cmd->valueT;

  if (seconds < 0.0f)
  {
    seconds = 0.0f;
  }

  if (seconds > 3600.0f)
  {
    seconds = 3600.0f;
  }

  *dwell_ms_out = (uint32_t)(seconds * 1000.0f);
  return ARM_ACK_OK;
}

/**
  * @brief  [学习标注] 执行一条已出队的命令（G0/G1/G4、M17/M18/M210、M3/M5/M106/M107），返回应答
  *         类型与警告文本。
  */
arm_ack_t arm_command_execute(const arm_cmd_t *cmd, const char **warn_out)
{
  arm_cmd_t local;
  uint32_t dwell_ms = 0u;
  arm_ack_t ack = ARM_ACK_OK;

  *warn_out = NULL;

  if (cmd == NULL)
  {
    *warn_out = "null command";
    return ARM_ACK_FAULT;
  }

  /* command.cpp:161-166 - id -1 marks a line that failed to parse. */
  if (cmd->id == (char)-1)
  {
    *warn_out = "parsing error";
    return ARM_ACK_FAULT;
  }

  local = *cmd;
  arm_command_default_coordinates(&local);

  if (local.id == 'G')
  {
    switch (local.num)
    {
      case 0:
      case 1:
        ack = arm_command_move(&local, warn_out);
        break;

      case 4:
        ack = arm_command_dwell(&local, &dwell_ms);
        break;

      default:
        /* executeCommand() default branch: handleAsErr() */
        *warn_out = "unknown G command";
        ack = ARM_ACK_FAULT;
        break;
    }
  }
  else if (local.id == 'M')
  {
    switch (local.num)
    {
      case 3:
      case 5:
        /* The gripper is not built in this revision (section 3.4).  The
           protocol answer is kept so the host does not stall, and the alarm
           lamp blinks for one second so the operator sees the request. */
        *warn_out = "gripper not implemented";
        arm_limits_flash_warn();
        ack = ARM_ACK_WARN;
        break;

      case 17:
        arm_limits_clear_estop();
        arm_stepper_enable_all(true);
        break;

      case 18:
        arm_limits_clear_fault();
        arm_stepper_enable_all(false);
        break;

      case 210:
        /* M210 - re-zero: "the pose this arm is standing in is home again".
           Open loop means one lost step, one nudge, one power cycle and the
           model no longer matches the machine; until now the only cure was to
           put the arm back into Gamma by eye and press reset, which also threw
           away every other bit of state.  This does the declaration in place:
           the three step counters become the power-on values (rot 0, low 0,
           high -90 deg) and the interpolator is anchored at the power-on XYZ,
           so the next G1 is planned from a pose the operator has just
           guaranteed.  Nothing moves: pulses still queued belong to the old
           frame and are dropped by arm_stepper_set_home_pose().

           Procedure: M18 (release the joints), put the arm into Gamma by hand
           (lower arm horizontal, forearm straight down, gripper near the
           table), M210, M17.  Sent while the drivers are still energised the
           command is harmless but pointless, hence the note below. */
        arm_stepper_set_home_pose();
        arm_interp_set_position(ARM_BOOT_X_MM, ARM_BOOT_Y_MM, ARM_BOOT_Z_MM);

        arm_uart_debug("M210: re-zeroed here - rot 0, low 0, high -90, "
                       "XYZ 0/120/120\r\n");

        if ((arm_stepper_is_enabled(ARM_AXIS_ROT) != false) ||
            (arm_stepper_is_enabled(ARM_AXIS_LOW) != false) ||
            (arm_stepper_is_enabled(ARM_AXIS_HIGH) != false))
        {
          arm_uart_debug("M210: NOTE the drivers are still enabled, so the arm "
                         "could not have been placed by hand - send M18, place "
                         "it in Gamma, then M210 again\r\n");
        }
        break;

      case 106:
      case 107:
        /* Fan support was removed with the heater, section 3.4. */
        *warn_out = "fan removed";
        arm_limits_flash_warn();
        ack = ARM_ACK_WARN;
        break;

      default:
        *warn_out = "unknown M command";
        ack = ARM_ACK_FAULT;
        break;
    }
  }
  else
  {
    /* executeCommand() final else: handleAsErr() */
    *warn_out = "unknown command";
    ack = ARM_ACK_FAULT;
  }

  return ack;
}

/*============================================================================*/
/* Route one line into the command queue                                      */
/*============================================================================*/

/** Provided by app_freertos.c.  Returns false when the queue is full. */
bool arm_app_post_command(const arm_cmd_t *cmd);

/**
  * @brief  [学习标注] 串口一行 → 解析 → 入队 → 立刻回 ok；解析失败回 rs（不入队）。
  */
bool arm_command_handle_line(const char *line)
{
  arm_cmd_t cmd;

  if (arm_command_parse(line, &cmd) == false)
  {
    /* command.cpp:37-40 and :42-47 - the parse failed, so the host is asked to
       resend.  Nothing is queued and no "ok" is sent. */
    arm_uart_print_resend();
    return true;
  }

  if (arm_app_post_command(&cmd) == false)
  {
    /* Queue full: the original silently dropped the command and still sent
       "ok" (robotArm.ino:93-97).  Reporting a resend here would be friendlier
       but would differ from the original byte stream, so the drop is logged
       on the debug link and "ok" is sent. */
    arm_uart_debug("command queue full, command dropped\r\n");
  }

  /* "ok" is sent at enqueue time, exactly like printOk() in the original. */
  arm_uart_print_ok();
  return true;
}

/*============================================================================*/
/* Self test                                                                  */
/*============================================================================*/

bool arm_command_selftest(void)
{
  static const struct
  {
    const char *line;
    bool        expect_ok;
    char        expect_id;
    int16_t     expect_num;
    float       expect_x;
  } cases[] =
  {
    { "G1 X10 Y-2.5 Z0",  true,  'G', 1,   10.0f },
    { "g0 x1.25",         true,  'G', 0,    1.25f },
    { "M17",              true,  'M', 17,   0.0f },
    { "M106",             true,  'M', 106,  0.0f },
    { "T1",               false, 0,   0,    0.0f },
    { "G1",               true,  'G', 1,    0.0f },
  };

  for (uint32_t i = 0u; i < (sizeof(cases) / sizeof(cases[0])); i++)
  {
    arm_cmd_t cmd;
    bool ok = arm_command_parse(cases[i].line, &cmd);

    if (ok != cases[i].expect_ok)
    {
      arm_uart_debugf("parser self test FAIL at case %lu\r\n", (unsigned long)i);
      return false;
    }

    if (ok == false)
    {
      continue;
    }

    if ((cmd.id != cases[i].expect_id) || (cmd.num != cases[i].expect_num))
    {
      arm_uart_debugf("parser self test FAIL id/num at case %lu\r\n", (unsigned long)i);
      return false;
    }

    if ((cases[i].expect_x != 0.0f) && (cmd.valueX != cases[i].expect_x))
    {
      arm_uart_debugf("parser self test FAIL X at case %lu\r\n", (unsigned long)i);
      return false;
    }
  }

  return true;
}
