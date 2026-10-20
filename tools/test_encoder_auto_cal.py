#!/usr/bin/env python3
"""Host regressions of the real firmware calibration and control paths; no hardware."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class EncoderAutoCalTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="enc-auto-cal-")
        cls.addClassCleanup(cls.temp.cleanup)
        path = Path(cls.temp.name)
        (path / "main.h").write_text('''
#pragma once
#include <stdint.h>
typedef struct { uint32_t BDTR; } TIM_TypeDef;
extern TIM_TypeDef Test_TIM;
#define TIM1 (&Test_TIM)
#define TIM_BDTR_MOE 0x8000U
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t Value) { (void)Value; }
''')
        (path / "tx_api.h").write_text('''
#pragma once
typedef unsigned int UINT;
typedef uint32_t ULONG;
typedef void VOID;
typedef struct { int Unused; } TX_BYTE_POOL;
typedef struct { int Unused; } TX_QUEUE;
typedef struct { int Unused; } TX_SEMAPHORE;
#define TX_SUCCESS 0U
#define TX_NO_WAIT 0U
''')
        # Compile the command and completion functions verbatim, omitting only
        # RTOS thread creation/timing. Queue, protocol and worker are boundaries.
        thread = (ROOT / "AZURE_RTOS/App/motor_thread.c").read_text()
        commands = "\n".join(line for line in thread.splitlines()
                             if re.match(r"static (bool|Motor_Cmd_Msg_T) \w+_Action_", line))
        for name in ("Motor_Action_Id_Get", "Motor_Action_Txn_Get",
                     "Motor_Action_Response", "Motor_Action_Complete",
                     "Motor_Async_Action_Poll", "Motor_Cmd_Run"):
            match = re.search(r"^static [^\n]+\b" + name + r"\(", thread, re.M)
            start = thread.index("{", match.start())
            depth, end = 1, start + 1
            while depth:
                depth += (thread[end] == "{") - (thread[end] == "}")
                end += 1
            commands += "\n" + thread[match.start():end] + "\n"
        (path / "commands.inc").write_text(commands)
        cls.binary = path / "check"
        sources = ["tools/tests/encoder_auto_cal.c", "Motor/Motor_Control.c",
                   "Motor/Motor_Config.c", "Motor/Motion_Loop.c", "Motor/Current_Loop.c",
                   "Motor/Signal_Injection.c", "Motion/Motion.c", "Motion/Ramp.c",
                   "Motion/Trapezoid.c", "Motion/SCurve.c", "Algo/Math.c", "Algo/PID.c",
                   "Algo/LADRC.c", "Algo/Sin_LUT.c", "Sensorless/VF.c"]
        subprocess.run(["cc", "-O2", "-Wall", "-Wextra", "-Werror", "-ffp-contract=off",
                        "-I" + str(path), "-IMotor", "-IMotion", "-IAlgo", "-IUser",
                        "-IObserver", "-IIdentification", "-ISensorless", "-IAZURE_RTOS/App",
                        "-IParameter", "-IComm",
                        *sources, "-lm", "-o", str(cls.binary)], cwd=ROOT, check=True)

    def run_case(self, name):
        subprocess.run([str(self.binary), name], check=True)

    def test_all_user_modes_and_existing_phase_required(self): self.run_case("modes")
    def test_rotating_transfer_pi_ladrc_feedforward(self): self.run_case("transfer")
    def test_fit_apply_and_rephase(self): self.run_case("fit")
    def test_sample_quality_and_sync(self): self.run_case("quality")
    def test_measured_stop_and_timeout(self): self.run_case("decel")
    def test_stop_disable_at_every_stage(self): self.run_case("abort")
    def test_commands_cancel_and_save_completion(self): self.run_case("commands")
    def test_normal_servo_after_calibration(self): self.run_case("normal")


if __name__ == "__main__":
    unittest.main()
