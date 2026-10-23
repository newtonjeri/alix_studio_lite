# RealMan SDK calls that never return (20-23 Oct 2026)

Twenty times in four lab days a call into RealMan's SDK did not return. Lite gave the arm's connection up and connected it
again: after 20 s for a quick call, and after the program's length plus a margin for a program start. The SDK's
reply timeout is set to 1.5 s (`alix_dispatch.connect.ACK_TIMEOUT_MS`), and both wrappers document a `-2` return
when the controller does not answer in time. This note says where the SDK can wait outside that timeout. It does not
yet say which thread made it wait.

## The calls

| Day | Time | Arm | Call | Given up after | Doing |
|---|---|---|---|---|---|
| 20 Oct | 12:23:31 | RIGHT | `rm_get_current_work_frame` | 20 s | MoveJ to R_R05_STP |
| 20 Oct | 12:36:27 | LEFT | `rm_get_current_work_frame` | 21 s | saving global point L_R06_STP |
| 20 Oct | 15:40:30 | LEFT | `rm_set_program_id_run` | 71 s | running program 57 |
| 20 Oct | 15:44:44 | LEFT | `rm_get_current_work_frame` | 21 s | MoveJ to L_R07_P1_STP |
| 20 Oct | 16:05:34 | LEFT | `rm_get_current_work_frame` | 20 s | saving global point L_R07_P2_STP |
| 20 Oct | 16:16:38 | LEFT | `rm_get_current_work_frame` | 21 s | saving global point L_R08_STP |
| 20 Oct | 16:28:52 | LEFT | `rm_get_current_work_frame` | 21 s | MoveJ to L_R08_STP |
| 20 Oct | 18:19:11 | RIGHT | `rm_get_current_work_frame` | 20 s | saving global point R_R08_P1_STP |
| 20 Oct | 18:20:49 | LEFT | `rm_get_current_work_frame` | 20 s | saving global point L_R10_P2_STP |
| 21 Oct | 10:25:13 | RIGHT | `rm_get_current_work_frame` | 21 s | MoveJ to R_R08_P1_STP |
| 21 Oct | 10:33:23 | LEFT | `rm_set_arm_stop` | 20 s | a reading |
| 21 Oct | 10:33:24 | RIGHT | `rm_set_program_id_run` | 80 s | running program 58 |
| 21 Oct | 17:07:12 | LEFT | `rm_set_program_id_run` | 107 s | running program 80 |
| 21 Oct | 18:56:28 | LEFT | `rm_set_program_id_run` | 72 s | running program 55 |
| 21 Oct | 19:13:30 | LEFT | `rm_set_program_id_run` | 83 s | running program 81 |
| 22 Oct | 11:11:05 | RIGHT | `rm_set_program_id_run` | 98 s | running program 20 |
| 22 Oct | 16:34:09 | LEFT | `rm_get_current_work_frame` | 21 s | MoveJ to L_HME_POS |
| 23 Oct | 11:49:16 | RIGHT and LEFT | `rm_get_current_work_frame` | 20 s | S2, its first step (Global MoveJ Home) on both arms at once |
| 23 Oct | 12:21:26 | LEFT | `rm_set_program_id_run` | 89 s | running program 63 (Hardware sim) |

Normally the frame read takes 27-51 ms (the median of 45 Global moves, 21-23 Oct; at most 86 ms).

Where the calls come from:
- The frame reads are runner v14's `_capture_current_frames` (`vendor/control_app/rm_upload_fast_v14_safe_transit.py:1428`). It reads the work frame and then the tool frame before every Global MoveJ, so it can restore them afterwards. Lite's `RealArm.save_global_point` (`src/alix_studio_lite/server/robot_real.py:994`) calls the same function to store the frame names with a point.
- The program starts are `rm_upload.run` (S1 Run, Hardware sim) or runner v14 (S2), both non-blocking (`timeout` 0).

The thread dumps written at the give-up (`Resources/Logs/2026-10-23/threads-*-114916.txt`, `threads-left-122126.txt`) show the stuck Python thread inside the SDK function. The worker's other Python threads were idle at the time, none of them inside the SDK.

## What the SDK library does

Source: `vendor/alix/external/butterfli-realman-alix/Python/Robotic_Arm/libs/linux_x86/libapi_c.so`, disassembled with `objdump -d`.

1. **Every request goes through `rm_pack_lock_set`** (0x27b50), in this order:
   - `rm_rdlock_lock`, the read side of one process-wide `pthread_rwlock` (`rwlock`, shared by every arm handle);
   - the handle lookup;
   - `clear_fifo_all`;
   - the send (`common_json_pack_set`);
   - the reply wait (`handle_revice_data`);
   - `rm_rdlock_unlock`.
2. **The reply wait is bounded.** In triple-thread mode, which ALIX's `connect.open_arm` uses, `handle_revice_data` (0x277d0) runs this loop at most timeout/2 times, about 1.5 s here: take a string from the FIFO (`get_str_from_fifo`), parse it, compare its command, sleep 2 ms (`sleep_cp(2)`). Then it returns -1. Unrelated messages do not restart the count.
3. **The lock has no timeout.** `rm_rdlock_lock` and `rm_wrlock_lock` are plain `pthread_rwlock_rdlock` and `pthread_rwlock_wrlock` on that one lock (0x23900, 0x23910). Calls that also take the write lock:

   | Called | Functions |
   |---|---|
   | Each time it reads a reply | `rm_get_current_work_frame`, `rm_get_current_tool_frame` |
   | Writing the push setup (twice) | `rm_get_realtime_push` |
   | Otherwise | `rm_set_move_block_state`, `rm_create_robot_arm`, `rm_delete_robot_arm`, `rm_destroy`, `rm_get_arm_software_info`, `rm_get_DH_data`, `rm_get_install_pose`, `rm_get_joint_max_acc`, `rm_get_joint_max_pos`, `rm_get_joint_max_speed`, `rm_get_joint_min_pos`, `rm_set_robot_dof` |

4. **`rm_set_program_id_run` with `timeout` 0** (0x3f730) takes the read lock only for the handle lookup and releases it, then calls `rm_pack_lock_set` (step 1). Its loop over `rm_get_program_run_state` and `rm_get_flowchart_program_run_state` is on the blocking path, which Lite never takes.

So a call that does not return for 20-107 s is not waiting for a reply: that wait ends after 1.5 s. It is waiting either to take the process-wide lock or inside the send, and neither has a timeout. `rm_get_current_work_frame` also needs the write lock after its reply. That waits until no other thread holds the read lock, including the SDK's own threads.

## What is not known yet

Which thread holds the lock, or whether the send itself blocks. The Python thread dumps cannot show the SDK's native threads. On 23 Oct at 11:49:16 both arms' processes hung in the same call at the same moment, on two separate controllers. That points at something both had in common then, such as the network, rather than at chance.

## Options (the owner's to choose)

1. **Native stacks at the give-up.** When Lite gives a call up, also write the worker process's native thread stacks (`gdb -p <pid> -batch -ex "thread apply all bt"`, or `eu-stack -p <pid>`, whichever the lab computer has) next to `threads-*.txt`. The next hang then names the native function each thread waits in, and which one holds the lock. This is diagnostic only.
2. **Fewer write-lock calls where the arms work.**
   - Lite's `save_global_point` can read the frame names once per connection and keep them. Lite never changes frames.
   - The runner's `_capture_current_frames`, before every Global MoveJ, is the owner's code. Whether it should keep reading both frames each time is the owner's call.
3. **Report it to RealMan** with this evidence: one process-wide lock without a timeout around every call, and two getters that also need the write lock.
