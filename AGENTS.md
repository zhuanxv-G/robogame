# RoboGame collaboration rules

- Preserve unrelated worktree changes and stage only files belonging to the current task.
- Do not change robot control logic, verified vision/mechanical parameters, serial protocols, device paths, or STM32 code unless the task explicitly requests it.
- Do not run programs that may move the chassis, arm, Stepper, or Servo unless the user explicitly requests that execution.
- Never commit passwords, SSH private keys, or other credentials.

## Raspberry Pi deployment

After completing a Raspberry Pi Python task, use this deployment workflow by default:

1. Inspect local `git status` and `git diff`.
2. Stage only files changed for the current task, then commit and push normally. Never force-push.
3. Before pulling on the Pi, run `ssh robopi "cd ~/robogame && git status --short"`.
4. If the Pi reports any uncommitted change, stop: do not pull, reset, clean, or overwrite it.
5. If the Pi is clean, run `ssh robopi "cd ~/robogame && git pull --ff-only"`.
6. Verify with `ssh robopi "cd ~/robogame && git status --short && git log -1 --oneline"`.
7. Report the commit, push result, pull result, Pi HEAD, and Pi worktree status.

Deployment must not automatically start any robot program after synchronization. Never run `git reset --hard` or `git clean -fd` as part of this workflow.
