# Raspberry Pi Environment and Git Sync

## Connection

```text
SSH alias: robopi
Actual host: raspberrypi.local
User: pi
Remote project directory: ~/robogame
```

The SSH alias is configured on the Windows development machine. Passwords, private keys, and other SSH secrets must never be committed to this repository.

## Git sync workflow

Windows local workflow:

```text
edit
  -> git status / git diff
  -> commit
  -> push
```

Raspberry Pi update command:

```bash
ssh robopi "cd ~/robogame && git pull --ff-only"
```

Before pulling, always check that the Raspberry Pi worktree is clean:

```bash
ssh robopi "cd ~/robogame && git status --short"
```

After synchronization, verify the worktree and latest commit:

```bash
ssh robopi "cd ~/robogame && git status --short && git log -1 --oneline"
```

## Safety rules

- Never run `git reset --hard` automatically.
- Never run `git clean -fd` automatically.
- Never use `git push --force`.
- If the Raspberry Pi has uncommitted changes, stop synchronization and report them.
- Never overwrite unknown local or remote modifications.
- Never commit passwords, SSH private keys, or other credentials.
- After synchronization, do not automatically run any program that may move the chassis, arm, Stepper, or Servo.

## Serial devices

All listed serial links use `115200` baud.

STM32:

```text
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

K230D / CanMV:

```text
/dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00
```
