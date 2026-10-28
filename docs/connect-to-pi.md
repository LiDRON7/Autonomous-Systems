# Connect to the drone's Pi

```bash
ssh lidron-cranberry@lidron-pi
```

That's the command at the university, at home, or anywhere with internet, once you've done the one-time setup below. In the field there's no internet, so see [In the field](#in-the-field).

Ask the leads for the Pi password. It's never written in the repos.

## Use the lab computer if you can

The lab computer already has Tailscale set up, so just open a terminal and run the command above.

Tailscale allows only 6 devices on the LiDRON account, and some are already taken by the Pi and the lab computer. Only add your own computer if you really need to work from somewhere else.

## Set up your own computer (only if needed, about 5 minutes)

1. Install Tailscale from [tailscale.com/download](https://tailscale.com/download). It's available for macOS, Windows and Linux.
2. Open Tailscale and log in with the LiDRON Gmail account. Ask the leads for the password.
3. Check that you can see the Pi: `tailscale status` should list `lidron-pi`.
4. Connect: `ssh lidron-cranberry@lidron-pi`. Type `yes` the first time it asks about the fingerprint.

On Windows, run the `ssh` command in PowerShell. It's built in.

Tailscale has to be running whenever you connect. It works on campus Wi-Fi, which blocks other ways of finding the Pi.

When you no longer need it, log out of Tailscale on your computer so the device slot is freed for someone else.

## In the field

The Pi makes its own Wi-Fi network when there's no internet.

1. Join the Wi-Fi network `LiDRON`. Ask the leads for the Wi-Fi password.
2. Connect: `ssh lidron-cranberry@10.42.0.1`

Tailscale doesn't work here, because it needs internet.

## If it doesn't connect

| Problem | Fix |
| --- | --- |
| `Could not resolve hostname lidron-pi` | Tailscale isn't running or isn't logged in on your computer. Open it and check `tailscale status`. |
| `lidron-pi` is missing from `tailscale status` | You're logged in with a different account, or the Pi is off or offline. |
| `Permission denied` | Wrong password. Ask the leads. |
| `Connection timed out` in the field | Check you're on the `LiDRON` Wi-Fi and the Pi is powered. |

## Once you're in

The code is in the two repos on the Pi. Start the containers as described in the Sensor-Control-Data-Gathering README, then the autonomy README.
