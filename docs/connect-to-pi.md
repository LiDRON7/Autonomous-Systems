# Connect to the drone's Pi

```bash
ssh lidron-cranberry@lidron-pi
```

That's the command at the university, at home, or anywhere with internet, once you've done the one-time setup below. In the field there's no internet, so see [In the field](#in-the-field).

Ask Alex for the Pi password. It's never written in the repos.

## One-time setup (about 5 minutes)

1. Install Tailscale on your computer from [tailscale.com/download](https://tailscale.com/download). It's available for macOS, Windows and Linux.
2. Open Tailscale and log in with any account (Google, GitHub or Microsoft).
3. Ask Alex for the share invite to `lidron-pi`, then open the link and accept it.
4. Check that you can see it: `tailscale status` should list `lidron-pi`.
5. Connect: `ssh lidron-cranberry@lidron-pi`. Type `yes` the first time it asks about the fingerprint.

On Windows, run the `ssh` command in PowerShell. It's built in.

Tailscale has to be running on your computer whenever you connect. It works on campus Wi-Fi, which blocks other ways of finding the Pi.

## In the field

The Pi makes its own Wi-Fi network when there's no internet.

1. Join the Wi-Fi network `LiDRON`. Ask Alex for the Wi-Fi password.
2. Connect: `ssh lidron-cranberry@10.42.0.1`

Tailscale doesn't work here, because it needs internet.

## If it doesn't connect

| Problem | Fix |
| --- | --- |
| `Could not resolve hostname lidron-pi` | Tailscale isn't running or isn't logged in on your computer. Open it and check `tailscale status`. |
| `lidron-pi` is missing from `tailscale status` | You haven't accepted the share invite, or the Pi is off or offline. |
| `Permission denied` | Wrong password. Ask Alex. |
| `Connection timed out` in the field | Check you're on the `LiDRON` Wi-Fi and the Pi is powered. |

## Once you're in

The code is in the two repos on the Pi. Start the containers as described in the Sensor-Control-Data-Gathering README, then the autonomy README.
